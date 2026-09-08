/*
*
* Zero-config node ID negotiation
* -------------------------------
*
* A heartbeat message is a message with a 8 byte unique serial number as payload.
* A regular message is any message that is not a heartbeat message.
*
* All nodes MUST obey these four rules:
*
* a) At a given point in time, a node MUST consider a node ID taken (by others)
*   if any of the following is true:
*     - the node received a (not self-emitted) heartbeat message with that node ID
*       within the last second
*     - the node attempted and failed at sending a heartbeat message with that
*       node ID within the last second (failed in the sense of not ACK'd)
*
* b) At a given point in time, a node MUST NOT consider a node ID self-assigned
*   if, within the last second, it did not succeed in sending a heartbeat
*   message with that node ID.
*
* c) At a given point in time, a node MUST NOT send any heartbeat message with
*   a node ID that is taken.
*
* d) At a given point in time, a node MUST NOT send any regular message with
*   a node ID that is not self-assigned.
*
* Hardware allocation
* -------------------
*   RX FIFO0:
*       - filter bank 0: heartbeat messages
*/

#include "common_inc.h"
#include <stm32f4xx_hal.h>
#include <cmsis_os.h>
#include "../../../can_tx_lifecycle.hpp"

// defined in can.c
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_context can1Ctx;
CAN_context can2Ctx;
static CAN_context* ctxs = nullptr;

namespace
{
constexpr uint32_t kCanBlockingCompatibilityWaitMs = 8U;

osSemaphoreId TxSemaphore(CAN_HandleTypeDef* hcan)
{
    if (hcan->Instance == CAN1)
        return sem_can1_tx;
    if (hcan->Instance == CAN2)
        return sem_can2_tx;
    return nullptr;
}

void ReleaseTxToken(CAN_HandleTypeDef* hcan)
{
    const osSemaphoreId semaphore = TxSemaphore(hcan);
    if (semaphore != nullptr)
        (void) osSemaphoreRelease(semaphore);
}

IRQn_Type CanTxIrq(CAN_HandleTypeDef* hcan)
{
    return hcan->Instance == CAN1 ? CAN1_TX_IRQn : CAN2_TX_IRQn;
}

uint32_t MailboxTmeBit(uint8_t mailbox_idx)
{
    return mailbox_idx == 0U ? CAN_TSR_TME0
        : mailbox_idx == 1U ? CAN_TSR_TME1 : CAN_TSR_TME2;
}

uint32_t MailboxTxokBit(uint8_t mailbox_idx)
{
    return CAN_TSR_TXOK0 << (static_cast<uint32_t>(mailbox_idx) * 8U);
}

void PushTxCompletion(CAN_context* ctx, CanTxCompletionStatus status,
                      uint8_t mailbox_idx, bool notify_from_isr = true)
{
    if (ctx == nullptr)
        return;
    const uint32_t completed_us = micros();
    const uint8_t write = ctx->tx_completion_write;
    const uint8_t next = static_cast<uint8_t>(
        (write + 1U) % kCanTxCompletionCapacity);
    if (next == ctx->tx_completion_read)
    {
        ++ctx->tx_completion_overflow_count;
    }
    else
    {
        auto& completion = ctx->tx_completion_ring[write];
        completion = {};
        if (ctx->active_tx_metadata_valid)
            completion.metadata = ctx->active_tx_metadata;
        completion.completed_us = completed_us;
        completion.status = status;
        completion.mailbox_index = mailbox_idx;
        ctx->tx_completion_write = next;
    }
    ctx->active_tx_metadata = {};
    ctx->active_tx_metadata_valid = false;
    if (notify_from_isr)
        NotifyCanDispatcherFromIsr();
}

void SaturatingIncrement(volatile uint32_t& value)
{
    if (value != UINT32_MAX)
        ++value;
}

// Applies one lifecycle transition from the shared state machine (doc 07
// R01). The machine is the single authority for the business terminal, the
// token and the blocked flag; this function only maps its output onto the
// transport structures.
void ApplyTxTransition(CAN_context* ctx,
                       const dummy::can_tx::Transition& transition,
                       uint8_t mailbox_idx, bool notify_from_isr = true)
{
    if (ctx == nullptr)
        return;
    ctx->tx_state = transition.next;
    if (transition.count_stale)
        SaturatingIncrement(ctx->stale_tx_callback_count);
    if (transition.count_recovery)
        SaturatingIncrement(ctx->tx_abort_recovery_count);
    if (transition.terminal != dummy::can_tx::Terminal::None &&
        ctx->active_tx_metadata_valid)
    {
        const CanTxCompletionStatus status =
            transition.terminal == dummy::can_tx::Terminal::Complete
            ? CanTxCompletionStatus::Complete
            : transition.terminal == dummy::can_tx::Terminal::Aborted
                ? CanTxCompletionStatus::Aborted
                : CanTxCompletionStatus::Error;
        PushTxCompletion(ctx, status, mailbox_idx, notify_from_isr);
    }
    if (transition.release_token)
    {
        ctx->active_tx_metadata = {};
        ctx->active_tx_metadata_valid = false;
        ReleaseTxToken(ctx->handle);
    }
    ctx->tx_channel_blocked =
        transition.next == dummy::can_tx::State::Blocked;
}

void QueueRxFrame(CAN_HandleTypeDef* hcan, uint32_t fifo)
{
    CAN_context* ctx = get_can_ctx(hcan);
    if (ctx == nullptr)
        return;
    CanRxFrame frame{};
    frame.received_us = micros();
    SaturatingIncrement(ctx->received_msg_cnt);
    if (HAL_CAN_GetRxMessage(
            hcan, fifo, &frame.header, frame.data) != HAL_OK)
    {
        SaturatingIncrement(ctx->unexpected_errors);
        return;
    }
    ctx->busoff_active = false;
    if (!ctx->rx_ring.Push(frame))
    {
        SaturatingIncrement(ctx->rx_overflow_count);
    }
    else
    {
        const size_t depth = ctx->rx_ring.Size();
        if (depth > ctx->rx_high_water)
            ctx->rx_high_water = static_cast<uint8_t>(depth);
    }
    NotifyCanDispatcherFromIsr();
}
}


struct CAN_context* get_can_ctx(CAN_HandleTypeDef* hcan)
{
    if (hcan->Instance == CAN1)
        return &can1Ctx;
    else if (hcan->Instance == CAN2)
        return &can2Ctx;
    else
        return nullptr;
}

bool StartCanServer(CAN_TypeDef* hcan)
{
    if (hcan == CAN1)
    {
        ctxs = &can1Ctx;
        ctxs->handle = &hcan1;
    } else if (hcan == CAN2)
    {
        ctxs = &can2Ctx;
        ctxs->handle = &hcan2;
    } else
        return false; // fail if none of the above checks matched

    HAL_StatusTypeDef status;

    ctxs->node_id = 0;
    ctxs->serial_number = serialNumber;
    osSemaphoreDef(sem_send_heartbeat);
    ctxs->sem_send_heartbeat = osSemaphoreNew(1, 0, osSemaphore(sem_send_heartbeat));

    //// Set up filter
    CAN_FilterTypeDef sFilterConfig = {
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterBank = hcan == CAN1 ? 0U : 14U,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_16BIT, // two 16-bit filters
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 14U
    };
    status = HAL_CAN_ConfigFilter(ctxs->handle, &sFilterConfig);
    if (status != HAL_OK)
        return false;

    status = HAL_CAN_Start(ctxs->handle);
    if (status != HAL_OK)
        return false;

    status = HAL_CAN_ActivateNotification(ctxs->handle,
                                          CAN_IT_TX_MAILBOX_EMPTY |
                                          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |
                                          /* we probably only want this */
                                          CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO1_FULL |
                                          CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN |
                                          CAN_IT_WAKEUP | CAN_IT_SLEEP_ACK |
                                          CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                                          CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE |
                                          CAN_IT_ERROR);
    if (status != HAL_OK)
        return false;

    return true;
}

void tx_complete_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    CAN_context* ctx = get_can_ctx(hcan);
    if (ctx == nullptr)
        return;
    // Mailbox identity check: an IRQ for a mailbox that is not the active
    // one, or for a channel without a live transaction, is stale evidence
    // and must never settle a later frame (doc 07 R01).
    const UBaseType_t saved_interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
    const bool matched = ctx->tx_state != CanTxLifecycleState::Idle &&
        ctx->tx_state != CanTxLifecycleState::Blocked &&
        mailbox_idx == ctx->active_mailbox_index;
    const auto transition = dummy::can_tx::Advance(
        ctx->tx_state,
        matched ? dummy::can_tx::Event::CompleteCallback
                : dummy::can_tx::Event::StaleCallback);
    const bool was_complete =
        transition.terminal == dummy::can_tx::Terminal::Complete;
    ApplyTxTransition(ctx, transition, mailbox_idx);
    taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_mask);
    if (was_complete)
    {
        ctx->busoff_active = false;
        ctx->tx_msg_cnt++;
        ctx->TxMailboxCompleteCallbackCnt++;
    }
}

void tx_aborted_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    CAN_context* ctx = get_can_ctx(hcan);
    if (ctx == nullptr)
        return;
    const UBaseType_t saved_interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
    const bool matched = ctx->tx_state != CanTxLifecycleState::Idle &&
        ctx->tx_state != CanTxLifecycleState::Blocked &&
        mailbox_idx == ctx->active_mailbox_index;
    const auto transition = dummy::can_tx::Advance(
        ctx->tx_state,
        matched ? dummy::can_tx::Event::AbortCallback
                : dummy::can_tx::Event::StaleCallback);
    const bool was_aborted =
        transition.terminal == dummy::can_tx::Terminal::Aborted;
    ApplyTxTransition(ctx, transition, mailbox_idx);
    taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_mask);
    if (was_aborted)
        ctx->TxMailboxAbortCallbackCnt++;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{ tx_complete_callback(hcan, 0); }

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan)
{ tx_complete_callback(hcan, 1); }

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan)
{ tx_complete_callback(hcan, 2); }

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef* hcan)
{ tx_aborted_callback(hcan, 0); }

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef* hcan)
{ tx_aborted_callback(hcan, 1); }

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef* hcan)
{ tx_aborted_callback(hcan, 2); }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    QueueRxFrame(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->RxFifo0FullCallbackCnt++; }

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_context* ctx = get_can_ctx(hcan);
    if (ctx != nullptr)
        ++ctx->RxFifo1MsgPendingCallbackCnt;
    QueueRxFrame(hcan, CAN_RX_FIFO1);
}

void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->RxFifo1FullCallbackCnt++; }

void HAL_CAN_SleepCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->SleepCallbackCnt++; }

void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->WakeUpFromRxMsgCallbackCnt++; }

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
    CAN_context* ctx = get_can_ctx(hcan);
    if (!ctx) return;
    const uint32_t errors = hcan->ErrorCode;
    if ((errors & HAL_CAN_ERROR_BOF) != 0U && !ctx->busoff_active)
    {
        ++ctx->busoff_count;
        ctx->busoff_active = true;
    }
    constexpr uint32_t kRetryableTxErrors =
        HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR0 |
        HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_TERR1 |
        HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR2;
    if ((errors & kRetryableTxErrors) != 0U)
        SaturatingIncrement(ctx->tx_recovery_count);
    if (errors != HAL_CAN_ERROR_NONE)
        SaturatingIncrement(ctx->unexpected_errors);
}

CanTxStatus CanTrySendMessage(CAN_context* canCtx, uint8_t* txData,
                              CAN_TxHeaderTypeDef* txHeader,
                              CanTxQueuedCallback on_queued,
                              void* callback_context,
                              const CanTxMetadata* metadata)
{
    if (canCtx == nullptr || canCtx->handle == nullptr || txData == nullptr ||
        txHeader == nullptr)
        return CanTxStatus::Invalid;

    // A blocked channel has not yet confirmed that its old request is gone;
    // reusing it would break the one-frame-in-flight premise (doc 07 R01).
    if (canCtx->tx_channel_blocked)
        return CanTxStatus::Invalid;

    canCtx->tx_attempt_count++;

    const osSemaphoreId semaphore = TxSemaphore(canCtx->handle);
    if (semaphore == nullptr)
        return CanTxStatus::Invalid;

    if (osSemaphoreAcquire(semaphore, 0U) != osOK)
    {
        canCtx->tx_busy_count++;
        return CanTxStatus::Busy;
    }

    // Keep send accounting atomic with mailbox admission. Without this, a
    // motor response or the realtime control task could run between enqueue
    // and feedback bookkeeping and manufacture a false missed-response count.
    taskENTER_CRITICAL();
    // Before reusing a mailbox, drop any residual TX interrupt from the
    // previous transaction so a late ISR can never settle the new frame
    // (doc 07 R01). Requesting the mailbox clears its leftover TSR flags.
    NVIC_ClearPendingIRQ(CanTxIrq(canCtx->handle));
    const HAL_StatusTypeDef send_status = HAL_CAN_AddTxMessage(
        canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox);
    const auto transition = dummy::can_tx::Advance(
        canCtx->tx_state,
        send_status == HAL_OK ? dummy::can_tx::Event::EnqueueSucceeded
                              : dummy::can_tx::Event::EnqueueFailed);
    ApplyTxTransition(canCtx, transition, 0U, false);
    if (send_status == HAL_OK)
    {
        canCtx->tx_started_us = micros();
        canCtx->tx_recovery_attempts = 0U;
        if (canCtx->last_heartbeat_mailbox == CAN_TX_MAILBOX0)
            canCtx->active_mailbox_index = 0U;
        else if (canCtx->last_heartbeat_mailbox == CAN_TX_MAILBOX1)
            canCtx->active_mailbox_index = 1U;
        else
            canCtx->active_mailbox_index = 2U;
        canCtx->tx_queued_count++;
        canCtx->active_tx_metadata = metadata == nullptr
            ? CanTxMetadata{} : *metadata;
        canCtx->active_tx_metadata_valid = metadata != nullptr;
        if (on_queued != nullptr)
            on_queued(callback_context);
    }
    taskEXIT_CRITICAL();
    if (send_status != HAL_OK)
    {
        // No completion interrupt will be generated when the frame was never
        // queued; the lifecycle transition above already released the token
        // and delivered the Error terminal evidence.
        canCtx->unexpected_errors++;
        canCtx->tx_enqueue_error_count++;
        return CanTxStatus::Error;
    }
    return CanTxStatus::Queued;
}

void CanServiceTxDeadline(CAN_context* canCtx, uint32_t now_us,
                          uint32_t timeout_us)
{
    if (canCtx == nullptr || canCtx->handle == nullptr || timeout_us == 0U)
        return;

    // First deadline: an in-flight frame that has not completed is aborted.
    if (canCtx->tx_state == CanTxLifecycleState::InFlight &&
        now_us - canCtx->tx_started_us >= timeout_us)
    {
        taskENTER_CRITICAL();
        if (canCtx->tx_state != CanTxLifecycleState::InFlight ||
            now_us - canCtx->tx_started_us < timeout_us)
        {
            taskEXIT_CRITICAL();
            return;
        }
        canCtx->abort_requested_us = now_us;
        const uint8_t mailbox_index = canCtx->active_mailbox_index;
        taskEXIT_CRITICAL();

        const HAL_StatusTypeDef abort_status = HAL_CAN_AbortTxRequest(
            canCtx->handle, 1UL << mailbox_index);
        taskENTER_CRITICAL();
        if (abort_status != HAL_OK)
        {
            ++canCtx->tx_enqueue_error_count;
            ++canCtx->unexpected_errors;
        }
        ApplyTxTransition(
            canCtx,
            dummy::can_tx::Advance(
                canCtx->tx_state,
                abort_status == HAL_OK
                    ? dummy::can_tx::Event::AbortAccepted
                    : dummy::can_tx::Event::AbortRejected),
            mailbox_index, false);
        taskEXIT_CRITICAL();
        return;
    }

    // Second deadline (doc 05 section 3.5 / 9.5, doc 07 R01): an abort whose
    // callback never arrived must not hold the single TX token forever.
    // Converge on hardware evidence: TME+TXOK is a real completion, TME
    // without TXOK is an aborted/errored transmission, a still-pending
    // mailbox moves the channel into RecoveryRequired and KEEPS the token.
    if (canCtx->tx_state == CanTxLifecycleState::AbortRequested &&
        now_us - canCtx->abort_requested_us >= timeout_us)
    {
        taskENTER_CRITICAL();
        if (canCtx->tx_state != CanTxLifecycleState::AbortRequested ||
            now_us - canCtx->abort_requested_us < timeout_us)
        {
            taskEXIT_CRITICAL();
            return;
        }
        const uint8_t mailbox_index = canCtx->active_mailbox_index;
        const uint32_t tsr = canCtx->handle->Instance->TSR;
        dummy::can_tx::Event event = dummy::can_tx::Event::MailboxStillPending;
        if ((tsr & MailboxTmeBit(mailbox_index)) != 0U)
        {
            event = (tsr & MailboxTxokBit(mailbox_index)) != 0U
                ? dummy::can_tx::Event::MailboxEndedComplete
                : dummy::can_tx::Event::MailboxEndedAborted;
        }
        ApplyTxTransition(
            canCtx, dummy::can_tx::Advance(canCtx->tx_state, event),
            mailbox_index, false);
        taskEXIT_CRITICAL();
    }
}

bool CanResetCanChannel(CAN_context* canCtx)
{
    if (canCtx == nullptr || canCtx->handle == nullptr)
        return false;
    CAN_HandleTypeDef* hcan = canCtx->handle;

    taskENTER_CRITICAL();
    (void) HAL_CAN_Stop(hcan);
    if (HAL_CAN_Init(hcan) != HAL_OK)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    // HAL reinitialization is not evidence that an old TX request vanished.
    // Never restart/release this channel while any mailbox is still pending.
    constexpr uint32_t all_mailboxes_empty =
        CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2;
    if ((hcan->Instance->TSR & all_mailboxes_empty) != all_mailboxes_empty)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP0);
    __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP1);
    __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP2);
    // Re-apply the receive filter and notification set exactly as the boot
    // path configured them; Init alone does not restore them.
    CAN_FilterTypeDef filter_config = {
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterBank = hcan->Instance == CAN1 ? 0U : 14U,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_16BIT,
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 14U,
    };
    if (HAL_CAN_ConfigFilter(hcan, &filter_config) != HAL_OK ||
        HAL_CAN_Start(hcan) != HAL_OK)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    if (HAL_CAN_ActivateNotification(hcan,
            CAN_IT_TX_MAILBOX_EMPTY |
            CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |
            CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO1_FULL |
            CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN |
            CAN_IT_WAKEUP | CAN_IT_SLEEP_ACK |
            CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
            CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE |
            CAN_IT_ERROR) != HAL_OK)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    // Empty mailboxes were verified and peripheral completion flags consumed;
    // discard any residual NVIC TX notification before channel reuse.
    NVIC_ClearPendingIRQ(CanTxIrq(hcan));
    taskEXIT_CRITICAL();
    return true;
}

void CanServiceTxRecovery(CAN_context* canCtx, uint32_t now_us,
                          uint32_t max_attempts)
{
    (void) now_us;
    if (canCtx == nullptr || canCtx->handle == nullptr)
        return;

    if (canCtx->tx_state == CanTxLifecycleState::Blocked)
    {
        // One bounded reset attempt per wake; the state machine returns the
        // token exactly once when the reset succeeds.
        const bool reset_ok = CanResetCanChannel(canCtx);
        taskENTER_CRITICAL();
        ApplyTxTransition(
            canCtx,
            dummy::can_tx::Advance(
                canCtx->tx_state,
                reset_ok ? dummy::can_tx::Event::ChannelReset
                         : dummy::can_tx::Event::RecoveryExhausted),
            canCtx->active_mailbox_index, false);
        taskEXIT_CRITICAL();
        return;
    }

    if (canCtx->tx_state != CanTxLifecycleState::RecoveryRequired)
        return;

    taskENTER_CRITICAL();
    if (canCtx->tx_state != CanTxLifecycleState::RecoveryRequired)
    {
        taskEXIT_CRITICAL();
        return;
    }
    if (canCtx->tx_recovery_attempts != UINT32_MAX)
        ++canCtx->tx_recovery_attempts;
    const uint8_t mailbox_index = canCtx->active_mailbox_index;
    taskEXIT_CRITICAL();

    // Re-abort (idempotent; a late abort IRQ only converges the channel),
    // then poll the hardware for evidence.
    (void) HAL_CAN_AbortTxRequest(
        canCtx->handle, 1UL << mailbox_index);
    const uint32_t tsr = canCtx->handle->Instance->TSR;
    dummy::can_tx::Event event = dummy::can_tx::Event::MailboxStillPending;
    if ((tsr & MailboxTmeBit(mailbox_index)) != 0U)
    {
        event = (tsr & MailboxTxokBit(mailbox_index)) != 0U
            ? dummy::can_tx::Event::MailboxEndedComplete
            : dummy::can_tx::Event::MailboxEndedAborted;
    }
    else if (canCtx->tx_recovery_attempts >= max_attempts)
    {
        event = CanResetCanChannel(canCtx)
            ? dummy::can_tx::Event::ChannelReset
            : dummy::can_tx::Event::RecoveryExhausted;
    }
    taskENTER_CRITICAL();
    ApplyTxTransition(
        canCtx, dummy::can_tx::Advance(canCtx->tx_state, event),
        mailbox_index, false);
    taskEXIT_CRITICAL();
}

bool CanSendMessage(CAN_context* canCtx, uint8_t* txData,
                    CAN_TxHeaderTypeDef* txHeader,
                    CanTxQueuedCallback on_queued, void* callback_context,
                    const CanTxMetadata* metadata)
{
    const uint32_t started_ms = HAL_GetTick();
    do
    {
        const CanTxStatus status = CanTrySendMessage(
            canCtx, txData, txHeader, on_queued, callback_context, metadata);
        if (status == CanTxStatus::Queued)
            return true;
        if (status == CanTxStatus::Invalid)
            return false;
        osDelay(1U);
    } while (HAL_GetTick() - started_ms < kCanBlockingCompatibilityWaitMs);
    return false;
}

bool CanTakeTxCompletion(CAN_context* canCtx, CanTxCompletion& completion)
{
    if (canCtx == nullptr)
        return false;
    taskENTER_CRITICAL();
    if (canCtx->tx_completion_read == canCtx->tx_completion_write)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    const uint8_t read = canCtx->tx_completion_read;
    completion = canCtx->tx_completion_ring[read];
    canCtx->tx_completion_read = static_cast<uint8_t>(
        (read + 1U) % kCanTxCompletionCapacity);
    taskEXIT_CRITICAL();
    return true;
}

bool CanTakeRxFrame(CAN_context* canCtx, CanRxFrame& frame)
{
    return canCtx != nullptr && canCtx->rx_ring.Pop(frame);
}
