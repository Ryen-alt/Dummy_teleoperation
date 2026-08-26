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

// defined in can.c
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_context can1Ctx;
CAN_context can2Ctx;
static CAN_context* ctxs = nullptr;
static CAN_RxHeaderTypeDef headerRx;
static uint8_t data[8];

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

void PushTxCompletion(CAN_context* ctx, CanTxCompletionStatus status,
                      uint8_t mailbox_idx, bool notify_from_isr = true)
{
    if (ctx == nullptr)
        return;
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
        completion.status = status;
        completion.mailbox_index = mailbox_idx;
        ctx->tx_completion_write = next;
    }
    ctx->active_tx_metadata = {};
    ctx->active_tx_metadata_valid = false;
    if (notify_from_isr)
        NotifyCanDispatcherFromIsr();
}

bool FinishActiveTx(CAN_context* ctx, CanTxCompletionStatus status,
                    uint8_t mailbox_idx, bool notify_from_isr = true)
{
    if (ctx == nullptr || ctx->tx_state == CanTxLifecycleState::Idle)
        return false;
    PushTxCompletion(ctx, status, mailbox_idx, notify_from_isr);
    ctx->tx_state = CanTxLifecycleState::Idle;
    ReleaseTxToken(ctx->handle);
    return true;
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
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_16BIT, // two 16-bit filters
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 0
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
    if (FinishActiveTx(ctx, CanTxCompletionStatus::Complete, mailbox_idx))
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
    if (FinishActiveTx(ctx, CanTxCompletionStatus::Aborted, mailbox_idx))
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
    CAN_context* ctx = get_can_ctx(hcan);
    if (!ctx) return;
    ctx->received_msg_cnt++;

    HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &headerRx, data);
    if (status != HAL_OK)
    {
        ctx->unexpected_errors++;
        return;
    }
    ctx->busoff_active = false;

    OnCanMessage(ctx, &headerRx, data);
    NotifyCanDispatcherFromIsr();
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->RxFifo0FullCallbackCnt++; }

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->RxFifo1MsgPendingCallbackCnt++; }

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
    if (errors != HAL_CAN_ERROR_NONE)
        ++ctx->unexpected_errors;
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
    const HAL_StatusTypeDef send_status = HAL_CAN_AddTxMessage(
        canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox);
    if (send_status == HAL_OK)
    {
        canCtx->tx_state = CanTxLifecycleState::InFlight;
        canCtx->tx_started_us = micros();
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
        // queued, so restore the token immediately.
        canCtx->unexpected_errors++;
        canCtx->tx_enqueue_error_count++;
        ReleaseTxToken(canCtx->handle);
        return CanTxStatus::Error;
    }
    return CanTxStatus::Queued;
}

void CanServiceTxDeadline(CAN_context* canCtx, uint32_t now_us,
                          uint32_t timeout_us)
{
    if (canCtx == nullptr || canCtx->handle == nullptr || timeout_us == 0U ||
        canCtx->tx_state != CanTxLifecycleState::InFlight ||
        now_us - canCtx->tx_started_us < timeout_us)
        return;

    taskENTER_CRITICAL();
    if (canCtx->tx_state != CanTxLifecycleState::InFlight ||
        now_us - canCtx->tx_started_us < timeout_us)
    {
        taskEXIT_CRITICAL();
        return;
    }
    canCtx->tx_state = CanTxLifecycleState::AbortRequested;
    const uint8_t mailbox_index = canCtx->active_mailbox_index;
    taskEXIT_CRITICAL();

    const HAL_StatusTypeDef abort_status = HAL_CAN_AbortTxRequest(
        canCtx->handle, 1UL << mailbox_index);
    if (abort_status == HAL_OK)
        return;

    taskENTER_CRITICAL();
    if (canCtx->tx_state == CanTxLifecycleState::AbortRequested)
    {
        ++canCtx->tx_enqueue_error_count;
        ++canCtx->unexpected_errors;
        (void) FinishActiveTx(
            canCtx, CanTxCompletionStatus::Error, mailbox_index, false);
    }
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
