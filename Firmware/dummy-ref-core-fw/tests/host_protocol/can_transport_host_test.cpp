// Compile the actual BSP transport with HAL/RTOS boundary stubs. These tests
// check ISR API use, mailbox ownership, metadata and token accounting; they
// do not claim real bus timing or electrical recovery coverage.
#include "common_inc.h"
#include <cassert>
#include <iostream>
#include <new>

CAN_TypeDef test_can1{}, test_can2{};
CAN_HandleTypeDef hcan1{CAN1}, hcan2{CAN2};
TestSemaphore semaphore1{}, semaphore2{};
osSemaphoreId sem_can1_tx = &semaphore1, sem_can2_tx = &semaphore2;
uint64_t serialNumber = 1;
static uint32_t now_us = 100, interrupt_mask = 0, task_nesting = 0;
static bool in_isr = false, reject_abort = false, reject_enqueue = false, reject_reset = false;
static bool reset_keeps_pending_mailbox = false;
static unsigned reset_calls = 0;
constexpr uint32_t kEmpty = CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2;

void TestTaskEnterCritical() {
    // The production ARM_CM4F vPortEnterCritical/configASSERT rejects ISR use.
    assert(!in_isr); ++task_nesting; interrupt_mask = 5;
}
void TestTaskExitCritical() { assert(!in_isr && task_nesting); if (--task_nesting == 0) interrupt_mask = 0; }
UBaseType_t TestIsrEnterCritical() { assert(in_isr); auto saved = interrupt_mask; interrupt_mask = 5; return saved; }
void TestIsrExitCritical(UBaseType_t saved) { assert(in_isr); interrupt_mask = saved; }
uint32_t micros() { return now_us; }
uint32_t HAL_GetTick() { return now_us / 1000; }
void osDelay(uint32_t ms) { now_us += ms * 1000; }
void NotifyCanDispatcherFromIsr() { assert(in_isr); }
osStatus_t osSemaphoreAcquire(osSemaphoreId s, uint32_t) { if (!s->tokens) return osError; --s->tokens; return osOK; }
osStatus_t osSemaphoreRelease(osSemaphoreId s) { assert(s->tokens == 0); ++s->tokens; ++s->release_calls; return osOK; }
osSemaphoreId osSemaphoreNew(uint32_t, uint32_t, const void*) { return &semaphore1; }
void NVIC_ClearPendingIRQ(IRQn_Type) {}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef* h, CAN_TxHeaderTypeDef*, uint8_t*, uint32_t* mailbox) {
    if (reject_enqueue) return HAL_ERROR;
    for (unsigned i = 0; i < 3; ++i) {
        if (h->Instance->TSR & (CAN_TSR_TME0 << i)) {
            h->Instance->TSR &= ~((CAN_TSR_TME0 << i) | (0xFU << (8*i)));
            *mailbox = 1U << i; return HAL_OK;
        }
    }
    return HAL_ERROR;
}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*, uint32_t) { return reject_abort ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef* h) {
    ++reset_calls;
    if (reject_reset) return HAL_ERROR;
    if (!reset_keeps_pending_mailbox) h->Instance->TSR = kEmpty;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*, CAN_FilterTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*, uint32_t, CAN_RxHeaderTypeDef*, uint8_t*) { return HAL_ERROR; }

static void Reset() {
    can1Ctx.~CAN_context(); new (&can1Ctx) CAN_context;
    can2Ctx.~CAN_context(); new (&can2Ctx) CAN_context;
    can1Ctx.handle = &hcan1; can2Ctx.handle = &hcan2;
    test_can1.TSR = test_can2.TSR = kEmpty;
    semaphore1 = {}; semaphore2 = {};
    now_us = 100; interrupt_mask = task_nesting = 0;
    in_isr = reject_abort = reject_enqueue = reject_reset = false; reset_calls = 0;
    reset_keeps_pending_mailbox = false;
}
static CanTxStatus Send(uint32_t sequence = 42) {
    uint8_t data[8]{}; CAN_TxHeaderTypeDef header{};
    CanTxMetadata metadata{}; metadata.channel = CanTxChannel::Target;
    metadata.action_sequence = sequence; metadata.session_epoch = 1; metadata.fanout_generation = 1;
    return CanTrySendMessage(&can1Ctx, data, &header, nullptr, nullptr, &metadata);
}
static CanTxCompletion Completion() {
    CanTxCompletion c{}; assert(CanTakeTxCompletion(&can1Ctx, c));
    CanTxCompletion extra{}; assert(!CanTakeTxCompletion(&can1Ctx, extra)); return c;
}
static void Irq(void (*callback)(CAN_HandleTypeDef*)) {
    in_isr = true; interrupt_mask = 3; // Verify ISR restores the caller's mask.
    callback(&hcan1); assert(interrupt_mask == 3 && task_nesting == 0);
    in_isr = false; interrupt_mask = 0;
}
static void TestNormalAndDuplicateIrqs() {
    Reset(); assert(Send() == CanTxStatus::Queued);
    Irq(HAL_CAN_TxMailbox1CompleteCallback); // Wrong mailbox cannot settle frame.
    assert(semaphore1.tokens == 0 && can1Ctx.stale_tx_callback_count == 1);
    test_can1.TSR |= CAN_TSR_TME0 | CAN_TSR_TXOK0;
    Irq(HAL_CAN_TxMailbox0CompleteCallback);
    auto c = Completion(); assert(c.status == CanTxCompletionStatus::Complete && c.metadata.action_sequence == 42);
    Irq(HAL_CAN_TxMailbox0CompleteCallback);
    assert(semaphore1.release_calls == 1 && can1Ctx.stale_tx_callback_count == 2);
    assert(Send(43) == CanTxStatus::Queued);
    test_can1.TSR |= CAN_TSR_TME0;
    Irq(HAL_CAN_TxMailbox0AbortCallback);
    c = Completion(); assert(c.status == CanTxCompletionStatus::Aborted && c.metadata.action_sequence == 43);
}
static void TestAbortRejectedRetainsOwnership() {
    Reset(); assert(Send() == CanTxStatus::Queued); reject_abort = true;
    now_us = 5100; CanServiceTxDeadline(&can1Ctx, now_us, 5000);
    assert(can1Ctx.tx_state == CanTxLifecycleState::RecoveryRequired);
    assert(semaphore1.tokens == 0 && semaphore1.release_calls == 0);
    assert(Completion().status == CanTxCompletionStatus::Error);
    assert(Send(43) == CanTxStatus::Busy); // Other empty mailboxes must not be used.
    CanServiceTxRecovery(&can1Ctx, now_us, 4);
    assert(semaphore1.tokens == 0);
    test_can1.TSR |= CAN_TSR_TME0;
    Irq(HAL_CAN_TxMailbox0AbortCallback); // Converge without a second business terminal.
    CanTxCompletion c{}; assert(!CanTakeTxCompletion(&can1Ctx, c));
    assert(semaphore1.release_calls == 1);
    assert(Send(43) == CanTxStatus::Queued && can1Ctx.tx_recovery_attempts == 0);
}
static void TestLostCallbackRecoveryAndBlockedChannel() {
    Reset(); assert(Send() == CanTxStatus::Queued);
    now_us = 5100; CanServiceTxDeadline(&can1Ctx, now_us, 5000);
    now_us = 10100; CanServiceTxDeadline(&can1Ctx, now_us, 5000);
    assert(can1Ctx.tx_state == CanTxLifecycleState::RecoveryRequired && semaphore1.tokens == 0);
    assert(Completion().status == CanTxCompletionStatus::Error);
    reject_reset = true;
    for (unsigned i = 0; i < 4; ++i) CanServiceTxRecovery(&can1Ctx, now_us, 4);
    assert(can1Ctx.tx_state == CanTxLifecycleState::Blocked && reset_calls == 1);
    assert(Send() == CanTxStatus::Invalid && semaphore1.tokens == 0);
    // HAL_OK alone must not restore credit when its registers still show an
    // old request. Only a verified quiescent channel may be reused.
    reject_reset = false; reset_keeps_pending_mailbox = true;
    CanServiceTxRecovery(&can1Ctx, now_us, 4);
    assert(can1Ctx.tx_state == CanTxLifecycleState::Blocked && semaphore1.tokens == 0);
    reset_keeps_pending_mailbox = false;
    CanServiceTxRecovery(&can1Ctx, now_us, 4);
    assert(can1Ctx.tx_state == CanTxLifecycleState::Idle && semaphore1.release_calls == 1);
    Irq(HAL_CAN_TxMailbox0AbortCallback);
    CanTxCompletion c{}; assert(!CanTakeTxCompletion(&can1Ctx, c));
}
static void TestAdmissionErrorRestoresTokenWithoutCompletion() {
    Reset(); reject_enqueue = true; assert(Send() == CanTxStatus::Error);
    assert(semaphore1.tokens == 1 && semaphore1.release_calls == 1);
    CanTxCompletion c{}; assert(!CanTakeTxCompletion(&can1Ctx, c));
}
int main() {
    TestNormalAndDuplicateIrqs(); TestAbortRejectedRetainsOwnership();
    TestLostCallbackRecoveryAndBlockedChannel(); TestAdmissionErrorRestoresTokenWithoutCompletion();
    std::cout << "Actual CAN transport boundary tests passed\n";
}
