#ifndef __INTERFACE_CAN_HPP
#define __INTERFACE_CAN_HPP

#include "fibre/protocol.hpp"
#include <stm32f4xx_hal.h>
#include <cmsis_os.h>

struct CAN_context
{
    CAN_HandleTypeDef* handle = nullptr;
    uint8_t node_id = 0;
    uint64_t serial_number = 0;

    uint32_t node_ids_in_use_0[4]; // 128 bits (indicate if a node ID was in use up to 1 second ago)
    uint32_t node_ids_in_use_1[4]; // 128 bits (indicats if a node ID was in use 1-2 seconds ago)

    uint32_t last_heartbeat_mailbox = 0;
    uint32_t tx_msg_cnt = 0;
    uint32_t tx_attempt_count = 0;
    uint32_t tx_queued_count = 0;
    uint32_t tx_busy_count = 0;
    uint32_t tx_recovery_count = 0;
    uint32_t tx_enqueue_error_count = 0;
    volatile uint32_t tx_started_ms = 0;
    volatile bool tx_in_flight = false;

    uint8_t node_id_rng_state = 0;

    osSemaphoreId_t sem_send_heartbeat;

    // count occurrence various callbacks
    uint32_t TxMailboxCompleteCallbackCnt = 0;
    uint32_t TxMailboxAbortCallbackCnt = 0;
    int RxFifo0MsgPendingCallbackCnt = 0;
    int RxFifo0FullCallbackCnt = 0;
    int RxFifo1MsgPendingCallbackCnt = 0;
    int RxFifo1FullCallbackCnt = 0;
    int SleepCallbackCnt = 0;
    int WakeUpFromRxMsgCallbackCnt = 0;
    int ErrorCallbackCnt = 0;

    uint32_t received_msg_cnt = 0;
    uint32_t received_ack = 0;
    uint32_t unexpected_errors = 0;
    uint32_t unhandled_messages = 0;
};

struct CAN_context* get_can_ctx(CAN_HandleTypeDef* hcan);
bool StartCanServer(CAN_TypeDef* hcan);
using CanTxQueuedCallback = void (*)(void* context);
enum class CanTxStatus : uint8_t
{
    Queued,
    Busy,
    Error,
    Invalid,
};

// Realtime control must never wait for a CAN mailbox. This function makes one
// admission attempt and returns immediately. A stalled single-flight token is
// recovered asynchronously after the reviewed timeout.
CanTxStatus CanTrySendMessage(CAN_context* canCtx, uint8_t* txData,
                              CAN_TxHeaderTypeDef* txHeader,
                              CanTxQueuedCallback on_queued = nullptr,
                              void* callback_context = nullptr);

// Returns true only when the frame was accepted by a hardware TX mailbox. The
// optional callback runs in the same critical section as the mailbox enqueue.
// This compatibility API may wait and is restricted to startup/maintenance;
// realtime dispatch uses CanTrySendMessage().
bool CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader,
                    CanTxQueuedCallback on_queued = nullptr,
                    void* callback_context = nullptr);
void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data);

#endif // __INTERFACE_CAN_HPP
