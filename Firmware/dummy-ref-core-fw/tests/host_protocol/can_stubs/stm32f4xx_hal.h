#pragma once
#include <cstdint>
struct CAN_TypeDef { uint32_t TSR = 0; };
struct CAN_HandleTypeDef { CAN_TypeDef* Instance; uint32_t ErrorCode = 0; };
struct CAN_TxHeaderTypeDef { uint32_t StdId = 0, DLC = 0; };
using CAN_RxHeaderTypeDef = CAN_TxHeaderTypeDef;
struct CAN_FilterTypeDef {
    uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
    uint32_t FilterFIFOAssignment, FilterBank, FilterMode, FilterScale;
    uint32_t FilterActivation, SlaveStartFilterBank;
};
extern CAN_TypeDef test_can1, test_can2;
#define CAN1 (&test_can1)
#define CAN2 (&test_can2)
enum IRQn_Type { CAN1_TX_IRQn, CAN2_TX_IRQn };
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
constexpr uint32_t CAN_TSR_TME0 = 1U << 26, CAN_TSR_TME1 = 1U << 27, CAN_TSR_TME2 = 1U << 28;
constexpr uint32_t CAN_TSR_TXOK0 = 2U;
constexpr uint32_t CAN_FLAG_RQCP0 = 0U, CAN_FLAG_RQCP1 = 8U, CAN_FLAG_RQCP2 = 16U;
#define __HAL_CAN_CLEAR_FLAG(hcan, flag) ((hcan)->Instance->TSR &= ~(0xFU << (flag)))
constexpr uint32_t CAN_TX_MAILBOX0 = 1U, CAN_TX_MAILBOX1 = 2U, CAN_TX_MAILBOX2 = 4U;
constexpr uint32_t CAN_RX_FIFO0 = 0, CAN_RX_FIFO1 = 1;
constexpr uint32_t CAN_FILTERMODE_IDMASK = 0, CAN_FILTERSCALE_16BIT = 0, ENABLE = 1;
constexpr uint32_t CAN_IT_TX_MAILBOX_EMPTY = 1U << 0;
constexpr uint32_t CAN_IT_RX_FIFO0_MSG_PENDING = 1U << 1, CAN_IT_RX_FIFO1_MSG_PENDING = 1U << 2;
constexpr uint32_t CAN_IT_RX_FIFO0_FULL = 1U << 3, CAN_IT_RX_FIFO1_FULL = 1U << 4;
constexpr uint32_t CAN_IT_RX_FIFO0_OVERRUN = 1U << 5, CAN_IT_RX_FIFO1_OVERRUN = 1U << 6;
constexpr uint32_t CAN_IT_WAKEUP = 1U << 7, CAN_IT_SLEEP_ACK = 1U << 8;
constexpr uint32_t CAN_IT_ERROR_WARNING = 1U << 9, CAN_IT_ERROR_PASSIVE = 1U << 10;
constexpr uint32_t CAN_IT_BUSOFF = 1U << 11, CAN_IT_LAST_ERROR_CODE = 1U << 12, CAN_IT_ERROR = 1U << 13;
constexpr uint32_t HAL_CAN_ERROR_NONE = 0, HAL_CAN_ERROR_BOF = 1U << 0;
constexpr uint32_t HAL_CAN_ERROR_TX_ALST0 = 1U << 1, HAL_CAN_ERROR_TX_TERR0 = 1U << 2;
constexpr uint32_t HAL_CAN_ERROR_TX_ALST1 = 1U << 3, HAL_CAN_ERROR_TX_TERR1 = 1U << 4;
constexpr uint32_t HAL_CAN_ERROR_TX_ALST2 = 1U << 5, HAL_CAN_ERROR_TX_TERR2 = 1U << 6;
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef*, CAN_TxHeaderTypeDef*, uint8_t*, uint32_t*);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*, CAN_FilterTypeDef*);
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*, uint32_t, CAN_RxHeaderTypeDef*, uint8_t*);
uint32_t HAL_GetTick();
void NVIC_ClearPendingIRQ(IRQn_Type);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef*);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef*);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef*);
