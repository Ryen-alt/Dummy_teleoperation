#pragma once
#include "interface_can.hpp"
using UBaseType_t = uint32_t;
extern CAN_HandleTypeDef hcan1, hcan2;
extern CAN_context can1Ctx, can2Ctx;
extern osSemaphoreId sem_can1_tx, sem_can2_tx;
extern uint64_t serialNumber;
uint32_t micros();
void NotifyCanDispatcherFromIsr();
void TestTaskEnterCritical();
void TestTaskExitCritical();
UBaseType_t TestIsrEnterCritical();
void TestIsrExitCritical(UBaseType_t);
#define taskENTER_CRITICAL() TestTaskEnterCritical()
#define taskEXIT_CRITICAL() TestTaskExitCritical()
#define taskENTER_CRITICAL_FROM_ISR() TestIsrEnterCritical()
#define taskEXIT_CRITICAL_FROM_ISR(saved) TestIsrExitCritical(saved)
