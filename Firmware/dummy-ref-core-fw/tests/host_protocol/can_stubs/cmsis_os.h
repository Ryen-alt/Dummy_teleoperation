#pragma once
#include <cstdint>
struct TestSemaphore { unsigned tokens = 1, release_calls = 0; };
using osSemaphoreId = TestSemaphore*;
using osSemaphoreId_t = TestSemaphore*;
enum osStatus_t { osOK, osError };
#define osSemaphoreDef(name) ((void)0)
#define osSemaphore(name) nullptr
osStatus_t osSemaphoreAcquire(osSemaphoreId, uint32_t);
osStatus_t osSemaphoreRelease(osSemaphoreId);
osSemaphoreId osSemaphoreNew(uint32_t, uint32_t, const void*);
void osDelay(uint32_t);
