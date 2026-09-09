#pragma once
#include <cstdint>
using osSemaphoreId = void*;
using osThreadId_t = void*;
enum osStatus { osOK, osError };
using osPriority_t = int;
constexpr int osPriorityAboveNormal = 1;
struct osThreadAttr_t { const char* name; uint32_t stack_size; osPriority_t priority; };
inline osStatus osSemaphoreAcquire(osSemaphoreId, uint32_t) { return osOK; }
inline osStatus osSemaphoreRelease(osSemaphoreId) { return osOK; }
inline osThreadId_t osThreadNew(void (*)(void*), void*, const osThreadAttr_t*) { return nullptr; }
