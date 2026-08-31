#ifndef DUMMY_MOTOR_TIMING_PROFILER_H
#define DUMMY_MOTOR_TIMING_PROFILER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void MotorTimingProfilerInit(void);
uint32_t MotorTimingProfilerControlBegin(void);
void MotorTimingProfilerControlEnd(uint32_t start_cycles);
uint32_t MotorTimingProfilerCanBegin(void);
void MotorTimingProfilerRecordCan05(uint32_t start_cycles);
void MotorTimingProfilerRefresh(void);
bool MotorTimingProfilerEncodePage(uint8_t page, uint8_t output[8]);

#ifdef __cplusplus
}
#endif

#endif
