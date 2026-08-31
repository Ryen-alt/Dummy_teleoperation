#include "timing_profiler.h"

#include "stm32f1xx.h"
#include "../../../can_transport_contract.h"

#include <stddef.h>

namespace
{
constexpr uint32_t kControlFrequencyHz = 20000U;
constexpr uint32_t kHistogramBinCycles = 32U;
constexpr size_t kHistogramBinCount = 128U;
constexpr uint32_t kControlHistogramDecimation = 20U;
constexpr uint32_t kRefreshIntervalMs = 100U;
constexpr uint32_t kMinimumCanSamples = 100U;
constexpr uint32_t kMinimumControlSamples = 1000U;

struct Histogram
{
    volatile uint32_t bins[kHistogramBinCount]{};
    volatile uint32_t samples = 0U;
    volatile uint32_t maximum_cycles = 0U;
};

struct CachedProfile
{
    uint8_t flags = 0U;
    uint16_t can_p999_x10_us = 0U;
    uint16_t can_max_x10_us = 0U;
    uint16_t jitter_p999_x10_us = 0U;
    uint16_t jitter_max_x10_us = 0U;
    uint16_t control_p999_x10_us = 0U;
    uint16_t control_max_x10_us = 0U;
    uint16_t can_samples = 0U;
    uint16_t missed_ticks = 0U;
};

Histogram can_service_histogram;
Histogram control_jitter_histogram;
Histogram control_execution_histogram;
CachedProfile cached_profiles[2];
volatile uint8_t active_cached_profile = 0U;
volatile uint32_t missed_control_ticks = 0U;
volatile uint32_t active_window_token = 0U;
volatile uint32_t window_can_sample_baseline = 0U;
volatile uint32_t window_missed_tick_baseline = 0U;
uint32_t previous_control_start = 0U;
uint32_t control_tick_count = 0U;
uint32_t last_refresh_ms = 0U;
bool cycle_counter_enabled = false;

void SaturatingIncrement(volatile uint32_t& value)
{
    if (value != UINT32_MAX)
        ++value;
}

void RecordHistogram(Histogram& histogram, uint32_t cycles)
{
    size_t bin = cycles / kHistogramBinCycles;
    if (bin >= kHistogramBinCount)
        bin = kHistogramBinCount - 1U;
    SaturatingIncrement(histogram.bins[bin]);
    SaturatingIncrement(histogram.samples);
    if (cycles > histogram.maximum_cycles)
        histogram.maximum_cycles = cycles;
}

uint32_t PercentileCycles(const Histogram& histogram, uint32_t numerator,
                          uint32_t denominator)
{
    const uint32_t samples = histogram.samples;
    if (samples == 0U)
        return 0U;
    const uint32_t rank = static_cast<uint32_t>(
        (static_cast<uint64_t>(samples) * numerator + denominator - 1U) /
        denominator);
    uint32_t cumulative = 0U;
    for (size_t index = 0U; index < kHistogramBinCount; ++index)
    {
        const uint32_t count = histogram.bins[index];
        if (UINT32_MAX - cumulative < count)
            cumulative = UINT32_MAX;
        else
            cumulative += count;
        if (cumulative >= rank)
            return static_cast<uint32_t>(index + 1U) * kHistogramBinCycles;
    }
    return kHistogramBinCount * kHistogramBinCycles;
}

uint16_t CyclesToDeciMicroseconds(uint32_t cycles)
{
    if (SystemCoreClock == 0U)
        return UINT16_MAX;
    const uint64_t scaled = static_cast<uint64_t>(cycles) * 10000000ULL;
    const uint64_t rounded = (scaled + SystemCoreClock - 1U) / SystemCoreClock;
    return rounded > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(rounded);
}

uint16_t Saturate16(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(value);
}

void Store16(uint8_t* output, uint16_t value)
{
    output[0] = static_cast<uint8_t>(value & 0xFFU);
    output[1] = static_cast<uint8_t>(value >> 8U);
}
}

extern "C" void MotorTimingProfilerInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cycle_counter_enabled = (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}

extern "C" uint32_t MotorTimingProfilerControlBegin(void)
{
    if (!cycle_counter_enabled)
        return 0U;
    const uint32_t now = DWT->CYCCNT;
    if (previous_control_start != 0U)
    {
        const uint32_t elapsed = now - previous_control_start;
        const uint32_t expected = SystemCoreClock / kControlFrequencyHz;
        const uint32_t jitter = elapsed > expected ? elapsed - expected : expected - elapsed;
        if ((control_tick_count % kControlHistogramDecimation) == 0U)
            RecordHistogram(control_jitter_histogram, jitter);
        if (expected != 0U && elapsed > expected + expected / 2U)
        {
            uint32_t elapsed_ticks = (elapsed + expected / 2U) / expected;
            if (elapsed_ticks > 1U)
            {
                elapsed_ticks -= 1U;
                while (elapsed_ticks-- != 0U)
                    SaturatingIncrement(missed_control_ticks);
            }
        }
    }
    previous_control_start = now;
    ++control_tick_count;
    return now;
}

extern "C" void MotorTimingProfilerControlEnd(uint32_t start_cycles)
{
    if (!cycle_counter_enabled)
        return;
    if ((control_tick_count % kControlHistogramDecimation) == 0U)
        RecordHistogram(control_execution_histogram, DWT->CYCCNT - start_cycles);
}

extern "C" uint32_t MotorTimingProfilerCanBegin(void)
{
    return cycle_counter_enabled ? DWT->CYCCNT : 0U;
}

extern "C" void MotorTimingProfilerRecordCan05(uint32_t start_cycles)
{
    if (cycle_counter_enabled)
        RecordHistogram(can_service_histogram, DWT->CYCCNT - start_cycles);
}

extern "C" void MotorTimingProfilerStartWindow(uint32_t window_token)
{
    if (window_token == 0U || window_token == active_window_token)
        return;
    window_can_sample_baseline = can_service_histogram.samples;
    window_missed_tick_baseline = missed_control_ticks;
    __DMB();
    active_window_token = window_token;
}

extern "C" void MotorTimingProfilerRefresh(void)
{
    const uint32_t now_ms = HAL_GetTick();
    if (now_ms - last_refresh_ms < kRefreshIntervalMs)
        return;
    last_refresh_ms = now_ms;

    const uint8_t inactive = static_cast<uint8_t>(active_cached_profile ^ 1U);
    CachedProfile& profile = cached_profiles[inactive];
    profile.flags = cycle_counter_enabled ? DUMMY_MOTOR_TIMING_FLAG_DWT_ENABLED : 0U;
    if (can_service_histogram.samples >= kMinimumCanSamples)
        profile.flags |= DUMMY_MOTOR_TIMING_FLAG_CAN_VALID;
    if (control_jitter_histogram.samples >= kMinimumControlSamples &&
        control_execution_histogram.samples >= kMinimumControlSamples)
        profile.flags |= DUMMY_MOTOR_TIMING_FLAG_CONTROL_VALID;
    const uint32_t window_can_samples =
        can_service_histogram.samples - window_can_sample_baseline;
    const uint32_t window_missed_ticks =
        missed_control_ticks - window_missed_tick_baseline;
    if (window_missed_ticks == 0U)
        profile.flags |= DUMMY_MOTOR_TIMING_FLAG_NO_MISSED_TICKS;
    profile.can_p999_x10_us = CyclesToDeciMicroseconds(
        PercentileCycles(can_service_histogram, 999U, 1000U));
    profile.can_max_x10_us = CyclesToDeciMicroseconds(can_service_histogram.maximum_cycles);
    profile.jitter_p999_x10_us = CyclesToDeciMicroseconds(
        PercentileCycles(control_jitter_histogram, 999U, 1000U));
    profile.jitter_max_x10_us =
        CyclesToDeciMicroseconds(control_jitter_histogram.maximum_cycles);
    profile.control_p999_x10_us = CyclesToDeciMicroseconds(
        PercentileCycles(control_execution_histogram, 999U, 1000U));
    profile.control_max_x10_us =
        CyclesToDeciMicroseconds(control_execution_histogram.maximum_cycles);
    profile.can_samples = Saturate16(window_can_samples);
    profile.missed_ticks = Saturate16(window_missed_ticks);
    __DMB();
    active_cached_profile = inactive;
}

extern "C" bool MotorTimingProfilerEncodePage(uint8_t page, uint8_t output[8])
{
    if (output == nullptr || page >= DUMMY_MOTOR_TIMING_PAGE_COUNT)
        return false;
    const CachedProfile& profile = cached_profiles[active_cached_profile];
    output[0] = DUMMY_MOTOR_TIMING_FORMAT_V1;
    output[1] = page;
    output[2] = profile.flags;
    output[3] = 0U;
    switch (page)
    {
        case DUMMY_MOTOR_TIMING_PAGE_CAN_SERVICE:
            Store16(output + 4U, profile.can_p999_x10_us);
            Store16(output + 6U, profile.can_max_x10_us);
            break;
        case DUMMY_MOTOR_TIMING_PAGE_CONTROL_JITTER:
            Store16(output + 4U, profile.jitter_p999_x10_us);
            Store16(output + 6U, profile.jitter_max_x10_us);
            break;
        case DUMMY_MOTOR_TIMING_PAGE_CONTROL_EXECUTION:
            Store16(output + 4U, profile.control_p999_x10_us);
            Store16(output + 6U, profile.control_max_x10_us);
            break;
        case DUMMY_MOTOR_TIMING_PAGE_COUNTS:
            Store16(output + 4U, profile.can_samples);
            Store16(output + 6U, profile.missed_ticks);
            break;
        default:
            return false;
    }
    return true;
}
