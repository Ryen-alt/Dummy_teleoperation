#include "can_timing_profiler.hpp"

#include "../../../can_transport_contract.h"

#include <algorithm>
#include <limits>

namespace dummy::protocol
{
namespace
{
uint32_t SaturatingIncrement(uint32_t value)
{
    return value == std::numeric_limits<uint32_t>::max() ? value : value + 1U;
}

template<typename T, size_t N>
void CopyArray(T (&destination)[N], const std::array<T, N>& source)
{
    std::copy(source.begin(), source.end(), destination);
}
}

void TimingHistogram::Record(uint32_t value_us)
{
    size_t bin = value_us / kBinWidthUs;
    if (bin >= bins_.size())
        bin = bins_.size() - 1U;
    bins_[bin] = SaturatingIncrement(bins_[bin]);
    samples_ = SaturatingIncrement(samples_);
    maximum_us_ = std::max(maximum_us_, value_us);
}

void TimingHistogram::Reset()
{
    bins_.fill(0U);
    samples_ = 0U;
    maximum_us_ = 0U;
}

uint32_t TimingHistogram::Percentile(uint32_t numerator,
                                     uint32_t denominator) const
{
    if (samples_ == 0U || denominator == 0U || numerator > denominator)
        return 0U;
    const uint32_t rank = static_cast<uint32_t>(
        (static_cast<uint64_t>(samples_) * numerator + denominator - 1U) /
        denominator);
    uint32_t cumulative = 0U;
    for (size_t index = 0U; index < bins_.size(); ++index)
    {
        const uint32_t remaining = std::numeric_limits<uint32_t>::max() -
            cumulative;
        cumulative += bins_[index] > remaining ? remaining : bins_[index];
        if (cumulative >= rank)
            return static_cast<uint32_t>(index + 1U) * kBinWidthUs;
    }
    return static_cast<uint32_t>(bins_.size()) * kBinWidthUs;
}

size_t CanTimingProfiler::NodeIndex(uint8_t node_id)
{
    return node_id >= 1U && node_id <= kActuatorNodeCount
        ? static_cast<size_t>(node_id - 1U) : kActuatorNodeCount;
}

uint16_t CanTimingProfiler::Load16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
        static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

void CanTimingProfiler::Reset(uint32_t session_epoch, uint64_t start_us)
{
    if (reset_count_ != std::numeric_limits<uint32_t>::max())
        ++reset_count_;
    session_epoch_ = session_epoch;
    start_us_ = start_us;
    active_ = true;
    epoch_stable_ = true;
    position_requests_ = {};
    temperature_requests_ = {};
    for (auto& histogram : position_histograms_)
        histogram.Reset();
    for (auto& histogram : temperature_histograms_)
        histogram.Reset();
    motor_profiles_ = {};
    motor_page_valid_mask_.fill(0U);
}

void CanTimingProfiler::RecordPositionRequest(uint8_t node_id,
                                              uint32_t now_us)
{
    if (!active_)
        return;
    const size_t index = NodeIndex(node_id);
    if (index < position_requests_.size())
        position_requests_[index] = RequestState{now_us, true};
}

void CanTimingProfiler::RecordResponse(
    uint8_t node_id, uint32_t received_us,
    std::array<RequestState, kActuatorNodeCount>& requests,
    std::array<TimingHistogram, kActuatorNodeCount>& histograms)
{
    if (!active_)
        return;
    const size_t index = NodeIndex(node_id);
    if (index >= requests.size() || !requests[index].pending)
        return;
    histograms[index].Record(received_us - requests[index].started_us);
    requests[index].pending = false;
}

void CanTimingProfiler::RecordPositionResponse(uint8_t node_id,
                                               uint32_t received_us)
{
    RecordResponse(node_id, received_us, position_requests_,
                   position_histograms_);
}

void CanTimingProfiler::RecordPositionTimeout(uint8_t node_id)
{
    if (!active_)
        return;
    const size_t index = NodeIndex(node_id);
    if (index < position_requests_.size())
        position_requests_[index].pending = false;
}

void CanTimingProfiler::RecordTemperatureRequest(uint8_t node_id,
                                                 uint32_t now_us)
{
    if (!active_)
        return;
    const size_t index = NodeIndex(node_id);
    if (index < temperature_requests_.size())
        temperature_requests_[index] = RequestState{now_us, true};
}

void CanTimingProfiler::RecordTemperatureResponse(uint8_t node_id,
                                                  uint32_t received_us)
{
    RecordResponse(node_id, received_us, temperature_requests_,
                   temperature_histograms_);
}

void CanTimingProfiler::RecordTemperatureTimeout(uint8_t node_id)
{
    if (!active_)
        return;
    const size_t index = NodeIndex(node_id);
    if (index < temperature_requests_.size())
        temperature_requests_[index].pending = false;
}

bool CanTimingProfiler::RecordMotorPage(uint8_t node_id, const uint8_t* data,
                                        uint32_t length)
{
    if (!active_)
        return false;
    const size_t index = NodeIndex(node_id);
    if (index >= motor_profiles_.size() || data == nullptr || length < 8U ||
        data[0] != DUMMY_MOTOR_TIMING_FORMAT_V1 ||
        data[1] >= DUMMY_MOTOR_TIMING_PAGE_COUNT)
        return false;
    const uint8_t page = data[1];
    MotorProfile& profile = motor_profiles_[index];
    profile.flags = data[2];
    const uint16_t first = Load16(data + 4U);
    const uint16_t second = Load16(data + 6U);
    if (page == DUMMY_MOTOR_TIMING_PAGE_CAN_SERVICE)
    {
        profile.can_p999_x10_us = first;
        profile.can_max_x10_us = second;
    }
    else if (page == DUMMY_MOTOR_TIMING_PAGE_CONTROL_JITTER)
    {
        profile.jitter_p999_x10_us = first;
        profile.jitter_max_x10_us = second;
    }
    else if (page == DUMMY_MOTOR_TIMING_PAGE_CONTROL_EXECUTION)
    {
        profile.control_p999_x10_us = first;
        profile.control_max_x10_us = second;
    }
    else
    {
        profile.can_samples = first;
        profile.missed_ticks = second;
    }
    motor_page_valid_mask_[page] = static_cast<uint8_t>(
        motor_page_valid_mask_[page] | (1U << index));
    return true;
}

bool CanTimingProfiler::MotorPagesComplete() const
{
    constexpr uint8_t kAllNodes = static_cast<uint8_t>(
        (1U << kActuatorNodeCount) - 1U);
    return std::all_of(
        motor_page_valid_mask_.begin(), motor_page_valid_mask_.end(),
        [](uint8_t mask) { return mask == kAllNodes; });
}

bool CanTimingProfiler::LatencySamplesValid() const
{
    constexpr uint32_t kMinimumPositionSamples = 1000U;
    constexpr uint32_t kMinimumTemperatureSamples = 100U;
    for (size_t index = 0U; index < kActuatorNodeCount; ++index)
    {
        if (position_histograms_[index].samples() < kMinimumPositionSamples ||
            temperature_histograms_[index].samples() <
                kMinimumTemperatureSamples)
            return false;
    }
    return true;
}

CanTimingProfilePayload CanTimingProfiler::MakePayload(
    uint64_t now_us, const CanDispatchDiagnostics& scheduler) const
{
    CanTimingProfilePayload output{};
    output.format_version = kCanTimingProfileFormatVersion;
    output.payload_size = kCanTimingProfilePayloadSize;
    output.session_epoch = session_epoch_;
    output.window_reset_count = reset_count_;
    output.window_start_us = start_us_;
    output.window_duration_us = start_us_ == 0U || now_us < start_us_
        ? 0U : now_us - start_us_;
    std::copy(motor_page_valid_mask_.begin(), motor_page_valid_mask_.end(),
              output.motor_page_valid_mask);
    if (active_)
        output.window_flags |= kCanTimingProfileWindowActive;
    if (epoch_stable_)
        output.window_flags |= kCanTimingProfileEpochStable;
    if (MotorPagesComplete())
        output.window_flags |= kCanTimingProfileMotorPagesComplete;
    if (LatencySamplesValid())
        output.window_flags |= kCanTimingProfileLatencySamplesValid;

    std::array<uint32_t, kActuatorNodeCount> position_samples{};
    std::array<uint32_t, kActuatorNodeCount> position_p50{};
    std::array<uint32_t, kActuatorNodeCount> position_p99{};
    std::array<uint32_t, kActuatorNodeCount> position_p999{};
    std::array<uint32_t, kActuatorNodeCount> position_max{};
    std::array<uint32_t, kActuatorNodeCount> temperature_samples{};
    std::array<uint32_t, kActuatorNodeCount> temperature_p50{};
    std::array<uint32_t, kActuatorNodeCount> temperature_p99{};
    std::array<uint32_t, kActuatorNodeCount> temperature_p999{};
    std::array<uint32_t, kActuatorNodeCount> temperature_max{};
    for (size_t index = 0U; index < kActuatorNodeCount; ++index)
    {
        const TimingHistogram& position = position_histograms_[index];
        const TimingHistogram& temperature = temperature_histograms_[index];
        position_samples[index] = position.samples();
        position_p50[index] = position.Percentile(50U, 100U);
        position_p99[index] = position.Percentile(99U, 100U);
        position_p999[index] = position.Percentile(999U, 1000U);
        position_max[index] = position.maximum_us();
        temperature_samples[index] = temperature.samples();
        temperature_p50[index] = temperature.Percentile(50U, 100U);
        temperature_p99[index] = temperature.Percentile(99U, 100U);
        temperature_p999[index] = temperature.Percentile(999U, 1000U);
        temperature_max[index] = temperature.maximum_us();

        const MotorProfile& motor = motor_profiles_[index];
        output.motor_flags[index] = motor.flags;
        output.motor_can_samples[index] = motor.can_samples;
        output.motor_can_p999_x10_us[index] = motor.can_p999_x10_us;
        output.motor_can_max_x10_us[index] = motor.can_max_x10_us;
        output.motor_jitter_p999_x10_us[index] =
            motor.jitter_p999_x10_us;
        output.motor_jitter_max_x10_us[index] = motor.jitter_max_x10_us;
        output.motor_control_p999_x10_us[index] =
            motor.control_p999_x10_us;
        output.motor_control_max_x10_us[index] = motor.control_max_x10_us;
        output.motor_missed_ticks[index] = motor.missed_ticks;
    }
    CopyArray(output.position_samples, position_samples);
    CopyArray(output.position_p50_us, position_p50);
    CopyArray(output.position_p99_us, position_p99);
    CopyArray(output.position_p999_us, position_p999);
    CopyArray(output.position_max_us, position_max);
    CopyArray(output.temperature_samples, temperature_samples);
    CopyArray(output.temperature_p50_us, temperature_p50);
    CopyArray(output.temperature_p99_us, temperature_p99);
    CopyArray(output.temperature_p999_us, temperature_p999);
    CopyArray(output.temperature_max_us, temperature_max);
    CopyArray(output.timing_request, scheduler.timing_profile_requested);
    CopyArray(output.timing_response, scheduler.timing_profile_responded);
    CopyArray(output.timing_timeout, scheduler.timing_profile_timed_out);
    return output;
}

} // namespace dummy::protocol
