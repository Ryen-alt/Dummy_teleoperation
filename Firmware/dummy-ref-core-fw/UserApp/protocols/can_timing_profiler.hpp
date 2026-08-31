#ifndef DUMMY_CAN_TIMING_PROFILER_HPP
#define DUMMY_CAN_TIMING_PROFILER_HPP

#include "binary_protocol.hpp"
#include "can_feedback_monitor.hpp"
#include "feedback_poll_scheduler.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dummy::protocol
{

class TimingHistogram
{
public:
    void Record(uint32_t value_us);
    void Reset();
    uint32_t samples() const { return samples_; }
    uint32_t maximum_us() const { return maximum_us_; }
    uint32_t Percentile(uint32_t numerator, uint32_t denominator) const;

private:
    static constexpr uint32_t kBinWidthUs = 8U;
    static constexpr size_t kBinCount = 128U;
    std::array<uint32_t, kBinCount> bins_{};
    uint32_t samples_ = 0U;
    uint32_t maximum_us_ = 0U;
};

class CanTimingProfiler
{
public:
    void Reset(uint32_t session_epoch, uint64_t start_us);
    void SetActive(bool active) { active_ = active; }
    void SetEpochStable(bool stable) { epoch_stable_ = stable; }
    void RecordPositionRequest(uint8_t node_id, uint32_t now_us);
    void RecordPositionResponse(uint8_t node_id, uint32_t received_us);
    void RecordPositionTimeout(uint8_t node_id);
    void RecordTemperatureRequest(uint8_t node_id, uint32_t now_us);
    void RecordTemperatureResponse(uint8_t node_id, uint32_t received_us);
    void RecordTemperatureTimeout(uint8_t node_id);
    bool RecordMotorPage(uint8_t node_id, const uint8_t* data,
                         uint32_t length);
    CanTimingProfilePayload MakePayload(
        uint64_t now_us, const CanDispatchDiagnostics& scheduler) const;

private:
    struct RequestState
    {
        uint32_t started_us = 0U;
        bool pending = false;
    };

    struct MotorProfile
    {
        uint8_t flags = 0U;
        uint16_t can_samples = 0U;
        uint16_t can_p999_x10_us = 0U;
        uint16_t can_max_x10_us = 0U;
        uint16_t jitter_p999_x10_us = 0U;
        uint16_t jitter_max_x10_us = 0U;
        uint16_t control_p999_x10_us = 0U;
        uint16_t control_max_x10_us = 0U;
        uint16_t missed_ticks = 0U;
    };

    static size_t NodeIndex(uint8_t node_id);
    static uint16_t Load16(const uint8_t* data);
    void RecordResponse(uint8_t node_id, uint32_t received_us,
                        std::array<RequestState, kActuatorNodeCount>& requests,
                        std::array<TimingHistogram, kActuatorNodeCount>& histograms);
    bool MotorPagesComplete() const;
    bool LatencySamplesValid() const;

    std::array<RequestState, kActuatorNodeCount> position_requests_{};
    std::array<RequestState, kActuatorNodeCount> temperature_requests_{};
    std::array<TimingHistogram, kActuatorNodeCount> position_histograms_{};
    std::array<TimingHistogram, kActuatorNodeCount> temperature_histograms_{};
    std::array<MotorProfile, kActuatorNodeCount> motor_profiles_{};
    std::array<uint8_t, 4U> motor_page_valid_mask_{};
    uint32_t session_epoch_ = 0U;
    uint32_t reset_count_ = 0U;
    uint64_t start_us_ = 0U;
    bool active_ = false;
    bool epoch_stable_ = false;
};

} // namespace dummy::protocol

#endif
