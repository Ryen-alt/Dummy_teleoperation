#ifndef DUMMY_FEEDBACK_SAFETY_SUPERVISOR_HPP
#define DUMMY_FEEDBACK_SAFETY_SUPERVISOR_HPP

#include "can_feedback_monitor.hpp"

#include <array>
#include <cstdint>

namespace dummy::protocol
{

enum HoldReasonBits : uint16_t
{
    kHoldReasonTargetTimeout = 1U << 0U,
    kHoldReasonLeaseTimeout = 1U << 1U,
    kHoldReasonFollowingError = 1U << 2U,
    kHoldReasonFeedbackStale = 1U << 3U,
    kHoldReasonOperator = 1U << 4U,
    kHoldReasonRuntimeLimit = 1U << 5U,
};

enum FaultBits : uint16_t
{
    kFaultEmergencyStop = 1U << 0U,
    kFaultFeedbackLost = 1U << 1U,
    kFaultOverTemperature = 1U << 2U,
    // Reserved until a motor-board CAN response actually exposes these
    // sources. They must never be inferred from position error alone.
    kFaultEncoder = 1U << 3U,
    kFaultStall = 1U << 4U,
    kFaultOverCurrent = 1U << 5U,
};

enum NodeFaultBits : uint16_t
{
    kNodeFaultFeedbackStale = 1U << 0U,
    kNodeFaultFollowingError = 1U << 1U,
    kNodeFaultOverTemperature = 1U << 2U,
    kNodeFaultEncoder = 1U << 3U,
    kNodeFaultStall = 1U << 4U,
    kNodeFaultOverCurrent = 1U << 5U,
};

enum NodeValidityBits : uint8_t
{
    kNodePositionValid = 1U << 0U,
    kNodeTemperatureValid = 1U << 1U,
    kNodeEncoderFaultSourceValid = 1U << 2U,
    kNodeStallSourceValid = 1U << 3U,
    kNodeCurrentSourceValid = 1U << 4U,
};

enum TelemetryValidityBits : uint16_t
{
    kFollowingErrorTelemetryValid = 1U << 0U,
    kCanFeedbackTelemetryValid = 1U << 1U,
    kTemperatureTelemetryValid = 1U << 2U,
};

struct FeedbackSafetyConfig
{
    std::array<float, kActuatorNodeCount> following_error_limit{};
    uint32_t following_error_hold_ms = 250;
    uint32_t feedback_hold_ms = 100;
    uint32_t feedback_fault_ms = 500;
    uint32_t temperature_max_age_ms = 2500;
    float temperature_fault_c = 75.0F;
    uint32_t temperature_fault_ms = 1000;
};

struct FeedbackSafetyInput
{
    uint64_t now_us = 0;
    bool control_active = false;
    bool following_active = false;
    std::array<float, kActuatorNodeCount> commanded_position{};
    std::array<float, kActuatorNodeCount> measured_position{};
    std::array<NodeFeedbackStatus, kActuatorNodeCount> feedback{};
};

struct FeedbackSafetyOutput
{
    std::array<float, kActuatorNodeCount> following_error{};
    std::array<uint32_t, kActuatorNodeCount> following_error_duration_ms{};
    std::array<uint32_t, kActuatorNodeCount> feedback_age_ms{};
    std::array<uint32_t, kActuatorNodeCount> feedback_loss_count{};
    std::array<uint16_t, kActuatorNodeCount> consecutive_feedback_loss{};
    std::array<uint16_t, kActuatorNodeCount> node_fault_bits{};
    std::array<uint8_t, kActuatorNodeCount> node_validity{};
    uint16_t hold_reason_bits = 0;
    uint16_t fault_bits = 0;
    uint16_t telemetry_validity = 0;
    bool arm_position_valid = false;
    bool gripper_position_valid = false;
};

// Stateful persistence checker. General following/short feedback anomalies
// request HOLD. Feedback loss that persists and measured over-temperature are
// severe and request a latched FAULT from ControlSession.
class FeedbackSafetySupervisor
{
public:
    explicit FeedbackSafetySupervisor(const FeedbackSafetyConfig& config);

    FeedbackSafetyOutput Update(const FeedbackSafetyInput& input);
    void Reset();

private:
    static uint32_t DurationMs(uint64_t now_us, uint64_t start_us);

    FeedbackSafetyConfig config_{};
    std::array<uint64_t, kActuatorNodeCount> following_start_us_{};
    std::array<uint64_t, kActuatorNodeCount> missing_start_us_{};
    std::array<uint64_t, kActuatorNodeCount> overtemperature_start_us_{};
    std::array<bool, kActuatorNodeCount> following_active_{};
    std::array<bool, kActuatorNodeCount> missing_active_{};
    std::array<bool, kActuatorNodeCount> overtemperature_active_{};
};

} // namespace dummy::protocol

#endif // DUMMY_FEEDBACK_SAFETY_SUPERVISOR_HPP
