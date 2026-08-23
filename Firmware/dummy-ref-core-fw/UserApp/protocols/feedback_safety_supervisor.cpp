#include "feedback_safety_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dummy::protocol
{

FeedbackSafetySupervisor::FeedbackSafetySupervisor(const FeedbackSafetyConfig& config)
    : config_(config)
{
}

uint32_t FeedbackSafetySupervisor::DurationMs(uint64_t now_us, uint64_t start_us)
{
    if (now_us <= start_us)
        return 0;
    return static_cast<uint32_t>(std::min<uint64_t>(
        (now_us - start_us) / 1000U, std::numeric_limits<uint32_t>::max()));
}

FeedbackSafetyOutput FeedbackSafetySupervisor::Update(const FeedbackSafetyInput& input)
{
    FeedbackSafetyOutput output{};
    output.telemetry_validity = kCanFeedbackTelemetryValid;
    if (input.following_active)
        output.telemetry_validity |= kFollowingErrorTelemetryValid;
    bool any_temperature_valid = false;
    bool arm_valid = true;
    for (size_t index = 0; index < kActuatorNodeCount; ++index)
    {
        const NodeFeedbackStatus& feedback = input.feedback[index];
        output.feedback_age_ms[index] = feedback.position_age_ms;
        output.feedback_loss_count[index] = feedback.total_position_losses;
        output.consecutive_feedback_loss[index] = feedback.consecutive_position_losses;

        const bool position_fresh = feedback.position_seen &&
            feedback.position_age_ms <= config_.feedback_hold_ms;
        if (position_fresh)
            output.node_validity[index] |= kNodePositionValid;
        if (index < 6)
            arm_valid = arm_valid && position_fresh;
        else
            output.gripper_position_valid = position_fresh;

        const bool temperature_fresh = feedback.temperature_seen &&
            feedback.temperature_age_ms <= config_.temperature_max_age_ms &&
            std::isfinite(feedback.temperature_c);
        if (temperature_fresh)
        {
            output.node_validity[index] |= kNodeTemperatureValid;
            any_temperature_valid = true;
        }

        // Measured over-temperature is a hardware condition and remains
        // supervised even when no host control lease is active.
        if (temperature_fresh && feedback.temperature_c >= config_.temperature_fault_c)
        {
            output.node_fault_bits[index] |= kNodeFaultOverTemperature;
            if (!overtemperature_active_[index])
            {
                overtemperature_active_[index] = true;
                overtemperature_start_us_[index] = input.now_us;
            }
            if (DurationMs(input.now_us, overtemperature_start_us_[index]) >=
                config_.temperature_fault_ms)
                output.fault_bits |= kFaultOverTemperature;
        }
        else
        {
            overtemperature_active_[index] = false;
        }

        if (!input.control_active)
        {
            missing_active_[index] = false;
            following_active_[index] = false;
            continue;
        }

        if (!position_fresh)
        {
            output.node_fault_bits[index] |= kNodeFaultFeedbackStale;
            if (!missing_active_[index])
            {
                missing_active_[index] = true;
                missing_start_us_[index] = input.now_us;
            }
            uint32_t missing_ms = DurationMs(input.now_us, missing_start_us_[index]);
            if (feedback.position_seen)
                missing_ms = std::max(missing_ms, feedback.position_age_ms);
            if (missing_ms >= config_.feedback_hold_ms)
                output.hold_reason_bits |= kHoldReasonFeedbackStale;
            if (missing_ms >= config_.feedback_fault_ms)
                output.fault_bits |= kFaultFeedbackLost;
        }
        else
        {
            missing_active_[index] = false;
        }

        const bool following_valid = input.following_active && position_fresh &&
            std::isfinite(input.commanded_position[index]) &&
            std::isfinite(input.measured_position[index]);
        if (following_valid)
        {
            const float error = input.commanded_position[index] - input.measured_position[index];
            output.following_error[index] = error;
            if (std::fabs(error) > config_.following_error_limit[index])
            {
                output.node_fault_bits[index] |= kNodeFaultFollowingError;
                if (!following_active_[index])
                {
                    following_active_[index] = true;
                    following_start_us_[index] = input.now_us;
                }
                output.following_error_duration_ms[index] = DurationMs(
                    input.now_us, following_start_us_[index]);
                if (output.following_error_duration_ms[index] >=
                    config_.following_error_hold_ms)
                    output.hold_reason_bits |= kHoldReasonFollowingError;
            }
            else
            {
                following_active_[index] = false;
            }
        }
        else
        {
            following_active_[index] = false;
        }

    }
    output.arm_position_valid = arm_valid;
    if (any_temperature_valid)
        output.telemetry_validity |= kTemperatureTelemetryValid;
    return output;
}

void FeedbackSafetySupervisor::Reset()
{
    following_start_us_.fill(0);
    missing_start_us_.fill(0);
    overtemperature_start_us_.fill(0);
    following_active_.fill(false);
    missing_active_.fill(false);
    overtemperature_active_.fill(false);
}

} // namespace dummy::protocol
