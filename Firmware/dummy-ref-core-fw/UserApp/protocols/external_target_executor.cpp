#include "external_target_executor.hpp"

#include <algorithm>
#include <cmath>

namespace dummy::protocol
{

ExternalTargetExecutor::ExternalTargetExecutor(const ExecutorConfig& config)
    : config_(config)
{
    if (config_.loop_rate_hz == 0)
        config_.loop_rate_hz = 200;
}

ExecutorStep ExternalTargetExecutor::Step(const ExecutorTarget& target, bool motion_allowed,
                                          const std::array<float, 7>& measured_position)
{
    ExecutorStep output{};
    if (!motion_allowed || !target.valid)
    {
        output.entered_hold = active_;
        Reset();
        return output;
    }

    if (!initialized_)
    {
        commanded_position_ = measured_position;
        commanded_velocity_.fill(0.0F);
        initialized_ = true;
    }
    active_ = true;

    const float dt = 1.0F / static_cast<float>(config_.loop_rate_hz);
    for (size_t index = 0; index < 7; ++index)
    {
        const float error = target.position[index] - commanded_position_[index];
        const float acceleration = index < 6
            ? config_.max_acceleration_rad_s2[index]
            : config_.gripper_max_acceleration_per_s2;
        const float velocity_limit = index < 6
            ? target.max_velocity_rad_s[index]
            : config_.gripper_max_velocity_per_s;
        if (!std::isfinite(error) || !std::isfinite(acceleration) ||
            !std::isfinite(velocity_limit) || acceleration <= 0.0F || velocity_limit <= 0.0F)
        {
            output.entered_hold = true;
            Reset();
            return output;
        }

        const float stopping_speed = std::sqrt(2.0F * acceleration * std::fabs(error));
        const float desired_speed = std::min(velocity_limit, stopping_speed);
        const float desired_velocity = std::copysign(desired_speed, error);
        const float max_delta_velocity = acceleration * dt;
        const float next_velocity = std::clamp(
            desired_velocity,
            commanded_velocity_[index] - max_delta_velocity,
            commanded_velocity_[index] + max_delta_velocity);
        const float proposed_step = next_velocity * dt;

        if (std::fabs(proposed_step) >= std::fabs(error))
        {
            commanded_position_[index] = target.position[index];
            commanded_velocity_[index] = 0.0F;
        }
        else
        {
            commanded_position_[index] += proposed_step;
            commanded_velocity_[index] = next_velocity;
        }
    }

    output.position = commanded_position_;
    output.sequence = target.sequence;
    output.command_valid = true;
    return output;
}

void ExternalTargetExecutor::Reset()
{
    commanded_velocity_.fill(0.0F);
    initialized_ = false;
    active_ = false;
}

} // namespace dummy::protocol
