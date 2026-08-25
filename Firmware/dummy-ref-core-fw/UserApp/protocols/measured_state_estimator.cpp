#include "measured_state_estimator.hpp"

#include <cstddef>
#include <cmath>

namespace dummy::protocol
{

VelocityEstimate MeasuredStateEstimator::Update(
    const std::array<float, 7>& position,
    const std::array<uint32_t, 7>& sample_time_us,
    uint32_t sweep_id, bool position_valid)
{
    VelocityEstimate output{};
    if (!position_valid)
    {
        Reset();
        return output;
    }
    for (float value : position)
    {
        if (!std::isfinite(value))
        {
            Reset();
            return output;
        }
    }
    if (initialized_ && sweep_id == previous_sweep_id_)
    {
        VelocityEstimate repeated = cached_;
        repeated.repeated = true;
        return repeated;
    }
    if (!initialized_ || sweep_id == 0U)
    {
        previous_position_ = position;
        previous_time_us_ = sample_time_us;
        previous_sweep_id_ = sweep_id;
        initialized_ = true;
        return output;
    }
    for (size_t index = 0; index < output.velocity.size(); ++index)
    {
        const uint32_t dt_us = sample_time_us[index] - previous_time_us_[index];
        if (dt_us < kMinIntervalUs || dt_us > max_interval_us_)
        {
            previous_position_ = position;
            previous_time_us_ = sample_time_us;
            previous_sweep_id_ = sweep_id;
            cached_ = {};
            return output;
        }
        const float inverse_dt = 1000000.0F / static_cast<float>(dt_us);
        output.velocity[index] = (position[index] - previous_position_[index]) * inverse_dt;
        if (!std::isfinite(output.velocity[index]))
        {
            Reset();
            return VelocityEstimate{};
        }
    }
    previous_position_ = position;
    previous_time_us_ = sample_time_us;
    previous_sweep_id_ = sweep_id;
    output.valid = true;
    cached_ = output;
    return output;
}

void MeasuredStateEstimator::Reset()
{
    previous_position_.fill(0.0F);
    previous_time_us_.fill(0U);
    cached_ = {};
    previous_sweep_id_ = 0U;
    initialized_ = false;
}

} // namespace dummy::protocol
