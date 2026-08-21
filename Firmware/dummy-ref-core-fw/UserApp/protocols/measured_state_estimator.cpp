#include "measured_state_estimator.hpp"

#include <cstddef>
#include <cmath>

namespace dummy::protocol
{

VelocityEstimate MeasuredStateEstimator::Update(
    const std::array<float, 7>& position, uint64_t now_us, bool position_valid)
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
    if (!initialized_)
    {
        previous_position_ = position;
        previous_time_us_ = now_us;
        initialized_ = true;
        return output;
    }
    if (now_us <= previous_time_us_ || now_us - previous_time_us_ < kMinIntervalUs ||
        now_us - previous_time_us_ > max_interval_us_)
    {
        previous_position_ = position;
        previous_time_us_ = now_us;
        return output;
    }
    const float inverse_dt = 1000000.0F / static_cast<float>(now_us - previous_time_us_);
    for (size_t index = 0; index < output.velocity.size(); ++index)
    {
        output.velocity[index] = (position[index] - previous_position_[index]) * inverse_dt;
        if (!std::isfinite(output.velocity[index]))
        {
            Reset();
            return VelocityEstimate{};
        }
    }
    previous_position_ = position;
    previous_time_us_ = now_us;
    output.valid = true;
    return output;
}

void MeasuredStateEstimator::Reset()
{
    previous_position_.fill(0.0F);
    previous_time_us_ = 0;
    initialized_ = false;
}

} // namespace dummy::protocol
