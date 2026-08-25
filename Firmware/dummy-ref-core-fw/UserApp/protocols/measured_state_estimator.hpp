#ifndef DUMMY_MEASURED_STATE_ESTIMATOR_HPP
#define DUMMY_MEASURED_STATE_ESTIMATOR_HPP

#include <array>
#include <cstdint>

namespace dummy::protocol
{

struct VelocityEstimate
{
    std::array<float, 7> velocity{};
    bool valid = false;
    bool repeated = false;
};

// Monotonic finite-difference estimator for host telemetry. It never marks a
// first sample, a repeated timestamp, or a long feedback gap as valid.
class MeasuredStateEstimator
{
public:
    explicit MeasuredStateEstimator(uint64_t max_interval_us = 250000U)
        : max_interval_us_(max_interval_us)
    {}

    VelocityEstimate Update(
        const std::array<float, 7>& position,
        const std::array<uint32_t, 7>& sample_time_us,
        uint32_t sweep_id, bool position_valid);
    void Reset();

private:
    static constexpr uint64_t kMinIntervalUs = 1000U;
    std::array<float, 7> previous_position_{};
    std::array<uint32_t, 7> previous_time_us_{};
    VelocityEstimate cached_{};
    uint32_t previous_sweep_id_ = 0;
    uint64_t max_interval_us_ = 250000U;
    bool initialized_ = false;
};

} // namespace dummy::protocol

#endif // DUMMY_MEASURED_STATE_ESTIMATOR_HPP
