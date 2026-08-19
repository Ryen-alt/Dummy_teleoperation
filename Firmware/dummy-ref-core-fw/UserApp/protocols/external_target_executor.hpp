#ifndef DUMMY_EXTERNAL_TARGET_EXECUTOR_HPP
#define DUMMY_EXTERNAL_TARGET_EXECUTOR_HPP

#include <array>
#include <cstdint>

namespace dummy::protocol
{

struct ExecutorConfig
{
    std::array<float, 6> max_acceleration_rad_s2{};
    uint32_t loop_rate_hz = 200;
};

struct ExecutorTarget
{
    std::array<float, 7> position{};
    std::array<float, 6> max_velocity_rad_s{};
    uint32_t sequence = 0;
    bool valid = false;
};

struct ExecutorStep
{
    std::array<float, 7> position{};
    uint32_t sequence = 0;
    bool command_valid = false;
    bool entered_hold = false;
};

// Pure C++ 200 Hz target interpolator. Session/TTL/lease validation remains in
// ControlSession; this class only converts the latest validated target into a
// velocity- and acceleration-limited position command.
class ExternalTargetExecutor
{
public:
    explicit ExternalTargetExecutor(const ExecutorConfig& config);

    ExecutorStep Step(const ExecutorTarget& target, bool motion_allowed,
                      const std::array<float, 7>& measured_position);
    void Reset();

    bool active() const { return active_; }
    const std::array<float, 7>& commanded_position() const { return commanded_position_; }
    const std::array<float, 6>& commanded_velocity() const { return commanded_velocity_; }

private:
    ExecutorConfig config_{};
    std::array<float, 7> commanded_position_{};
    std::array<float, 6> commanded_velocity_{};
    bool initialized_ = false;
    bool active_ = false;
};

} // namespace dummy::protocol

#endif // DUMMY_EXTERNAL_TARGET_EXECUTOR_HPP
