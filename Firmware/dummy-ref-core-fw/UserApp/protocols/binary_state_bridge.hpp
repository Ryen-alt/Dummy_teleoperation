#ifndef DUMMY_BINARY_STATE_BRIDGE_HPP
#define DUMMY_BINARY_STATE_BRIDGE_HPP

#include "feedback_safety_supervisor.hpp"

#include <array>
#include <cstdint>

namespace dummy::protocol
{

constexpr uint8_t kStatePositionValid = 1U << 0U;
constexpr uint8_t kStateVelocityValid = 1U << 1U;
constexpr uint8_t kStateGripperValid = 1U << 2U;

constexpr uint8_t PositionFeedbackValidityBits(bool arm_valid, bool gripper_valid)
{
    return static_cast<uint8_t>(
        (arm_valid ? kStatePositionValid : 0U) |
        (gripper_valid ? kStateGripperValid : 0U));
}

struct BinaryRobotMeasurement
{
    std::array<float, 7> position{};
    std::array<float, 7> velocity{};
    uint8_t validity = 0;
};

// Builds measured position, velocity and validity from the exact safety
// snapshot that will be serialized alongside them in STATE.
BinaryRobotMeasurement ReadRobotStateForBinaryProtocol(
    uint64_t now_us, const FeedbackSafetyOutput& safety);

} // namespace dummy::protocol

#endif // DUMMY_BINARY_STATE_BRIDGE_HPP
