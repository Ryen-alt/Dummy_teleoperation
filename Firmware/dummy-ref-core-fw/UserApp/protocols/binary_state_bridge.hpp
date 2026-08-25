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
    std::array<uint32_t, 7> position_sample_us{};
    std::array<uint32_t, 7> position_sweep_id{};
    uint32_t coherent_sweep_id = 0;
    uint32_t max_skew_us = 0;
    bool repeated = false;
};

// Called by the CAN dispatcher immediately after consuming a completed sweep.
// It freezes all seven positions before the next node request can make the
// live motor objects temporally mixed.
void LatchCoherentRobotMeasurement();

// Builds measured position, velocity and validity from the exact safety
// snapshot that will be serialized alongside them in STATE.
BinaryRobotMeasurement ReadRobotStateForBinaryProtocol(
    uint64_t now_us, const FeedbackSafetyOutput& safety);

} // namespace dummy::protocol

#endif // DUMMY_BINARY_STATE_BRIDGE_HPP
