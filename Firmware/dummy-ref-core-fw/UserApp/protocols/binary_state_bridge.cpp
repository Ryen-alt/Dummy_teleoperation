#include "common_inc.h"

#include "binary_state_bridge.hpp"
#include "joint_space_mapping.hpp"
#include "measured_state_estimator.hpp"
#include "feedback_runtime.hpp"
#include "configurations/robot_config_generated.hpp"

#include <algorithm>
#include <cmath>

extern DummyRobot robot;

namespace dummy::protocol
{
namespace
{
BinaryRobotMeasurement coherent_measurement;

std::array<float, 7> ReadLivePosition()
{
    std::array<float, 7> position{};
    for (size_t index = 0; index < 6; ++index)
        position[index] = LegacyFirmwareDegreesToUrdfRadians(
            robot.currentJoints.a[index], index);
    if (robot.hand != nullptr)
    {
        const float travel = robot.hand->closedAngle - robot.hand->openedAngle;
        if (std::fabs(travel) > 1e-6F)
            position[6] = std::clamp(
                (robot.hand->angle - robot.hand->openedAngle) / travel, 0.0F, 1.0F);
    }
    return position;
}
}

void LatchCoherentRobotMeasurement()
{
    const auto coherence = ReadCoherentFeedbackStatus();
    if (!coherence.valid || coherence.sweep_id == 0U)
        return;
    taskENTER_CRITICAL();
    if (coherent_measurement.coherent_sweep_id != coherence.sweep_id)
    {
        coherent_measurement.position = ReadLivePosition();
        coherent_measurement.position_sample_us = coherence.position_sample_us;
        coherent_measurement.position_sweep_id = coherence.position_sweep_id;
        coherent_measurement.coherent_sweep_id = coherence.sweep_id;
        coherent_measurement.max_skew_us = coherence.max_skew_us;
        coherent_measurement.absolute_position_generation =
            robot.AbsoluteJointPositionGeneration();
    }
    taskEXIT_CRITICAL();
}

BinaryRobotMeasurement ReadRobotStateForBinaryProtocol(
    uint64_t now_us, const FeedbackSafetyOutput& safety)
{
    (void) now_us;
    taskENTER_CRITICAL();
    BinaryRobotMeasurement output = coherent_measurement;
    taskEXIT_CRITICAL();
    const bool coherent_valid = output.coherent_sweep_id != 0U &&
        output.max_skew_us <= dummy::generated_config::kCoherentMaxSkewUs;
    const uint32_t absolute_position_generation =
        robot.AbsoluteJointPositionGeneration();
    const bool arm_absolute_valid = absolute_position_generation != 0U &&
        output.absolute_position_generation == absolute_position_generation;
    const bool gripper_position_valid = coherent_valid &&
        robot.hand != nullptr && safety.gripper_position_valid;
    output.validity = PositionFeedbackValidityBits(
        coherent_valid && arm_absolute_valid && safety.arm_position_valid,
        gripper_position_valid);

    static MeasuredStateEstimator estimator;
    const auto estimate = estimator.Update(
        output.position, output.position_sample_us,
        output.coherent_sweep_id,
        coherent_valid && arm_absolute_valid && safety.arm_position_valid &&
            gripper_position_valid);
    output.velocity = estimate.velocity;
    output.repeated = estimate.repeated;
    if (estimate.valid)
        output.validity |= kStateVelocityValid;
    return output;
}

} // namespace dummy::protocol
