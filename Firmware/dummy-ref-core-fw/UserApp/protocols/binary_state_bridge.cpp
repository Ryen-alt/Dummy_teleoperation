#include "common_inc.h"

#include "binary_state_bridge.hpp"
#include "joint_space_mapping.hpp"
#include "measured_state_estimator.hpp"

#include <algorithm>
#include <cmath>

extern DummyRobot robot;

namespace dummy::protocol
{

BinaryRobotMeasurement ReadRobotStateForBinaryProtocol(
    uint64_t now_us, const FeedbackSafetyOutput& safety)
{
    BinaryRobotMeasurement output{};
    for (size_t index = 0; index < 6; ++index)
    {
        output.position[index] = LegacyFirmwareDegreesToUrdfRadians(
            robot.currentJoints.a[index], index);
    }

    output.position[6] = 0.0F;
    bool gripper_position_valid = false;
    if (robot.hand != nullptr)
    {
        const float travel = robot.hand->closedAngle - robot.hand->openedAngle;
        if (std::fabs(travel) > 1e-6F)
        {
            output.position[6] = std::clamp(
                (robot.hand->angle - robot.hand->openedAngle) / travel, 0.0F, 1.0F);
            gripper_position_valid = safety.gripper_position_valid;
        }
    }
    output.validity = PositionFeedbackValidityBits(
        safety.arm_position_valid, gripper_position_valid);

    static MeasuredStateEstimator estimator;
    const auto estimate = estimator.Update(
        output.position, now_us,
        safety.arm_position_valid && gripper_position_valid);
    output.velocity = estimate.velocity;
    if (estimate.valid)
        output.validity |= kStateVelocityValid;
    return output;
}

} // namespace dummy::protocol
