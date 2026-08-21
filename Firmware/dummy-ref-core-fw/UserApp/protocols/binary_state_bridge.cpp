#include "common_inc.h"

#include "joint_space_mapping.hpp"
#include "measured_state_estimator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

extern DummyRobot robot;

extern "C" void ReadRobotStateForBinaryProtocol(float position[7], float velocity[7],
                                                  uint8_t* validity, uint64_t now_us)
{
    std::array<float, 7> measured_position{};
    for (size_t index = 0; index < 6; ++index)
    {
        measured_position[index] = dummy::protocol::LegacyFirmwareDegreesToUrdfRadians(
            robot.currentJoints.a[index], index);
    }

    measured_position[6] = 0.0F;
    *validity = 0x01U;
    if (robot.hand != nullptr)
    {
        const float travel = robot.hand->closedAngle - robot.hand->openedAngle;
        if (std::fabs(travel) > 1e-6F)
        {
            measured_position[6] = std::clamp(
                (robot.hand->angle - robot.hand->openedAngle) / travel, 0.0F, 1.0F);
            *validity |= 0x04U;
        }
    }

    static dummy::protocol::MeasuredStateEstimator estimator;
    const auto estimate = estimator.Update(measured_position, now_us, true);
    std::copy(measured_position.begin(), measured_position.end(), position);
    std::copy(estimate.velocity.begin(), estimate.velocity.end(), velocity);
    if (estimate.valid)
        *validity |= 0x02U;
}
