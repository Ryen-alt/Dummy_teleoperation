#include "common_inc.h"

#include "joint_space_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

extern DummyRobot robot;

extern "C" void ReadRobotStateForBinaryProtocol(float position[7], float velocity[7],
                                                  uint8_t* validity)
{
    for (size_t index = 0; index < 6; ++index)
    {
        position[index] = dummy::protocol::LegacyFirmwareDegreesToUrdfRadians(
            robot.currentJoints.a[index], index);
        velocity[index] = 0.0F;
    }

    position[6] = 0.0F;
    velocity[6] = 0.0F;
    *validity = 0x01U; // Joint position valid; velocity is not measured yet.
    if (robot.hand != nullptr)
    {
        const float travel = robot.hand->closedAngle - robot.hand->openedAngle;
        if (std::fabs(travel) > 1e-6F)
        {
            position[6] = std::clamp((robot.hand->angle - robot.hand->openedAngle) / travel,
                                     0.0F, 1.0F);
            *validity |= 0x04U;
        }
    }
}

