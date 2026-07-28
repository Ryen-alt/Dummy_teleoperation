#include "common_inc.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

extern DummyRobot dummy;

extern "C" void ReadRobotStateForBinaryProtocol(float position[7], float velocity[7],
                                                  uint8_t* validity)
{
    constexpr float kDegreesToRadians = 0.01745329251994329577F;
    for (size_t index = 0; index < 6; ++index)
    {
        position[index] = dummy.currentJoints.a[index] * kDegreesToRadians;
        velocity[index] = 0.0F;
    }

    position[6] = 0.0F;
    velocity[6] = 0.0F;
    *validity = 0x01U; // Joint position valid; velocity is not measured yet.
    if (dummy.hand != nullptr)
    {
        const float travel = dummy.hand->closedAngle - dummy.hand->openedAngle;
        if (std::fabs(travel) > 1e-6F)
        {
            position[6] = std::clamp((dummy.hand->angle - dummy.hand->openedAngle) / travel,
                                     0.0F, 1.0F);
            *validity |= 0x04U;
        }
    }
}

