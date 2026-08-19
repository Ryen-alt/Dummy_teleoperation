#include "common_inc.h"

extern DummyRobot robot;


void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data)
{
    if (canCtx->handle->Instance != CAN1)
        return;

    const uint8_t id = rxHeader->StdId >> 7;
    const uint8_t cmd = rxHeader->StdId & 0x7F;

    CtrlStepMotor* actuator = nullptr;
    bool armJointResponse = false;
    if (id >= 1 && id <= 6)
    {
        actuator = robot.motorJ[id];
        armJointResponse = true;
    } else if (robot.hand != nullptr && id == robot.hand->nodeID)
    {
        actuator = robot.hand;
    }

    // Ignore responses from unconfigured CAN node IDs instead of indexing
    // beyond motorJ[0..6], as the original gripper reference code did.
    if (actuator == nullptr)
        return;

    switch (cmd)
    {
        case 0x23:
            if (rxHeader->DLC >= 5)
            {
                float position;
                memcpy(&position, data, sizeof(position));
                actuator->UpdateAngleCallback(position, data[4] != 0);
                if (armJointResponse)
                    robot.UpdateJointAnglesCallback();
            }
            break;
        case 0x25:
            if (rxHeader->DLC >= 4)
                memcpy(&actuator->temperature, data, sizeof(actuator->temperature));
            break;
        default:
            break;
    }
}
