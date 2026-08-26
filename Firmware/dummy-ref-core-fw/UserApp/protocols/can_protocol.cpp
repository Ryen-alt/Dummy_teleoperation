#include "common_inc.h"

#include "feedback_runtime.hpp"

extern DummyRobot robot;


void OnCanMessage(CAN_context* canCtx, const CAN_RxHeaderTypeDef* rxHeader,
                  const uint8_t* data, uint32_t received_us)
{
    if (canCtx == nullptr || canCtx->handle == nullptr ||
        canCtx->handle->Instance != CAN1)
        return;
    if (rxHeader == nullptr || data == nullptr ||
        rxHeader->IDE != CAN_ID_STD || rxHeader->RTR != CAN_RTR_DATA)
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
                dummy::protocol::RecordPositionFeedbackResponse(
                    id, received_us);
                if (armJointResponse)
                    robot.UpdateJointAnglesCallback();
            }
            break;
        case 0x25:
            if (rxHeader->DLC >= 4)
            {
                memcpy(&actuator->temperature, data, sizeof(actuator->temperature));
                dummy::protocol::RecordTemperatureFeedbackResponse(
                    id, actuator->temperature, received_us);
                dummy::protocol::RecordMotorTransportDiagnostics(
                    id, data, rxHeader->DLC);
            }
            break;
        default:
            break;
    }
}
