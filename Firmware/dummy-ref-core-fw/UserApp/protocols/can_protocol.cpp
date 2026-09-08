#include "common_inc.h"

#include "feedback_runtime.hpp"
#include "ieee754_finite.hpp"
#include "../../../can_transport_contract.h"

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
            // Validate DLC, payload finiteness and the feedback transaction
            // BEFORE mutating shared realtime state (doc 05 section 3.2 /
            // 10.1). A response the monitor rejects must never update the
            // live actuator angle.
            if (rxHeader->DLC >= 5)
            {
                float position;
                memcpy(&position, data, sizeof(position));
                if (dummy::protocol::Ieee754IsFinite(position) &&
                    dummy::protocol::RecordPositionFeedbackResponse(
                        id, received_us))
                {
                    actuator->UpdateAngleCallback(position, data[4] != 0);
                    if (armJointResponse)
                        robot.UpdateJointAnglesCallback();
                    // Seal after the absolute-branch resolution so value and
                    // metadata describe the same sample.
                    dummy::protocol::SealJointPositionSample(
                        id, received_us);
                }
            }
            break;
        case 0x25:
            // Same rule: the shared temperature value is committed only after
            // the monitor accepted the response for the pending transaction.
            if (rxHeader->DLC >= 4)
            {
                float temperature;
                memcpy(&temperature, data, sizeof(temperature));
                if (dummy::protocol::Ieee754IsFinite(temperature) &&
                    dummy::protocol::RecordTemperatureFeedbackResponse(
                        id, temperature, received_us))
                {
                    actuator->temperature = temperature;
                }
                dummy::protocol::RecordMotorTransportDiagnostics(
                    id, data, rxHeader->DLC);
            }
            break;
        case DUMMY_MOTOR_TIMING_COMMAND:
            dummy::protocol::RecordMotorTimingProfile(
                id, data, rxHeader->DLC);
            break;
        default:
            break;
    }
}
