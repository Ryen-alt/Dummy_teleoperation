#include "ctrl_step.hpp"
#include "communication.hpp"
#include "protocols/feedback_runtime.hpp"

namespace
{
struct PositionRequestContext
{
    uint8_t node_id = 0U;
    uint32_t sweep_id = 0U;
};

void RecordPositionRequest(void* context)
{
    if (context != nullptr)
    {
        const auto* request = static_cast<PositionRequestContext*>(context);
        dummy::protocol::RecordPositionFeedbackRequest(
            request->node_id, request->sweep_id);
    }
}

void RecordTemperatureRequest(void* context)
{
    if (context != nullptr)
        dummy::protocol::RecordTemperatureFeedbackRequest(*static_cast<uint8_t*>(context));
}
}


CtrlStepMotor::CtrlStepMotor(CAN_HandleTypeDef* _hcan, uint8_t _id, bool _inverse,
                             uint8_t _reduction, float _angleLimitMin, float _angleLimitMax) :
    nodeID(_id), hcan(_hcan), inverseDirection(_inverse), reduction(_reduction),
    angleLimitMin(_angleLimitMin), angleLimitMax(_angleLimitMax)
{
    txHeader =
        {
            .StdId = 0,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE
        };
}


void CtrlStepMotor::SetEnable(bool _enable)
{
    state = _enable ? FINISH : STOP;

    uint8_t mode = 0x01;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


bool CtrlStepMotor::TrySetEnable(bool _enable, const CanTxMetadata* metadata)
{
    uint8_t mode = 0x01;
    CAN_TxHeaderTypeDef request_header = txHeader;
    request_header.StdId = nodeID << 7 | mode;
    uint8_t request_data[8] = {};
    const uint32_t value = _enable ? 1U : 0U;
    memcpy(request_data, &value, sizeof(value));

    const bool queued = CanTrySendMessage(
        get_can_ctx(hcan), request_data, &request_header, nullptr, nullptr,
        metadata) == CanTxStatus::Queued;
    if (queued)
        state = _enable ? FINISH : STOP;
    return queued;
}

void CtrlStepMotor::SetEnableTemp(bool _enable)
{
    uint8_t mode = 0x7d;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void CtrlStepMotor::DoCalibration()
{
    uint8_t mode = 0x02;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentSetPoint(float _val)
{
    state = RUNNING;

    uint8_t mode = 0x03;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetVelocitySetPoint(float _val)
{
    state = RUNNING;

    uint8_t mode = 0x04;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetPositionSetPoint(float _val)
{
    (void) SendPositionSetPoint(_val, true, false);
}


bool CtrlStepMotor::SendPositionSetPoint(float _val, bool request_ack,
                                         bool non_blocking,
                                         const CanTxMetadata* metadata)
{
    uint8_t mode = 0x05;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = request_ack ? 1U : 0U;
    canBuf[5] = 0U;
    canBuf[6] = 0U;
    canBuf[7] = 0U;

    if (non_blocking)
        return CanTrySendMessage(
            get_can_ctx(hcan), canBuf, &txHeader, nullptr, nullptr, metadata) ==
            CanTxStatus::Queued;
    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetPositionWithVelocityLimit(float _pos, float _vel)
{
    (void) SendPositionWithVelocityLimit(_pos, _vel, false);
}


bool CtrlStepMotor::SendPositionWithVelocityLimit(float _pos, float _vel,
                                                   bool non_blocking,
                                                   const CanTxMetadata* metadata)
{
    uint8_t mode = 0x07;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_pos;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    b = (unsigned char*) &_vel;
    for (int i = 4; i < 8; i++)
        canBuf[i] = *(b + i - 4);

    if (non_blocking)
        return CanTrySendMessage(
            get_can_ctx(hcan), canBuf, &txHeader, nullptr, nullptr, metadata) ==
            CanTxStatus::Queued;
    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetNodeID(uint32_t _id)
{
    uint8_t mode = 0x11;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    auto* b = (unsigned char*) &_id;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentLimit(float _val)
{
    uint8_t mode = 0x12;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetVelocityLimit(float _val)
{
    uint8_t mode = 0x13;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetAcceleration(float _val)
{
    uint8_t mode = 0x14;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 0; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::ApplyPositionAsHome()
{
    uint8_t mode = 0x15;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetEnableOnBoot(bool _enable)
{
    uint8_t mode = 0x16;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetEnableStallProtect(bool _enable)
{
    uint8_t mode = 0x1B;
    txHeader.StdId = nodeID << 7 | mode;

    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::Reboot()
{
    uint8_t mode = 0x7f;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

float CtrlStepMotor::GetTemp()
{
    constexpr uint8_t mode = 0x25;
    CAN_TxHeaderTypeDef request_header = txHeader;
    request_header.StdId = nodeID << 7 | mode;
    uint8_t request_data[8] = {};

    CanSendMessage(get_can_ctx(hcan), request_data, &request_header,
                   RecordTemperatureRequest, &nodeID);
    return temperature;
}


bool CtrlStepMotor::TryGetTemp(const CanTxMetadata* metadata)
{
    constexpr uint8_t mode = 0x25;
    CAN_TxHeaderTypeDef request_header = txHeader;
    request_header.StdId = nodeID << 7 | mode;
    uint8_t request_data[8] = {};

    return CanTrySendMessage(get_can_ctx(hcan), request_data, &request_header,
                             RecordTemperatureRequest, &nodeID, metadata) ==
        CanTxStatus::Queued;
}

void CtrlStepMotor::EraseConfigs()
{
    uint8_t mode = 0x7e;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetAngle(float _angle)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionSetPoint(stepMotorCnt);
}


bool CtrlStepMotor::SetStreamingAngle(float _angle,
                                      const CanTxMetadata* metadata)
{
    _angle = inverseDirection ? -_angle : _angle;
    const float stepMotorCnt = _angle / 360.0f * (float) reduction;
    return SendPositionSetPoint(stepMotorCnt, false, true, metadata);
}


void CtrlStepMotor::SetAngleWithVelocityLimit(float _angle, float _vel)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionWithVelocityLimit(stepMotorCnt, _vel);
}


bool CtrlStepMotor::SetStreamingAngleWithVelocityLimit(
    float _angle, float _vel, const CanTxMetadata* metadata)
{
    _angle = inverseDirection ? -_angle : _angle;
    const float stepMotorCnt = _angle / 360.0f * (float) reduction;
    return SendPositionWithVelocityLimit(stepMotorCnt, _vel, true, metadata);
}


void CtrlStepMotor::UpdateAngle()
{
    constexpr uint8_t mode = 0x23;
    CAN_TxHeaderTypeDef request_header = txHeader;
    request_header.StdId = nodeID << 7 | mode;
    uint8_t request_data[8] = {};
    PositionRequestContext request_context{nodeID, 0U};

    CanSendMessage(get_can_ctx(hcan), request_data, &request_header,
                   RecordPositionRequest, &request_context);
}


bool CtrlStepMotor::TryUpdateAngle(const CanTxMetadata* metadata)
{
    constexpr uint8_t mode = 0x23;
    CAN_TxHeaderTypeDef request_header = txHeader;
    request_header.StdId = nodeID << 7 | mode;
    uint8_t request_data[8] = {};
    PositionRequestContext request_context{
        nodeID, metadata == nullptr ? 0U : metadata->feedback_sweep_id};

    return CanTrySendMessage(get_can_ctx(hcan), request_data, &request_header,
                             RecordPositionRequest, &request_context, metadata) ==
        CanTxStatus::Queued;
}


void CtrlStepMotor::UpdateAngleCallback(float _pos, bool _isFinished)
{
    state = _isFinished ? FINISH : RUNNING;

    float tmp = _pos / (float) reduction * 360;
    angle = inverseDirection ? -tmp : tmp;
}


void CtrlStepMotor::SetDceKp(int32_t _val)
{
    uint8_t mode = 0x17;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKv(int32_t _val)
{
    uint8_t mode = 0x18;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKi(int32_t _val)
{
    uint8_t mode = 0x19;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKd(int32_t _val)
{
    uint8_t mode = 0x1A;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}
