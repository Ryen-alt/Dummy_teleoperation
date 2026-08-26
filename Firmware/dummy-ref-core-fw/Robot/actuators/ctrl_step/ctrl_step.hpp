#ifndef DUMMY_CORE_FW_CTRL_STEP_HPP
#define DUMMY_CORE_FW_CTRL_STEP_HPP

#include "fibre/protocol.hpp"
#include "can.h"

struct CanTxMetadata;

class CtrlStepMotor
{
public:
    enum State
    {
        RUNNING,
        FINISH,
        STOP
    };


    const uint32_t CTRL_CIRCLE_COUNT = 200 * 256;

    CtrlStepMotor(CAN_HandleTypeDef* _hcan, uint8_t _id, bool _inverse = false, uint8_t _reduction = 1,
                  float _angleLimitMin = -180, float _angleLimitMax = 180);

    uint8_t nodeID;
    float angle = 0;
    float angleLimitMax;
    float angleLimitMin;
    float temperature = 0.0F;
    bool inverseDirection;
    uint8_t reduction;
    State state = STOP;

    void SetAngle(float _angle);
    // Streaming control targets are already supervised by the 200 Hz firmware
    // executor. Suppress the motor-board ACK to keep CAN feedback deterministic.
    bool SetStreamingAngle(float _angle,
                           const CanTxMetadata* metadata = nullptr);
    bool SetStreamingAngleWithVelocityLimit(
        float _angle, float _vel, const CanTxMetadata* metadata = nullptr);
    void SetAngleWithVelocityLimit(float _angle, float _vel);
    // CAN Command
    void SetEnable(bool _enable);
    bool TrySetEnable(bool _enable, const CanTxMetadata* metadata = nullptr);
    void SetEnableTemp(bool _enable);
    void DoCalibration();
    void SetCurrentSetPoint(float _val);
    void SetVelocitySetPoint(float _val);
    void SetPositionSetPoint(float _val);
    void SetPositionWithVelocityLimit(float _pos, float _vel);
    void SetNodeID(uint32_t _id);
    void SetCurrentLimit(float _val);
    void SetVelocityLimit(float _val);
    bool TrySetVelocityLimitRam(
        float _val, const CanTxMetadata* metadata = nullptr);
    void SetAcceleration(float _val);
    void SetDceKp(int32_t _val);
    void SetDceKv(int32_t _val);
    void SetDceKi(int32_t _val);
    void SetDceKd(int32_t _val);
    void ApplyPositionAsHome();
    void SetEnableOnBoot(bool _enable);
    void SetEnableStallProtect(bool _enable);
    void Reboot();
    float GetTemp();
    bool TryGetTemp(const CanTxMetadata* metadata = nullptr);
    void EraseConfigs();

    void UpdateAngle();
    bool TryUpdateAngle(const CanTxMetadata* metadata = nullptr);
    void UpdateAngleCallback(float _pos, bool _isFinished);


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_ro_property("angle", &angle),
            make_protocol_function("reboot", *this, &CtrlStepMotor::Reboot),
            make_protocol_function("get_temperature", *this, &CtrlStepMotor::GetTemp),
            make_protocol_function("set_enable_temperature", *this, &CtrlStepMotor::SetEnableTemp, "enable"),
            make_protocol_function("erase_configs", *this, &CtrlStepMotor::EraseConfigs),
            make_protocol_function("set_enable", *this, &CtrlStepMotor::SetEnable, "enable"),
            make_protocol_function("set_position_with_time", *this,
                                   &CtrlStepMotor::SetPositionWithVelocityLimit, "pos", "time"),
            make_protocol_function("set_position", *this, &CtrlStepMotor::SetPositionSetPoint, "pos"),
            make_protocol_function("set_velocity", *this, &CtrlStepMotor::SetVelocitySetPoint, "vel"),
            make_protocol_function("set_velocity_limit", *this, &CtrlStepMotor::SetVelocityLimit, "vel"),
            make_protocol_function("set_current", *this, &CtrlStepMotor::SetCurrentSetPoint, "current"),
            make_protocol_function("set_current_limit", *this, &CtrlStepMotor::SetCurrentLimit, "current"),
            make_protocol_function("set_node_id", *this, &CtrlStepMotor::SetNodeID, "id"),
            make_protocol_function("set_acceleration", *this, &CtrlStepMotor::SetAcceleration, "acc"),
            make_protocol_function("apply_home_offset", *this, &CtrlStepMotor::ApplyPositionAsHome),
            make_protocol_function("do_calibration", *this, &CtrlStepMotor::DoCalibration),
            make_protocol_function("set_enable_on_boot", *this, &CtrlStepMotor::SetEnableOnBoot, "enable"),
            make_protocol_function("set_dce_kp", *this, &CtrlStepMotor::SetDceKp, "vel"),
            make_protocol_function("set_dce_kv", *this, &CtrlStepMotor::SetDceKv, "vel"),
            make_protocol_function("set_dce_ki", *this, &CtrlStepMotor::SetDceKi, "vel"),
            make_protocol_function("set_dce_kd", *this, &CtrlStepMotor::SetDceKd, "vel"),
            make_protocol_function("set_enable_stall_protect", *this, &CtrlStepMotor::SetEnableStallProtect,
                                   "enable"),
            make_protocol_function("update_angle", *this, &CtrlStepMotor::UpdateAngle)
        );
    }


private:
    bool SendPositionSetPoint(float _val, bool request_ack, bool non_blocking,
                              const CanTxMetadata* metadata = nullptr);
    bool SendPositionWithVelocityLimit(
        float _pos, float _vel, bool non_blocking,
        const CanTxMetadata* metadata = nullptr);

    CAN_HandleTypeDef* hcan;
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = {};
};

#endif //DUMMY_CORE_FW_CTRL_STEP_HPP
