#include <algorithm>

#include "communication.hpp"
#include "dummy_robot.h"
#include "../../UserApp/protocols/joint_space_mapping.hpp"

namespace
{
// Metres. Derived from Dummy_URDF/dummy.urdf joint axes and the supplied CAD
// dimension drawing. Keep these values synchronized with validate_model.py.
constexpr float L_BASE = 0.15475f;
constexpr float D_BASE = 0.03500f;
constexpr float L_ARM = 0.14600f;
constexpr float L_FOREARM = 0.115455f;
constexpr float D_ELBOW = 0.05200f;
constexpr float L_WRIST = 0.09900f;
constexpr float RADIANS_TO_DEGREES = 57.295779513082320876f;

float LegacyLimitEndpointDegrees(float urdfRadians, size_t jointIndex)
{
    return dummy::protocol::UrdfRadiansToLegacyFirmwareRadians(urdfRadians, jointIndex) *
           RADIANS_TO_DEGREES;
}

float LegacyLimitMinDegrees(size_t jointIndex)
{
    const float endpointA = LegacyLimitEndpointDegrees(
        dummy::generated_config::kJointMinRad[jointIndex], jointIndex);
    const float endpointB = LegacyLimitEndpointDegrees(
        dummy::generated_config::kJointMaxRad[jointIndex], jointIndex);
    return endpointA < endpointB ? endpointA : endpointB;
}

float LegacyLimitMaxDegrees(size_t jointIndex)
{
    const float endpointA = LegacyLimitEndpointDegrees(
        dummy::generated_config::kJointMinRad[jointIndex], jointIndex);
    const float endpointB = LegacyLimitEndpointDegrees(
        dummy::generated_config::kJointMaxRad[jointIndex], jointIndex);
    return endpointA > endpointB ? endpointA : endpointB;
}
}

inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index)
{
    float max = -1;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (abs(_joints.a[i]) > max)
        {
            max = abs(_joints.a[i]);
            _index = i;
        }
    }

    return max;
}


DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);
    motorJ[1] = new CtrlStepMotor(_hcan, 1, false, 50, LegacyLimitMinDegrees(0), LegacyLimitMaxDegrees(0));
    motorJ[2] = new CtrlStepMotor(_hcan, 2, true, 50, LegacyLimitMinDegrees(1), LegacyLimitMaxDegrees(1));
    motorJ[3] = new CtrlStepMotor(_hcan, 3, true, 50, LegacyLimitMinDegrees(2), LegacyLimitMaxDegrees(2));
    motorJ[4] = new CtrlStepMotor(_hcan, 4, true, 50, LegacyLimitMinDegrees(3), LegacyLimitMaxDegrees(3));
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true, 50, LegacyLimitMinDegrees(4), LegacyLimitMaxDegrees(4));
    motorJ[6] = new CtrlStepMotor(_hcan, 6, true, 50, LegacyLimitMinDegrees(5), LegacyLimitMaxDegrees(5));
    hand = new StepHand(_hcan, 7);

    dof6Solver = new DOF6Kinematic(L_BASE, D_BASE, L_ARM, L_FOREARM, D_ELBOW, L_WRIST);
}


DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];

    delete hand;
    delete dof6Solver;
}


void DummyRobot::Init()
{
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);
    // Motor firmware only updates its temperature measurement when this flag
    // is enabled. Node 0 broadcasts to the six arm joints and gripper node.
    motorJ[ALL]->SetEnableTemp(true);
}


void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();
    osDelay(500); // waiting for all joints done
    HAL_NVIC_SystemReset();
}

void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    for (int j = 1; j <= 6; j++)
    {
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                             dynamicJointSpeeds.a[j - 1]);
    }
}


void DummyRobot::ApplyExternalUrdfTargetRad(const std::array<float, 7>& target)
{
    constexpr float kRadiansToDegrees = 57.295779513082320876F;
    for (int index = 0; index < 6; ++index)
    {
        const float target_degrees =
            dummy::protocol::UrdfRadiansToLegacyFirmwareRadians(target[index], index) *
            kRadiansToDegrees;
        targetJoints.a[index] = target_degrees;
        // The 200 Hz executor has already bounded position, velocity and
        // acceleration. Send its incremental absolute target directly.
        motorJ[index + 1]->SetAngle(target_degrees - initPose.a[index]);
    }
    hand->SetNormalizedPosition(
        target[6], dummy::generated_config::kGripperVelocityLimitPerS);
}


void DummyRobot::HoldCurrentPosition()
{
    targetJoints = currentJoints;
    for (int index = 0; index < 6; ++index)
        motorJ[index + 1]->SetAngle(currentJoints.a[index] - initPose.a[index]);
    if (hand != nullptr)
    {
        const float travel = hand->closedAngle - hand->openedAngle;
        if (fabsf(travel) > 1e-6F)
        {
            const float normalized = std::clamp(
                (hand->angle - hand->openedAngle) / travel, 0.0F, 1.0F);
            hand->SetNormalizedPosition(
                normalized, dummy::generated_config::kGripperVelocityLimitPerS);
        }
    }
}


bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    bool valid = true;

    for (int j = 1; j <= 6; j++)
    {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }

    if (valid)
    {
        DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;
        uint8_t index;
        float maxAngle = AbsMaxOf6(deltaJoints, index);
        float time = maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;
        for (int j = 1; j <= 6; j++)
        {
            dynamicJointSpeeds.a[j - 1] =
                abs(deltaJoints.a[j - 1] * (float) (motorJ[j]->reduction) / time * 0.1f); //0~10r/s
        }

        jointsStateFlag = 0;
        targetJoints = targetJointsTmp;

        return true;
    }

    return false;
}


bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};

    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);

    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;

        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                continue;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        return MoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5]);
    }

    return false;
}

void DummyRobot::RequestPositionFeedback(uint8_t node_id)
{
    if (node_id >= 1U && node_id <= 6U)
        motorJ[node_id]->UpdateAngle();
    else if (hand != nullptr && node_id == hand->nodeID)
        hand->UpdateAngle();
}


void DummyRobot::RequestTemperatureFeedback(uint8_t node_id)
{
    if (node_id >= 1U && node_id <= 6U)
        motorJ[node_id]->GetTemp();
    else if (hand != nullptr && node_id == hand->nodeID)
        hand->GetTemp();
}


void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];

        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
}


void DummyRobot::SetJointSpeed(float _speed)
{
    if (_speed < 0)_speed = 0;
    else if (_speed > 100) _speed = 100;

    jointSpeed = _speed * jointSpeedRatio;
}


void DummyRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 0)_acc = 0;
    else if (_acc > 100) _acc = 100;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(_acc / 100 * DEFAULT_JOINT_ACCELERATION_BASES.a[i - 1]);
}


void DummyRobot::CalibrateHomeOffset()
{
    // Disable FixUpdate, but not disable motors
    isEnabled = false;
    motorJ[ALL]->SetEnable(true);

    // 1.Manually move joints to L-Pose [precisely]
    // ...
    motorJ[2]->SetCurrentLimit(0.5);
    motorJ[3]->SetCurrentLimit(0.5);
    osDelay(500);

    // 2.Apply Home-Offset the first time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);

    // 3.Go to Resting-Pose
    initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    Resting();
    osDelay(500);

    // 4.Apply Home-Offset the second time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);
    motorJ[2]->SetCurrentLimit(1);
    motorJ[3]->SetCurrentLimit(1);
    osDelay(500);

    Reboot();
}


void DummyRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5]);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DummyRobot::SetEnable(bool _enable)
{
    // The control loop continuously transmits targetJoints as soon as isEnabled
    // becomes true.  targetJoints is initialized to REST_POSE, so enabling before
    // latching the measured pose makes every joint jump toward REST_POSE.  Load a
    // hold target into both the robot state and the motor boards before energizing
    // the axes.  This also makes repeated !START commands stop at the measured
    // pose instead of resurrecting a stale target.
    if (_enable)
        HoldCurrentPosition();

    motorJ[ALL]->SetEnable(_enable);
    isEnabled = _enable;
}

void DummyRobot::SetRGBEnable(bool _enable)
{
    isRGBEnabled = _enable;
}

bool DummyRobot::GetRGBEnabled()
{
    return isRGBEnabled;
}

void DummyRobot::SetRGBMode(uint32_t mode)
{
    rgbMode = mode;
}

uint32_t DummyRobot::GetRGBMode()
{
    return rgbMode;
}

void DummyRobot::UpdateJointPose6D()
{
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    currentPose6D.X *= 1000; // m -> mm
    currentPose6D.Y *= 1000; // m -> mm
    currentPose6D.Z *= 1000; // m -> mm
}


bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;
}


bool DummyRobot::IsEnabled()
{
    return isEnabled;
}


void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_MOTOR_TUNING)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1;
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            break;
        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH);
            jointSpeedRatio = 0.3;
            break;
        case COMMAND_MOTOR_TUNING:
            break;
    }
}


StepHand::StepHand(CAN_HandleTypeDef* _hcan, uint8_t _id) :
    CtrlStepMotor(_hcan, _id, false, 8, -115.0f, 115.0f)
{
}


void StepHand::SetPercent(float _percent)
{
    if (_percent < 0.0f)_percent = 0.0f;
    else if (_percent > 100.0f)_percent = 100.0f;

    SetNormalizedPosition(
        _percent / 100.0F, dummy::generated_config::kGripperVelocityLimitPerS);
}


void StepHand::SetNormalizedPosition(float normalized, float max_velocity_per_s)
{
    if (normalized < 0.0F) normalized = 0.0F;
    else if (normalized > 1.0F) normalized = 1.0F;
    if (max_velocity_per_s <= 0.0F)
        return;

    const float travel_degrees = closedAngle - openedAngle;
    const float target_angle = openedAngle + normalized * travel_degrees;
    // CtrlStep velocity uses motor revolutions per second. Convert from the
    // configured normalized gripper travel per second.
    const float motor_velocity =
        fabsf(travel_degrees) / 360.0F * static_cast<float>(reduction) *
        max_velocity_per_s;
    SetAngleWithVelocityLimit(target_angle, motor_velocity);
}


void StepHand::SetGripCurrent(float _current)
{
    if (_current < 0.0f)_current = 0.0f;
    else if (_current > 2.0f)_current = 2.0f;
    gripCurrent = _current;
}


void StepHand::DriveWithCurrent(float _direction)
{
    if (_direction < -1.0f)_direction = -1.0f;
    else if (_direction > 1.0f)_direction = 1.0f;
    SetCurrentSetPoint(_direction * gripCurrent);
}


void StepHand::HandCalibration()
{
    if (isCalibrating)
        return;

    isCalibrating = true;
    SetEnable(true);

    SetAngleWithVelocityLimit(openedAngle, 7.0f);
    HAL_Delay(900);
    ApplyPositionAsHome();
    HAL_Delay(100);
    UpdateAngle();
    HAL_Delay(100);
    openedAngle = angle;

    SetAngleWithVelocityLimit(closedAngle, 7.0f);
    HAL_Delay(900);
    UpdateAngle();
    HAL_Delay(100);
    closedAngle = angle;

    SetEnable(false);
    isCalibrating = false;
}


void StepHand::SetGripperEnable(bool _enable)
{
    SetEnable(_enable);
}


void StepHand::RequestAngle()
{
    UpdateAngle();
}


bool StepHand::IsEnabled() const
{
    return state != STOP;
}


DummyHand::DummyHand(CAN_HandleTypeDef* _hcan, uint8_t
_id) :
    nodeID(_id), hcan(_hcan)
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


void DummyHand::SetAngle(float _angle)
{
    if (_angle > 30)_angle = 30;
    if (_angle < 0)_angle = 0;

    uint8_t mode = 0x02;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_angle;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetMaxCurrent(float _val)
{
    if (_val > 1)_val = 1;
    if (_val < 0)_val = 0;

    uint8_t mode = 0x01;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetEnable(bool _enable)
{
    if (_enable)
        SetMaxCurrent(maxCurrent);
    else
        SetMaxCurrent(0);
}


uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd)
{
    osStatus_t status = osMessageQueuePut(commandFifo, _cmd.c_str(), 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; // failed
}


void DummyRobot::CommandHandler::EmergencyStop()
{
    context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
                   context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    context->MoveJoints(context->targetJoints);
    context->hand->DriveWithCurrent(0.0f);
    context->hand->SetGripperEnable(false);
    context->isEnabled = false;
    ClearFifo();
}


std::string DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);

    return std::string{strBuffer};
}


uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}


uint32_t DummyRobot::CommandHandler::ParseCommand(const std::string &_cmd)
{
    uint8_t argNum;

    switch (context->commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_CONTINUES_TRAJECTORY:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float joints[6];
                float speed;

                if (_cmd[0] == '>')
                    argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (_cmd[0] == '&')
                    argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                context->MoveJoints(context->targetJoints);

                while (context->IsMoving() && context->IsEnabled())
                    osDelay(5);
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }

            break;

        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float joints[6];
                float speed;

                if (_cmd[0] == '>')
                    argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (_cmd[0] == '&')
                    argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }
            break;

        case COMMAND_MOTOR_TUNING:
            break;
    }

    return osMessageQueueGetSpace(commandFifo);
}


void DummyRobot::CommandHandler::ClearFifo()
{
    osMessageQueueReset(commandFifo);
}


void DummyRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}


void DummyRobot::TuningHelper::Tick(uint32_t _timeMillis)
{
    time += PI * 2 * frequency * (float) _timeMillis / 1000.0f;
    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (tuningFlag & (1 << (i - 1)))
            context->motorJ[i]->SetAngle(delta);
}


void DummyRobot::TuningHelper::SetFreqAndAmp(float _freq, float _amp)
{
    if (_freq > 5)_freq = 5;
    else if (_freq < 0.1) _freq = 0.1;
    if (_amp > 50)_amp = 50;
    else if (_amp < 1) _amp = 1;

    frequency = _freq;
    amplitude = _amp;
}
