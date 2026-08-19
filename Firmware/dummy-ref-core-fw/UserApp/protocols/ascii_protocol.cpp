#include "common_inc.h"

extern DummyRobot robot;


static CtrlStepMotor* FindActuator(uint32_t node)
{
    if (node >= 1 && node <= 6)
        return robot.motorJ[node];
    if (robot.hand != nullptr && node == robot.hand->nodeID)
        return robot.hand;
    return nullptr;
}


static bool HandleHandCommand(const char* cmd, const std::string &command, StreamSink &responseChannel)
{
    if (command.rfind("!HAND_POS", 0) == 0)
    {
        uint32_t percent;
        if (sscanf(cmd, "!HAND_POS %lu", &percent) != 1)
        {
            Respond(responseChannel, "error hand position - use !HAND_POS <0-100>");
        } else if (percent > 100)
        {
            Respond(responseChannel, "error hand position %lu - expected 0-100", percent);
        } else
        {
            robot.hand->SetPercent(static_cast<float>(percent));
            Respond(responseChannel, "ok hand position %lu", percent);
        }
        return true;
    }

    if (command.rfind("!HAND_I", 0) == 0)
    {
        float current;
        if (sscanf(cmd, "!HAND_I %f", &current) != 1)
        {
            Respond(responseChannel, "error hand current - use !HAND_I <0-2.0>");
        } else if (current < 0.0f || current > 2.0f)
        {
            Respond(responseChannel, "error hand current %.3f - expected 0-2.0", current);
        } else
        {
            robot.hand->SetGripCurrent(current);
            Respond(responseChannel, "ok hand current %.3f", current);
        }
        return true;
    }

    if (command.rfind("!HAND_ZERO", 0) == 0)
    {
        Respond(responseChannel, "hand calibration start");
        robot.hand->HandCalibration();
        Respond(responseChannel, "ok hand calibration open=%.2f closed=%.2f",
                robot.hand->openedAngle, robot.hand->closedAngle);
        return true;
    }

    if (command.rfind("!HAND_EN", 0) == 0)
    {
        robot.hand->SetGripperEnable(true);
        Respond(responseChannel, "ok hand enabled=%d", static_cast<int>(robot.hand->IsEnabled()));
        return true;
    }

    if (command.rfind("!HAND_DIS", 0) == 0)
    {
        robot.hand->DriveWithCurrent(0.0f);
        robot.hand->SetGripperEnable(false);
        Respond(responseChannel, "ok hand enabled=%d", static_cast<int>(robot.hand->IsEnabled()));
        return true;
    }

    if (command.rfind("!HAND_O", 0) == 0)
    {
        robot.hand->DriveWithCurrent(1.0f);
        Respond(responseChannel, "ok hand open");
        return true;
    }

    if (command.rfind("!HAND_C", 0) == 0)
    {
        robot.hand->DriveWithCurrent(-1.0f);
        Respond(responseChannel, "ok hand close");
        return true;
    }

    return false;
}


static void HandleBangCommand(const char* cmd, StreamSink &responseChannel)
{
    std::string command(cmd);
    if (command.find("STOP") != std::string::npos)
    {
        robot.commandHandler.EmergencyStop();
        Respond(responseChannel, "Stopped ok");
    } else if (command.find("START") != std::string::npos)
    {
        robot.SetEnable(true);
        Respond(responseChannel, "Started ok");
    } else if (command.find("HOME") != std::string::npos)
    {
        robot.Homing();
        Respond(responseChannel, "Started ok");
    } else if (command.find("CALIBRATION") != std::string::npos)
    {
        robot.CalibrateHomeOffset();
        Respond(responseChannel, "calibration ok");
    } else if (command.find("RESET") != std::string::npos)
    {
        robot.Resting();
        Respond(responseChannel, "Started ok");
    } else if (command.find("DISABLE") != std::string::npos)
    {
        robot.SetEnable(false);
        Respond(responseChannel, "Disabled ok");
    } else if (!HandleHandCommand(cmd, command, responseChannel))
    {
        Respond(responseChannel, "error unknown command");
    }
}


void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    if (_len == 0)
        return;

    if (_cmd[0] == '!')
    {
        HandleBangCommand(_cmd, _responseChannel);
    } else if (_cmd[0] == '#')
    {
        std::string command(_cmd);
        if (command.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    robot.currentJoints.a[0], robot.currentJoints.a[1],
                    robot.currentJoints.a[2], robot.currentJoints.a[3],
                    robot.currentJoints.a[4], robot.currentJoints.a[5]);
        } else if (command.find("GETLPOS") != std::string::npos)
        {
            robot.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    robot.currentPose6D.X, robot.currentPose6D.Y,
                    robot.currentPose6D.Z, robot.currentPose6D.A,
                    robot.currentPose6D.B, robot.currentPose6D.C);
        } else if (command.find("SET_DCE_KV") != std::string::npos)
        {
            uint32_t node, value;
            if (sscanf(_cmd, "#SET_DCE_KV %lu %lu", &node, &value) == 2 && node >= 1 && node <= 6)
            {
                robot.motorJ[node]->SetDceKv(value);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KV [%lu]", node, value);
            } else
                Respond(_responseChannel, "error SET_DCE_KV - use node 1-6");
        } else if (command.find("SET_DCE_KP") != std::string::npos)
        {
            uint32_t node, value;
            if (sscanf(_cmd, "#SET_DCE_KP %lu %lu", &node, &value) == 2 && node >= 1 && node <= 6)
            {
                robot.motorJ[node]->SetDceKp(value);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KP [%lu]", node, value);
            } else
                Respond(_responseChannel, "error SET_DCE_KP - use node 1-6");
        } else if (command.find("SET_DCE_KI") != std::string::npos)
        {
            uint32_t node, value;
            if (sscanf(_cmd, "#SET_DCE_KI %lu %lu", &node, &value) == 2 && node >= 1 && node <= 6)
            {
                robot.motorJ[node]->SetDceKi(value);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KI [%lu]", node, value);
            } else
                Respond(_responseChannel, "error SET_DCE_KI - use node 1-6");
        } else if (command.find("SET_DCE_KD") != std::string::npos)
        {
            uint32_t node, value;
            if (sscanf(_cmd, "#SET_DCE_KD %lu %lu", &node, &value) == 2 && node >= 1 && node <= 6)
            {
                robot.motorJ[node]->SetDceKd(value);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KD [%lu]", node, value);
            } else
                Respond(_responseChannel, "error SET_DCE_KD - use node 1-6");
        } else if (command.find("REBOOT") != std::string::npos)
        {
            uint32_t node;
            CtrlStepMotor* actuator = nullptr;
            if (sscanf(_cmd, "#REBOOT %lu", &node) == 1)
                actuator = FindActuator(node);
            if (actuator != nullptr)
            {
                actuator->Reboot();
                Respond(_responseChannel, "ok REBOOT MOTOR [%lu]", node);
            } else
                Respond(_responseChannel, "error REBOOT - use node 1-7");
        } else if (command.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            if (sscanf(_cmd, "#CMDMODE %lu", &mode) == 1)
            {
                robot.SetCommandMode(mode);
                Respond(_responseChannel, "ok Set command mode to [%lu]", mode);
            } else
                Respond(_responseChannel, "error CMDMODE");
        } else if (command.find("OFFSET_J") != std::string::npos)
        {
            uint32_t node;
            CtrlStepMotor* actuator = nullptr;
            if (sscanf(_cmd, "#OFFSET_J %lu", &node) == 1)
                actuator = FindActuator(node);
            if (actuator != nullptr)
            {
                actuator->ApplyPositionAsHome();
                Respond(_responseChannel, "ok HOMEOFFSET MOTOR [%lu]", node);
            } else
                Respond(_responseChannel, "error OFFSET_J - use node 1-7");
        } else if (command.find("ACC_J") != std::string::npos)
        {
            uint32_t node;
            float value;
            CtrlStepMotor* actuator = nullptr;
            if (sscanf(_cmd, "#ACC_J %lu %f", &node, &value) == 2)
                actuator = FindActuator(node);
            if (actuator != nullptr)
            {
                actuator->SetAcceleration(value);
                Respond(_responseChannel, "ok SET MOTOR [%lu] ACCELERATION [%.3f]", node, value);
            } else
                Respond(_responseChannel, "error ACC_J - use node 1-7");
        } else if (command.find("SPEED_J") != std::string::npos)
        {
            uint32_t node;
            float value;
            CtrlStepMotor* actuator = nullptr;
            if (sscanf(_cmd, "#SPEED_J %lu %f", &node, &value) == 2)
                actuator = FindActuator(node);
            if (actuator != nullptr)
            {
                actuator->SetVelocityLimit(value);
                Respond(_responseChannel, "ok SET MOTOR [%lu] SPEED [%.3f]", node, value);
            } else
                Respond(_responseChannel, "error SPEED_J - use node 1-7");
        } else if (command.find("I_LIMIT_J") != std::string::npos)
        {
            uint32_t node;
            float value;
            CtrlStepMotor* actuator = nullptr;
            if (sscanf(_cmd, "#I_LIMIT_J %lu %f", &node, &value) == 2)
                actuator = FindActuator(node);
            if (actuator != nullptr)
            {
                actuator->SetCurrentLimit(value);
                Respond(_responseChannel, "ok SET MOTOR [%lu] CURRENT_LIMIT [%.3f]", node, value);
            } else
                Respond(_responseChannel, "error I_LIMIT_J - use node 1-7");
        } else
            Respond(_responseChannel, "ok");
    } else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&')
    {
        uint32_t freeSize = robot.commandHandler.Push(_cmd);
        Respond(_responseChannel, "%lu", freeSize);
    }
}


void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    if (_len == 0)
        return;

    if (_cmd[0] == '!')
    {
        HandleBangCommand(_cmd, _responseChannel);
    } else if (_cmd[0] == '#')
    {
        std::string command(_cmd);
        if (command.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    robot.currentJoints.a[0], robot.currentJoints.a[1],
                    robot.currentJoints.a[2], robot.currentJoints.a[3],
                    robot.currentJoints.a[4], robot.currentJoints.a[5]);
        } else if (command.find("GETLPOS") != std::string::npos)
        {
            robot.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    robot.currentPose6D.X, robot.currentPose6D.Y,
                    robot.currentPose6D.Z, robot.currentPose6D.A,
                    robot.currentPose6D.B, robot.currentPose6D.C);
        } else if (command.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            if (sscanf(_cmd, "#CMDMODE %lu", &mode) == 1)
            {
                robot.SetCommandMode(mode);
                Respond(_responseChannel, "Set command mode to [%lu]", mode);
            } else
                Respond(_responseChannel, "error CMDMODE");
        } else
            Respond(_responseChannel, "ok");
    } else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&')
    {
        uint32_t freeSize = robot.commandHandler.Push(_cmd);
        Respond(_responseChannel, "%lu", freeSize);
    }
}


void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    (void) _cmd;
    (void) _len;
    (void) _responseChannel;
}
