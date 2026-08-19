#include "common_inc.h"
#include "configurations/robot_config_generated.hpp"
#include "protocols/binary_control_bridge.hpp"
#include "protocols/external_target_executor.hpp"
#include "protocols/joint_space_mapping.hpp"

#include <algorithm>
#include <array>
#include <cmath>


// On-board Screen, can choose from hi2c2 or hi2c0(soft i2c)
SSD1306 oled(&hi2c0);
// On-board Sensor, used hi2c1
MPU6050 mpu6050(&hi2c1);
// 5 User-Timers, can choose from htim7/htim10/htim11/htim13/htim14
Timer timerCtrlLoop(&htim7, 200);
// 2x2-channel PWMs, used htim9 & htim12, each has 2-channel outputs
PWM pwm(21000, 21000);

RGB rgb(0);
// Robot instance
DummyRobot robot(&hcan1);

namespace
{
dummy::protocol::ExecutorConfig MakeExternalExecutorConfig()
{
    dummy::protocol::ExecutorConfig config{};
    config.max_acceleration_rad_s2 = dummy::generated_config::kMaxAccelerationRadS2;
    config.loop_rate_hz = dummy::generated_config::kFirmwareLoopHz;
    return config;
}

dummy::protocol::ExternalTargetExecutor external_target_executor(MakeExternalExecutorConfig());
}


/* Thread Definitions -----------------------------------------------------*/
osThreadId_t controlLoopFixUpdateHandle;
void ThreadControlLoopFixUpdate(void* argument)
{
    for (;;)
    {
        // Suspended here until got Notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        std::array<float, 7> measured_position{};
        for (size_t index = 0; index < 6; ++index)
            measured_position[index] = dummy::protocol::LegacyFirmwareDegreesToUrdfRadians(
                robot.currentJoints.a[index], index);
        if (robot.hand != nullptr)
        {
            const float travel = robot.hand->closedAngle - robot.hand->openedAngle;
            if (fabsf(travel) > 1e-6F)
                measured_position[6] = std::clamp(
                    (robot.hand->angle - robot.hand->openedAngle) / travel, 0.0F, 1.0F);
        }

        const auto binary_snapshot = dummy::protocol::ReadBinaryControlSnapshot(
            dummy::protocol::BinaryControlMonotonicMicros());
        const bool binary_motion_mode =
            binary_snapshot.mode == dummy::protocol::ControlMode::Teleop ||
            binary_snapshot.mode == dummy::protocol::ControlMode::Policy;
        const bool binary_context_active = binary_snapshot.lease_active ||
            external_target_executor.active() ||
            binary_snapshot.mode == dummy::protocol::ControlMode::Fault;

        if (binary_context_active)
        {
            const auto step = external_target_executor.Step(
                binary_snapshot.target,
                binary_snapshot.lease_active && binary_motion_mode,
                measured_position);
            if (step.entered_hold)
                robot.HoldCurrentPosition();
            if (binary_snapshot.mode == dummy::protocol::ControlMode::Fault)
                robot.commandHandler.EmergencyStop();
            else if (step.command_valid)
            {
                if (!robot.IsEnabled())
                    robot.SetEnable(true);
                robot.ApplyExternalUrdfTargetRad(step.position);
                dummy::protocol::MarkBinaryTargetApplied(step.sequence);
            }
            robot.UpdateJointAngles();
            robot.UpdateJointPose6D();
        }
        else if (robot.IsEnabled())
        {
            // Send control command to Motors & update Joint states
            switch (robot.commandMode)
            {
                case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
                case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
                case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
                    robot.MoveJoints(robot.targetJoints);
                    robot.UpdateJointPose6D();
                    break;
                case DummyRobot::COMMAND_MOTOR_TUNING:
                    robot.tuningHelper.Tick(10);
                    robot.UpdateJointPose6D();
                    break;
            }
        } else
        {
            // Just update Joint states
            robot.UpdateJointAngles();
            robot.UpdateJointPose6D();
        }
    }
}


osThreadId_t ControlLoopUpdateHandle;
void ThreadControlLoopUpdate(void* argument)
{
    for (;;)
    {
        robot.commandHandler.ParseCommand(robot.commandHandler.Pop(osWaitForever));
    }
}


osThreadId_t oledTaskHandle;
void ThreadOledUpdate(void* argument)
{
    uint32_t t = micros();
    char buf[16];
    char cmdModeNames[4][4] = {"SEQ", "INT", "TRJ", "TUN"};

    for (;;)
    {
        mpu6050.Update(true);

        oled.clearBuffer();
        oled.setFont(u8g2_font_5x8_tr);
        oled.setCursor(0, 10);
        oled.printf("IMU:%.3f/%.3f", mpu6050.data.ax, mpu6050.data.ay);
        oled.setCursor(85, 10);
        oled.printf("| FPS:%lu", 1000000 / (micros() - t));
        t = micros();

        oled.drawBox(0, 15, 128, 3);
        oled.setCursor(0, 30);
        oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d",
                    (int) roundf(robot.currentJoints.a[0]), (int) roundf(robot.currentJoints.a[1]),
                    (int) roundf(robot.currentJoints.a[2]), (int) roundf(robot.currentJoints.a[3]),
                    (int) roundf(robot.currentJoints.a[4]), (int) roundf(robot.currentJoints.a[5]));

        oled.drawBox(40, 35, 128, 24);
        oled.setFont(u8g2_font_6x12_tr);
        oled.setDrawColor(0);
        oled.setCursor(42, 45);
        oled.printf("%4d|%4d|%4d", (int) roundf(robot.currentPose6D.X),
                    (int) roundf(robot.currentPose6D.Y), (int) roundf(robot.currentPose6D.Z));
        oled.setCursor(42, 56);
        oled.printf("%4d|%4d|%4d", (int) roundf(robot.currentPose6D.A),
                    (int) roundf(robot.currentPose6D.B), (int) roundf(robot.currentPose6D.C));
        oled.setDrawColor(1);
        oled.setCursor(0, 45);
        oled.printf("[XYZ]:");
        oled.setCursor(0, 56);
        oled.printf("[ABC]:");

        oled.setFont(u8g2_font_10x20_tr);
        oled.setCursor(0, 78);
        if (robot.IsEnabled())
        {
            for (int i = 1; i <= 6; i++)
                buf[i - 1] = (robot.jointsStateFlag & (1 << i) ? '*' : '_');
            buf[6] = 0;
            oled.printf("[%s] %s", cmdModeNames[robot.commandMode - 1], buf);
        } else
        {
            oled.printf("[%s] %s", cmdModeNames[robot.commandMode - 1], "======");
        }

        oled.sendBuffer();
    }
}


/* Timer Callbacks -------------------------------------------------------*/
void OnTimer7Callback()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Wake & invoke thread IMMEDIATELY.
    vTaskNotifyGiveFromISR(TaskHandle_t(controlLoopFixUpdateHandle), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

osThreadId_t rgbTaskHandle;
void ThreadRGBUpdate(void* argument) {
    for (;;) {
        if (robot.GetRGBEnabled())
        {
            rgb.Run((RGB::Rgb_style_t)robot.GetRGBMode());
            osDelay(30);
        }else
        {
            rgb.Run(RGB::ALLOff);
            osDelay(30);
        }
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM2)
    {
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
        rgb.Interrupt(1);
    }
}

/* Default Entry -------------------------------------------------------*/
void Main(void)
{
    // Init all communication staff, including USB-CDC/VCP/UART/CAN etc.
    InitCommunication();

    // Init Robot.
    robot.Init();

    // Init IMU.
    do
    {
        mpu6050.Init();
        osDelay(100);
    } while (!mpu6050.testConnection());
    mpu6050.InitFilter(200, 100, 50);

    // Init OLED 128x80.
    oled.Init();
    pwm.Start();

    // Init & Run User Threads.
    const osThreadAttr_t controlLoopTask_attributes = {
        .name = "ControlLoopFixUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityRealtime,
    };
    controlLoopFixUpdateHandle = osThreadNew(ThreadControlLoopFixUpdate, nullptr,
                                             &controlLoopTask_attributes);

    const osThreadAttr_t ControlLoopUpdateTask_attributes = {
        .name = "ControlLoopUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,
    };
    ControlLoopUpdateHandle = osThreadNew(ThreadControlLoopUpdate, nullptr,
                                          &ControlLoopUpdateTask_attributes);

    const osThreadAttr_t oledTask_attributes = {
        .name = "OledTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    };
    oledTaskHandle = osThreadNew(ThreadOledUpdate, nullptr, &oledTask_attributes);

    const osThreadAttr_t rgbTask_attributes = {
            .name = "RGBTask",
            .stack_size = 2000,
            .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    };
    rgbTaskHandle = osThreadNew(ThreadRGBUpdate, nullptr, &rgbTask_attributes);

    // Start Timer Callbacks.
    timerCtrlLoop.SetCallback(OnTimer7Callback);
    timerCtrlLoop.Start();

    // System started, light switch-led up.
    Respond(*uart4StreamOutputPtr, "[sys] Heap remain: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
    pwm.SetDuty(PWM::CH_A1, 0.5);
}

