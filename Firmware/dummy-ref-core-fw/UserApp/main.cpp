#include "common_inc.h"
#include "configurations/robot_config_generated.hpp"
#include "protocols/binary_control_bridge.hpp"
#include "protocols/external_target_executor.hpp"
#include "protocols/feedback_runtime.hpp"
#include "protocols/feedback_safety_supervisor.hpp"
#include "protocols/feedback_poll_scheduler.hpp"
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
Timer timerFeedbackPoll(&htim10, dummy::generated_config::kFeedbackPollHz);
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
    config.gripper_max_velocity_per_s =
        dummy::generated_config::kGripperVelocityLimitPerS;
    config.gripper_max_acceleration_per_s2 =
        dummy::generated_config::kGripperAccelerationLimitPerS2;
    config.loop_rate_hz = dummy::generated_config::kFirmwareLoopHz;
    return config;
}

dummy::protocol::ExternalTargetExecutor external_target_executor(MakeExternalExecutorConfig());

dummy::protocol::FeedbackSafetyConfig MakeFeedbackSafetyConfig()
{
    dummy::protocol::FeedbackSafetyConfig config{};
    std::copy(dummy::generated_config::kJointFollowingErrorLimitRad.begin(),
              dummy::generated_config::kJointFollowingErrorLimitRad.end(),
              config.following_error_limit.begin());
    config.following_error_limit[6] =
        dummy::generated_config::kGripperFollowingErrorLimit;
    config.following_error_hold_ms = dummy::generated_config::kFollowingErrorHoldMs;
    config.feedback_hold_ms = dummy::generated_config::kFeedbackHoldMs;
    config.feedback_fault_ms = dummy::generated_config::kFeedbackFaultMs;
    config.temperature_max_age_ms = dummy::generated_config::kTemperatureMaxAgeMs;
    config.temperature_fault_c = dummy::generated_config::kTemperatureFaultC;
    config.temperature_fault_ms = dummy::generated_config::kTemperatureFaultMs;
    return config;
}

dummy::protocol::FeedbackSafetySupervisor feedback_safety_supervisor(
    MakeFeedbackSafetyConfig());

constexpr uint32_t kTemperatureSlotInterval =
    dummy::generated_config::kFeedbackPollHz /
    static_cast<uint32_t>(dummy::protocol::kActuatorNodeCount);
static_assert(kTemperatureSlotInterval > 0U,
              "feedback scheduler requires temperature slots");
dummy::protocol::FeedbackPollScheduler feedback_poll_scheduler(
    kTemperatureSlotInterval);

constexpr uint32_t kActuatorCommandHz = 100U;
static_assert(dummy::generated_config::kFirmwareLoopHz % kActuatorCommandHz == 0U,
              "actuator command rate must divide the control loop rate");
dummy::protocol::ActuatorCommandScheduler actuator_command_scheduler(
    dummy::generated_config::kFirmwareLoopHz / kActuatorCommandHz);
bool actuator_hold_latched = false;
bool actuator_fault_latched = false;

constexpr uint32_t kFeedbackPollStackBytes = 768U;
static_assert(kFeedbackPollStackBytes % sizeof(StackType_t) == 0U,
              "feedback task stack must be word aligned");
StaticTask_t feedback_poll_task_control_block{};
StackType_t feedback_poll_task_stack[kFeedbackPollStackBytes / sizeof(StackType_t)]{};
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

        const uint64_t now_us = dummy::protocol::BinaryControlMonotonicMicros();
        const auto binary_snapshot = dummy::protocol::ReadBinaryControlSnapshot(now_us);
        const bool binary_motion_mode =
            binary_snapshot.mode == dummy::protocol::ControlMode::Teleop ||
            binary_snapshot.mode == dummy::protocol::ControlMode::Policy;
        const bool binary_context_active = binary_snapshot.lease_active ||
            external_target_executor.active() ||
            binary_snapshot.mode == dummy::protocol::ControlMode::Fault;

        dummy::protocol::ExecutorStep step{};
        if (binary_context_active)
        {
            step = external_target_executor.Step(
                binary_snapshot.target,
                binary_snapshot.lease_active && binary_motion_mode,
                measured_position);
        }

        dummy::protocol::FeedbackSafetyInput safety_input{};
        safety_input.now_us = now_us;
        safety_input.control_active = binary_snapshot.lease_active;
        safety_input.following_active = step.command_valid;
        safety_input.commanded_position = step.command_valid
            ? step.position : external_target_executor.commanded_position();
        safety_input.measured_position = measured_position;
        safety_input.feedback = dummy::protocol::ReadCanFeedbackStatus(
            static_cast<uint32_t>(now_us));
        const auto safety = feedback_safety_supervisor.Update(safety_input);
        dummy::protocol::ApplyBinarySafetyOutcome(safety);
        const bool safety_stop = safety.hold_reason_bits != 0 || safety.fault_bits != 0;

        if (binary_context_active)
        {
            const bool fault_requested =
                binary_snapshot.mode == dummy::protocol::ControlMode::Fault ||
                safety.fault_bits != 0;
            const bool hold_requested = step.entered_hold || safety_stop;

            if (fault_requested)
            {
                actuator_command_scheduler.Reset();
                actuator_hold_latched = false;
                if (!actuator_fault_latched)
                    robot.commandHandler.EmergencyStop();
                actuator_fault_latched = true;
            }
            else
            {
                actuator_fault_latched = false;
                if (hold_requested)
                {
                    actuator_command_scheduler.Reset();
                    if (!actuator_hold_latched)
                        robot.HoldCurrentPosition();
                    actuator_hold_latched = true;
                }
                else if (step.command_valid)
                {
                    actuator_hold_latched = false;
                    if (actuator_command_scheduler.ShouldTransmit(true))
                    {
                        if (!robot.IsEnabled())
                            robot.SetEnable(true);
                        robot.ApplyExternalUrdfTargetRad(step.position);
                        dummy::protocol::MarkBinaryTargetApplied(step.sequence);
                    }
                }
                else
                {
                    actuator_command_scheduler.Reset();
                }
            }
            robot.UpdateJointPose6D();
        }
        else
        {
            actuator_command_scheduler.Reset();
            actuator_hold_latched = false;
            actuator_fault_latched = false;
            if (robot.IsEnabled())
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
                robot.UpdateJointPose6D();
            }
        }
    }
}


osThreadId_t feedbackPollTaskHandle;
void ThreadFeedbackPoll(void* argument)
{
    (void) argument;
    for (;;)
    {
        // Clear any accumulated notifications so a delayed task never emits a
        // catch-up burst onto CAN. A missed slot remains visible as feedback age.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const auto request = feedback_poll_scheduler.Next();
        if (request.kind == dummy::protocol::FeedbackPollKind::Position)
            robot.RequestPositionFeedback(request.node_id);
        else
            robot.RequestTemperatureFeedback(request.node_id);
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

void OnTimer10FeedbackPollCallback()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(TaskHandle_t(feedbackPollTaskHandle), &xHigherPriorityTaskWoken);
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

    const osThreadAttr_t feedbackPollTask_attributes = {
        .name = "FeedbackPollTask",
        .cb_mem = &feedback_poll_task_control_block,
        .cb_size = sizeof(feedback_poll_task_control_block),
        .stack_mem = feedback_poll_task_stack,
        .stack_size = kFeedbackPollStackBytes,
        .priority = (osPriority_t) osPriorityHigh,
    };
    feedbackPollTaskHandle = osThreadNew(ThreadFeedbackPoll, nullptr,
                                         &feedbackPollTask_attributes);
    if (feedbackPollTaskHandle == nullptr)
        Error_Handler();

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
    timerFeedbackPoll.SetCallback(OnTimer10FeedbackPollCallback);
    timerCtrlLoop.Start();
    timerFeedbackPoll.Start();

    // System started, light switch-led up.
    Respond(*uart4StreamOutputPtr, "[sys] Heap remain: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
    pwm.SetDuty(PWM::CH_A1, 0.5);
}
