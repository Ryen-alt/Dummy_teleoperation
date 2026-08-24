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

static_assert(dummy::generated_config::kFeedbackPollHz % 2U == 0U,
              "CAN slot rate must split evenly between targets and feedback");
constexpr uint32_t kFeedbackSlotHz =
    dummy::generated_config::kFeedbackPollHz / 2U;
constexpr uint32_t kTemperatureFeedbackSlotInterval =
    kFeedbackSlotHz /
    static_cast<uint32_t>(dummy::protocol::kActuatorNodeCount);
static_assert(kTemperatureFeedbackSlotInterval > 0U,
              "feedback scheduler requires temperature slots");
dummy::protocol::CanSlotScheduler can_slot_scheduler(
    kTemperatureFeedbackSlotInterval);

enum class ScheduledActuatorMode : uint8_t
{
    Idle,
    Stream,
    Hold,
    Fault,
};

struct ScheduledActuatorRequest
{
    ScheduledActuatorMode mode = ScheduledActuatorMode::Idle;
    std::array<float, 7> position{};
    uint32_t sequence = 0;
};

ScheduledActuatorRequest scheduled_actuator_request{};

void PublishStreamingActuatorTarget(const std::array<float, 7>& position,
                                    uint32_t sequence)
{
    taskENTER_CRITICAL();
    scheduled_actuator_request.position = position;
    scheduled_actuator_request.sequence = sequence;
    scheduled_actuator_request.mode = ScheduledActuatorMode::Stream;
    taskEXIT_CRITICAL();
}

void PublishActuatorMode(ScheduledActuatorMode mode)
{
    taskENTER_CRITICAL();
    scheduled_actuator_request.mode = mode;
    taskEXIT_CRITICAL();
}

ScheduledActuatorRequest ReadScheduledActuatorRequest()
{
    taskENTER_CRITICAL();
    const ScheduledActuatorRequest request = scheduled_actuator_request;
    taskEXIT_CRITICAL();
    return request;
}

// This task now owns enable/HOLD/FAULT and per-node target writes in addition
// to feedback polling, so give it the same reviewed stack budget as control.
constexpr uint32_t kFeedbackPollStackBytes = 2000U;
static_assert(kFeedbackPollStackBytes % sizeof(StackType_t) == 0U,
              "feedback task stack must be word aligned");
StaticTask_t feedback_poll_task_control_block{};
StackType_t feedback_poll_task_stack[kFeedbackPollStackBytes / sizeof(StackType_t)]{};
}


/* Thread Definitions -----------------------------------------------------*/
osThreadId_t controlLoopFixUpdateHandle;
void ThreadControlLoopFixUpdate(void* argument)
{
    // Once the binary safety session has taken ownership, never fall back to
    // the legacy 200 Hz whole-arm writer after a lease release. That fallback
    // would silently reintroduce 1,200 command frames/s while the host is idle.
    bool binary_actuator_owner_latched = false;
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
            binary_actuator_owner_latched = true;
            const bool fault_requested =
                binary_snapshot.mode == dummy::protocol::ControlMode::Fault ||
                safety.fault_bits != 0;
            const bool hold_requested = step.entered_hold || safety_stop;

            if (fault_requested)
            {
                PublishActuatorMode(ScheduledActuatorMode::Fault);
            }
            else if (hold_requested || !step.command_valid)
            {
                // The CAN task owns all binary-mode actuator writes. Keeping
                // HOLD latched in this mailbox prevents a delayed CAN task from
                // missing a one-control-tick transition during lease release.
                PublishActuatorMode(ScheduledActuatorMode::Hold);
            }
            else
            {
                // Latest-value mailbox: the 200 Hz executor may overwrite an
                // intermediate point before a node's 100 Hz slot. No stale
                // backlog is ever replayed onto the actuator bus.
                PublishStreamingActuatorTarget(step.position, step.sequence);
            }
            robot.UpdateJointPose6D();
        }
        else
        {
            if (binary_actuator_owner_latched)
            {
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
                robot.UpdateJointPose6D();
            }
        }
    }
}


osThreadId_t feedbackPollTaskHandle;
void ThreadFeedbackPoll(void* argument)
{
    (void) argument;
    ScheduledActuatorMode processed_mode = ScheduledActuatorMode::Idle;
    dummy::protocol::ActuatorApplicationTracker application_tracker;
    for (;;)
    {
        // Clear any accumulated notifications so a delayed task never emits a
        // catch-up burst onto CAN. A missed slot remains visible as feedback age.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const auto request = can_slot_scheduler.Next();

        const ScheduledActuatorRequest scheduled = ReadScheduledActuatorRequest();
        if (scheduled.mode == ScheduledActuatorMode::Stream)
        {
            if (processed_mode != ScheduledActuatorMode::Stream)
            {
                application_tracker.Reset();
                if (!robot.IsEnabled())
                    robot.SetEnable(true);
                processed_mode = ScheduledActuatorMode::Stream;
                // Enabling is a one-time multi-frame transition. Give it this
                // entire slot so no target/query is appended to the burst.
                continue;
            }

            if (request.kind == dummy::protocol::CanSlotKind::ActuatorTarget)
            {
                // Re-read immediately before transmitting. If the realtime
                // safety task published HOLD/FAULT meanwhile, skip this stale
                // target and process the transition in the next (<= 1.43 ms)
                // CAN slot.
                const ScheduledActuatorRequest latest = ReadScheduledActuatorRequest();
                if (latest.mode == ScheduledActuatorMode::Stream)
                {
                    const bool transmitted = robot.ApplyExternalUrdfTargetNodeRad(
                        request.node_id, latest.position);
                    if (application_tracker.RecordTransmission(
                            latest.sequence, request.node_id, transmitted))
                        dummy::protocol::MarkBinaryTargetApplied(latest.sequence);
                }
                else
                {
                    application_tracker.Reset();
                }
            }
            else if (request.kind == dummy::protocol::CanSlotKind::PositionFeedback)
                robot.RequestPositionFeedback(request.node_id);
            else
                robot.RequestTemperatureFeedback(request.node_id);
        }
        else
        {
            application_tracker.Reset();
            if (scheduled.mode != processed_mode)
            {
                if (scheduled.mode == ScheduledActuatorMode::Hold)
                    robot.HoldCurrentPosition();
                else if (scheduled.mode == ScheduledActuatorMode::Fault)
                    robot.commandHandler.EmergencyStop();
                processed_mode = scheduled.mode;
                // HOLD/FAULT can fan out to all nodes. Do not append another
                // CAN frame until the following timer slot.
                continue;
            }

            // With control inactive, reuse target slots for position queries.
            // This preserves high-rate idle diagnostics without ever sending
            // more than one normal-operation frame in a timer slot.
            if (request.kind == dummy::protocol::CanSlotKind::TemperatureFeedback)
                robot.RequestTemperatureFeedback(request.node_id);
            else
                robot.RequestPositionFeedback(request.node_id);
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
