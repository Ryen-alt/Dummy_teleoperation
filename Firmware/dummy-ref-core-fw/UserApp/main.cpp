#include "common_inc.h"
#include "configurations/robot_config_generated.hpp"
#include "protocols/binary_control_bridge.hpp"
#include "protocols/external_target_executor.hpp"
#include "protocols/feedback_runtime.hpp"
#include "protocols/binary_state_bridge.hpp"
#include "protocols/feedback_safety_supervisor.hpp"
#include "protocols/feedback_poll_scheduler.hpp"
#include "protocols/joint_space_mapping.hpp"
#include "protocols/monotonic_micros.hpp"

#include <algorithm>
#include <array>
#include <cmath>


// On-board Screen, can choose from hi2c2 or hi2c0(soft i2c)
SSD1306 oled(&hi2c0);
// On-board Sensor, used hi2c1
MPU6050 mpu6050(&hi2c1);
// 5 User-Timers, can choose from htim7/htim10/htim11/htim13/htim14
Timer timerCtrlLoop(&htim7, 200);
Timer timerCanDispatch(
    &htim10, dummy::generated_config::kCanSchedulerWatchdogHz);
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

static_assert(dummy::generated_config::kCanSchedulerWatchdogHz == 1000U,
              "v2.2 CAN watchdog frequency is reviewed at 1000 Hz");

dummy::protocol::CanDispatchConfig MakeCanDispatchConfig()
{
    dummy::protocol::CanDispatchConfig config{};
    config.scheduler_watchdog_hz =
        dummy::generated_config::kCanSchedulerWatchdogHz;
    config.target_hz_per_node = dummy::generated_config::kCanTargetHzPerNode;
    config.position_hz_per_node = dummy::generated_config::kCanPositionHzPerNode;
    config.temperature_hz_per_node =
        dummy::generated_config::kCanTemperatureHzPerNode;
    return config;
}

dummy::protocol::CanDispatchScheduler can_dispatch_scheduler(
    MakeCanDispatchConfig());

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

void PublishActuatorMode(ScheduledActuatorMode mode,
                         const std::array<float, 7>& hold_position)
{
    taskENTER_CRITICAL();
    scheduled_actuator_request.position = hold_position;
    scheduled_actuator_request.sequence = 0U;
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
constexpr uint32_t kCanDispatchStackBytes = 2000U;
static_assert(kCanDispatchStackBytes % sizeof(StackType_t) == 0U,
              "CAN dispatcher stack must be word aligned");
StaticTask_t can_dispatch_task_control_block{};
StackType_t can_dispatch_task_stack[kCanDispatchStackBytes / sizeof(StackType_t)]{};
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
        dummy::protocol::PublishCanFeedbackReady(
            safety.arm_position_valid && safety.gripper_position_valid);
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
                PublishActuatorMode(ScheduledActuatorMode::Fault, measured_position);
            }
            else if (hold_requested || !step.command_valid)
            {
                // The CAN task owns all binary-mode actuator writes. Keeping
                // HOLD latched in this mailbox prevents a delayed CAN task from
                // missing a one-control-tick transition during lease release.
                PublishActuatorMode(ScheduledActuatorMode::Hold, measured_position);
            }
            else
            {
                // Latest-value mailbox: the 200 Hz executor may overwrite an
                // intermediate point before a node's 50 Hz slot. No stale
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


osThreadId_t canDispatchTaskHandle;
void NotifyCanDispatcherFromIsr()
{
    if (canDispatchTaskHandle == nullptr)
        return;
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(
        TaskHandle_t(canDispatchTaskHandle), &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void ThreadCanDispatch(void* argument)
{
    (void) argument;
    dummy::protocol::ActuatorApplicationTracker application_tracker;
    ScheduledActuatorRequest target_fanout{};
    bool target_fanout_active = false;
    uint32_t fanout_generation = 0U;
    dummy::protocol::ActuatorApplicationTracker completion_tracker;
    uint32_t completion_generation = 0U;
    uint32_t completion_sequence = 0U;
    uint32_t completion_first_enqueue_us = 0U;
    uint32_t observed_completion_overflow_count = 0U;
    std::array<uint32_t, dummy::protocol::kActuatorNodeCount>
        target_tx_complete_count{};
    uint32_t can_diagnostics_window_start_us = micros();
    uint32_t max_fanout_us = 0U;
    uint32_t safety_preemption_count = 0U;
    uint32_t max_safety_wait_us = 0U;
    for (;;)
    {
        // TX-complete, RX-response and the 1 kHz watchdog all wake this task.
        // One iteration admits at most one frame; completion interrupts drive
        // a seven-node fan-out without turning the timer into a throughput cap.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        CAN_context* can_context = get_can_ctx(&hcan1);
        if (can_context != nullptr &&
            can_context->tx_completion_overflow_count !=
                observed_completion_overflow_count)
        {
            observed_completion_overflow_count =
                can_context->tx_completion_overflow_count;
            if (target_fanout_active && target_fanout.sequence != 0U)
                dummy::protocol::RecordBinaryTargetFailed(
                    target_fanout.sequence,
                    dummy::protocol::BinaryControlMonotonicMicros());
            if (completion_sequence != 0U)
                dummy::protocol::RecordBinaryTargetFailed(
                    completion_sequence,
                    dummy::protocol::BinaryControlMonotonicMicros());
            target_fanout = {};
            target_fanout_active = false;
            application_tracker.Reset();
            completion_tracker.Reset();
            completion_sequence = 0U;
            dummy::protocol::RequestBinaryRuntimeHold();
        }
        CanTxCompletion completion{};
        while (CanTakeTxCompletion(can_context, completion))
        {
            const uint64_t completed_us =
                dummy::protocol::BinaryControlMonotonicMicros();
            if (completion.metadata.channel == CanTxChannel::Safety &&
                completion.status == CanTxCompletionStatus::Complete)
            {
                max_safety_wait_us = std::max(
                    max_safety_wait_us,
                    static_cast<uint32_t>(completed_us -
                        completion.metadata.enqueued_time_us));
            }
            if (completion.metadata.channel != CanTxChannel::Target ||
                completion.metadata.action_sequence == 0U)
                continue;
            if (completion.status != CanTxCompletionStatus::Complete)
            {
                dummy::protocol::RecordBinaryTargetFailed(
                    completion.metadata.action_sequence, completed_us);
                dummy::protocol::RequestBinaryRuntimeHold();
                completion_tracker.Reset();
                continue;
            }
            if (completion.metadata.node_id >= 1U &&
                completion.metadata.node_id <=
                    dummy::protocol::kActuatorNodeCount)
                ++target_tx_complete_count[
                    completion.metadata.node_id - 1U];
            if (completion_generation !=
                completion.metadata.fanout_generation)
            {
                completion_generation =
                    completion.metadata.fanout_generation;
                completion_sequence = completion.metadata.action_sequence;
                completion_first_enqueue_us =
                    completion.metadata.enqueued_time_us;
                completion_tracker.Reset();
            }
            const bool tx_complete_exact =
                completion_tracker.RecordTransmission(
                    completion.metadata.action_sequence,
                    completion.metadata.node_id, true);
            if (tx_complete_exact)
            {
                const uint32_t elapsed_us = static_cast<uint32_t>(
                    completed_us - completion_first_enqueue_us);
                max_fanout_us = std::max(max_fanout_us, elapsed_us);
                if (elapsed_us > 10000U)
                {
                    dummy::protocol::RecordBinaryTargetFailed(
                        completion.metadata.action_sequence, completed_us);
                    dummy::protocol::RequestBinaryRuntimeHold();
                }
                else
                {
                    dummy::protocol::RecordBinaryTargetCanTxCompleteExact(
                        completion.metadata.action_sequence, completed_us);
                }
                completion_tracker.Reset();
                completion_sequence = 0U;
            }
        }
        const ScheduledActuatorRequest scheduled = ReadScheduledActuatorRequest();
        dummy::protocol::CanDispatchMode dispatch_mode =
            dummy::protocol::CanDispatchMode::Bootstrap;
        if (scheduled.mode == ScheduledActuatorMode::Stream)
            dispatch_mode = dummy::protocol::CanDispatchMode::Stream;
        else if (scheduled.mode == ScheduledActuatorMode::Hold)
            dispatch_mode = dummy::protocol::CanDispatchMode::Hold;
        else if (scheduled.mode == ScheduledActuatorMode::Fault)
            dispatch_mode = dummy::protocol::CanDispatchMode::Fault;

        if (dispatch_mode != can_dispatch_scheduler.mode())
        {
            if (target_fanout_active && target_fanout.sequence != 0U)
            {
                dummy::protocol::RecordBinaryTargetPreemptedBySafety(
                    target_fanout.sequence,
                    dummy::protocol::BinaryControlMonotonicMicros());
                ++safety_preemption_count;
            }
            target_fanout = {};
            target_fanout_active = false;
            application_tracker.Reset();
            can_dispatch_scheduler.SetMode(dispatch_mode);
        }

        const uint32_t now_us = micros();
        const auto responses = dummy::protocol::ConsumeFeedbackResponseEvents();
        dummy::protocol::LatchCoherentRobotMeasurement();
        const auto coherent = dummy::protocol::ReadCoherentFeedbackStatus();
        if (coherent.valid)
        {
            uint64_t earliest_sample_us = 0U;
            const uint64_t coherent_now_us =
                dummy::protocol::BinaryControlMonotonicMicros();
            for (const uint32_t sample_low_us : coherent.position_sample_us)
            {
                const uint64_t sample_us =
                    dummy::protocol::ExtendRecentMicros32(
                        coherent_now_us, sample_low_us);
                earliest_sample_us = earliest_sample_us == 0U
                    ? sample_us : std::min(earliest_sample_us, sample_us);
            }
            dummy::protocol::RecordBinaryCoherentSweep(
                coherent.sweep_id, coherent_now_us, earliest_sample_us);
        }
        const auto step = can_dispatch_scheduler.Next(now_us, responses);
        if (step.timed_out_action ==
            dummy::protocol::CanDispatchAction::PositionRequest)
            dummy::protocol::RecordPositionFeedbackTimeout(
                step.timed_out_node_id);
        else if (step.timed_out_action ==
                 dummy::protocol::CanDispatchAction::TemperatureRequest)
            dummy::protocol::RecordTemperatureFeedbackTimeout(
                step.timed_out_node_id);

        bool queued = false;
        ScheduledActuatorRequest latest = scheduled;
        CanTxMetadata tx_metadata{};
        tx_metadata.session_epoch =
            dummy::protocol::ReadBinaryControlSnapshot(
                dummy::protocol::BinaryControlMonotonicMicros()).session_epoch;
        tx_metadata.node_id = step.node_id;
        tx_metadata.enqueued_time_us = now_us;
        if (step.action == dummy::protocol::CanDispatchAction::ActuatorTarget)
            tx_metadata.channel = step.transition
                ? CanTxChannel::Safety : CanTxChannel::Target;
        else if (step.action == dummy::protocol::CanDispatchAction::PositionRequest)
            tx_metadata.channel = CanTxChannel::Position;
        else if (step.action == dummy::protocol::CanDispatchAction::TemperatureRequest)
            tx_metadata.channel = CanTxChannel::Temperature;
        else if (step.action == dummy::protocol::CanDispatchAction::EnableBroadcast ||
                 step.action == dummy::protocol::CanDispatchAction::DisableBroadcast)
            tx_metadata.channel =
                step.action == dummy::protocol::CanDispatchAction::DisableBroadcast &&
                dispatch_mode == dummy::protocol::CanDispatchMode::Fault
                ? CanTxChannel::Emergency : CanTxChannel::Safety;
        switch (step.action)
        {
            case dummy::protocol::CanDispatchAction::ActuatorTarget:
                if (dispatch_mode == dummy::protocol::CanDispatchMode::Stream)
                {
                    if (!target_fanout_active)
                    {
                        const ScheduledActuatorRequest candidate =
                            ReadScheduledActuatorRequest();
                        if (candidate.mode == ScheduledActuatorMode::Stream &&
                            candidate.sequence != 0U &&
                            dummy::protocol::TryStartBinaryTargetDispatch(
                                candidate.sequence))
                        {
                            target_fanout = candidate;
                            target_fanout_active = true;
                            ++fanout_generation;
                            if (fanout_generation == 0U)
                                ++fanout_generation;
                            application_tracker.Reset();
                        }
                    }
                    if (target_fanout_active)
                    {
                        latest = target_fanout;
                        tx_metadata.action_sequence = latest.sequence;
                        tx_metadata.fanout_generation = fanout_generation;
                        queued = robot.ApplyExternalUrdfTargetNodeRad(
                            step.node_id, latest.position, &tx_metadata);
                    }
                }
                else
                {
                    latest = ReadScheduledActuatorRequest();
                    if (latest.mode == scheduled.mode)
                        queued = robot.ApplyExternalUrdfTargetNodeRad(
                            step.node_id, latest.position, &tx_metadata);
                }
                break;
            case dummy::protocol::CanDispatchAction::PositionRequest:
                queued = robot.TryRequestPositionFeedback(
                    step.node_id, &tx_metadata);
                break;
            case dummy::protocol::CanDispatchAction::TemperatureRequest:
                queued = robot.TryRequestTemperatureFeedback(
                    step.node_id, &tx_metadata);
                break;
            case dummy::protocol::CanDispatchAction::EnableBroadcast:
                latest = ReadScheduledActuatorRequest();
                if (latest.mode == ScheduledActuatorMode::Stream)
                    queued = robot.TrySetExternalEnable(true, &tx_metadata);
                break;
            case dummy::protocol::CanDispatchAction::DisableBroadcast:
                queued = robot.TrySetExternalEnable(false, &tx_metadata);
                break;
            case dummy::protocol::CanDispatchAction::None:
                break;
        }

        if (step.action != dummy::protocol::CanDispatchAction::None)
        {
            if (queued)
            {
                can_dispatch_scheduler.OnQueued(step, now_us);
                if (step.action == dummy::protocol::CanDispatchAction::ActuatorTarget &&
                    latest.mode == ScheduledActuatorMode::Stream &&
                    latest.sequence != 0U)
                {
                    const bool completed = application_tracker.RecordTransmission(
                        latest.sequence, step.node_id, true);
                    if (completed)
                    {
                        dummy::protocol::RecordBinaryTargetCanQueuedExact(
                            latest.sequence,
                            dummy::protocol::BinaryControlMonotonicMicros(),
                            coherent.sweep_id);
                        target_fanout = {};
                        target_fanout_active = false;
                        application_tracker.Reset();
                    }
                }
            }
            else
            {
                can_dispatch_scheduler.OnDeferred();
            }
        }

        const auto diagnostics = can_dispatch_scheduler.diagnostics();
        uint8_t runtime_status = dummy::protocol::kCanRuntimeDispatcherAlive;
        if (can_context != nullptr && can_context->tx_queued_count != 0U)
            runtime_status |= dummy::protocol::kCanRuntimeTxQueued;
        bool position_requested = false;
        bool position_responded = false;
        for (size_t index = 0; index < dummy::protocol::kActuatorNodeCount; ++index)
        {
            position_requested = position_requested ||
                diagnostics.position_requested[index] != 0U;
            position_responded = position_responded ||
                diagnostics.position_responded[index] != 0U;
        }
        if (position_requested)
            runtime_status |= dummy::protocol::kCanRuntimePositionRequested;
        if (position_responded)
            runtime_status |= dummy::protocol::kCanRuntimePositionResponded;
        if (diagnostics.deferred_send_count != 0U ||
            (can_context != nullptr && can_context->tx_busy_count != 0U))
            runtime_status |= dummy::protocol::kCanRuntimeTxDeferred;
        if (diagnostics.query_pending)
            runtime_status |= dummy::protocol::kCanRuntimeQueryPending;
        if (dummy::protocol::ReadCanFeedbackReady())
            runtime_status |= dummy::protocol::kCanRuntimeFeedbackReady;
        if (can_context != nullptr && can_context->tx_recovery_count != 0U)
            runtime_status |= dummy::protocol::kCanRuntimeTxRecovered;
        dummy::protocol::PublishCanRuntimeStatus(runtime_status);

        dummy::protocol::CanDiagnosticsPayload can_diagnostics{};
        can_diagnostics.window_start_us = can_diagnostics_window_start_us;
        can_diagnostics.window_duration_us =
            static_cast<uint32_t>(now_us - can_diagnostics_window_start_us);
        for (size_t index = 0;
             index < dummy::protocol::kActuatorNodeCount; ++index)
        {
            can_diagnostics.target_tx_complete[index] =
                target_tx_complete_count[index];
            can_diagnostics.position_response[index] =
                diagnostics.position_responded[index];
            can_diagnostics.temperature_response[index] =
                diagnostics.temperature_responded[index];
            can_diagnostics.position_timeout_count +=
                diagnostics.position_timed_out[index];
            can_diagnostics.temperature_timeout_count +=
                diagnostics.temperature_timed_out[index];
        }
        if (can_context != nullptr)
        {
            can_diagnostics.tx_abort_count =
                can_context->TxMailboxAbortCallbackCnt;
            can_diagnostics.tx_error_count =
                can_context->tx_enqueue_error_count +
                can_context->tx_completion_overflow_count;
            can_diagnostics.tx_recovery_count =
                can_context->tx_recovery_count;
        }
        can_diagnostics.safety_preemption_count = safety_preemption_count;
        can_diagnostics.max_safety_wait_us = max_safety_wait_us;
        can_diagnostics.max_fanout_us = max_fanout_us;
        dummy::protocol::PublishCanDiagnostics(can_diagnostics);
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

void OnTimer10CanDispatchCallback()
{
    NotifyCanDispatcherFromIsr();
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

    const osThreadAttr_t canDispatchTask_attributes = {
        .name = "CanDispatchTask",
        .cb_mem = &can_dispatch_task_control_block,
        .cb_size = sizeof(can_dispatch_task_control_block),
        .stack_mem = can_dispatch_task_stack,
        .stack_size = kCanDispatchStackBytes,
        .priority = (osPriority_t) osPriorityHigh,
    };
    canDispatchTaskHandle = osThreadNew(ThreadCanDispatch, nullptr,
                                        &canDispatchTask_attributes);
    if (canDispatchTaskHandle == nullptr)
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
    timerCanDispatch.SetCallback(OnTimer10CanDispatchCallback);
    timerCtrlLoop.Start();
    timerCanDispatch.Start();

    // System started, light switch-led up.
    Respond(*uart4StreamOutputPtr, "[sys] Heap remain: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
    pwm.SetDuty(PWM::CH_A1, 0.5);
}
