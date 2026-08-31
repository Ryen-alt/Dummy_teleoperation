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
constexpr uint32_t kCanTxAbortTimeoutUs =
    dummy::generated_config::kCanTxAbortTimeoutUs;
constexpr uint32_t kCanTargetFanoutTimeoutUs =
    dummy::generated_config::kCanTargetFanoutTimeoutUs;

struct CanContextWindowBaseline
{
    uint32_t busoff_count = 0U;
    uint32_t rx_overflow_count = 0U;
    uint32_t tx_abort_count = 0U;
    uint32_t tx_error_count = 0U;
    uint32_t tx_recovery_count = 0U;
    uint32_t completion_overflow_count = 0U;
    uint32_t rx_frame_count = 0U;
    uint32_t tx_busy_count = 0U;
};

struct CanDiagnosticsWindow
{
    bool active = false;
    bool epoch_stable = false;
    bool counters_monotonic = false;
    uint32_t session_epoch = 0U;
    uint32_t reset_count = 0U;
    uint64_t start_us = 0U;
    uint8_t motor_marker_mask = 0U;
    dummy::protocol::CanDispatchDiagnostics scheduler{};
    dummy::protocol::MotorTransportDiagnostics motor{};
    std::array<uint32_t, dummy::protocol::kActuatorNodeCount>
        target_tx_complete{};
    std::array<CanContextWindowBaseline, 2U> can{};
    uint32_t safety_preemption_count = 0U;
    uint32_t transition_failure_count = 0U;
};

uint32_t WindowCounterDelta(uint32_t current, uint32_t baseline,
                            bool& monotonic)
{
    if (current < baseline)
    {
        monotonic = false;
        return 0U;
    }
    return current - baseline;
}

uint32_t timing_profile_window_token = 0U;

uint8_t WindowMotorCounterDelta(uint8_t current, uint8_t baseline,
                                bool& monotonic)
{
    if (current < baseline)
    {
        monotonic = false;
        return 0U;
    }
    return static_cast<uint8_t>(current - baseline);
}

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
    // A9 internal profiler traffic is diagnostic-only and intentionally
    // excluded from the robot configuration hash. One request per node per
    // second refreshes all four motor pages every four seconds.
    config.timing_profile_hz_per_node = 1U;
    config.response_timeout_us =
        dummy::generated_config::kCanResponseTimeoutUs;
    config.node_quiet_us = dummy::generated_config::kCanNodeQuietUs;
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
constexpr uint32_t kCanDispatchStackBytes = 4096U;
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
        // A configured binary HELLO claims the actuator path even before a
        // control lease is acquired.  Read-only diagnostics deliberately run
        // in that pre-lease interval; allowing the legacy 200 Hz writer to
        // continue would emit 0x07 commands whose mandatory 0x23 ACKs are
        // indistinguishable from position-query responses and corrupt
        // coherent sweep attribution.  Ownership remains latched so a serial
        // disconnect can never silently restore the legacy writer.
        if (binary_snapshot.hello_valid && !binary_actuator_owner_latched)
        {
            binary_actuator_owner_latched = true;
            PublishActuatorMode(ScheduledActuatorMode::Hold, measured_position);
        }
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
                PublishActuatorMode(ScheduledActuatorMode::Hold, measured_position);
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
    dummy::protocol::TargetCompletionTracker completion_tracker(
        kCanTargetFanoutTimeoutUs);
    ScheduledActuatorRequest completion_target{};
    bool stream_fail_closed = false;
    std::array<uint32_t, 2U> observed_completion_overflow_count{};
    std::array<uint32_t, 2U> observed_rx_overflow_count{};
    std::array<uint32_t, dummy::protocol::kActuatorNodeCount>
        target_tx_complete_count{};
    CanDiagnosticsWindow diagnostics_window{};
    uint32_t safety_preemption_count = 0U;
    uint32_t max_safety_wait_us = 0U;
    uint32_t max_rx_dispatch_latency_us = 0U;
    uint32_t transition_failure_count = 0U;
    for (;;)
    {
        // TX-complete, RX-response and the 1 kHz watchdog all wake this task.
        // One iteration admits at most one frame; completion interrupts drive
        // a seven-node fan-out without turning the timer into a throughput cap.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const std::array<CAN_context*, 2U> can_contexts{
            get_can_ctx(&hcan1), get_can_ctx(&hcan2)};
        CAN_context* can_context = can_contexts[0];
        for (CAN_context* context : can_contexts)
            CanServiceTxDeadline(
                context, micros(), kCanTxAbortTimeoutUs);

        const ScheduledActuatorRequest scheduled =
            ReadScheduledActuatorRequest();
        dummy::protocol::CanDispatchMode dispatch_mode =
            dummy::protocol::CanDispatchMode::Bootstrap;
        if (scheduled.mode == ScheduledActuatorMode::Stream)
            dispatch_mode = dummy::protocol::CanDispatchMode::Stream;
        else if (scheduled.mode == ScheduledActuatorMode::Hold)
            dispatch_mode = dummy::protocol::CanDispatchMode::Hold;
        else if (scheduled.mode == ScheduledActuatorMode::Fault)
            dispatch_mode = dummy::protocol::CanDispatchMode::Fault;
        const uint64_t dispatch_now_us =
            dummy::protocol::BinaryControlMonotonicMicros();
        const auto control_snapshot =
            dummy::protocol::ReadBinaryControlSnapshot(dispatch_now_us);

        // Observe the safety mailbox before crediting or retrying any target
        // completion from the same wake. Once the mode changes, stale target
        // completions can no longer revive the preempted generation.
        if (dispatch_mode != can_dispatch_scheduler.mode())
        {
            uint32_t preempted_sequence = 0U;
            if (completion_tracker.active())
                preempted_sequence =
                    completion_tracker.key().action_sequence;
            else if (target_fanout_active)
                preempted_sequence = target_fanout.sequence;
            if (preempted_sequence != 0U)
            {
                dummy::protocol::RecordBinaryTargetPreemptedBySafety(
                    preempted_sequence,
                    dummy::protocol::BinaryControlMonotonicMicros());
                ++safety_preemption_count;
            }
            target_fanout = {};
            target_fanout_active = false;
            completion_target = {};
            application_tracker.Reset();
            completion_tracker.Cancel();
            stream_fail_closed = false;
            dummy::protocol::CancelPendingFeedbackRequests();
            if (dispatch_mode == dummy::protocol::CanDispatchMode::Stream)
            {
                dummy::protocol::ResetMotorTransportDiagnostics();
                dummy::protocol::SetCanTimingProfileActive(false);
            }
            else if (diagnostics_window.active)
            {
                diagnostics_window.active = false;
                dummy::protocol::SetCanTimingProfileActive(false);
            }
            can_dispatch_scheduler.SetMode(dispatch_mode);
        }

        if (diagnostics_window.active &&
            control_snapshot.session_epoch != diagnostics_window.session_epoch)
        {
            diagnostics_window.epoch_stable = false;
            dummy::protocol::SetCanTimingProfileEpochStable(false);
            stream_fail_closed = true;
            dummy::protocol::RequestBinaryRuntimeHold();
        }

        const bool binary_stream_authorized =
            control_snapshot.lease_active &&
            (control_snapshot.mode == dummy::protocol::ControlMode::Teleop ||
             control_snapshot.mode == dummy::protocol::ControlMode::Policy);
        if (dispatch_mode == dummy::protocol::CanDispatchMode::Stream &&
            (!binary_stream_authorized ||
             (completion_tracker.active() &&
              control_snapshot.session_epoch !=
                  completion_tracker.key().session_epoch)))
        {
            const uint32_t preempted_sequence = completion_tracker.active()
                ? completion_tracker.key().action_sequence
                : target_fanout.sequence;
            if (preempted_sequence != 0U)
            {
                dummy::protocol::RecordBinaryTargetPreemptedBySafety(
                    preempted_sequence, dispatch_now_us);
                ++safety_preemption_count;
            }
            target_fanout = {};
            target_fanout_active = false;
            completion_target = {};
            application_tracker.Reset();
            completion_tracker.Cancel();
            stream_fail_closed = true;
            dummy::protocol::RequestBinaryRuntimeHold();
        }

        bool transport_overflow = false;
        for (size_t index = 0U; index < can_contexts.size(); ++index)
        {
            CAN_context* context = can_contexts[index];
            if (context == nullptr)
                continue;
            if (context->tx_completion_overflow_count !=
                observed_completion_overflow_count[index])
            {
                observed_completion_overflow_count[index] =
                    context->tx_completion_overflow_count;
                transport_overflow = true;
            }
            if (context->rx_overflow_count !=
                observed_rx_overflow_count[index])
            {
                observed_rx_overflow_count[index] =
                    context->rx_overflow_count;
                transport_overflow = true;
            }
        }
        if (transport_overflow)
        {
            const uint32_t failed_sequence = completion_tracker.active()
                ? completion_tracker.key().action_sequence
                : target_fanout.sequence;
            if (failed_sequence != 0U)
                dummy::protocol::RecordBinaryTargetFailed(
                    failed_sequence,
                    dummy::protocol::BinaryControlMonotonicMicros());
            target_fanout = {};
            target_fanout_active = false;
            completion_target = {};
            application_tracker.Reset();
            completion_tracker.Cancel();
            stream_fail_closed = true;
            dummy::protocol::RequestBinaryRuntimeHold();
        }

        CanTxCompletion completion{};
        for (CAN_context* completion_context : can_contexts)
        {
            while (CanTakeTxCompletion(completion_context, completion))
            {
                const uint64_t completed_us =
                    dummy::protocol::BinaryControlMonotonicMicros();
                if ((completion.metadata.channel == CanTxChannel::Safety ||
                     completion.metadata.channel ==
                         CanTxChannel::EnableTransition) &&
                    completion.status == CanTxCompletionStatus::Complete)
                {
                    max_safety_wait_us = std::max(
                        max_safety_wait_us,
                        static_cast<uint32_t>(completed_us -
                            completion.metadata.enqueued_time_us));
                }
                if (completion.metadata.channel == CanTxChannel::Safety &&
                    completion.status != CanTxCompletionStatus::Complete &&
                    dispatch_mode == dummy::protocol::CanDispatchMode::Stream)
                {
                    if (transition_failure_count != UINT32_MAX)
                        ++transition_failure_count;
                    stream_fail_closed = true;
                    dummy::protocol::RequestBinaryRuntimeHold();
                    continue;
                }
                if (completion.metadata.channel ==
                        CanTxChannel::Configuration &&
                    completion.status != CanTxCompletionStatus::Complete)
                {
                    if (transition_failure_count != UINT32_MAX)
                        ++transition_failure_count;
                    stream_fail_closed = true;
                    dummy::protocol::RequestBinaryRuntimeHold();
                    continue;
                }
                if (completion.metadata.channel ==
                    CanTxChannel::EnableTransition)
                {
                    const auto motor_diagnostics =
                        dummy::protocol::ReadMotorTransportDiagnostics();
                    constexpr uint8_t kAllMotorMarkers = static_cast<uint8_t>(
                        (1U << dummy::protocol::kActuatorNodeCount) - 1U);
                    if (completion.status != CanTxCompletionStatus::Complete ||
                        dispatch_mode !=
                            dummy::protocol::CanDispatchMode::Stream ||
                        !binary_stream_authorized || stream_fail_closed ||
                        control_snapshot.session_epoch == 0U ||
                        completion.metadata.session_epoch !=
                            control_snapshot.session_epoch ||
                        motor_diagnostics.valid_mask != kAllMotorMarkers)
                    {
                        if (transition_failure_count != UINT32_MAX)
                            ++transition_failure_count;
                        stream_fail_closed = true;
                        dummy::protocol::RequestBinaryRuntimeHold();
                        continue;
                    }

                    if (diagnostics_window.reset_count != UINT32_MAX)
                        ++diagnostics_window.reset_count;
                    diagnostics_window.active = true;
                    diagnostics_window.epoch_stable = true;
                    diagnostics_window.counters_monotonic = true;
                    diagnostics_window.session_epoch =
                        control_snapshot.session_epoch;
                    diagnostics_window.start_us = completed_us;
                    timing_profile_window_token =
                        control_snapshot.session_epoch ^
                        diagnostics_window.reset_count * 0x9E3779B9U;
                    if (timing_profile_window_token == 0U)
                        timing_profile_window_token = 1U;
                    dummy::protocol::ResetCanTimingProfile(
                        control_snapshot.session_epoch, completed_us,
                        can_dispatch_scheduler.diagnostics());
                    diagnostics_window.motor_marker_mask =
                        motor_diagnostics.valid_mask;
                    diagnostics_window.scheduler =
                        can_dispatch_scheduler.diagnostics();
                    diagnostics_window.motor = motor_diagnostics;
                    diagnostics_window.target_tx_complete =
                        target_tx_complete_count;
                    diagnostics_window.safety_preemption_count =
                        safety_preemption_count;
                    diagnostics_window.transition_failure_count =
                        transition_failure_count;
                    for (size_t index = 0U;
                         index < can_contexts.size(); ++index)
                    {
                        CAN_context* context = can_contexts[index];
                        if (context == nullptr)
                            continue;
                        auto& baseline = diagnostics_window.can[index];
                        baseline.busoff_count = context->busoff_count;
                        baseline.rx_overflow_count =
                            context->rx_overflow_count;
                        baseline.tx_abort_count =
                            context->TxMailboxAbortCallbackCnt;
                        baseline.tx_error_count =
                            context->tx_enqueue_error_count;
                        baseline.tx_recovery_count =
                            context->tx_recovery_count;
                        baseline.completion_overflow_count =
                            context->tx_completion_overflow_count;
                        baseline.rx_frame_count = context->received_msg_cnt;
                        baseline.tx_busy_count = context->tx_busy_count;
                    }
                    taskENTER_CRITICAL();
                    for (CAN_context* context : can_contexts)
                    {
                        if (context != nullptr)
                            context->rx_high_water = 0U;
                    }
                    taskEXIT_CRITICAL();
                    completion_tracker.ResetDiagnostics();
                    max_safety_wait_us = 0U;
                    max_rx_dispatch_latency_us = 0U;
                    continue;
                }
                if (completion.status == CanTxCompletionStatus::Complete &&
                    completion.metadata.channel == CanTxChannel::Position)
                    dummy::protocol::RecordPositionTimingStart(
                        completion.metadata.node_id,
                        static_cast<uint32_t>(completed_us));
                else if (
                    completion.status == CanTxCompletionStatus::Complete &&
                    completion.metadata.channel == CanTxChannel::Temperature)
                    dummy::protocol::RecordTemperatureTimingStart(
                        completion.metadata.node_id,
                        static_cast<uint32_t>(completed_us));
                if (completion.metadata.channel != CanTxChannel::Target ||
                    completion.metadata.action_sequence == 0U)
                    continue;
                const dummy::protocol::TargetFanoutKey completion_key{
                    completion.metadata.session_epoch,
                    completion.metadata.action_sequence,
                    completion.metadata.fanout_generation};
                const auto completion_result =
                    completion_tracker.RecordCompletion(
                        completion_key, completion.metadata.node_id,
                        completion.status == CanTxCompletionStatus::Complete,
                        static_cast<uint32_t>(completed_us));
                if (completion_result !=
                        dummy::protocol::TargetCompletionResult::Ignored &&
                    completion.metadata.node_id >= 1U &&
                    completion.metadata.node_id <=
                        dummy::protocol::kActuatorNodeCount &&
                    completion.status == CanTxCompletionStatus::Complete)
                {
                    ++target_tx_complete_count[
                        completion.metadata.node_id - 1U];
                }
                if (completion_result ==
                    dummy::protocol::TargetCompletionResult::CompleteExact)
                {
                    dummy::protocol::RecordBinaryTargetCanTxCompleteExact(
                        completion.metadata.action_sequence, completed_us,
                        completion_tracker.last_fanout_us());
                    completion_target = {};
                    completion_tracker.Cancel();
                }
                else if (completion_result ==
                         dummy::protocol::TargetCompletionResult::Failed)
                {
                    dummy::protocol::RecordBinaryTargetFailed(
                        completion.metadata.action_sequence, completed_us);
                    target_fanout = {};
                    target_fanout_active = false;
                    completion_target = {};
                    application_tracker.Reset();
                    stream_fail_closed = true;
                    dummy::protocol::RequestBinaryRuntimeHold();
                }
            }
        }

        CanRxFrame rx_frame{};
        for (CAN_context* rx_context : can_contexts)
        {
            while (CanTakeRxFrame(rx_context, rx_frame))
            {
                const uint32_t dispatch_latency_us =
                    dummy::protocol::RecentElapsedMicros32(
                        micros(), rx_frame.received_us);
                max_rx_dispatch_latency_us = std::max(
                    max_rx_dispatch_latency_us, dispatch_latency_us);
                OnCanMessage(
                    rx_context, &rx_frame.header, rx_frame.data,
                    rx_frame.received_us);
            }
        }
        const uint32_t now_us = micros();
        dummy::protocol::PublishFeedbackSnapshot(now_us);
        if (dispatch_mode == dummy::protocol::CanDispatchMode::Stream &&
            completion_tracker.CheckDeadline(now_us) ==
                dummy::protocol::TargetCompletionResult::Failed)
        {
            dummy::protocol::RecordBinaryTargetFailed(
                completion_tracker.key().action_sequence,
                dummy::protocol::BinaryControlMonotonicMicros());
            target_fanout = {};
            target_fanout_active = false;
            completion_target = {};
            application_tracker.Reset();
            stream_fail_closed = true;
            dummy::protocol::RequestBinaryRuntimeHold();
        }

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
        if (step.accepted_timing_profile_node_id != 0U)
            (void) dummy::protocol::AcceptMotorTimingProfile(
                step.accepted_timing_profile_node_id,
                step.accepted_timing_profile_page, now_us);
        if (step.timed_out_final)
        {
            if (step.timed_out_action ==
                dummy::protocol::CanDispatchAction::PositionRequest)
            {
                dummy::protocol::RecordPositionFeedbackTimeout(
                    step.timed_out_node_id);
            }
            else if (step.timed_out_action ==
                         dummy::protocol::CanDispatchAction::TemperatureRequest ||
                     step.timed_out_action ==
                         dummy::protocol::CanDispatchAction::MotorDiagnosticsRequest)
            {
                dummy::protocol::RecordTemperatureFeedbackTimeout(
                    step.timed_out_node_id);
            }
            if (step.timed_out_action ==
                dummy::protocol::CanDispatchAction::MotorDiagnosticsRequest)
            {
                if (transition_failure_count != UINT32_MAX)
                    ++transition_failure_count;
                stream_fail_closed = true;
                dummy::protocol::RequestBinaryRuntimeHold();
            }
        }

        bool queued = false;
        ScheduledActuatorRequest latest = scheduled;
        const auto target_retry = completion_tracker.retry_request();
        const bool dispatching_target_retry = target_retry.valid &&
            dispatch_mode == dummy::protocol::CanDispatchMode::Stream &&
            !stream_fail_closed;
        const bool block_normal_stream_dispatch = stream_fail_closed &&
            dispatch_mode == dummy::protocol::CanDispatchMode::Stream;
        CanTxMetadata tx_metadata{};
        tx_metadata.session_epoch = control_snapshot.session_epoch;
        tx_metadata.node_id = dispatching_target_retry
            ? target_retry.node_id : step.node_id;
        tx_metadata.feedback_sweep_id = step.feedback_sweep_id;
        tx_metadata.enqueued_time_us = now_us;
        if (dispatching_target_retry)
        {
            tx_metadata.channel = CanTxChannel::Target;
            tx_metadata.session_epoch = target_retry.key.session_epoch;
            tx_metadata.action_sequence = target_retry.key.action_sequence;
            tx_metadata.fanout_generation =
                target_retry.key.fanout_generation;
        }
        else if (step.action ==
                 dummy::protocol::CanDispatchAction::ActuatorTarget)
            tx_metadata.channel = step.transition
                ? CanTxChannel::Safety : CanTxChannel::Target;
        else if (step.action == dummy::protocol::CanDispatchAction::PositionRequest)
            tx_metadata.channel = CanTxChannel::Position;
        else if (step.action == dummy::protocol::CanDispatchAction::TemperatureRequest)
            tx_metadata.channel = CanTxChannel::Temperature;
        else if (step.action ==
                 dummy::protocol::CanDispatchAction::MotorTimingRequest)
            tx_metadata.channel = CanTxChannel::TimingProfile;
        else if (step.action ==
                 dummy::protocol::CanDispatchAction::MotorDiagnosticsRequest)
            tx_metadata.channel = CanTxChannel::Diagnostics;
        else if (step.action ==
                 dummy::protocol::CanDispatchAction::ConfigureGripperVelocity)
            tx_metadata.channel = CanTxChannel::Configuration;
        else if (step.action == dummy::protocol::CanDispatchAction::EnableBroadcast ||
                 step.action == dummy::protocol::CanDispatchAction::DisableBroadcast)
            tx_metadata.channel = step.action ==
                    dummy::protocol::CanDispatchAction::EnableBroadcast
                ? CanTxChannel::EnableTransition
                : (dispatch_mode == dummy::protocol::CanDispatchMode::Fault
                    ? CanTxChannel::Emergency : CanTxChannel::Safety);
        if (dispatching_target_retry)
        {
            if (completion_target.mode == ScheduledActuatorMode::Stream &&
                completion_target.sequence ==
                    target_retry.key.action_sequence)
            {
                latest = completion_target;
                queued = robot.ApplyExternalUrdfTargetNodeRad(
                    target_retry.node_id, completion_target.position,
                    &tx_metadata);
            }
        }
        else if (!block_normal_stream_dispatch)
        {
            switch (step.action)
            {
            case dummy::protocol::CanDispatchAction::ActuatorTarget:
                if (dispatch_mode == dummy::protocol::CanDispatchMode::Stream &&
                    !step.transition)
                {
                    if (!target_fanout_active &&
                        !completion_tracker.active())
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
            case dummy::protocol::CanDispatchAction::MotorDiagnosticsRequest:
                queued = robot.TryRequestTemperatureFeedback(
                    step.node_id, &tx_metadata);
                break;
            case dummy::protocol::CanDispatchAction::MotorTimingRequest:
                queued = robot.TryRequestTimingProfile(
                    step.node_id, step.timing_profile_page,
                    timing_profile_window_token, &tx_metadata);
                break;
            case dummy::protocol::CanDispatchAction::ConfigureGripperVelocity:
                if (dummy::protocol::ReadMotorTransportDiagnostics().valid_mask !=
                    static_cast<uint8_t>(
                        (1U << dummy::protocol::kActuatorNodeCount) - 1U))
                {
                    if (transition_failure_count != UINT32_MAX)
                        ++transition_failure_count;
                    stream_fail_closed = true;
                    dummy::protocol::RequestBinaryRuntimeHold();
                }
                else
                {
                    queued = robot.TryConfigureGripperStreaming(
                        dummy::generated_config::kGripperVelocityLimitPerS,
                        &tx_metadata);
                }
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
        }

        if (dispatching_target_retry)
        {
            if (queued)
            {
                if (!completion_tracker.MarkRetryQueued(target_retry))
                {
                    dummy::protocol::RecordBinaryTargetFailed(
                        target_retry.key.action_sequence, dispatch_now_us);
                    target_fanout = {};
                    target_fanout_active = false;
                    completion_target = {};
                    application_tracker.Reset();
                    completion_tracker.Cancel();
                    stream_fail_closed = true;
                    dummy::protocol::RequestBinaryRuntimeHold();
                }
            }
            else
            {
                can_dispatch_scheduler.OnDeferred();
            }
            if (step.action != dummy::protocol::CanDispatchAction::None)
                can_dispatch_scheduler.OnDeferred();
        }
        else if (step.action != dummy::protocol::CanDispatchAction::None)
        {
            if (queued)
            {
                can_dispatch_scheduler.OnQueued(step, now_us);
                if (step.action == dummy::protocol::CanDispatchAction::ActuatorTarget &&
                    !step.transition &&
                    latest.mode == ScheduledActuatorMode::Stream &&
                    latest.sequence != 0U)
                {
                    if (!completion_tracker.active())
                    {
                        const dummy::protocol::TargetFanoutKey key{
                            tx_metadata.session_epoch,
                            latest.sequence,
                            tx_metadata.fanout_generation};
                        if (!completion_tracker.Begin(key, now_us))
                        {
                            dummy::protocol::RecordBinaryTargetFailed(
                                latest.sequence, dispatch_now_us);
                            target_fanout = {};
                            target_fanout_active = false;
                            application_tracker.Reset();
                            stream_fail_closed = true;
                            dummy::protocol::RequestBinaryRuntimeHold();
                        }
                        else
                        {
                            completion_target = latest;
                        }
                    }
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
                if (step.action ==
                        dummy::protocol::CanDispatchAction::ConfigureGripperVelocity ||
                    step.action ==
                        dummy::protocol::CanDispatchAction::EnableBroadcast)
                {
                    if (transition_failure_count != UINT32_MAX)
                        ++transition_failure_count;
                    stream_fail_closed = true;
                    dummy::protocol::RequestBinaryRuntimeHold();
                }
                else
                {
                    can_dispatch_scheduler.OnDeferred();
                }
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
        bool can_degraded = false;
        for (const CAN_context* context : can_contexts)
        {
            can_degraded = can_degraded ||
                (context != nullptr &&
                 (context->tx_recovery_count != 0U ||
                  context->tx_completion_overflow_count != 0U ||
                  context->rx_overflow_count != 0U ||
                  context->busoff_count != 0U));
        }
        if (can_degraded)
            runtime_status |= dummy::protocol::kCanRuntimeDegraded;
        dummy::protocol::PublishCanRuntimeStatus(runtime_status);

        dummy::protocol::CanDiagnosticsPayload can_diagnostics{};
        can_diagnostics.format_version =
            dummy::protocol::kCanDiagnosticsFormatVersion;
        can_diagnostics.payload_size =
            dummy::protocol::kCanDiagnosticsPayloadSize;
        can_diagnostics.session_epoch = diagnostics_window.session_epoch;
        can_diagnostics.window_reset_count = diagnostics_window.reset_count;
        can_diagnostics.window_start_us = diagnostics_window.start_us;
        if (diagnostics_window.start_us != 0U)
        {
            can_diagnostics.window_duration_us =
                dummy::protocol::BinaryControlMonotonicMicros() -
                diagnostics_window.start_us;
        }

        const auto motor_diagnostics =
            dummy::protocol::ReadMotorTransportDiagnostics();
        can_diagnostics.motor_marker_mask = motor_diagnostics.valid_mask;
        bool counters_monotonic = diagnostics_window.counters_monotonic;
        if (diagnostics_window.start_us != 0U)
        {
            for (size_t index = 0;
                 index < dummy::protocol::kActuatorNodeCount; ++index)
            {
                can_diagnostics.target_tx_complete[index] =
                    WindowCounterDelta(
                        target_tx_complete_count[index],
                        diagnostics_window.target_tx_complete[index],
                        counters_monotonic);
                can_diagnostics.position_request[index] =
                    WindowCounterDelta(
                        diagnostics.position_requested[index],
                        diagnostics_window.scheduler.position_requested[index],
                        counters_monotonic);
                can_diagnostics.position_response[index] =
                    WindowCounterDelta(
                        diagnostics.position_responded[index],
                        diagnostics_window.scheduler.position_responded[index],
                        counters_monotonic);
                can_diagnostics.position_timeout[index] =
                    WindowCounterDelta(
                        diagnostics.position_timed_out[index],
                        diagnostics_window.scheduler.position_timed_out[index],
                        counters_monotonic);
                can_diagnostics.temperature_request[index] =
                    WindowCounterDelta(
                        diagnostics.temperature_requested[index],
                        diagnostics_window.scheduler.temperature_requested[index],
                        counters_monotonic);
                can_diagnostics.temperature_response[index] =
                    WindowCounterDelta(
                        diagnostics.temperature_responded[index],
                        diagnostics_window.scheduler.temperature_responded[index],
                        counters_monotonic);
                can_diagnostics.temperature_timeout[index] =
                    WindowCounterDelta(
                        diagnostics.temperature_timed_out[index],
                        diagnostics_window.scheduler.temperature_timed_out[index],
                        counters_monotonic);
                can_diagnostics.motor_tx_drop[index] =
                    WindowMotorCounterDelta(
                        motor_diagnostics.tx_drop[index],
                        diagnostics_window.motor.tx_drop[index],
                        counters_monotonic);
                can_diagnostics.motor_rx_error[index] =
                    WindowMotorCounterDelta(
                        motor_diagnostics.rx_error[index],
                        diagnostics_window.motor.rx_error[index],
                        counters_monotonic);
                can_diagnostics.motor_busoff[index] =
                    WindowMotorCounterDelta(
                        motor_diagnostics.busoff[index],
                        diagnostics_window.motor.busoff[index],
                        counters_monotonic);
            }
            for (size_t index = 0U; index < can_contexts.size(); ++index)
            {
                CAN_context* context = can_contexts[index];
                if (context == nullptr)
                    continue;
                const auto& baseline = diagnostics_window.can[index];
                can_diagnostics.main_can_busoff[index] =
                    WindowCounterDelta(
                        context->busoff_count, baseline.busoff_count,
                        counters_monotonic);
                can_diagnostics.main_can_rx_overflow[index] =
                    WindowCounterDelta(
                        context->rx_overflow_count,
                        baseline.rx_overflow_count, counters_monotonic);
                can_diagnostics.main_can_rx_high_water[index] =
                    context->rx_high_water;
                can_diagnostics.main_can_tx_abort[index] =
                    WindowCounterDelta(
                        context->TxMailboxAbortCallbackCnt,
                        baseline.tx_abort_count, counters_monotonic);
                can_diagnostics.main_can_tx_error[index] =
                    WindowCounterDelta(
                        context->tx_enqueue_error_count,
                        baseline.tx_error_count, counters_monotonic);
                can_diagnostics.main_can_tx_recovery[index] =
                    WindowCounterDelta(
                        context->tx_recovery_count,
                        baseline.tx_recovery_count, counters_monotonic);
                can_diagnostics.main_can_completion_overflow[index] =
                    WindowCounterDelta(
                        context->tx_completion_overflow_count,
                        baseline.completion_overflow_count,
                        counters_monotonic);
                can_diagnostics.main_can_rx_frame[index] =
                    WindowCounterDelta(
                        context->received_msg_cnt,
                        baseline.rx_frame_count, counters_monotonic);
                can_diagnostics.main_can_tx_busy[index] =
                    WindowCounterDelta(
                        context->tx_busy_count, baseline.tx_busy_count,
                        counters_monotonic);
            }
            can_diagnostics.unexpected_response_count =
                WindowCounterDelta(
                    diagnostics.unexpected_response_count,
                    diagnostics_window.scheduler.unexpected_response_count,
                    counters_monotonic);
            can_diagnostics.maintenance_response_count =
                WindowCounterDelta(
                    diagnostics.maintenance_response_count,
                    diagnostics_window.scheduler.maintenance_response_count,
                    counters_monotonic);
            can_diagnostics.query_target_overlap_count =
                WindowCounterDelta(
                    diagnostics.query_target_overlap_count,
                    diagnostics_window.scheduler.query_target_overlap_count,
                    counters_monotonic);
            const auto target_diagnostics =
                completion_tracker.diagnostics();
            can_diagnostics.target_retry_count =
                target_diagnostics.retry_count;
            can_diagnostics.target_retry_exhausted_count =
                target_diagnostics.retry_exhausted_count;
            can_diagnostics.target_deadline_failure_count =
                target_diagnostics.deadline_failure_count;
            can_diagnostics.max_fanout_us =
                target_diagnostics.max_fanout_us;
            can_diagnostics.safety_preemption_count =
                WindowCounterDelta(
                    safety_preemption_count,
                    diagnostics_window.safety_preemption_count,
                    counters_monotonic);
            can_diagnostics.transition_failure_count =
                WindowCounterDelta(
                    transition_failure_count,
                    diagnostics_window.transition_failure_count,
                    counters_monotonic);
        }
        can_diagnostics.max_safety_wait_us = max_safety_wait_us;
        can_diagnostics.max_rx_dispatch_latency_us =
            max_rx_dispatch_latency_us;

        constexpr uint8_t kAllMotorMarkers = static_cast<uint8_t>(
            (1U << dummy::protocol::kActuatorNodeCount) - 1U);
        if (diagnostics_window.active)
            can_diagnostics.window_flags |=
                dummy::protocol::kCanDiagnosticsWindowActive;
        if (diagnostics_window.epoch_stable)
            can_diagnostics.window_flags |=
                dummy::protocol::kCanDiagnosticsEpochStable;
        if (counters_monotonic)
            can_diagnostics.window_flags |=
                dummy::protocol::kCanDiagnosticsMotorCountersMonotonic;
        if (motor_diagnostics.valid_mask == kAllMotorMarkers)
            can_diagnostics.window_flags |=
                dummy::protocol::kCanDiagnosticsMarkersComplete;
        else if (diagnostics_window.active)
        {
            stream_fail_closed = true;
            dummy::protocol::RequestBinaryRuntimeHold();
        }
        if (diagnostics_window.counters_monotonic && !counters_monotonic)
        {
            diagnostics_window.counters_monotonic = false;
            stream_fail_closed = true;
            dummy::protocol::RequestBinaryRuntimeHold();
        }
        dummy::protocol::PublishCanDiagnostics(can_diagnostics);
        dummy::protocol::PublishCanTimingProfile(
            dummy::protocol::BinaryControlMonotonicMicros(), diagnostics);
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
