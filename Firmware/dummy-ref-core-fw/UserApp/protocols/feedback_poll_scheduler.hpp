#ifndef DUMMY_FEEDBACK_POLL_SCHEDULER_HPP
#define DUMMY_FEEDBACK_POLL_SCHEDULER_HPP

#include "can_feedback_monitor.hpp"

#include <array>
#include <cstdint>

namespace dummy::protocol
{

enum class CanDispatchMode : uint8_t
{
    Bootstrap,
    Hold,
    Stream,
    Fault,
};

// Explicit lifecycle of one Stream attempt (doc 05 section 8.2). Feedback
// service is allowed in every phase; only target fan-out admission differs.
enum class CanStreamPhase : uint8_t
{
    Idle,               // Not in Stream, or the attempt failed closed.
    Preflight,          // HoldTargets -> diagnostics -> gripper config -> enable.
    AwaitFreshFeedback, // Enable completed; position queries run, no fan-out.
    ReadyNoTarget,      // Fresh post-enable sweep sealed; first target pending.
    Streaming,          // Dispatchable target admitted; 50 Hz fan-out cadence.
};

enum class CanDispatchAction : uint8_t
{
    None,
    ActuatorTarget,
    PositionRequest,
    TemperatureRequest,
    MotorDiagnosticsRequest,
    MotorTimingRequest,
    ConfigureGripperVelocity,
    EnableBroadcast,
    DisableBroadcast,
};

struct FeedbackResponseEvents
{
    uint8_t position_mask = 0;
    uint8_t temperature_mask = 0;
    uint8_t timing_profile_mask = 0;
    std::array<uint8_t, kActuatorNodeCount> timing_profile_page{};
    uint32_t unexpected_position_count = 0;
    uint32_t unexpected_temperature_count = 0;
    uint32_t unexpected_timing_profile_count = 0;
};

struct CanDispatchStep
{
    CanDispatchAction action = CanDispatchAction::None;
    uint8_t node_id = 0;
    uint8_t timing_profile_page = 0;
    uint32_t feedback_sweep_id = 0;
    CanDispatchAction timed_out_action = CanDispatchAction::None;
    uint8_t timed_out_node_id = 0;
    uint8_t accepted_timing_profile_node_id = 0;
    uint8_t accepted_timing_profile_page = 0;
    bool timed_out_final = false;
    bool transition = false;
    // Set on the tick that seals the first complete post-enable sweep and
    // moves AwaitFreshFeedback -> ReadyNoTarget.
    bool fresh_feedback_ready = false;
    uint32_t fresh_feedback_sweep_id = 0U;
    // Set once when the AwaitFreshFeedback phase budget expires without a
    // complete sweep. Callers must fail closed and record the first cause.
    bool post_enable_feedback_timeout = false;
    uint8_t post_enable_feedback_missing_mask = 0U;
    // Set on steps produced by OnAdmissionFailed() when the failure must be
    // handled by the upper layer (preflight query or frozen target batch).
    bool admission_failed = false;
    // A failed tail admission ends any earlier on-wire request in this
    // sweep. This is cancellation evidence, not a fabricated TX/timeout.
    uint8_t cancelled_position_node_id = 0U;
    uint32_t cancelled_position_sweep_id = 0U;
};

inline bool ApplyFeedbackAdmissionOutcome(CanFeedbackMonitor& monitor,
                                          const CanDispatchStep& outcome)
{
    return monitor.OnPositionCancelled(outcome.cancelled_position_node_id,
                                        outcome.cancelled_position_sweep_id);
}

struct CanDispatchConfig
{
    uint32_t response_timeout_us = 4000U;
    uint32_t node_quiet_us = 5000U;
    uint32_t scheduler_watchdog_hz = 1000U;
    uint32_t target_hz_per_node = 50U;
    uint32_t position_hz_per_node = 40U;
    uint32_t temperature_hz_per_node = 1U;
    uint32_t timing_profile_hz_per_node = 0U;
    // AwaitFreshFeedback phase budget. Zero disables the explicit deadline;
    // the firmware wires this to the reviewed feedback-HOLD boundary.
    uint32_t post_enable_feedback_timeout_us = 100000U;
    // Bounded wait for the enable/disable frame terminal state. Zero disables.
    uint32_t enable_completion_timeout_us = 5000U;
    // Admission budget for a frozen fan-out whose first frame has not been
    // admitted yet (doc 07 R04). Once the first frame is queued the whole
    // batch is bounded by the caller's completion deadline instead.
    uint32_t target_fanout_admission_timeout_us = 15000U;
};

struct CanDispatchDiagnostics
{
    uint32_t tick_count = 0;
    uint32_t idle_slot_count = 0;
    uint32_t deferred_send_count = 0;
    uint32_t unexpected_response_count = 0;
    uint32_t maintenance_response_count = 0;
    uint32_t query_target_overlap_count = 0;
    std::array<uint32_t, kActuatorNodeCount> target_queued{};
    std::array<uint32_t, kActuatorNodeCount> position_requested{};
    std::array<uint32_t, kActuatorNodeCount> position_responded{};
    std::array<uint32_t, kActuatorNodeCount> position_timed_out{};
    std::array<uint32_t, kActuatorNodeCount> temperature_requested{};
    std::array<uint32_t, kActuatorNodeCount> temperature_responded{};
    std::array<uint32_t, kActuatorNodeCount> temperature_timed_out{};
    std::array<uint32_t, kActuatorNodeCount> timing_profile_requested{};
    std::array<uint32_t, kActuatorNodeCount> timing_profile_responded{};
    std::array<uint32_t, kActuatorNodeCount> timing_profile_timed_out{};
    bool query_pending = false;
    CanDispatchAction pending_action = CanDispatchAction::None;
    uint8_t pending_node_id = 0;
    bool config_valid = true;
    CanStreamPhase stream_phase = CanStreamPhase::Idle;
    uint32_t ready_sweep_id = 0U;
    uint32_t ready_time_us = 0U;
    uint32_t fresh_feedback_baseline_us = 0U;
    uint32_t fanout_action_sequence = 0U;
    uint32_t fanout_generation = 0U;
    uint32_t post_enable_feedback_timeout_count = 0U;
    uint32_t target_unavailable_skip_count = 0U;
    uint32_t cancelled_fanout_count = 0U;
    uint32_t enable_completion_timeout_count = 0U;
    std::array<uint32_t, kActuatorNodeCount> position_admission_failed{};
    std::array<uint32_t, kActuatorNodeCount> temperature_admission_failed{};
    std::array<uint32_t, kActuatorNodeCount> timing_profile_admission_failed{};
    uint32_t query_admission_failure_count = 0U;
    uint32_t target_admission_failed = 0U;
    uint32_t fanout_admission_timeout_count = 0U;
};

// Immutable per-wake scheduler input (doc 05 section 8.1). target_available
// must be judged from the same snapshot that later provides the payload, and
// must mean "valid, unexpired, same session, acceptable action sequence, no
// safety stop" - not merely "the mailbox is not empty".
struct DispatchInput
{
    uint32_t now_us = 0U;
    uint32_t session_epoch = 0U;
    bool motion_authorized = false;
    bool target_available = false;
    uint32_t action_sequence = 0U;
    uint32_t target_generation = 0U;
};

// v2.2 event-driven CAN traffic planner. TX-complete and RX-response wake the
// task immediately; the 1 kHz timer is only a watchdog/deadline fallback:
//   - active target:       50 Hz/node (350 frames/s)
//   - position feedback:  40 Hz/node (280 requests/s)
//   - temperature:         1 Hz/node (7 requests/s)
// Only one feedback transaction and one CAN frame may be outstanding. Target
// and position rates describe complete seven-node cycles, not timer slots.
class CanDispatchScheduler
{
public:
    explicit CanDispatchScheduler(const CanDispatchConfig& config = {});

    void SetMode(CanDispatchMode mode);
    void SetDispatchInput(const DispatchInput& input);
    CanDispatchStep Next(uint32_t now_us,
                         const FeedbackResponseEvents& responses = {});
    void OnQueued(const CanDispatchStep& step, uint32_t now_us);
    void OnTransmissionCompleted(CanDispatchAction action, uint8_t node_id,
                                 uint32_t completed_us, bool successful,
                                 uint32_t now_us = 0U);
    // Cancels a fan-out before any node of it was admitted. The next attempt
    // is deferred one target cycle; feedback service is never held back.
    void CancelUnstartedTargetFanout(uint32_t now_us);
    // Consumes a definitive admission failure for a step the caller could not
    // enqueue (doc 07 R03/R04). Queries are closed or skipped with their own
    // counters instead of being re-issued forever; a frozen target batch is
    // terminated with a timed_out event for the caller to fail closed.
    CanDispatchStep OnAdmissionFailed(const CanDispatchStep& step,
                                      bool definitive_failure,
                                      uint32_t now_us);
    void OnDeferred();
    void Reset();

    CanDispatchMode mode() const { return mode_; }
    CanStreamPhase stream_phase() const { return phase_; }
    DispatchInput dispatch_input() const { return dispatch_input_; }
    CanDispatchDiagnostics diagnostics() const;

private:
    enum class Transition : uint8_t
    {
        None,
        HoldTargets,
        MotorDiagnostics,
        ConfigureGripper,
        Enable,
        Disable,
    };

    static uint8_t NextNode(uint8_t node_id);
    bool NodeQuiet(uint8_t node_id, uint32_t now_us) const;
    bool AllNodesQuiet(uint32_t now_us) const;
    uint8_t SelectTargetNode(uint32_t now_us) const;
    uint8_t SelectPositionNode(uint32_t now_us) const;
    uint8_t SelectTemperatureNode(uint32_t now_us) const;
    uint8_t SelectTimingProfileNode(uint32_t now_us) const;
    void ConsumeResponses(const FeedbackResponseEvents& responses,
                          uint32_t now_us, CanDispatchStep& step);
    void AdvancePositionSweep(uint32_t now_us);
    void FinishPositionSweep(uint32_t now_us);
    void FlushPhaseEvents(CanDispatchStep& step);
    uint8_t SelectPositionRetryNode() const;
    static uint32_t PeriodUs(uint32_t hz_per_node);
    static uint32_t CyclePeriodUs(uint32_t hz_per_node);
    static bool DeadlineDue(uint32_t now_us, uint32_t deadline_us);
    void InitializeDeadlines(uint32_t now_us);
    void AdvanceDeadline(uint32_t& deadline_us, uint32_t hz_per_node,
                         uint32_t now_us);
    void FailClosedStream();

    CanDispatchConfig config_{};
    bool config_valid_ = true;
    CanDispatchMode mode_ = CanDispatchMode::Bootstrap;
    CanStreamPhase phase_ = CanStreamPhase::Idle;
    DispatchInput dispatch_input_{};
    Transition transition_ = Transition::None;
    bool transition_frame_queued_ = false;
    uint32_t transition_queued_us_ = 0U;
    uint8_t transition_node_ = 1U;
    bool deadlines_initialized_ = false;
    bool periodic_traffic_stopped_ = false;
    uint32_t fresh_feedback_baseline_us_ = 0U;
    uint32_t post_enable_feedback_deadline_us_ = 0U;
    uint8_t phase_valid_mask_ = 0U;
    bool phase_ready_pending_ = false;
    uint32_t phase_ready_sweep_id_ = 0U;
    uint32_t next_target_deadline_us_ = 0U;
    uint32_t next_position_deadline_us_ = 0U;
    uint32_t next_temperature_deadline_us_ = 0U;
    uint32_t next_timing_profile_deadline_us_ = 0U;
    uint8_t next_target_node_ = 1U;
    uint8_t next_position_node_ = 1U;
    uint8_t next_temperature_node_ = 1U;
    uint8_t next_timing_profile_node_ = 1U;
    std::array<uint8_t, kActuatorNodeCount> timing_profile_page_{};
    bool target_fanout_active_ = false;
    uint8_t target_fanout_node_ = 1U;
    uint32_t target_fanout_started_us_ = 0U;
    uint32_t target_fanout_action_sequence_ = 0U;
    uint32_t target_fanout_generation_ = 0U;
    bool position_sweep_active_ = false;
    uint8_t position_sweep_start_node_ = 1U;
    uint8_t position_sweep_node_ = 1U;
    uint8_t position_sweep_count_ = 0U;
    uint8_t position_sweep_valid_mask_ = 0U;
    uint32_t position_sweep_id_ = 0U;
    uint32_t next_position_sweep_id_ = 1U;
    uint8_t position_retry_mask_ = 0U;
    bool position_retry_phase_ = false;
    std::array<bool, kActuatorNodeCount> position_pending_{};
    std::array<uint8_t, kActuatorNodeCount> position_attempts_{};
    std::array<uint32_t, kActuatorNodeCount> last_node_tx_us_{};
    std::array<bool, kActuatorNodeCount> node_transmitted_{};
    bool query_pending_ = false;
    CanDispatchAction pending_action_ = CanDispatchAction::None;
    uint8_t pending_node_id_ = 0U;
    uint32_t pending_since_us_ = 0U;
    bool pending_transmitted_ = false;
    CanDispatchDiagnostics diagnostics_{};
};

class ActuatorApplicationTracker
{
public:
    bool RecordTransmission(uint32_t sequence, uint8_t node_id,
                            bool transmitted);
    uint32_t TakeSupersededSequence();
    void Reset();

private:
    uint32_t sequence_ = 0;
    uint8_t transmitted_nodes_ = 0;
    bool sequence_active_ = false;
    bool completion_reported_ = false;
    uint32_t superseded_sequence_ = 0;
};

struct TargetFanoutKey
{
    uint32_t session_epoch = 0U;
    uint32_t action_sequence = 0U;
    uint32_t fanout_generation = 0U;
};

struct TargetRetryRequest
{
    TargetFanoutKey key{};
    uint8_t node_id = 0U;
    bool valid = false;
};

enum class TargetCompletionResult : uint8_t
{
    Ignored,
    Awaiting,
    RetryRequired,
    CompleteExact,
    Failed,
};

struct TargetCompletionDiagnostics
{
    uint32_t retry_count = 0U;
    uint32_t retry_exhausted_count = 0U;
    uint32_t deadline_failure_count = 0U;
    uint32_t max_fanout_us = 0U;
};

// Tracks one frozen seven-node target from its first hardware admission until
// every node completes. A failed node may be admitted once more only under the
// exact original session/sequence/generation key. The total deadline includes
// both the first attempt and that retry.
class TargetCompletionTracker
{
public:
    explicit TargetCompletionTracker(uint32_t fanout_timeout_us = 15000U);

    bool Begin(const TargetFanoutKey& key, uint32_t first_enqueue_us);
    TargetCompletionResult RecordCompletion(
        const TargetFanoutKey& key, uint8_t node_id, bool complete,
        uint32_t completed_us);
    TargetCompletionResult CheckDeadline(uint32_t now_us);
    bool MarkRetryQueued(const TargetRetryRequest& request);
    // Resolve the owning action before clearing the tracker. Once all seven
    // frames are queued, only this tracker still owns a failed retry's key.
    template<typename FailureRecorder>
    void FailAndCancel(uint32_t unqueued_sequence, FailureRecorder record)
    {
        const uint32_t sequence = active_ ? key_.action_sequence : unqueued_sequence;
        if (sequence != 0U)
            record(sequence);
        Cancel();
    }
    void Cancel();
    void ResetDiagnostics();

    bool active() const { return active_; }
    TargetFanoutKey key() const { return key_; }
    TargetRetryRequest retry_request() const { return retry_request_; }
    uint32_t last_fanout_us() const { return last_fanout_us_; }
    TargetCompletionDiagnostics diagnostics() const { return diagnostics_; }

private:
    static bool KeysMatch(const TargetFanoutKey& left,
                          const TargetFanoutKey& right);
    TargetCompletionResult Fail(uint32_t elapsed_us, bool retry_exhausted,
                                bool deadline_failure);

    uint32_t fanout_timeout_us_ = 15000U;
    TargetFanoutKey key_{};
    uint32_t first_enqueue_us_ = 0U;
    uint8_t completed_mask_ = 0U;
    uint8_t retried_mask_ = 0U;
    TargetRetryRequest retry_request_{};
    bool active_ = false;
    uint32_t last_fanout_us_ = 0U;
    TargetCompletionDiagnostics diagnostics_{};
};

} // namespace dummy::protocol

#endif // DUMMY_FEEDBACK_POLL_SCHEDULER_HPP
