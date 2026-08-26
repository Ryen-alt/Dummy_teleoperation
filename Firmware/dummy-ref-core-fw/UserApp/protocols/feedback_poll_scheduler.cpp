#include "feedback_poll_scheduler.hpp"

#include <algorithm>
#include <limits>

namespace dummy::protocol
{
namespace
{
uint8_t NodeMask(uint8_t node_id)
{
    return node_id >= 1U && node_id <= kActuatorNodeCount
        ? static_cast<uint8_t>(1U << (node_id - 1U)) : 0U;
}

uint32_t SaturatingIncrement(uint32_t value)
{
    return value == std::numeric_limits<uint32_t>::max() ? value : value + 1U;
}

uint32_t SaturatingAdd(uint32_t value, uint32_t increment)
{
    const uint32_t remaining = std::numeric_limits<uint32_t>::max() - value;
    return increment > remaining
        ? std::numeric_limits<uint32_t>::max() : value + increment;
}
}

CanDispatchScheduler::CanDispatchScheduler(const CanDispatchConfig& config)
    : config_(config)
{
    config_valid_ = config_.scheduler_watchdog_hz >= 100U &&
        config_.scheduler_watchdog_hz <= 5000U &&
        config_.response_timeout_us != 0U &&
        config_.target_hz_per_node != 0U;
}

uint8_t CanDispatchScheduler::NextNode(uint8_t node_id)
{
    return node_id >= kActuatorNodeCount ? 1U : static_cast<uint8_t>(node_id + 1U);
}

uint32_t CanDispatchScheduler::PeriodUs(uint32_t hz_per_node)
{
    if (hz_per_node == 0U)
        return 0U;
    const uint32_t aggregate_hz = hz_per_node * kActuatorNodeCount;
    const uint32_t period_us = static_cast<uint32_t>(
        (1000000ULL + aggregate_hz / 2U) / aggregate_hz);
    return period_us == 0U ? uint32_t{1} : period_us;
}

uint32_t CanDispatchScheduler::CyclePeriodUs(uint32_t hz_per_node)
{
    if (hz_per_node == 0U)
        return 0U;
    return static_cast<uint32_t>(
        (1000000ULL + hz_per_node / 2U) / hz_per_node);
}

bool CanDispatchScheduler::DeadlineDue(uint32_t now_us,
                                       uint32_t deadline_us)
{
    return static_cast<int32_t>(now_us - deadline_us) >= 0;
}

void CanDispatchScheduler::InitializeDeadlines(uint32_t now_us)
{
    const auto initialize = [now_us](uint32_t hz_per_node)
    {
        const uint32_t period_us = CyclePeriodUs(hz_per_node);
        return period_us == 0U ? 0U : now_us + period_us;
    };
    next_target_deadline_us_ = initialize(config_.target_hz_per_node);
    next_position_deadline_us_ = initialize(config_.position_hz_per_node);
    const uint32_t temperature_period = PeriodUs(
        config_.temperature_hz_per_node);
    next_temperature_deadline_us_ = temperature_period == 0U
        ? 0U : now_us + temperature_period;
    deadlines_initialized_ = true;
}

void CanDispatchScheduler::AdvanceDeadline(uint32_t& deadline_us,
                                           uint32_t hz_per_node,
                                           uint32_t now_us)
{
    const uint32_t period_us = PeriodUs(hz_per_node);
    if (period_us == 0U)
    {
        deadline_us = 0U;
        return;
    }
    if (deadline_us == 0U)
    {
        deadline_us = now_us + period_us;
        return;
    }
    const uint32_t lateness_us = now_us - deadline_us;
    // Minor arbitration jitter preserves the configured phase. Missing a
    // complete period discards the old deadline and re-bases from now, so
    // delayed work can never be replayed as a catch-up burst.
    deadline_us = DeadlineDue(now_us, deadline_us) &&
        lateness_us >= period_us
        ? now_us + period_us : deadline_us + period_us;
}

bool CanDispatchScheduler::NodeQuiet(uint8_t node_id, uint32_t now_us) const
{
    if (node_id < 1U || node_id > kActuatorNodeCount)
        return false;
    const size_t index = node_id - 1U;
    return !node_transmitted_[index] ||
        now_us - last_node_tx_us_[index] >= config_.node_quiet_us;
}

bool CanDispatchScheduler::AllNodesQuiet(uint32_t now_us) const
{
    for (uint8_t node_id = 1U; node_id <= kActuatorNodeCount; ++node_id)
    {
        if (!NodeQuiet(node_id, now_us))
            return false;
    }
    return true;
}

uint8_t CanDispatchScheduler::SelectTargetNode(uint32_t now_us) const
{
    uint8_t selected = 0U;
    uint32_t minimum_count = std::numeric_limits<uint32_t>::max();
    uint8_t node_id = next_target_node_;
    for (size_t attempt = 0; attempt < kActuatorNodeCount; ++attempt)
    {
        const uint32_t count = diagnostics_.target_queued[node_id - 1U];
        if (NodeQuiet(node_id, now_us) && count < minimum_count)
        {
            selected = node_id;
            minimum_count = count;
        }
        node_id = NextNode(node_id);
    }
    return selected;
}

uint8_t CanDispatchScheduler::SelectPositionNode(uint32_t now_us) const
{
    uint8_t selected = 0U;
    uint32_t minimum_count = std::numeric_limits<uint32_t>::max();
    uint8_t node_id = next_position_node_;
    for (size_t attempt = 0; attempt < kActuatorNodeCount; ++attempt)
    {
        const uint32_t count = diagnostics_.position_requested[node_id - 1U];
        if (NodeQuiet(node_id, now_us) && count < minimum_count)
        {
            selected = node_id;
            minimum_count = count;
        }
        node_id = NextNode(node_id);
    }
    return selected;
}

uint8_t CanDispatchScheduler::SelectTemperatureNode(uint32_t now_us) const
{
    uint8_t selected = 0U;
    uint32_t minimum_count = std::numeric_limits<uint32_t>::max();
    uint8_t node_id = next_temperature_node_;
    for (size_t attempt = 0; attempt < kActuatorNodeCount; ++attempt)
    {
        const uint32_t count = diagnostics_.temperature_requested[node_id - 1U];
        if (NodeQuiet(node_id, now_us) && count < minimum_count)
        {
            selected = node_id;
            minimum_count = count;
        }
        node_id = NextNode(node_id);
    }
    return selected;
}

uint8_t CanDispatchScheduler::SelectPositionRetryNode() const
{
    uint8_t node_id = position_sweep_start_node_;
    for (size_t attempt = 0; attempt < kActuatorNodeCount; ++attempt)
    {
        if ((position_retry_mask_ & NodeMask(node_id)) != 0U)
            return node_id;
        node_id = NextNode(node_id);
    }
    return 0U;
}

void CanDispatchScheduler::FinishPositionSweep(uint32_t now_us)
{
    position_sweep_active_ = false;
    position_sweep_count_ = 0U;
    position_sweep_id_ = 0U;
    position_retry_mask_ = 0U;
    position_retry_phase_ = false;
    position_pending_.fill(false);
    position_attempts_.fill(0U);
    position_sweep_start_node_ = NextNode(position_sweep_start_node_);
    const uint32_t period_us = CyclePeriodUs(config_.position_hz_per_node);
    if (period_us == 0U)
    {
        next_position_deadline_us_ = 0U;
        return;
    }
    const uint32_t lateness_us = now_us - next_position_deadline_us_;
    next_position_deadline_us_ =
        DeadlineDue(now_us, next_position_deadline_us_) &&
        lateness_us >= period_us
        ? now_us + period_us : next_position_deadline_us_ + period_us;
}

void CanDispatchScheduler::AdvancePositionSweep(uint32_t now_us)
{
    if (position_retry_phase_)
    {
        const uint8_t next_retry = SelectPositionRetryNode();
        if (next_retry == 0U)
            FinishPositionSweep(now_us);
        else
            position_sweep_node_ = next_retry;
        return;
    }

    ++position_sweep_count_;
    if (position_sweep_count_ < kActuatorNodeCount)
    {
        position_sweep_node_ = NextNode(position_sweep_node_);
        return;
    }

    position_retry_phase_ = true;
    const uint8_t next_retry = SelectPositionRetryNode();
    if (next_retry == 0U)
        FinishPositionSweep(now_us);
    else
        position_sweep_node_ = next_retry;
}

void CanDispatchScheduler::ConsumeResponses(
    const FeedbackResponseEvents& responses, uint32_t now_us)
{
    const uint32_t unexpected = SaturatingAdd(
        responses.unexpected_position_count,
        responses.unexpected_temperature_count);
    if (mode_ == CanDispatchMode::Stream)
        diagnostics_.unexpected_response_count = SaturatingAdd(
            diagnostics_.unexpected_response_count, unexpected);
    else
        diagnostics_.maintenance_response_count = SaturatingAdd(
            diagnostics_.maintenance_response_count, unexpected);

    const bool current_position_matched = query_pending_ &&
        pending_action_ == CanDispatchAction::PositionRequest &&
        (responses.position_mask & NodeMask(pending_node_id_)) != 0U;
    const bool current_temperature_matched = query_pending_ &&
        (pending_action_ == CanDispatchAction::TemperatureRequest ||
         pending_action_ == CanDispatchAction::MotorDiagnosticsRequest) &&
        (responses.temperature_mask & NodeMask(pending_node_id_)) != 0U;

    for (uint8_t node_id = 1U; node_id <= kActuatorNodeCount; ++node_id)
    {
        const uint8_t mask = NodeMask(node_id);
        if ((responses.position_mask & mask) != 0U)
        {
            diagnostics_.position_responded[node_id - 1U] = SaturatingIncrement(
                diagnostics_.position_responded[node_id - 1U]);
            position_pending_[node_id - 1U] = false;
            position_retry_mask_ = static_cast<uint8_t>(
                position_retry_mask_ & ~mask);
        }
        if ((responses.temperature_mask & mask) != 0U)
            diagnostics_.temperature_responded[node_id - 1U] = SaturatingIncrement(
                diagnostics_.temperature_responded[node_id - 1U]);
    }

    if (current_position_matched || current_temperature_matched)
    {
        const CanDispatchAction completed_action = pending_action_;
        query_pending_ = false;
        pending_action_ = CanDispatchAction::None;
        pending_node_id_ = 0U;
        if (completed_action == CanDispatchAction::PositionRequest &&
            position_sweep_active_)
            AdvancePositionSweep(now_us);
        else if (completed_action ==
                 CanDispatchAction::MotorDiagnosticsRequest &&
                 transition_ == Transition::MotorDiagnostics)
        {
            if (transition_node_ < kActuatorNodeCount)
                transition_node_ = NextNode(transition_node_);
            else
            {
                transition_node_ = 1U;
                transition_ = Transition::ConfigureGripper;
            }
        }
    }

    if (position_sweep_active_ && position_retry_phase_ &&
        position_retry_mask_ == 0U &&
        (!query_pending_ ||
         pending_action_ != CanDispatchAction::PositionRequest))
        FinishPositionSweep(now_us);
}

void CanDispatchScheduler::SetMode(CanDispatchMode mode)
{
    if (mode == mode_)
        return;
    mode_ = mode;
    transition_node_ = 1U;
    deadlines_initialized_ = false;
    target_fanout_active_ = false;
    target_fanout_node_ = 1U;
    position_sweep_active_ = false;
    position_sweep_count_ = 0U;
    position_sweep_id_ = 0U;
    position_retry_mask_ = 0U;
    position_retry_phase_ = false;
    position_pending_.fill(false);
    position_attempts_.fill(0U);
    query_pending_ = false;
    pending_action_ = CanDispatchAction::None;
    pending_node_id_ = 0U;
    if (mode == CanDispatchMode::Stream || mode == CanDispatchMode::Hold)
        transition_ = Transition::HoldTargets;
    else if (mode == CanDispatchMode::Fault)
        transition_ = Transition::Disable;
    else
        transition_ = Transition::None;
}

CanDispatchStep CanDispatchScheduler::Next(
    uint32_t now_us, const FeedbackResponseEvents& responses)
{
    diagnostics_.tick_count = SaturatingIncrement(diagnostics_.tick_count);
    if (!config_valid_)
    {
        diagnostics_.idle_slot_count = SaturatingIncrement(
            diagnostics_.idle_slot_count);
        return {};
    }
    ConsumeResponses(responses, now_us);
    if (transition_ == Transition::None && !deadlines_initialized_)
        InitializeDeadlines(now_us);

    CanDispatchStep step{};
    if (query_pending_ &&
        now_us - pending_since_us_ >= config_.response_timeout_us)
    {
        step.timed_out_action = pending_action_;
        step.timed_out_node_id = pending_node_id_;
        const size_t index = pending_node_id_ - 1U;
        if (pending_action_ == CanDispatchAction::PositionRequest)
        {
            diagnostics_.position_timed_out[index] = SaturatingIncrement(
                diagnostics_.position_timed_out[index]);
            const bool retry_exhausted = position_retry_phase_ ||
                position_attempts_[index] >= 2U;
            if (retry_exhausted)
            {
                step.timed_out_final = true;
                position_pending_[index] = false;
                position_retry_mask_ = static_cast<uint8_t>(
                    position_retry_mask_ & ~NodeMask(pending_node_id_));
            }
            else
            {
                position_retry_mask_ = static_cast<uint8_t>(
                    position_retry_mask_ | NodeMask(pending_node_id_));
            }
        }
        else if (pending_action_ == CanDispatchAction::TemperatureRequest ||
                 pending_action_ ==
                     CanDispatchAction::MotorDiagnosticsRequest)
        {
            step.timed_out_final = true;
            diagnostics_.temperature_timed_out[index] = SaturatingIncrement(
                diagnostics_.temperature_timed_out[index]);
            if (pending_action_ == CanDispatchAction::TemperatureRequest)
            {
                const uint32_t period_us = PeriodUs(
                    config_.temperature_hz_per_node);
                next_temperature_deadline_us_ = period_us == 0U
                    ? 0U : now_us + period_us;
            }
            else
            {
                transition_ = Transition::None;
            }
        }
        query_pending_ = false;
        pending_action_ = CanDispatchAction::None;
        pending_node_id_ = 0U;
        if (step.timed_out_action == CanDispatchAction::PositionRequest &&
            position_sweep_active_)
            AdvancePositionSweep(now_us);
    }

    // Emergency disable and HOLD transitions always outrank normal traffic.
    // The Bsp layer keeps exactly one frame in flight, so the next completion
    // interrupt is the only unavoidable safety delay.
    if (transition_ == Transition::Disable)
    {
        step.action = CanDispatchAction::DisableBroadcast;
        step.transition = true;
        return step;
    }

    if (transition_ == Transition::HoldTargets)
    {
        if (NodeQuiet(transition_node_, now_us))
        {
            step.action = CanDispatchAction::ActuatorTarget;
            step.node_id = transition_node_;
            step.transition = true;
            return step;
        }
        diagnostics_.idle_slot_count = SaturatingIncrement(
            diagnostics_.idle_slot_count);
        return step;
    }

    if (transition_ == Transition::MotorDiagnostics)
    {
        if (query_pending_)
        {
            diagnostics_.idle_slot_count = SaturatingIncrement(
                diagnostics_.idle_slot_count);
            return step;
        }
        if (NodeQuiet(transition_node_, now_us))
        {
            step.action = CanDispatchAction::MotorDiagnosticsRequest;
            step.node_id = transition_node_;
            step.transition = true;
            return step;
        }
        diagnostics_.idle_slot_count = SaturatingIncrement(
            diagnostics_.idle_slot_count);
        return step;
    }

    if (transition_ == Transition::ConfigureGripper)
    {
        if (NodeQuiet(kActuatorNodeCount, now_us))
        {
            step.action = CanDispatchAction::ConfigureGripperVelocity;
            step.node_id = kActuatorNodeCount;
            step.transition = true;
            return step;
        }
        diagnostics_.idle_slot_count = SaturatingIncrement(
            diagnostics_.idle_slot_count);
        return step;
    }

    if (transition_ == Transition::Enable)
    {
        if (AllNodesQuiet(now_us))
        {
            step.action = CanDispatchAction::EnableBroadcast;
            step.transition = true;
            return step;
        }
        diagnostics_.idle_slot_count = SaturatingIncrement(
            diagnostics_.idle_slot_count);
        return step;
    }

    if (transition_ != Transition::None)
    {
        diagnostics_.idle_slot_count = SaturatingIncrement(
            diagnostics_.idle_slot_count);
        return step;
    }

    // A 50 Hz target deadline starts one frozen seven-node fan-out. Once
    // started, TX-complete notifications drive nodes 2..7 immediately; the
    // watchdog timer is no longer the throughput limiter.
    if (mode_ == CanDispatchMode::Stream && !target_fanout_active_ &&
        next_target_deadline_us_ != 0U &&
        DeadlineDue(now_us, next_target_deadline_us_))
    {
        target_fanout_active_ = true;
        target_fanout_node_ = 1U;
        target_fanout_started_us_ = now_us;
    }
    if (target_fanout_active_)
    {
        if (NodeQuiet(target_fanout_node_, now_us))
        {
            step.action = CanDispatchAction::ActuatorTarget;
            step.node_id = target_fanout_node_;
            return step;
        }
        diagnostics_.idle_slot_count = SaturatingIncrement(
            diagnostics_.idle_slot_count);
        return step;
    }

    if (query_pending_)
    {
        diagnostics_.idle_slot_count = SaturatingIncrement(
            diagnostics_.idle_slot_count);
        return step;
    }

    // A first timeout skips the node and continues the rotating sweep. Only
    // nodes still missing at the tail receive one bounded retry.
    if (!position_sweep_active_ && next_position_deadline_us_ != 0U &&
        DeadlineDue(now_us, next_position_deadline_us_))
    {
        position_sweep_active_ = true;
        position_sweep_count_ = 0U;
        position_sweep_node_ = position_sweep_start_node_;
        position_retry_mask_ = 0U;
        position_retry_phase_ = false;
        position_pending_.fill(false);
        position_attempts_.fill(0U);
        position_sweep_id_ = next_position_sweep_id_++;
        if (position_sweep_id_ == 0U)
        {
            position_sweep_id_ = 1U;
            next_position_sweep_id_ = 2U;
        }
    }
    if (position_sweep_active_)
    {
        if (NodeQuiet(position_sweep_node_, now_us))
        {
            step.action = CanDispatchAction::PositionRequest;
            step.node_id = position_sweep_node_;
            step.feedback_sweep_id = position_sweep_id_;
            return step;
        }
        diagnostics_.idle_slot_count = SaturatingIncrement(
            diagnostics_.idle_slot_count);
        return step;
    }

    if (next_temperature_deadline_us_ != 0U &&
        DeadlineDue(now_us, next_temperature_deadline_us_) &&
        NodeQuiet(next_temperature_node_, now_us))
    {
        step.action = CanDispatchAction::TemperatureRequest;
        step.node_id = next_temperature_node_;
        return step;
    }

    diagnostics_.idle_slot_count = SaturatingIncrement(diagnostics_.idle_slot_count);
    return step;
}

void CanDispatchScheduler::OnQueued(const CanDispatchStep& step, uint32_t now_us)
{
    if (step.action == CanDispatchAction::None)
        return;

    if (step.node_id >= 1U && step.node_id <= kActuatorNodeCount)
    {
        const size_t index = step.node_id - 1U;
        last_node_tx_us_[index] = now_us;
        node_transmitted_[index] = true;
    }
    else if (step.action == CanDispatchAction::EnableBroadcast ||
             step.action == CanDispatchAction::DisableBroadcast)
    {
        last_node_tx_us_.fill(now_us);
        node_transmitted_.fill(true);
    }

    if (step.action == CanDispatchAction::ActuatorTarget)
    {
        diagnostics_.target_queued[step.node_id - 1U] = SaturatingIncrement(
            diagnostics_.target_queued[step.node_id - 1U]);
        if (!step.transition && query_pending_)
            diagnostics_.query_target_overlap_count = SaturatingIncrement(
                diagnostics_.query_target_overlap_count);
        if (!step.transition)
        {
            if (step.node_id < kActuatorNodeCount)
            {
                target_fanout_node_ = NextNode(step.node_id);
            }
            else
            {
                target_fanout_active_ = false;
                target_fanout_node_ = 1U;
                const uint32_t period_us = CyclePeriodUs(
                    config_.target_hz_per_node);
                const uint32_t lateness_us = now_us - next_target_deadline_us_;
                next_target_deadline_us_ =
                    DeadlineDue(now_us, next_target_deadline_us_) &&
                    lateness_us >= period_us
                    ? now_us + period_us
                    : next_target_deadline_us_ + period_us;
            }
        }
    }
    else if (step.action == CanDispatchAction::PositionRequest)
    {
        diagnostics_.position_requested[step.node_id - 1U] = SaturatingIncrement(
            diagnostics_.position_requested[step.node_id - 1U]);
        position_pending_[step.node_id - 1U] = true;
        if (position_attempts_[step.node_id - 1U] != UINT8_MAX)
            ++position_attempts_[step.node_id - 1U];
        query_pending_ = true;
        pending_action_ = step.action;
        pending_node_id_ = step.node_id;
        pending_since_us_ = now_us;
    }
    else if (step.action == CanDispatchAction::TemperatureRequest)
    {
        AdvanceDeadline(next_temperature_deadline_us_,
                        config_.temperature_hz_per_node, now_us);
        diagnostics_.temperature_requested[step.node_id - 1U] = SaturatingIncrement(
            diagnostics_.temperature_requested[step.node_id - 1U]);
        next_temperature_node_ = NextNode(step.node_id);
        query_pending_ = true;
        pending_action_ = step.action;
        pending_node_id_ = step.node_id;
        pending_since_us_ = now_us;
    }
    else if (step.action == CanDispatchAction::MotorDiagnosticsRequest)
    {
        diagnostics_.temperature_requested[step.node_id - 1U] =
            SaturatingIncrement(
                diagnostics_.temperature_requested[step.node_id - 1U]);
        query_pending_ = true;
        pending_action_ = step.action;
        pending_node_id_ = step.node_id;
        pending_since_us_ = now_us;
    }

    if (!step.transition)
        return;
    if (transition_ == Transition::HoldTargets)
    {
        if (transition_node_ < kActuatorNodeCount)
            transition_node_ = NextNode(transition_node_);
        else
        {
            transition_node_ = 1U;
            transition_ = mode_ == CanDispatchMode::Stream
                ? Transition::MotorDiagnostics : Transition::None;
        }
    }
    else if (transition_ == Transition::ConfigureGripper)
    {
        transition_ = Transition::Enable;
    }
    else if (transition_ == Transition::Enable ||
             transition_ == Transition::Disable)
    {
        transition_ = Transition::None;
    }
    if (transition_ == Transition::None && !deadlines_initialized_)
        InitializeDeadlines(now_us);
}

void CanDispatchScheduler::OnDeferred()
{
    diagnostics_.deferred_send_count = SaturatingIncrement(
        diagnostics_.deferred_send_count);
}

void CanDispatchScheduler::Reset()
{
    mode_ = CanDispatchMode::Bootstrap;
    transition_ = Transition::None;
    transition_node_ = 1U;
    deadlines_initialized_ = false;
    next_target_deadline_us_ = 0U;
    next_position_deadline_us_ = 0U;
    next_temperature_deadline_us_ = 0U;
    next_target_node_ = 1U;
    next_position_node_ = 1U;
    next_temperature_node_ = 1U;
    target_fanout_active_ = false;
    target_fanout_node_ = 1U;
    target_fanout_started_us_ = 0U;
    position_sweep_active_ = false;
    position_sweep_start_node_ = 1U;
    position_sweep_node_ = 1U;
    position_sweep_count_ = 0U;
    position_sweep_id_ = 0U;
    next_position_sweep_id_ = 1U;
    position_retry_mask_ = 0U;
    position_retry_phase_ = false;
    position_pending_.fill(false);
    position_attempts_.fill(0U);
    last_node_tx_us_.fill(0U);
    node_transmitted_.fill(false);
    query_pending_ = false;
    pending_action_ = CanDispatchAction::None;
    pending_node_id_ = 0U;
    pending_since_us_ = 0U;
    diagnostics_ = {};
    diagnostics_.config_valid = config_valid_;
}

CanDispatchDiagnostics CanDispatchScheduler::diagnostics() const
{
    CanDispatchDiagnostics output = diagnostics_;
    output.query_pending = query_pending_;
    output.pending_action = pending_action_;
    output.pending_node_id = pending_node_id_;
    output.config_valid = config_valid_;
    return output;
}

bool ActuatorApplicationTracker::RecordTransmission(uint32_t sequence,
                                                    uint8_t node_id,
                                                    bool transmitted)
{
    if (!sequence_active_ || sequence != sequence_)
    {
        if (sequence_active_ && !completion_reported_)
            superseded_sequence_ = sequence_;
        sequence_ = sequence;
        transmitted_nodes_ = 0U;
        sequence_active_ = true;
        completion_reported_ = false;
    }

    if (transmitted && node_id >= 1U && node_id <= kActuatorNodeCount)
        transmitted_nodes_ |= static_cast<uint8_t>(1U << (node_id - 1U));

    constexpr uint8_t kAllActuatorNodes = static_cast<uint8_t>(
        (1U << kActuatorNodeCount) - 1U);
    if (transmitted_nodes_ == kAllActuatorNodes && !completion_reported_)
    {
        completion_reported_ = true;
        return true;
    }
    return false;
}

uint32_t ActuatorApplicationTracker::TakeSupersededSequence()
{
    const uint32_t sequence = superseded_sequence_;
    superseded_sequence_ = 0U;
    return sequence;
}

void ActuatorApplicationTracker::Reset()
{
    sequence_ = 0U;
    transmitted_nodes_ = 0U;
    sequence_active_ = false;
    completion_reported_ = false;
    superseded_sequence_ = 0U;
}

TargetCompletionTracker::TargetCompletionTracker(uint32_t fanout_timeout_us)
    : fanout_timeout_us_(fanout_timeout_us)
{
}

bool TargetCompletionTracker::KeysMatch(const TargetFanoutKey& left,
                                        const TargetFanoutKey& right)
{
    return left.session_epoch == right.session_epoch &&
        left.action_sequence == right.action_sequence &&
        left.fanout_generation == right.fanout_generation;
}

bool TargetCompletionTracker::Begin(const TargetFanoutKey& key,
                                    uint32_t first_enqueue_us)
{
    if (fanout_timeout_us_ == 0U || key.session_epoch == 0U ||
        key.action_sequence == 0U || key.fanout_generation == 0U)
        return false;
    key_ = key;
    first_enqueue_us_ = first_enqueue_us;
    completed_mask_ = 0U;
    retried_mask_ = 0U;
    retry_request_ = {};
    active_ = true;
    return true;
}

TargetCompletionResult TargetCompletionTracker::Fail(
    uint32_t elapsed_us, bool retry_exhausted, bool deadline_failure)
{
    diagnostics_.max_fanout_us = std::max(
        diagnostics_.max_fanout_us, elapsed_us);
    if (retry_exhausted)
        diagnostics_.retry_exhausted_count = SaturatingIncrement(
            diagnostics_.retry_exhausted_count);
    if (deadline_failure)
        diagnostics_.deadline_failure_count = SaturatingIncrement(
            diagnostics_.deadline_failure_count);
    retry_request_ = {};
    active_ = false;
    return TargetCompletionResult::Failed;
}

TargetCompletionResult TargetCompletionTracker::RecordCompletion(
    const TargetFanoutKey& key, uint8_t node_id, bool complete,
    uint32_t completed_us)
{
    if (!active_ || !KeysMatch(key_, key) || node_id < 1U ||
        node_id > kActuatorNodeCount)
        return TargetCompletionResult::Ignored;

    const uint32_t elapsed_us = completed_us - first_enqueue_us_;
    if (elapsed_us >= fanout_timeout_us_)
        return Fail(elapsed_us, false, true);

    const uint8_t node_mask = NodeMask(node_id);
    if (!complete)
    {
        if ((retried_mask_ & node_mask) != 0U)
            return Fail(elapsed_us, true, false);
        retried_mask_ = static_cast<uint8_t>(retried_mask_ | node_mask);
        retry_request_ = {key_, node_id, true};
        diagnostics_.retry_count = SaturatingIncrement(
            diagnostics_.retry_count);
        return TargetCompletionResult::RetryRequired;
    }

    completed_mask_ = static_cast<uint8_t>(completed_mask_ | node_mask);
    constexpr uint8_t kAllNodes = static_cast<uint8_t>(
        (1U << kActuatorNodeCount) - 1U);
    if (completed_mask_ != kAllNodes)
        return TargetCompletionResult::Awaiting;

    diagnostics_.max_fanout_us = std::max(
        diagnostics_.max_fanout_us, elapsed_us);
    retry_request_ = {};
    active_ = false;
    return TargetCompletionResult::CompleteExact;
}

TargetCompletionResult TargetCompletionTracker::CheckDeadline(uint32_t now_us)
{
    if (!active_)
        return TargetCompletionResult::Ignored;
    const uint32_t elapsed_us = now_us - first_enqueue_us_;
    if (elapsed_us < fanout_timeout_us_)
        return TargetCompletionResult::Awaiting;
    return Fail(elapsed_us, false, true);
}

bool TargetCompletionTracker::MarkRetryQueued(
    const TargetRetryRequest& request)
{
    if (!active_ || !request.valid || !retry_request_.valid ||
        request.node_id != retry_request_.node_id ||
        !KeysMatch(request.key, retry_request_.key))
        return false;
    retry_request_ = {};
    return true;
}

void TargetCompletionTracker::Cancel()
{
    key_ = {};
    first_enqueue_us_ = 0U;
    completed_mask_ = 0U;
    retried_mask_ = 0U;
    retry_request_ = {};
    active_ = false;
}

void TargetCompletionTracker::ResetDiagnostics()
{
    diagnostics_ = {};
}

} // namespace dummy::protocol
