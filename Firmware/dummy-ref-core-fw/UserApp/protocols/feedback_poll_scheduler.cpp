#include "feedback_poll_scheduler.hpp"

#include <algorithm>
#include <limits>

namespace dummy::protocol
{
namespace
{
constexpr uint8_t kAllNodeMask = static_cast<uint8_t>(
    (1U << kActuatorNodeCount) - 1U);

uint8_t NodeMask(uint8_t node_id)
{
    return node_id >= 1U && node_id <= kActuatorNodeCount
        ? static_cast<uint8_t>(1U << (node_id - 1U)) : 0U;
}

uint32_t SaturatingIncrement(uint32_t value)
{
    return value == std::numeric_limits<uint32_t>::max() ? value : value + 1U;
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

void CanDispatchScheduler::CountUnexpectedResponses(
    uint8_t mask, CanDispatchAction action, uint8_t expected_node)
{
    uint8_t expected_mask = 0U;
    if (query_pending_ && pending_action_ == action)
        expected_mask = NodeMask(expected_node);
    uint8_t unexpected = static_cast<uint8_t>(mask & ~expected_mask & kAllNodeMask);
    while (unexpected != 0U)
    {
        diagnostics_.unexpected_response_count = SaturatingIncrement(
            diagnostics_.unexpected_response_count);
        unexpected = static_cast<uint8_t>(unexpected & (unexpected - 1U));
    }
}

void CanDispatchScheduler::ConsumeResponses(
    const FeedbackResponseEvents& responses, uint32_t now_us)
{
    for (uint8_t node_id = 1U; node_id <= kActuatorNodeCount; ++node_id)
    {
        const uint8_t mask = NodeMask(node_id);
        if ((responses.position_mask & mask) != 0U)
            diagnostics_.position_responded[node_id - 1U] = SaturatingIncrement(
                diagnostics_.position_responded[node_id - 1U]);
        if ((responses.temperature_mask & mask) != 0U)
            diagnostics_.temperature_responded[node_id - 1U] = SaturatingIncrement(
                diagnostics_.temperature_responded[node_id - 1U]);
    }

    CountUnexpectedResponses(
        responses.position_mask, CanDispatchAction::PositionRequest,
        pending_node_id_);
    CountUnexpectedResponses(
        responses.temperature_mask, CanDispatchAction::TemperatureRequest,
        pending_node_id_);

    if (!query_pending_)
        return;
    const uint8_t mask = NodeMask(pending_node_id_);
    const bool matched =
        (pending_action_ == CanDispatchAction::PositionRequest &&
         (responses.position_mask & mask) != 0U) ||
        (pending_action_ == CanDispatchAction::TemperatureRequest &&
         (responses.temperature_mask & mask) != 0U);
    if (matched)
    {
        const CanDispatchAction completed_action = pending_action_;
        query_pending_ = false;
        pending_action_ = CanDispatchAction::None;
        pending_node_id_ = 0U;
        if (completed_action == CanDispatchAction::PositionRequest &&
            position_sweep_active_)
        {
            ++position_sweep_count_;
            if (position_sweep_count_ >= kActuatorNodeCount)
            {
                position_sweep_active_ = false;
                position_sweep_count_ = 0U;
                position_sweep_start_node_ = NextNode(
                    position_sweep_start_node_);
                const uint32_t period_us = CyclePeriodUs(
                    config_.position_hz_per_node);
                if (period_us == 0U)
                {
                    next_position_deadline_us_ = 0U;
                }
                else
                {
                    const uint32_t lateness_us =
                        now_us - next_position_deadline_us_;
                    next_position_deadline_us_ =
                        DeadlineDue(now_us, next_position_deadline_us_) &&
                        lateness_us >= period_us
                        ? now_us + period_us
                        : next_position_deadline_us_ + period_us;
                }
            }
            else
            {
                position_sweep_node_ = NextNode(position_sweep_node_);
            }
        }
    }
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
            position_sweep_active_ = false;
            position_sweep_count_ = 0U;
            position_sweep_start_node_ = NextNode(position_sweep_start_node_);
            const uint32_t period_us = CyclePeriodUs(
                config_.position_hz_per_node);
            next_position_deadline_us_ = period_us == 0U
                ? 0U : now_us + period_us;
        }
        else if (pending_action_ == CanDispatchAction::TemperatureRequest)
        {
            diagnostics_.temperature_timed_out[index] = SaturatingIncrement(
                diagnostics_.temperature_timed_out[index]);
            const uint32_t period_us = PeriodUs(
                config_.temperature_hz_per_node);
            next_temperature_deadline_us_ = period_us == 0U
                ? 0U : now_us + period_us;
        }
        query_pending_ = false;
        pending_action_ = CanDispatchAction::None;
        pending_node_id_ = 0U;
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

    // Position feedback is a complete rotating sweep. Each response wakes the
    // dispatcher and immediately schedules the next node. A timeout discards
    // the rest of the sweep and re-bases its 25 ms deadline.
    if (!position_sweep_active_ && next_position_deadline_us_ != 0U &&
        DeadlineDue(now_us, next_position_deadline_us_))
    {
        position_sweep_active_ = true;
        position_sweep_count_ = 0U;
        position_sweep_node_ = position_sweep_start_node_;
    }
    if (position_sweep_active_)
    {
        if (NodeQuiet(position_sweep_node_, now_us))
        {
            step.action = CanDispatchAction::PositionRequest;
            step.node_id = position_sweep_node_;
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

    if (!step.transition)
        return;
    if (transition_ == Transition::HoldTargets)
    {
        if (transition_node_ < kActuatorNodeCount)
            transition_node_ = NextNode(transition_node_);
        else
            transition_ = mode_ == CanDispatchMode::Stream
                ? Transition::Enable : Transition::None;
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

} // namespace dummy::protocol
