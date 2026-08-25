#include "can_feedback_monitor.hpp"

#include <algorithm>
#include <cmath>
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

uint32_t AgeMilliseconds(uint32_t now_us, uint32_t then_us)
{
    const uint32_t elapsed_us = now_us - then_us;
    constexpr uint32_t kHalfCounterRange = uint32_t{1} << 31U;

    // A CAN RX interrupt can publish a response timestamp just after a task
    // captured now_us but before that task snapshots the monitor. In modular
    // arithmetic that small future timestamp looks almost 2^32 us old. Treat
    // the negative half of the counter range as a concurrent future sample;
    // genuine 32-bit timer wrap still produces a small positive elapsed value.
    if (elapsed_us >= kHalfCounterRange)
        return 0U;
    return elapsed_us / 1000U;
}
}

template<typename Callback>
void CanFeedbackMonitor::ForRequestedNodes(uint8_t node_id, Callback callback)
{
    if (node_id == 0)
    {
        for (auto& node : nodes_)
            callback(node);
        return;
    }
    if (node_id >= 1 && node_id <= nodes_.size())
        callback(nodes_[node_id - 1]);
}

void CanFeedbackMonitor::OnPositionRequest(uint8_t node_id, uint32_t now_us)
{
    (void) now_us;
    if (node_id > kActuatorNodeCount)
        return;
    if (node_id == 0U)
    {
        if (current_sweep_request_mask_ != 0U)
        {
            ++current_sweep_id_;
            if (current_sweep_id_ == 0U)
                current_sweep_id_ = 1U;
        }
        current_sweep_request_mask_ = kAllNodeMask;
        ForRequestedNodes(node_id, [this](NodeState& node)
        {
            if (node.position_pending)
            {
                if (node.total_position_losses != std::numeric_limits<uint32_t>::max())
                    ++node.total_position_losses;
                if (node.consecutive_position_losses != std::numeric_limits<uint16_t>::max())
                    ++node.consecutive_position_losses;
            }
            node.position_pending = true;
            node.pending_sweep_id = current_sweep_id_;
        });
        return;
    }
    const uint8_t mask = NodeMask(node_id);
    if (current_sweep_request_mask_ == kAllNodeMask ||
        (current_sweep_request_mask_ & mask) != 0U)
    {
        ++current_sweep_id_;
        if (current_sweep_id_ == 0U)
            current_sweep_id_ = 1U;
        current_sweep_request_mask_ = 0U;
    }
    current_sweep_request_mask_ = static_cast<uint8_t>(
        current_sweep_request_mask_ | mask);
    ForRequestedNodes(node_id, [this](NodeState& node)
    {
        if (node.position_pending)
        {
            if (node.total_position_losses != std::numeric_limits<uint32_t>::max())
                ++node.total_position_losses;
            if (node.consecutive_position_losses != std::numeric_limits<uint16_t>::max())
                ++node.consecutive_position_losses;
        }
        node.position_pending = true;
        node.pending_sweep_id = current_sweep_id_;
    });
}

void CanFeedbackMonitor::OnPositionResponse(uint8_t node_id, uint32_t now_us)
{
    if (node_id < 1 || node_id > nodes_.size())
        return;
    NodeState& node = nodes_[node_id - 1];
    node.last_position_us = now_us;
    node.position_sweep_id = node.pending_sweep_id;
    node.position_seen = true;
    node.position_pending = false;
    node.consecutive_position_losses = 0;

    const uint32_t candidate_sweep = node.position_sweep_id;
    if (candidate_sweep == 0U)
        return;
    uint32_t newest_age_us = 0U;
    uint32_t oldest_age_us = 0U;
    bool first = true;
    for (const auto& candidate : nodes_)
    {
        if (!candidate.position_seen ||
            candidate.position_sweep_id != candidate_sweep)
            return;
        const uint32_t age_us = now_us - candidate.last_position_us;
        if (first)
        {
            newest_age_us = age_us;
            oldest_age_us = age_us;
            first = false;
        }
        else
        {
            newest_age_us = std::min(newest_age_us, age_us);
            oldest_age_us = std::max(oldest_age_us, age_us);
        }
    }
    coherent_.sweep_id = candidate_sweep;
    coherent_.max_skew_us = oldest_age_us - newest_age_us;
    coherent_.valid = coherent_.max_skew_us <= coherent_max_skew_us_;
    for (size_t index = 0; index < nodes_.size(); ++index)
    {
        coherent_.position_sample_us[index] = nodes_[index].last_position_us;
        coherent_.position_sweep_id[index] = nodes_[index].position_sweep_id;
    }
}

void CanFeedbackMonitor::OnPositionTimeout(uint8_t node_id)
{
    if (node_id < 1 || node_id > nodes_.size())
        return;
    NodeState& node = nodes_[node_id - 1];
    if (node.position_pending)
    {
        if (node.total_position_losses != std::numeric_limits<uint32_t>::max())
            ++node.total_position_losses;
        if (node.consecutive_position_losses != std::numeric_limits<uint16_t>::max())
            ++node.consecutive_position_losses;
    }
    node.position_pending = false;
}

void CanFeedbackMonitor::OnTemperatureRequest(uint8_t node_id, uint32_t now_us)
{
    (void) now_us;
    ForRequestedNodes(node_id, [](NodeState& node)
    {
        node.temperature_pending = true;
    });
}

void CanFeedbackMonitor::OnTemperatureResponse(uint8_t node_id, uint32_t now_us,
                                                float temperature_c)
{
    if (node_id < 1 || node_id > nodes_.size() || !std::isfinite(temperature_c))
        return;
    NodeState& node = nodes_[node_id - 1];
    node.last_temperature_us = now_us;
    node.temperature_c = temperature_c;
    node.temperature_seen = true;
    node.temperature_pending = false;
}

void CanFeedbackMonitor::OnTemperatureTimeout(uint8_t node_id)
{
    if (node_id < 1 || node_id > nodes_.size())
        return;
    nodes_[node_id - 1].temperature_pending = false;
}

std::array<NodeFeedbackStatus, kActuatorNodeCount>
CanFeedbackMonitor::Snapshot(uint32_t now_us) const
{
    std::array<NodeFeedbackStatus, kActuatorNodeCount> output{};
    for (size_t index = 0; index < nodes_.size(); ++index)
    {
        const NodeState& source = nodes_[index];
        NodeFeedbackStatus& target = output[index];
        target.position_seen = source.position_seen;
        target.temperature_seen = source.temperature_seen;
        target.position_age_ms = source.position_seen
            ? AgeMilliseconds(now_us, source.last_position_us) : kFeedbackAgeUnknown;
        target.temperature_age_ms = source.temperature_seen
            ? AgeMilliseconds(now_us, source.last_temperature_us) : kFeedbackAgeUnknown;
        target.total_position_losses = source.total_position_losses;
        target.consecutive_position_losses = source.consecutive_position_losses;
        target.temperature_c = source.temperature_c;
        target.position_sample_us = source.last_position_us;
        target.position_sweep_id = source.position_sweep_id;
    }
    return output;
}

CoherentFeedbackStatus CanFeedbackMonitor::CoherentSnapshot() const
{
    return coherent_;
}

void CanFeedbackMonitor::Reset()
{
    nodes_ = {};
    current_sweep_id_ = 1U;
    current_sweep_request_mask_ = 0U;
    coherent_ = {};
}

} // namespace dummy::protocol
