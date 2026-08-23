#include "can_feedback_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dummy::protocol
{
namespace
{
uint32_t AgeMilliseconds(uint32_t now_us, uint32_t then_us)
{
    return (now_us - then_us) / 1000U;
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
    ForRequestedNodes(node_id, [](NodeState& node)
    {
        if (node.position_pending)
        {
            if (node.total_position_losses != std::numeric_limits<uint32_t>::max())
                ++node.total_position_losses;
            if (node.consecutive_position_losses != std::numeric_limits<uint16_t>::max())
                ++node.consecutive_position_losses;
        }
        node.position_pending = true;
    });
}

void CanFeedbackMonitor::OnPositionResponse(uint8_t node_id, uint32_t now_us)
{
    if (node_id < 1 || node_id > nodes_.size())
        return;
    NodeState& node = nodes_[node_id - 1];
    node.last_position_us = now_us;
    node.position_seen = true;
    node.position_pending = false;
    node.consecutive_position_losses = 0;
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
    }
    return output;
}

void CanFeedbackMonitor::Reset()
{
    nodes_ = {};
}

} // namespace dummy::protocol
