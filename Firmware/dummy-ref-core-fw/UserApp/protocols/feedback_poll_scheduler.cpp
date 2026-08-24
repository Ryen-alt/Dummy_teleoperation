#include "feedback_poll_scheduler.hpp"

namespace dummy::protocol
{
namespace
{
uint8_t NextNode(uint8_t node_id)
{
    return node_id >= kActuatorNodeCount ? 1U : static_cast<uint8_t>(node_id + 1U);
}
}

FeedbackPollScheduler::FeedbackPollScheduler(uint32_t temperature_slot_interval)
    : temperature_slot_interval_(temperature_slot_interval)
{
}

FeedbackPollRequest FeedbackPollScheduler::Next()
{
    const uint8_t actuator_node_id = next_actuator_node_;
    next_actuator_node_ = NextNode(next_actuator_node_);

    if (temperature_slot_interval_ != 0U &&
        ++slots_since_temperature_ >= temperature_slot_interval_)
    {
        slots_since_temperature_ = 0;
        const uint8_t node_id = next_temperature_node_;
        next_temperature_node_ = NextNode(next_temperature_node_);
        return {FeedbackPollKind::Temperature, node_id, actuator_node_id};
    }

    const uint8_t node_id = next_position_node_;
    next_position_node_ = NextNode(next_position_node_);
    return {FeedbackPollKind::Position, node_id, actuator_node_id};
}

void FeedbackPollScheduler::Reset()
{
    slots_since_temperature_ = 0;
    next_position_node_ = 1;
    next_temperature_node_ = 1;
    next_actuator_node_ = 1;
}

bool ActuatorApplicationTracker::RecordTransmission(uint32_t sequence, uint8_t node_id,
                                                    bool transmitted)
{
    if (!sequence_active_ || sequence != sequence_)
    {
        sequence_ = sequence;
        transmitted_nodes_ = 0U;
        sequence_active_ = true;
        completion_reported_ = false;
    }

    if (transmitted && node_id >= 1U && node_id <= kActuatorNodeCount)
        transmitted_nodes_ |= static_cast<uint8_t>(1U << (node_id - 1U));

    constexpr uint8_t kAllActuatorNodes =
        static_cast<uint8_t>((1U << kActuatorNodeCount) - 1U);
    if (transmitted_nodes_ == kAllActuatorNodes && !completion_reported_)
    {
        completion_reported_ = true;
        return true;
    }
    return false;
}

void ActuatorApplicationTracker::Reset()
{
    sequence_ = 0U;
    transmitted_nodes_ = 0U;
    sequence_active_ = false;
    completion_reported_ = false;
}

} // namespace dummy::protocol
