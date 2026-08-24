#include "feedback_poll_scheduler.hpp"

namespace dummy::protocol
{
namespace
{
uint8_t NextNode(uint8_t node_id)
{
    return node_id >= kActuatorNodeCount ? 1U : static_cast<uint8_t>(node_id + 1U);
}

uint8_t OffsetNode(uint8_t node_id, uint8_t offset)
{
    return static_cast<uint8_t>(
        ((static_cast<uint32_t>(node_id) - 1U + offset) % kActuatorNodeCount) + 1U);
}
}

CanSlotScheduler::CanSlotScheduler(uint32_t temperature_feedback_slot_interval)
    : temperature_feedback_slot_interval_(temperature_feedback_slot_interval)
{
}

CanSlotRequest CanSlotScheduler::Next()
{
    if (target_slot_next_)
    {
        target_slot_next_ = false;
        current_pair_target_node_ = next_target_node_;
        next_target_node_ = NextNode(next_target_node_);
        return {CanSlotKind::ActuatorTarget, current_pair_target_node_};
    }

    target_slot_next_ = true;
    constexpr uint8_t kFeedbackNodePhaseOffset = 3U;
    const uint8_t feedback_node = OffsetNode(
        current_pair_target_node_, kFeedbackNodePhaseOffset);
    if (temperature_feedback_slot_interval_ != 0U &&
        ++feedback_slots_since_temperature_ >= temperature_feedback_slot_interval_)
    {
        feedback_slots_since_temperature_ = 0U;
        return {CanSlotKind::TemperatureFeedback, feedback_node};
    }
    return {CanSlotKind::PositionFeedback, feedback_node};
}

void CanSlotScheduler::Reset()
{
    feedback_slots_since_temperature_ = 0U;
    next_target_node_ = 1U;
    current_pair_target_node_ = 1U;
    target_slot_next_ = true;
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
