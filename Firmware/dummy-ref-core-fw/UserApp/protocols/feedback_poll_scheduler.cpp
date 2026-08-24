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
    if (temperature_slot_interval_ != 0U &&
        ++slots_since_temperature_ >= temperature_slot_interval_)
    {
        slots_since_temperature_ = 0;
        const uint8_t node_id = next_temperature_node_;
        next_temperature_node_ = NextNode(next_temperature_node_);
        return {FeedbackPollKind::Temperature, node_id};
    }

    const uint8_t node_id = next_position_node_;
    next_position_node_ = NextNode(next_position_node_);
    return {FeedbackPollKind::Position, node_id};
}

void FeedbackPollScheduler::Reset()
{
    slots_since_temperature_ = 0;
    next_position_node_ = 1;
    next_temperature_node_ = 1;
}

ActuatorCommandScheduler::ActuatorCommandScheduler(uint32_t control_tick_divisor)
    : control_tick_divisor_(control_tick_divisor == 0U ? 1U : control_tick_divisor)
{
}

bool ActuatorCommandScheduler::ShouldTransmit(bool command_valid)
{
    if (!command_valid)
    {
        Reset();
        return false;
    }

    if (ticks_until_transmit_ == 0U)
    {
        ticks_until_transmit_ = control_tick_divisor_ - 1U;
        return true;
    }

    --ticks_until_transmit_;
    return false;
}

void ActuatorCommandScheduler::Reset()
{
    ticks_until_transmit_ = 0U;
}

} // namespace dummy::protocol
