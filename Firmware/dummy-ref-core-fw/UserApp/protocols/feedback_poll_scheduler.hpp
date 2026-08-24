#ifndef DUMMY_FEEDBACK_POLL_SCHEDULER_HPP
#define DUMMY_FEEDBACK_POLL_SCHEDULER_HPP

#include "can_feedback_monitor.hpp"

#include <cstdint>

namespace dummy::protocol
{

enum class FeedbackPollKind : uint8_t
{
    Position,
    Temperature,
};

struct FeedbackPollRequest
{
    FeedbackPollKind kind = FeedbackPollKind::Position;
    uint8_t node_id = 1;
};

// Produces exactly one unicast CAN request per timer slot. Position and
// temperature cursors advance independently so a temperature sample never
// creates a second response burst in a position slot.
class FeedbackPollScheduler
{
public:
    explicit FeedbackPollScheduler(uint32_t temperature_slot_interval);

    FeedbackPollRequest Next();
    void Reset();

private:
    uint32_t temperature_slot_interval_ = 0;
    uint32_t slots_since_temperature_ = 0;
    uint8_t next_position_node_ = 1;
    uint8_t next_temperature_node_ = 1;
};

} // namespace dummy::protocol

#endif // DUMMY_FEEDBACK_POLL_SCHEDULER_HPP
