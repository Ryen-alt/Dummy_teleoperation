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
    // Independent round-robin cursor used for one streaming target per CAN
    // slot. It advances even when a temperature request replaces position.
    uint8_t actuator_node_id = 1;
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
    uint8_t next_actuator_node_ = 1;
};

// Tracks when all seven actuator nodes have accepted at least one CAN frame for
// a host target sequence. Failed mailbox admissions remain pending and are
// retried when that node's next slot arrives.
class ActuatorApplicationTracker
{
public:
    bool RecordTransmission(uint32_t sequence, uint8_t node_id, bool transmitted);
    void Reset();

private:
    uint32_t sequence_ = 0;
    uint8_t transmitted_nodes_ = 0;
    bool sequence_active_ = false;
    bool completion_reported_ = false;
};

} // namespace dummy::protocol

#endif // DUMMY_FEEDBACK_POLL_SCHEDULER_HPP
