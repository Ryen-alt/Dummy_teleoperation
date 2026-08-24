#ifndef DUMMY_FEEDBACK_POLL_SCHEDULER_HPP
#define DUMMY_FEEDBACK_POLL_SCHEDULER_HPP

#include "can_feedback_monitor.hpp"

#include <cstdint>

namespace dummy::protocol
{

enum class CanSlotKind : uint8_t
{
    ActuatorTarget,
    PositionFeedback,
    TemperatureFeedback,
};

struct CanSlotRequest
{
    CanSlotKind kind = CanSlotKind::ActuatorTarget;
    uint8_t node_id = 1;
};

// Produces at most one outbound CAN frame per 700 Hz timer slot. Target and
// feedback slots alternate. Feedback is phase-shifted by three nodes so a
// motor never receives its target and query in adjacent slots.
class CanSlotScheduler
{
public:
    explicit CanSlotScheduler(uint32_t temperature_feedback_slot_interval);

    CanSlotRequest Next();
    void Reset();

private:
    uint32_t temperature_feedback_slot_interval_ = 0;
    uint32_t feedback_slots_since_temperature_ = 0;
    uint8_t next_target_node_ = 1;
    uint8_t current_pair_target_node_ = 1;
    bool target_slot_next_ = true;
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
