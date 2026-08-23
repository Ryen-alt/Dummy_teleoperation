#include "common_inc.h"
#include "feedback_runtime.hpp"

namespace dummy::protocol
{
namespace
{
CanFeedbackMonitor feedback_monitor;
}

void RecordPositionFeedbackRequest(uint8_t node_id)
{
    taskENTER_CRITICAL();
    feedback_monitor.OnPositionRequest(node_id, micros());
    taskEXIT_CRITICAL();
}

void RecordPositionFeedbackResponse(uint8_t node_id)
{
    // Called from the CAN RX ISR. Normal task code masks interrupts while it
    // mutates or snapshots the monitor.
    feedback_monitor.OnPositionResponse(node_id, micros());
}

void RecordTemperatureFeedbackRequest(uint8_t node_id)
{
    taskENTER_CRITICAL();
    feedback_monitor.OnTemperatureRequest(node_id, micros());
    taskEXIT_CRITICAL();
}

void RecordTemperatureFeedbackResponse(uint8_t node_id, float temperature_c)
{
    feedback_monitor.OnTemperatureResponse(node_id, micros(), temperature_c);
}

std::array<NodeFeedbackStatus, kActuatorNodeCount> ReadCanFeedbackStatus(uint32_t now_us)
{
    taskENTER_CRITICAL();
    const auto snapshot = feedback_monitor.Snapshot(now_us);
    taskEXIT_CRITICAL();
    return snapshot;
}

} // namespace dummy::protocol
