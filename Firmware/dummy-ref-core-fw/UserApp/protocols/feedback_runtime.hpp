#ifndef DUMMY_FEEDBACK_RUNTIME_HPP
#define DUMMY_FEEDBACK_RUNTIME_HPP

#include "can_feedback_monitor.hpp"

#include <array>
#include <cstdint>

namespace dummy::protocol
{

// Firmware-only bridge around the pure C++ monitor. Request functions run in
// task context; response functions are safe to call from the CAN RX interrupt.
void RecordPositionFeedbackRequest(uint8_t node_id);
void RecordPositionFeedbackResponse(uint8_t node_id);
void RecordTemperatureFeedbackRequest(uint8_t node_id);
void RecordTemperatureFeedbackResponse(uint8_t node_id, float temperature_c);
std::array<NodeFeedbackStatus, kActuatorNodeCount> ReadCanFeedbackStatus(uint32_t now_us);

} // namespace dummy::protocol

#endif // DUMMY_FEEDBACK_RUNTIME_HPP
