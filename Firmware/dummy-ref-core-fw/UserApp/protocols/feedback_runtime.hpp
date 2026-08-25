#ifndef DUMMY_FEEDBACK_RUNTIME_HPP
#define DUMMY_FEEDBACK_RUNTIME_HPP

#include "can_feedback_monitor.hpp"
#include "feedback_poll_scheduler.hpp"
#include "binary_protocol.hpp"

#include <array>
#include <cstdint>

namespace dummy::protocol
{

// Firmware-only bridge around the pure C++ monitor. Request functions run in
// task context; response functions are safe to call from the CAN RX interrupt.
void RecordPositionFeedbackRequest(uint8_t node_id);
void RecordPositionFeedbackResponse(uint8_t node_id);
void RecordPositionFeedbackTimeout(uint8_t node_id);
void RecordTemperatureFeedbackRequest(uint8_t node_id);
void RecordTemperatureFeedbackResponse(uint8_t node_id, float temperature_c);
void RecordTemperatureFeedbackTimeout(uint8_t node_id);
FeedbackResponseEvents ConsumeFeedbackResponseEvents();
std::array<NodeFeedbackStatus, kActuatorNodeCount> ReadCanFeedbackStatus(uint32_t now_us);
CoherentFeedbackStatus ReadCoherentFeedbackStatus();

enum CanRuntimeStatusBits : uint8_t
{
    kCanRuntimeDispatcherAlive = 1U << 0U,
    kCanRuntimeTxQueued = 1U << 1U,
    kCanRuntimePositionRequested = 1U << 2U,
    kCanRuntimePositionResponded = 1U << 3U,
    kCanRuntimeTxDeferred = 1U << 4U,
    kCanRuntimeQueryPending = 1U << 5U,
    kCanRuntimeFeedbackReady = 1U << 6U,
    kCanRuntimeTxRecovered = 1U << 7U,
};

void PublishCanRuntimeStatus(uint8_t status);
uint8_t ReadCanRuntimeStatus();
void PublishCanFeedbackReady(bool ready);
bool ReadCanFeedbackReady();
void PublishCanDiagnostics(const CanDiagnosticsPayload& diagnostics);
CanDiagnosticsPayload ReadCanDiagnostics();

} // namespace dummy::protocol

#endif // DUMMY_FEEDBACK_RUNTIME_HPP
