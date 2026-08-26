#ifndef DUMMY_FEEDBACK_RUNTIME_HPP
#define DUMMY_FEEDBACK_RUNTIME_HPP

#include "can_feedback_monitor.hpp"
#include "feedback_poll_scheduler.hpp"
#include "binary_protocol.hpp"

#include <array>
#include <cstdint>

namespace dummy::protocol
{

struct MotorTransportDiagnostics
{
    uint8_t valid_mask = 0U;
    std::array<uint8_t, kActuatorNodeCount> tx_drop{};
    std::array<uint8_t, kActuatorNodeCount> rx_error{};
    std::array<uint8_t, kActuatorNodeCount> busoff{};
};

// Firmware-only bridge around the pure C++ monitor. The CAN dispatcher is the
// sole writer; RX timestamps are captured in the ISR and parsed in task context.
void RecordPositionFeedbackRequest(uint8_t node_id, uint32_t sweep_id = 0U);
bool RecordPositionFeedbackResponse(uint8_t node_id, uint32_t received_us);
void RecordPositionFeedbackTimeout(uint8_t node_id);
void RecordTemperatureFeedbackRequest(uint8_t node_id);
void RecordTemperatureFeedbackResponse(uint8_t node_id, float temperature_c,
                                       uint32_t received_us);
void RecordMotorTransportDiagnostics(uint8_t node_id, const uint8_t* data,
                                     uint32_t length);
void RecordTemperatureFeedbackTimeout(uint8_t node_id);
FeedbackResponseEvents ConsumeFeedbackResponseEvents();
void CancelPendingFeedbackRequests();
void PublishFeedbackSnapshot(uint32_t now_us);
std::array<NodeFeedbackStatus, kActuatorNodeCount> ReadCanFeedbackStatus(uint32_t now_us);
CoherentFeedbackStatus ReadCoherentFeedbackStatus();
MotorTransportDiagnostics ReadMotorTransportDiagnostics();
void ResetMotorTransportDiagnostics();

enum CanRuntimeStatusBits : uint8_t
{
    kCanRuntimeDispatcherAlive = 1U << 0U,
    kCanRuntimeTxQueued = 1U << 1U,
    kCanRuntimePositionRequested = 1U << 2U,
    kCanRuntimePositionResponded = 1U << 3U,
    kCanRuntimeTxDeferred = 1U << 4U,
    kCanRuntimeQueryPending = 1U << 5U,
    kCanRuntimeFeedbackReady = 1U << 6U,
    kCanRuntimeDegraded = 1U << 7U,
};

void PublishCanRuntimeStatus(uint8_t status);
uint8_t ReadCanRuntimeStatus();
void PublishCanFeedbackReady(bool ready);
bool ReadCanFeedbackReady();
void PublishCanDiagnostics(const CanDiagnosticsPayload& diagnostics);
CanDiagnosticsPayload ReadCanDiagnostics();

} // namespace dummy::protocol

#endif // DUMMY_FEEDBACK_RUNTIME_HPP
