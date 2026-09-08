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

// Value and provenance committed together for one accepted position response
// (doc 05 sections 3.2 and 10.1). Old-protocol replies carry no request
// echo, so the sweep identity still comes from the monitor's pending-request
// association; a rejected response never seals a sample.
struct SealedJointSample
{
    float position = 0.0F;      // URDF rad for J1..J6, normalized for J7
    uint32_t received_us = 0U;  // main-controller monotonic clock
    uint32_t sweep_id = 0U;
    bool valid = false;
};

// Dispatcher-to-consumer liveness evidence. The dispatcher is the sole
// snapshot publisher; consumers compare the last successful publish time
// against their own clock (doc 05 section 10.3).
struct FeedbackRuntimeProgress
{
    uint32_t last_publish_us = 0U;
    uint32_t publish_failure_count = 0U;
};

// Firmware-only bridge around the pure C++ monitor. The CAN dispatcher is the
// sole writer; RX timestamps are captured in the ISR and parsed in task context.
void RecordPositionFeedbackRequest(uint8_t node_id, uint32_t sweep_id = 0U);
void RecordPositionTimingStart(uint8_t node_id, uint32_t completed_us);
bool RecordPositionFeedbackResponse(uint8_t node_id, uint32_t received_us);
// Seals the node's current actuator value together with the accepted
// response metadata. Must be called after the actuator update and the
// monitor acceptance in the same dispatch wake.
void SealJointPositionSample(uint8_t node_id, uint32_t received_us);
void RecordPositionFeedbackTimeout(uint8_t node_id);
void RecordFeedbackAdmissionOutcome(const CanDispatchStep& outcome);
void RecordTemperatureFeedbackRequest(uint8_t node_id);
void RecordTemperatureTimingStart(uint8_t node_id, uint32_t completed_us);
// Returns true only when the monitor accepted the response for the pending
// transaction; rejected responses must not commit shared realtime state.
bool RecordTemperatureFeedbackResponse(uint8_t node_id, float temperature_c,
                                       uint32_t received_us);
void RecordMotorTransportDiagnostics(uint8_t node_id, const uint8_t* data,
                                     uint32_t length);
void RecordTemperatureFeedbackTimeout(uint8_t node_id);
void RecordMotorDiagnosticsTimeout(uint8_t node_id);
bool RecordMotorTimingProfile(uint8_t node_id, const uint8_t* data,
                              uint32_t length);
bool AcceptMotorTimingProfile(uint8_t node_id, uint8_t page,
                              uint32_t received_us);
FeedbackResponseEvents ConsumeFeedbackResponseEvents();
void CancelPendingFeedbackRequests();
void PublishFeedbackSnapshot(uint32_t now_us);
std::array<NodeFeedbackStatus, kActuatorNodeCount> ReadCanFeedbackStatus(uint32_t now_us);
std::array<SealedJointSample, kActuatorNodeCount> ReadSealedJointSamples();
FeedbackRuntimeProgress ReadFeedbackRuntimeProgress();
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
void ResetCanTimingProfile(uint32_t session_epoch, uint64_t start_us,
                           const CanDispatchDiagnostics& scheduler);
void SetCanTimingProfileActive(bool active);
void SetCanTimingProfileEpochStable(bool stable);
void PublishCanTimingProfile(uint64_t now_us,
                             const CanDispatchDiagnostics& scheduler);
CanTimingProfilePayload ReadCanTimingProfile();

} // namespace dummy::protocol

#endif // DUMMY_FEEDBACK_RUNTIME_HPP
