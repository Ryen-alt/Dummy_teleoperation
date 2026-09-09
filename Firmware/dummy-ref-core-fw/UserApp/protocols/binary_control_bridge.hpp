#ifndef DUMMY_BINARY_CONTROL_BRIDGE_HPP
#define DUMMY_BINARY_CONTROL_BRIDGE_HPP

#include "binary_protocol.hpp"
#include "external_target_executor.hpp"
#include "feedback_safety_supervisor.hpp"

#include <cstdint>

namespace dummy::protocol
{

struct BinaryControlSnapshot
{
    ExecutorTarget target{};
    uint32_t session_epoch = 0;
    ControlMode mode = ControlMode::Disabled;
    bool hello_valid = false;
    bool lease_active = false;
};

// These functions are implemented by the USB bridge. Access to the session is
// protected because USB parsing and the 200 Hz control task run concurrently.
uint64_t BinaryControlMonotonicMicros();
BinaryControlSnapshot ReadBinaryControlSnapshot(uint64_t now_us);
void RecordBinaryTargetCanQueuedExact(uint32_t sequence, uint64_t now_us,
                                      uint32_t coherent_sweep_id, uint32_t session_epoch);
void RecordBinaryTargetCanTxCompleteExact(uint32_t sequence, uint64_t now_us,
                                          uint32_t fanout_us, uint32_t session_epoch);
void RecordBinaryTargetAccepted(uint32_t sequence, uint64_t now_us, uint32_t session_epoch);
bool TryStartBinaryTargetDispatch(uint32_t sequence, uint32_t session_epoch);
void RecordBinaryTargetSuperseded(uint32_t sequence, uint64_t now_us, uint32_t session_epoch);
void RecordBinaryTargetPreemptedBySafety(uint32_t sequence, uint64_t now_us, uint32_t session_epoch);
void RecordBinaryTargetFailed(uint32_t sequence, uint64_t now_us, uint32_t session_epoch);
void RecordBinaryCoherentSweep(uint32_t coherent_sweep_id, uint64_t now_us,
                               uint64_t earliest_sample_us, uint32_t session_epoch);
void RequestBinaryRuntimeHold();
void ApplyBinarySafetyOutcome(const FeedbackSafetyOutput& safety);
FeedbackSafetyOutput ReadBinarySafetyTelemetry();
bool BinaryControlLeaseActive();

} // namespace dummy::protocol

#endif // DUMMY_BINARY_CONTROL_BRIDGE_HPP
