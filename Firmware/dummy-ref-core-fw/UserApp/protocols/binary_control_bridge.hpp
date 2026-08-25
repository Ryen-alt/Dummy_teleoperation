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
    ControlMode mode = ControlMode::Disabled;
    bool hello_valid = false;
    bool lease_active = false;
};

// These functions are implemented by the USB bridge. Access to the session is
// protected because USB parsing and the 200 Hz control task run concurrently.
uint64_t BinaryControlMonotonicMicros();
BinaryControlSnapshot ReadBinaryControlSnapshot(uint64_t now_us);
void RecordBinaryTargetCanQueuedExact(uint32_t sequence, uint64_t now_us,
                                      uint32_t coherent_sweep_id);
void RecordBinaryTargetAccepted(uint32_t sequence, uint64_t now_us);
bool TryStartBinaryTargetDispatch(uint32_t sequence);
void RecordBinaryTargetSuperseded(uint32_t sequence, uint64_t now_us);
void RecordBinaryCoherentSweep(uint32_t coherent_sweep_id, uint64_t now_us);
void ApplyBinarySafetyOutcome(const FeedbackSafetyOutput& safety);
FeedbackSafetyOutput ReadBinarySafetyTelemetry();
bool BinaryControlLeaseActive();

} // namespace dummy::protocol

#endif // DUMMY_BINARY_CONTROL_BRIDGE_HPP
