#ifndef DUMMY_BINARY_CONTROL_BRIDGE_HPP
#define DUMMY_BINARY_CONTROL_BRIDGE_HPP

#include "binary_protocol.hpp"
#include "external_target_executor.hpp"

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
void MarkBinaryTargetApplied(uint32_t sequence);
bool BinaryControlLeaseActive();

} // namespace dummy::protocol

#endif // DUMMY_BINARY_CONTROL_BRIDGE_HPP
