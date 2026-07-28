# Linux Host ↔ STM32 Binary Protocol v1

This document is the firmware copy of the contract frozen on `upper_computer`.
All multi-byte values are little-endian. A frame is
`COBS(header || payload || CRC32C) || 0x00`, with CRC32C Castagnoli covering the
decoded header and payload. Maximum decoded size is 512 bytes.

The packed 24-byte header is:

```c
uint16_t magic;          // 0x4459
uint8_t  version;        // 1
uint8_t  message_type;
uint16_t payload_length;
uint16_t flags;
uint32_t session_id;
uint32_t sequence;
uint64_t sender_time_us;
```

Messages are `HELLO=0x01`, `ACQUIRE_CONTROL=0x02`, `RELEASE_CONTROL=0x03`,
`SET_MODE=0x04`, `HEARTBEAT=0x05`, `SET_JOINT_TARGET=0x06`, `HOLD=0x07`,
`ESTOP=0x08`, `CLEAR_FAULT=0x09`, and responses `HELLO_ACK=0x81`, `STATE=0x82`,
`ACK=0x83`, `NACK=0x84`, `FAULT=0x85`, `EVENT=0x86`.

Payload layouts are defined with compile-time size checks in
`UserApp/protocols/binary_protocol.hpp`. The important sizes are header 24,
joint target 56, and state 112 bytes. Joint targets contain seven float32 absolute
positions (`joint1..joint6` radians, then normalized gripper), six float32 maximum
velocities, uint16 TTL in milliseconds, and uint16 flags.

Control requires a matching 32-byte configuration hash, a fresh non-zero session,
an exclusive bounded lease, TELEOP/POLICY mode, increasing command sequence and a
locally unexpired target. Targets use latest-wins storage. Lease or target timeout
enters HOLD. ESTOP is accepted regardless of session and latches FAULT.

The generated configuration currently has
`kHardwareParametersVerified=false` and `kExternalTargetExecutionReady=false`.
Consequently production acquisition is
rejected until direction, zero, reduction, limits and gripper mapping are verified
on a mechanically supported, low-speed robot, a new config hash is generated, and
the 200 Hz latest-wins target consumer has been reviewed and enabled.

The pure C++ host tests are intentionally separate from STM32 HAL:

```bash
cmake -S tests/host_protocol -B build-host-protocol
cmake --build build-host-protocol
ctest --test-dir build-host-protocol --output-on-failure
```
