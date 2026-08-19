# Linux Host ↔ STM32 Binary Protocol v1

This document is the firmware copy of the host/MCU contract used by the current workspace.
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

All six binary joint positions, in both `SET_JOINT_TARGET` and `STATE`, use the
URDF joint convention. The URDF/CAD zero pose is `[0,0,0,0,0,0]`. Version 2 maps
the historical degree-based firmware coordinates only at the binary boundary:

```text
q_urdf     = joint_sign * (q_firmware - joint_zero_offset_rad)
q_firmware = joint_zero_offset_rad + joint_sign * q_urdf
joint_zero_offset_rad = [0, -73°, 180°, 0, 0, 0]
joint_sign            = [+1,+1,+1,-1,+1,-1]
```

The 200 Hz executor and binary limit checks operate in URDF coordinates. Legacy
ASCII maintenance commands still use firmware coordinates in degrees. They are a
separate diagnostic interface and their values must not be copied directly into a
binary target. The configuration version/hash prevents version-1 sessions from
being silently interpreted using the new mapping.
Before physical limit calibration, generated controller limits use the intersection
of the URDF domain and the historical firmware-safe range, still expressed in URDF
coordinates. This prevents `SetAngle` from receiving the wider unverified J1/J2/J3
URDF endpoints.

Control requires a matching 32-byte configuration hash, a fresh non-zero session,
an exclusive bounded lease, TELEOP/POLICY mode, increasing command sequence and a
locally unexpired target. Targets use latest-wins storage. Lease or target timeout
enters HOLD. ESTOP is accepted regardless of session and latches FAULT.

The generated configuration records the operator-confirmed calibration baseline in
`kRobotCalibrationId`. Production acquisition is enabled only when both
`kHardwareParametersVerified` and `kExternalTargetExecutionReady` are true and the
host presents the exact matching canonical configuration hash.

The target consumer is now wired into the 200 Hz task. USB processing validates
and publishes only the newest target; the control task copies a protected snapshot,
applies velocity/acceleration limiting, writes the motor targets, and only then
updates `last_applied_sequence`. TTL/lease expiry enters HOLD, and ASCII maintenance
commands are dropped while a binary lease is active. These paths are covered by
pure C++ tests and cross-compilation. The execution gate must stay false until the
motor-off timing/concurrency checks and the mechanically supported real-machine
tests in the calibration guide pass.

Keyboard and gamepad formats deliberately do not exist in this firmware protocol.
Linux records the raw evdev state, applies the per-run joint/gripper allow-list,
integrates velocity input and sends only the common absolute float32[7] target,
six velocity ceilings, TTL, session and sequence. Consequently a controller model
or key mapping can change without changing firmware. The firmware acknowledgement
contract for data collection is the target ACK followed by STATE
`last_received_sequence` and `last_applied_sequence`; a delayed STATE is joined to
the host record by sequence rather than assumed to belong to the current row.
While a control lease is active, STATE uses that lease's session ID. Once the lease
is released or expires, STATE uses the most recently validated HELLO session ID so
a new read-only client or reconnecting host does not discard telemetry as stale.

The pure C++ host tests are intentionally separate from STM32 HAL:

```bash
cmake -S tests/host_protocol -B build-host-protocol
cmake --build build-host-protocol
ctest --test-dir build-host-protocol --output-on-failure
```
