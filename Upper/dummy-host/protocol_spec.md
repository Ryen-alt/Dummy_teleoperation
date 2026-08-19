# Dummy Host ↔ MCU Binary Protocol v1

All multi-byte values are little-endian. A wire frame is
`COBS(header || payload || crc32c) || 0x00`. CRC32C covers the unencoded header
and payload. The maximum decoded frame is 512 bytes.

## Header

```c
struct PacketHeader {       // 24 bytes, packed
    uint16_t magic;         // 0x4459
    uint8_t  version;       // 1
    uint8_t  message_type;
    uint16_t payload_length;
    uint16_t flags;
    uint32_t session_id;
    uint32_t sequence;
    uint64_t sender_time_us;
};
```

Message values: `HELLO=0x01`, `ACQUIRE_CONTROL=0x02`, `RELEASE_CONTROL=0x03`,
`SET_MODE=0x04`, `HEARTBEAT=0x05`, `SET_JOINT_TARGET=0x06`, `HOLD=0x07`,
`ESTOP=0x08`, `CLEAR_FAULT=0x09`, `HELLO_ACK=0x81`, `STATE=0x82`,
`ACK=0x83`, `NACK=0x84`, `FAULT=0x85`, `EVENT=0x86`.

Modes: `DISABLED=1`, `HOLD=2`, `TELEOP=3`, `POLICY=4`, `GRAVITY=5`,
`FAULT=6`. Training/policy code cannot send messages directly; it must pass
through the host safety layer.

## Payloads

- `HELLO`: `uint8 config_sha256[32]; uint32 capabilities`
- `HELLO_ACK`: `uint8 config_sha256[32]; uint32 capabilities; char firmware_version[32]`
- `ACQUIRE_CONTROL`: `uint32 lease_ms`
- `SET_MODE`: `uint8 mode`
- `SET_JOINT_TARGET`: `float target[7]; float max_velocity[6]; uint16 valid_for_ms; uint16 target_flags`
- `ACK/NACK`: `uint8 request_type; uint8 result; uint16 detail`
- `STATE`: `uint64 mcu_time_us; float position[7]; float velocity[7];`
  `uint32 last_received_sequence; uint32 last_applied_sequence; uint8 mode;`
  `uint8 validity; uint16 fault_bits; uint32 target_age_ms; uint8 config_sha256[32]`

`STATE.validity`: bit 0 joint position, bit 1 velocity, bit 2 gripper feedback.
`ACK.result=0` means accepted. NACK result codes are defined in the source enum.

The response packet repeats the request sequence. A new control acquisition uses
a new non-zero `session_id`. Targets are absolute `[joint1..joint6, gripper]`,
joint units are radians in the URDF joint convention, and the gripper is normalized
to `[0,1]`. Both `SET_JOINT_TARGET` and `STATE` use this same convention; the CAD/
URDF zero pose is therefore six joint zeros. The MCU keeps the historical firmware
angles internal and converts only at the binary boundary:

```text
q_urdf     = joint_sign * (q_firmware - joint_zero_offset_rad)
q_firmware = joint_zero_offset_rad + joint_sign * q_urdf
```

For configuration version 2 the firmware-zero vector is
`[0, -73°, 180°, 0, 0, 0]` and the sign vector is `[+1,+1,+1,-1,+1,-1]`.
ASCII maintenance commands remain historical firmware coordinates in degrees and
must not be mixed with binary/URDF values. Version-1 recordings must not be replayed
as version-2 targets; the configuration hash makes this mismatch explicit.
Until physical limit calibration is complete, the controller soft limits are the
intersection of the URDF limits and the historical firmware-safe range, expressed
in URDF coordinates. They may therefore be narrower than the mechanical URDF limits.

Targets are latest-wins. The MCU must reject wrong version/hash/session/mode,
non-increasing sequence, expired TTL, non-finite or out-of-limit values. Target
timeout and lease timeout transition to HOLD locally; Linux is not a hard-real-time
safety boundary.

## Keyboard/gamepad application boundary

Keyboard key codes, gamepad axes, button names, dead-man state and Episode keys
are Linux application data. They are never sent to the MCU as device-specific
messages. The host performs the following conversion at the configured 20 Hz:

1. evdev snapshot -> timestamped `TeleopCommand` containing six joint velocities,
   gripper velocity, dead-man, HOLD/ESTOP and source;
2. per-run joint/gripper allow-list -> acceleration-limited integration from the
   latest valid measured state;
3. common `DummyRobot` safety filter -> absolute float32 target and a velocity
   ceiling no greater than `robot_config.yaml`;
4. `SET_JOINT_TARGET` -> firmware latest-wins buffer -> 200 Hz executor;
5. STATE `last_received_sequence`/`last_applied_sequence` closes the link back to
   the recorded action sequence.

Dead-man release does not send a zero velocity and assume the robot will stop. The
host sends HOLD, releases the lease and resets the integrator; USB/process failure
is independently covered by target TTL and lease timeout in firmware. Raw input,
requested action, applied action and the later STATE acknowledgement are stored in
the Linux session and associated by sequence. A STATE packet is not required to
acknowledge the same action in the same 20 Hz row; delayed acknowledgement remains
traceable in subsequent rows.

