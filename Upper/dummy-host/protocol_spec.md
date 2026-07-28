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
joint units are radians, and the gripper is normalized to `[0,1]`.

Targets are latest-wins. The MCU must reject wrong version/hash/session/mode,
non-increasing sequence, expired TTL, non-finite or out-of-limit values. Target
timeout and lease timeout transition to HOLD locally; Linux is not a hard-real-time
safety boundary.

