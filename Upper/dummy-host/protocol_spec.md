# Dummy Host ↔ MCU Binary Protocol v4

All multi-byte values are little-endian. A wire frame is
`COBS(header || payload || crc32c) || 0x00`. CRC32C covers the unencoded header
and payload. The maximum decoded frame is 512 bytes. Protocol v4 production
peers must report firmware version `dummy-ref-v2.1`; the host rejects every
other real-firmware version even when the configuration hash matches.

## Header

```c
struct PacketHeader {       // 24 bytes, packed
    uint16_t magic;         // 0x4459
    uint8_t  version;       // 4
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
- `ACTION_PROGRESS EVENT`（20 bytes）: `uint32 action_sequence; uint8 stage;`
  `uint8 reserved[3]; uint64 stage_time_us; uint32 feedback_sweep_id`
- `STATE`（484 bytes）: `uint64 mcu_time_us; float position[7]; float velocity[7];`
  `uint32 last_received_sequence; uint8 mode;`
  `uint8 validity; uint16 fault_bits; uint32 target_age_ms; uint8 config_sha256[32];`
  `float following_error[7]; uint32 following_error_duration_ms[7];`
  `uint32 feedback_age_ms[7]; uint32 feedback_loss_count[7];`
  `uint16 consecutive_feedback_loss[7]; uint16 node_fault_bits[7];`
  `uint8 node_validity[7]; uint8 can_transport_status; uint16 hold_reason_bits;`
  `uint16 telemetry_validity; uint64 feedback_sample_mcu_us[7];`
  `uint32 feedback_sweep_id[7]; uint32 coherent_sweep_id;`
  `uint32 feedback_max_skew_us; uint64 coherent_reference_mcu_us;`
  `uint8 state_flags; uint8 action_progress_count; uint8 action_progress_head;`
  `uint8 progress_reserved; ActionProgressRecord action_progress[6]`

`ActionProgressRecord` is a compact 20-byte replay entry containing sequence,
flags, the low 32 bits of CAN/post-feedback MCU times and the feedback sweep ID.
The host extends those low timestamps against `STATE.mcu_time_us`. Flags are
`CAN_QUEUED_EXACT=0x01`, `POST_COMMAND_FEEDBACK=0x02` and
`SUPERSEDED=0x04`. `state_flags & 0x01` means this 50 Hz STATE repeats the
previous coherent sweep and therefore reuses its velocity estimate.

`STATE.validity`: bit 0 joint position, bit 1 velocity, bit 2 gripper feedback.
`ACK.result=0` means accepted. NACK result codes are defined in the source enum.

`hold_reason_bits`: target TTL `0x0001`, lease `0x0002`, following error `0x0004`,
CAN feedback stale `0x0008`, operator HOLD `0x0010`, runtime limiter `0x0020`.
`fault_bits`: ESTOP `0x0001`, persistent feedback loss `0x0002`, over-temperature
`0x0004`; encoder/stall/over-current reserve `0x0008/0x0010/0x0020`.

`node_validity` makes source availability explicit: bit 0 position and bit 1
temperature. Bits 2/3/4 are encoder-fault/stall/current source availability. The
current CtrlStep CAN response does not expose those three sources, so firmware must
leave both their validity and fault bits clear rather than infer them from following
error. `feedback_age_ms=0xffffffff` means that node has never supplied position.

The firmware computes following error against the 200 Hz acceleration-limited
command, not the farther-away Linux input target. A persisted following error or
short feedback outage requests HOLD. Feedback loss beyond the configured severe
threshold and persisted measured over-temperature request a latched FAULT.

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

For configuration version 7 the firmware-zero vector is
`[0, -73°, 180°, 0, 0, 0]` and the sign vector is `[+1,+1,+1,-1,+1,-1]`.
ASCII maintenance commands remain historical firmware coordinates in degrees and
must not be mixed with binary/URDF values. Older recordings must not be replayed
as version-7 targets; the configuration hash makes this mismatch explicit.
Until physical limit calibration is complete, the controller soft limits are the
intersection of the URDF limits and the historical firmware-safe range, expressed
in URDF coordinates. They may therefore be narrower than the mechanical URDF limits.

Protocol v4 and older protocol versions must not share a control session. Targets
use uint32 serial-number ordering across wrap. The MCU must reject wrong
version/hash/session/mode, non-increasing sequence, expired TTL, non-finite or
out-of-limit values. Target
timeout and lease timeout transition to HOLD locally; Linux is not a hard-real-time
safety boundary.

The v2.1 production CAN plan runs a 700 Hz dispatcher: target writes are
50 Hz/node, position queries 40 Hz/node and temperature queries 1 Hz/node. This
is 637 MCU-scheduled frames/s and about 924 total frames/s including responses.
Only one query may be outstanding, missed deadlines are rebased without burst
replay, and coherent sweeps are valid only when seven-node skew is at most 30 ms.

An action advances only through exact evidence:
`RECEIVED → SAFETY_ACCEPTED → SEND_ENQUEUED → SERIAL_SEND_STARTED →`
`SERIAL_SEND_FINISHED → ACKNOWLEDGED → CAN_QUEUED_EXACT →`
`POST_COMMAND_FEEDBACK`. `CAN_QUEUED_EXACT` means that exact sequence was queued
for all seven actuator nodes. `POST_COMMAND_FEEDBACK` means a later complete
coherent sweep was received; it does not mean the target was reached. Firmware
emits fast EVENT notifications and repeats the latest six consolidated progress
records in STATE. Host and exporter de-duplicate strictly by equal sequence—newer
sequences never imply completion of older ones.

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
5. ACK, exact CAN progress and post-command coherent feedback close the link back
   to the recorded action sequence.

Dead-man release does not send a zero velocity and assume the robot will stop. The
host sends priority HOLD, releases the lease and resets the integrator; USB/process failure
is independently covered by target TTL and lease timeout in firmware. Raw input,
requested action, applied action and exact lifecycle timestamps are stored in Raw
Session schema v4 and associated by sequence. Schema v3 remains inspectable, but
strict v2.1 export refuses it because it cannot reconstruct missing exact-sequence
CAN and post-feedback evidence.

The host serial writer uses separate safety and reliable-control FIFOs plus a
single latest-target mailbox. ESTOP is first, then HOLD/RELEASE/SET_MODE,
reliable session traffic, and finally the current target. HOLD/ESTOP atomically
clear the motion mailbox. On receive, ACK/NACK/EVENT/FAULT remain reliable while
STATE uses a replaceable latest-value slot; reliable-queue overflow is fatal.
