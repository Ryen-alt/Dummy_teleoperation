# Dummy Host ↔ MCU Binary Protocol v5

All multi-byte values are little-endian. A wire frame is
`COBS(header || payload || crc32c) || 0x00`; CRC32C covers the decoded header
and payload. The maximum decoded frame is 576 bytes. A real peer must report
firmware `dummy-ref-v2.2.2`; v4/v5 peers never share a control session, and a
v2.2 peer is rejected even though it also speaks protocol v5.

## Header and epoch

```c
struct PacketHeader {       // 24 bytes, packed
    uint16_t magic;         // 0x4459
    uint8_t  version;       // 5
    uint8_t  message_type;
    uint16_t payload_length;
    uint16_t flags;
    uint32_t session_id;    // non-zero session_epoch
    uint32_t sequence;
    uint64_t sender_time_us;
};
```

Every connection creates a random non-zero `session_id`, used end-to-end as the
`session_epoch`. HELLO clears stale firmware progress and sequence replay state.
ACQUIRE activates the epoch but does not reset sequence when repeated within the
same epoch. Sequence comparison is uint32 modular ordering and skips zero.

Message values are:

- Requests: `HELLO=0x01`, `ACQUIRE_CONTROL=0x02`, `RELEASE_CONTROL=0x03`,
  `SET_MODE=0x04`, `HEARTBEAT=0x05`, `SET_JOINT_TARGET=0x06`, `HOLD=0x07`,
  `ESTOP=0x08`, `CLEAR_FAULT=0x09`, `TARGET_KEEPALIVE=0x0a`,
  `TIME_SYNC=0x0b`, `GET_CAN_DIAGNOSTICS=0x0c`,
  `GET_CAN_TIMING_PROFILE=0x0d`.
- Responses/telemetry: `HELLO_ACK=0x81`, `STATE=0x82`, `ACK=0x83`,
  `NACK=0x84`, `FAULT=0x85`, `EVENT=0x86`, `TIME_SYNC_ACK=0x87`,
  `CAN_DIAGNOSTICS=0x88`, `CAN_TIMING_PROFILE=0x89`.

Modes are `DISABLED=1`, `HOLD=2`, `TELEOP=3`, `POLICY=4`, `GRAVITY=5`,
`FAULT=6`. Policy code cannot write protocol packets directly; all actions pass
through the host ActionGateway and SafetyFilter.

## Capabilities

HELLO and HELLO_ACK carry a uint32 capability mask. The v5 host requires:

```text
MULTI_CHANNEL_SEQUENCE     0x00000001
TARGET_KEEPALIVE           0x00000002
CAN_TX_COMPLETE_EXACT      0x00000004
CONTROL_FRESHNESS_TOKEN    0x00000008
TIME_SYNC                  0x00000010
CAN_DIAGNOSTICS            0x00000020
CAN_DIAGNOSTICS_V2         0x00000040
CAN_TIMING_PROFILE         0x00000080
```

## Control payloads

- `HELLO`: `uint8 config_sha256[32]; uint32 capabilities`
- `HELLO_ACK`: `uint8 config_sha256[32]; uint32 capabilities; char firmware_version[32]`
- `ACQUIRE_CONTROL`: `uint32 lease_ms`
- `SET_MODE`: `uint8 mode`
- `SET_JOINT_TARGET` (58 bytes): `float target[7]; float max_velocity[6];`
  `uint16 valid_for_ms; uint16 target_flags; uint32 control_tick_id`
- `TARGET_KEEPALIVE`: `uint32 action_sequence; uint32 control_tick_id`
- `ACK/NACK`: `uint8 request_type; uint8 result; uint16 detail`
- `TIME_SYNC`: `uint64 host_t0_ns`
- `TIME_SYNC_ACK`: `uint64 host_t0_ns; uint64 mcu_rx_us; uint64 mcu_tx_us`

Targets are absolute `[joint1..joint6, gripper]`. Joint values use URDF radians;
the gripper is normalized to `[0,1]`. The firmware keeps historical motor angles
internal and converts only at the binary boundary:

```text
q_urdf     = joint_sign * (q_firmware - joint_zero_offset_rad)
q_firmware = joint_zero_offset_rad + joint_sign * q_urdf
```

`HEARTBEAT` extends only the 500 ms lease. Each healthy 20 Hz host control tick
creates a new non-zero `control_tick_id`. A new target consumes that token;
TARGET_KEEPALIVE may consume a later token exactly once for the named active
action. Duplicate, backward, stale-epoch or wrong-action tokens are rejected.
The 200 ms target TTL therefore still enters HOLD if the control loop stalls,
even when the independent lease heartbeat remains healthy.

## State and action progress

`STATE` is 508 bytes and the complete decoded frame is 536 bytes. Its fixed
fields contain MCU time, seven positions and velocities, mode/fault/validity,
configuration hash, following-error and per-node feedback diagnostics, coherent
sweep identity/reference time, repeated-state flag, and six progress records.

Each 24-byte `ActionProgressRecord` contains:

```c
uint32_t action_sequence;
uint8_t  flags;
uint8_t  reserved[3];
uint32_t can_queued_mcu_us_low;
uint32_t can_tx_complete_mcu_us_low;
uint32_t post_feedback_mcu_us_low;
uint32_t feedback_sweep_id;
```

The host extends low timestamps against `STATE.mcu_time_us`. Flags are
`CAN_QUEUED_EXACT=0x01`, `CAN_TX_COMPLETE_EXACT=0x02`,
`POST_COMMAND_FEEDBACK=0x04`, `SUPERSEDED=0x08`,
`PREEMPTED_BY_SAFETY=0x10`, and `FAILED=0x20`.

EVENT uses the full 20-byte action-progress payload:
`uint32 sequence; uint8 stage; uint8 reserved[3]; uint64 stage_time_us;`
`uint32 stage_value`. For `CAN_TX_COMPLETE_EXACT`, `stage_value` is the measured
first-enqueue→last-TX-complete fanout duration in microseconds; for queued/post-
feedback stages it remains the coherent sweep identifier, and otherwise is zero.
EVENT is the fast path; STATE replay is the loss recovery path. If STATE replay
arrives first without the fanout measurement, the later reliable EVENT enriches
that lifecycle exactly once.

The monotonic action lifecycle is:

```text
RECEIVED → SAFETY_ACCEPTED → SEND_ENQUEUED → SERIAL_SEND_STARTED →
SERIAL_SEND_FINISHED → ACKNOWLEDGED → CAN_QUEUED_EXACT →
CAN_TX_COMPLETE_EXACT → POST_COMMAND_FEEDBACK
```

`CAN_QUEUED_EXACT` means all seven frozen-generation frames entered the hardware
send path. `CAN_TX_COMPLETE_EXACT` means all seven completed transmission.
`POST_COMMAND_FEEDBACK` requires a complete coherent seven-node sweep whose node
sample times are all later than the last TX completion; it is execution-latency
evidence, not proof that the target pose was reached. Exact completion forbids a
later SUPERSEDED result. Abort/error, ACK timeout, or feedback timeout produces
FAILED and a safety HOLD.

`state_flags & 0x01` marks a repeated STATE that reuses the last coherent velocity.
A coherent sweep is published only when all seven nodes responded and skew is at
most 30 ms. The coherent reference time is the midpoint of the earliest/latest
node sample times.

## CAN diagnostics and scheduling

`CAN_DIAGNOSTICS` is fixed at 380 bytes and starts with
`format_version=2`, `payload_size=380`. The host rejects the legacy 132-byte
payload and requires both diagnostics capability bits.

`CAN_TIMING_PROFILE` is a separate fixed 520-byte v1 payload. It contains the
active Stream epoch/window, four 0x26 page masks, seven-node position and
temperature TX-complete→RX-ISR histograms, motor DWT P99.9/max values, windowed
0x05 sample/missed-tick counters, and timing-query request/response/timeout
counters. Every 0x26 request also carries a per-Stream window token so a motor's
old uptime counters cannot contaminate the current acceptance window. This is the
canonical A9 measurement path; an external CAN analyzer is optional. Main-
controller latency percentiles are conservative 64 us bin upper bounds over
0..8192 us; the maximum field remains exact beyond that range.

At boot, firmware opens a measurement-only maintenance window with
`session_epoch=0`. It includes Stream-transition motor-diagnostics requests and
late responses after a fail-closed timeout, so pre-Stream tails can be diagnosed
without enabling motion. The host must never accept epoch zero as a formal R5
pass. A successful Stream transition resets all timing evidence into its non-zero
session epoch.
`window_duration_us` is conservatively capped at the earliest latest-counts-page
timestamp across all seven nodes, so an uncovered tail cannot satisfy a duration
gate.

```c
uint16_t format_version;
uint16_t payload_size;
uint32_t session_epoch;
uint8_t motor_marker_mask;
uint8_t window_flags;       // active, epoch-stable, counters-monotonic, markers-complete
uint16_t reserved0;
uint32_t window_reset_count;
uint64_t window_start_us;
uint64_t window_duration_us;
uint32_t target_tx_complete[7];
uint32_t position_request[7];
uint32_t position_response[7];
uint32_t position_timeout[7];
uint32_t temperature_request[7];
uint32_t temperature_response[7];
uint32_t temperature_timeout[7];
uint8_t motor_tx_drop[7], motor_rx_error[7], motor_busoff[7];
uint8_t reserved_motor[3];
uint32_t main_can_busoff[2], main_can_rx_overflow[2], main_can_rx_high_water[2];
uint32_t unexpected_response_count, maintenance_response_count;
uint32_t query_target_overlap_count, target_retry_count;
uint32_t target_retry_exhausted_count, target_deadline_failure_count;
uint32_t main_can_tx_abort[2], main_can_tx_error[2];
uint32_t main_can_tx_recovery[2], main_can_completion_overflow[2];
uint32_t safety_preemption_count;
uint32_t max_safety_wait_us;
uint32_t max_fanout_us;
uint32_t max_rx_dispatch_latency_us;
uint32_t main_can_rx_frame[2], main_can_tx_busy[2];
uint32_t transition_failure_count;
uint32_t last_transition_failure_code;
uint32_t last_transition_failure_node_id;
uint32_t last_transition_failure_detail;
```

The final three words were reserved in the original diagnostics-v2 layout and
are now used without changing the 380-byte payload or protocol version. They
latch the first failure from the most recent Stream transition attempt and
remain readable after fail-closed HOLD. Codes are: `0=NONE`, `1=EPOCH_CHANGED`,
`2=AUTHORIZATION_LOST`, `3=TRANSPORT_OVERFLOW`, `4=SAFETY_TX_COMPLETION`,
`5=CONFIGURATION_TX_COMPLETION`, `6=ENABLE_VALIDATION`,
`7=MOTOR_DIAGNOSTICS_TIMEOUT`, `8=MOTOR_MARKERS_INCOMPLETE`,
`9=CONFIGURATION_QUEUE`, and `10=ENABLE_QUEUE`. The node is zero for a
non-node-specific failure. Detail is code-specific (for code 7 it is the
response timeout in microseconds).

The window opens only after the Stream enable frame completes successfully.
Motor counters are reported relative to the seven-node preflight baseline.
Epoch changes, window resets, missing markers, or counter rollback invalidate
the soak evidence and cause a fail-closed HOLD. The host records diagnostics at
1 Hz and evaluates the first/last snapshots from one unchanged window. Firmware keeps a single CAN frame in
flight and advances on TX-complete/RX/deadline events. Priority is ESTOP/FAULT,
then HOLD/RELEASE/mode, normal target/position traffic, then temperature and
diagnostics. The 1 kHz timer is only a watchdog; it does not rate-limit normal
dispatch or replay missed bursts. Nominal per-node rates remain target 50 Hz,
position 40 Hz and temperature 1 Hz.

## Time mapping and Raw Session evidence

The host performs the four-timestamp exchange at 2 Hz, rejects high-RTT samples,
and fits an affine mapping:

```text
host_monotonic_ns = slope_ns_per_us * mcu_time_us + intercept_ns
```

Every accepted update receives a new model ID. MCU/host rollback or a model jump
starts a new segment, across which strict export never interpolates.

Raw Session schema v6 maps to binary protocol v5 and stores session epoch/control tick, all lifecycle stages,
three action-latency components, time models/exchanges, CAN diagnostics, coherent
reference time and camera timestamp source. Strict LeRobot export is fixed at
20 Hz, interpolates observations by mapped coherent reference time, chooses
camera frames by hardware exposure time or explicit host arrival time, and keeps
only actions with ACK + exact TX completion + latency-qualified post feedback.
Actions themselves are never interpolated. HOLD, invalid sweep, over-budget
control gaps, and clock segment changes split exported episodes.

Raw v2/v3 remain integrity-inspectable only. Raw v4 export requires an explicit
`legacy_mode: true` recipe and is labeled `v4_legacy_can_queued_only`; it cannot
be merged into the exact-completion evidence tier. Raw v5 remains read-only
compatible; new recordings are always v6 and are not rewritten into old sessions.

## Safety boundary

The MCU rejects wrong version/hash/session/mode, non-increasing channel sequence,
expired TTL, non-finite targets and out-of-limit values. Target or lease timeout
enters HOLD locally. Persistent feedback loss, over-temperature and ESTOP enter
FAULT according to configuration. Linux, USB and dataset tooling are not the
hard-real-time safety boundary.
