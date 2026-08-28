from __future__ import annotations

import enum
import struct
from dataclasses import dataclass

import numpy as np

from .domain.models import ActionProgressRecord
from .schema import ControlMode, RobotState

MAGIC = 0x4459
PROTOCOL_VERSION = 5
MAX_DECODED_FRAME = 576
HEADER = struct.Struct("<HBBHHIIQ")
CRC = struct.Struct("<I")
HELLO = struct.Struct("<32sI")
HELLO_ACK = struct.Struct("<32sI32s")
ACQUIRE_CONTROL = struct.Struct("<I")
SET_MODE = struct.Struct("<B")
JOINT_TARGET = struct.Struct("<7f6fHHI")
TARGET_KEEPALIVE = struct.Struct("<II")
ACK = struct.Struct("<BBH")
ACTION_PROGRESS = struct.Struct("<IB3xQI")
ACTION_PROGRESS_RECORD = struct.Struct("<IB3xIIII")
TIME_SYNC = struct.Struct("<Q")
TIME_SYNC_ACK = struct.Struct("<QQQ")
CAN_DIAGNOSTICS_FORMAT_VERSION = 2
CAN_DIAGNOSTICS_PAYLOAD_SIZE = 380
CAN_DIAGNOSTICS = struct.Struct(
    "<HHIBBHIQQ" + "7I" * 7 + "7B7B7B3x" + "2I" * 3
    + "6I" + "2I" * 4 + "4I" + "2I2I" + "I3I"
)
assert CAN_DIAGNOSTICS.size == CAN_DIAGNOSTICS_PAYLOAD_SIZE
ACTION_PROGRESS_CAPACITY = 6
STATE = struct.Struct(
    "<Q7f7fIBBHI32s7f7I7I7I7H7H7BBHH7Q7IIIQ4B"
    + "IB3xIIII" * ACTION_PROGRESS_CAPACITY
)


class ProtocolError(ValueError):
    pass


class MessageType(enum.IntEnum):
    HELLO = 0x01
    ACQUIRE_CONTROL = 0x02
    RELEASE_CONTROL = 0x03
    SET_MODE = 0x04
    HEARTBEAT = 0x05
    SET_JOINT_TARGET = 0x06
    HOLD = 0x07
    ESTOP = 0x08
    CLEAR_FAULT = 0x09
    TARGET_KEEPALIVE = 0x0A
    TIME_SYNC = 0x0B
    GET_CAN_DIAGNOSTICS = 0x0C
    HELLO_ACK = 0x81
    STATE = 0x82
    ACK = 0x83
    NACK = 0x84
    FAULT = 0x85
    EVENT = 0x86
    TIME_SYNC_ACK = 0x87
    CAN_DIAGNOSTICS = 0x88


class ResultCode(enum.IntEnum):
    OK = 0
    BAD_VERSION = 1
    BAD_LENGTH = 2
    BAD_CONFIG = 3
    BAD_SESSION = 4
    BAD_SEQUENCE = 5
    BAD_MODE = 6
    NO_LEASE = 7
    EXPIRED = 8
    NON_FINITE = 9
    OUT_OF_RANGE = 10
    FAULT_ACTIVE = 11
    LEASE_CONFLICT = 12
    UNSUPPORTED = 13


class ActionProgressStage(enum.IntEnum):
    CAN_QUEUED_EXACT = 1
    CAN_TX_COMPLETE_EXACT = 2
    POST_COMMAND_FEEDBACK = 3
    SUPERSEDED = 4
    PREEMPTED_BY_SAFETY = 5
    FAILED = 6


ACK_DETAIL_FEEDBACK_NOT_READY = 1
CAPABILITY_MULTI_CHANNEL_SEQUENCE = 1 << 0
CAPABILITY_TARGET_KEEPALIVE = 1 << 1
CAPABILITY_CAN_TX_COMPLETE_EXACT = 1 << 2
CAPABILITY_CONTROL_FRESHNESS_TOKEN = 1 << 3
CAPABILITY_TIME_SYNC = 1 << 4
CAPABILITY_CAN_DIAGNOSTICS = 1 << 5
CAPABILITY_CAN_DIAGNOSTICS_V2 = 1 << 6
CAN_DIAGNOSTICS_WINDOW_ACTIVE = 1 << 0
CAN_DIAGNOSTICS_EPOCH_STABLE = 1 << 1
CAN_DIAGNOSTICS_MOTOR_COUNTERS_MONOTONIC = 1 << 2
CAN_DIAGNOSTICS_MARKERS_COMPLETE = 1 << 3
CAN_DIAGNOSTICS_WINDOW_VALID = (
    CAN_DIAGNOSTICS_WINDOW_ACTIVE
    | CAN_DIAGNOSTICS_EPOCH_STABLE
    | CAN_DIAGNOSTICS_MOTOR_COUNTERS_MONOTONIC
    | CAN_DIAGNOSTICS_MARKERS_COMPLETE
)


@dataclass(frozen=True)
class Packet:
    message_type: MessageType
    session_id: int
    sequence: int
    sender_time_us: int
    payload: bytes = b""
    flags: int = 0


@dataclass(frozen=True)
class AckPayload:
    request_type: MessageType
    result: ResultCode
    detail: int


@dataclass(frozen=True)
class CanDiagnostics:
    format_version: int
    payload_size: int
    session_epoch: int
    motor_marker_mask: int
    window_flags: int
    window_reset_count: int
    window_start_us: int
    window_duration_us: int
    target_tx_complete: tuple[int, ...]
    position_request: tuple[int, ...]
    position_response: tuple[int, ...]
    position_timeout: tuple[int, ...]
    temperature_request: tuple[int, ...]
    temperature_response: tuple[int, ...]
    temperature_timeout: tuple[int, ...]
    motor_tx_drop: tuple[int, ...]
    motor_rx_error: tuple[int, ...]
    motor_busoff: tuple[int, ...]
    main_can_busoff: tuple[int, ...]
    main_can_rx_overflow: tuple[int, ...]
    main_can_rx_high_water: tuple[int, ...]
    unexpected_response_count: int
    maintenance_response_count: int
    query_target_overlap_count: int
    target_retry_count: int
    target_retry_exhausted_count: int
    target_deadline_failure_count: int
    main_can_tx_abort: tuple[int, ...]
    main_can_tx_error: tuple[int, ...]
    main_can_tx_recovery: tuple[int, ...]
    main_can_completion_overflow: tuple[int, ...]
    safety_preemption_count: int
    max_safety_wait_us: int
    max_fanout_us: int
    max_rx_dispatch_latency_us: int
    main_can_rx_frame: tuple[int, ...]
    main_can_tx_busy: tuple[int, ...]
    transition_failure_count: int

    def __post_init__(self) -> None:
        arrays = (
            self.target_tx_complete,
            self.position_request,
            self.position_response,
            self.position_timeout,
            self.temperature_request,
            self.temperature_response,
            self.temperature_timeout,
            self.motor_tx_drop,
            self.motor_rx_error,
            self.motor_busoff,
        )
        if any(len(values) != 7 for values in arrays):
            raise ValueError("CAN diagnostic node counters must contain seven values")
        can_arrays = (
            self.main_can_busoff,
            self.main_can_rx_overflow,
            self.main_can_rx_high_water,
            self.main_can_tx_abort,
            self.main_can_tx_error,
            self.main_can_tx_recovery,
            self.main_can_completion_overflow,
            self.main_can_rx_frame,
            self.main_can_tx_busy,
        )
        if any(len(values) != 2 for values in can_arrays):
            raise ValueError("main CAN diagnostic counters must contain two values")
        if self.format_version != CAN_DIAGNOSTICS_FORMAT_VERSION:
            raise ValueError("CAN diagnostics format version must be 2")
        if self.payload_size != CAN_DIAGNOSTICS_PAYLOAD_SIZE:
            raise ValueError("CAN diagnostics payload size must be 380")
        if not 0 <= self.motor_marker_mask <= 0xFF or not 0 <= self.window_flags <= 0xFF:
            raise ValueError("CAN diagnostic masks must be uint8")
        if any(value > 0xFF for values in (
            self.motor_tx_drop, self.motor_rx_error, self.motor_busoff
        ) for value in values):
            raise ValueError("motor CAN diagnostic counters must be uint8")
        scalars = (
            self.format_version,
            self.payload_size,
            self.session_epoch,
            self.motor_marker_mask,
            self.window_flags,
            self.window_reset_count,
            self.window_start_us,
            self.window_duration_us,
            *(value for values in arrays for value in values),
            *(value for values in can_arrays for value in values),
            self.unexpected_response_count,
            self.maintenance_response_count,
            self.query_target_overlap_count,
            self.target_retry_count,
            self.target_retry_exhausted_count,
            self.target_deadline_failure_count,
            self.safety_preemption_count,
            self.max_safety_wait_us,
            self.max_fanout_us,
            self.max_rx_dispatch_latency_us,
            self.transition_failure_count,
        )
        if any(value < 0 for value in scalars):
            raise ValueError("CAN diagnostic counters must be non-negative")

    @property
    def window_valid(self) -> bool:
        return self.window_flags & CAN_DIAGNOSTICS_WINDOW_VALID == CAN_DIAGNOSTICS_WINDOW_VALID

    @property
    def position_timeout_count(self) -> int:
        return sum(self.position_timeout)

    @property
    def temperature_timeout_count(self) -> int:
        return sum(self.temperature_timeout)

    @property
    def tx_abort_count(self) -> int:
        return sum(self.main_can_tx_abort)

    @property
    def tx_error_count(self) -> int:
        return sum(self.main_can_tx_error)

    @property
    def tx_recovery_count(self) -> int:
        return sum(self.main_can_tx_recovery)


def crc32c(data: bytes, initial: int = 0) -> int:
    """CRC-32C (Castagnoli), compatible with the firmware implementation."""
    crc = initial ^ 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def cobs_encode(data: bytes) -> bytes:
    output = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    if not data:
        raise ProtocolError("empty COBS frame")
    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise ProtocolError("zero byte inside COBS frame")
        index += 1
        end = index + code - 1
        if end > len(data):
            raise ProtocolError("truncated COBS block")
        output.extend(data[index:end])
        index = end
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


def encode_packet(packet: Packet) -> bytes:
    if len(packet.payload) + HEADER.size + CRC.size > MAX_DECODED_FRAME:
        raise ProtocolError("packet exceeds maximum decoded size")
    header = HEADER.pack(
        MAGIC,
        PROTOCOL_VERSION,
        int(packet.message_type),
        len(packet.payload),
        packet.flags,
        packet.session_id,
        packet.sequence,
        packet.sender_time_us,
    )
    decoded = header + packet.payload
    return cobs_encode(decoded + CRC.pack(crc32c(decoded))) + b"\x00"


def decode_packet(frame: bytes) -> Packet:
    if frame.endswith(b"\x00"):
        frame = frame[:-1]
    decoded = cobs_decode(frame)
    if len(decoded) < HEADER.size + CRC.size:
        raise ProtocolError("frame is shorter than header and CRC")
    if len(decoded) > MAX_DECODED_FRAME:
        raise ProtocolError("decoded frame exceeds maximum size")
    body, received_crc = decoded[:-CRC.size], CRC.unpack(decoded[-CRC.size:])[0]
    if crc32c(body) != received_crc:
        raise ProtocolError("CRC32C mismatch")
    magic, version, raw_type, payload_length, flags, session_id, sequence, sender_time_us = HEADER.unpack(
        body[:HEADER.size]
    )
    if magic != MAGIC:
        raise ProtocolError(f"bad magic 0x{magic:04x}")
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    if payload_length != len(body) - HEADER.size:
        raise ProtocolError("payload length mismatch")
    try:
        message_type = MessageType(raw_type)
    except ValueError as exc:
        raise ProtocolError(f"unknown message type 0x{raw_type:02x}") from exc
    return Packet(message_type, session_id, sequence, sender_time_us, body[HEADER.size:], flags)


class StreamDecoder:
    def __init__(
        self,
        max_encoded_size: int = 600,
        *,
        allow_initial_partial_frame: bool = False,
    ) -> None:
        self._buffer = bytearray()
        self.max_encoded_size = max_encoded_size
        self.dropped_frames = 0
        self.initial_partial_frames = 0
        self._allow_initial_partial_frame = allow_initial_partial_frame
        self._has_frame_boundary = not allow_initial_partial_frame
        self._discard_until_boundary = False

    def feed(self, data: bytes) -> list[Packet]:
        packets: list[Packet] = []
        for byte in data:
            if byte == 0:
                if self._discard_until_boundary:
                    self._discard_until_boundary = False
                    self._buffer.clear()
                elif self._buffer:
                    try:
                        packets.append(decode_packet(bytes(self._buffer)))
                    except ProtocolError:
                        if self._has_frame_boundary:
                            self.dropped_frames += 1
                        else:
                            # A continuously streaming serial producer can be
                            # opened in the middle of a valid frame.  Bytes
                            # before the first observed delimiter have no
                            # trustworthy start boundary and are alignment,
                            # not evidence of an on-wire CRC/COBS failure.
                            self.initial_partial_frames += 1
                    self._buffer.clear()
                self._has_frame_boundary = True
                continue
            if self._discard_until_boundary:
                continue
            if len(self._buffer) >= self.max_encoded_size:
                self._buffer.clear()
                if self._has_frame_boundary:
                    self.dropped_frames += 1
                else:
                    self.initial_partial_frames += 1
                self._discard_until_boundary = True
            else:
                self._buffer.append(byte)
        return packets


def pack_hello(config_hash: bytes, capabilities: int = 0) -> bytes:
    if len(config_hash) != 32:
        raise ProtocolError("configuration hash must be 32 bytes")
    return HELLO.pack(config_hash, capabilities)


def unpack_hello(payload: bytes) -> tuple[bytes, int]:
    if len(payload) != HELLO.size:
        raise ProtocolError("invalid HELLO payload length")
    return HELLO.unpack(payload)


def pack_hello_ack(config_hash: bytes, capabilities: int, firmware_version: str) -> bytes:
    version = firmware_version.encode("ascii", errors="strict")
    if len(config_hash) != 32 or len(version) > 31:
        raise ProtocolError("invalid HELLO_ACK fields")
    return HELLO_ACK.pack(config_hash, capabilities, version.ljust(32, b"\x00"))


def unpack_hello_ack(payload: bytes) -> tuple[bytes, int, str]:
    if len(payload) != HELLO_ACK.size:
        raise ProtocolError("invalid HELLO_ACK payload length")
    config_hash, capabilities, version = HELLO_ACK.unpack(payload)
    return config_hash, capabilities, version.split(b"\x00", 1)[0].decode("ascii")


def pack_ack(request_type: MessageType, result: ResultCode = ResultCode.OK, detail: int = 0) -> bytes:
    return ACK.pack(int(request_type), int(result), detail)


def unpack_ack(payload: bytes) -> AckPayload:
    if len(payload) != ACK.size:
        raise ProtocolError("invalid ACK payload length")
    raw_type, raw_result, detail = ACK.unpack(payload)
    try:
        return AckPayload(MessageType(raw_type), ResultCode(raw_result), detail)
    except ValueError as exc:
        raise ProtocolError("invalid ACK enum value") from exc


def pack_joint_target(
    action: np.ndarray,
    max_velocity_rad_s: np.ndarray,
    valid_for_ms: int,
    control_tick_id: int,
    target_flags: int = 0,
) -> bytes:
    action = np.asarray(action, dtype=np.float32)
    velocity = np.asarray(max_velocity_rad_s, dtype=np.float32)
    if action.shape != (7,) or velocity.shape != (6,):
        raise ProtocolError("joint target dimensions must be 7 and 6")
    if not np.isfinite(action).all() or not np.isfinite(velocity).all():
        raise ProtocolError("joint target contains NaN or Inf")
    if not 1 <= valid_for_ms <= 0xFFFF:
        raise ProtocolError("target TTL is outside uint16 range")
    if not 0 < control_tick_id <= 0xFFFFFFFF:
        raise ProtocolError("control tick ID must be uint32 and non-zero")
    return JOINT_TARGET.pack(
        *action, *velocity, valid_for_ms, target_flags, control_tick_id
    )


def unpack_joint_target(
    payload: bytes,
) -> tuple[np.ndarray, np.ndarray, int, int, int]:
    if len(payload) != JOINT_TARGET.size:
        raise ProtocolError("invalid SET_JOINT_TARGET payload length")
    values = JOINT_TARGET.unpack(payload)
    action = np.asarray(values[:7], dtype=np.float32)
    velocity = np.asarray(values[7:13], dtype=np.float32)
    if not np.isfinite(action).all() or not np.isfinite(velocity).all():
        raise ProtocolError("joint target contains NaN or Inf")
    if values[15] == 0:
        raise ProtocolError("control tick ID must be non-zero")
    return action, velocity, values[13], values[14], values[15]


def pack_target_keepalive(action_sequence: int, control_tick_id: int) -> bytes:
    if not 0 < action_sequence <= 0xFFFFFFFF:
        raise ProtocolError("target keepalive action sequence must be uint32 and non-zero")
    if not 0 < control_tick_id <= 0xFFFFFFFF:
        raise ProtocolError("target keepalive control tick ID must be non-zero")
    return TARGET_KEEPALIVE.pack(action_sequence, control_tick_id)


def unpack_target_keepalive(payload: bytes) -> tuple[int, int]:
    if len(payload) != TARGET_KEEPALIVE.size:
        raise ProtocolError("invalid TARGET_KEEPALIVE payload length")
    action_sequence, control_tick_id = TARGET_KEEPALIVE.unpack(payload)
    if action_sequence == 0 or control_tick_id == 0:
        raise ProtocolError("target keepalive identifiers must be non-zero")
    return action_sequence, control_tick_id


def pack_time_sync(host_t0_ns: int) -> bytes:
    if not 0 <= host_t0_ns <= 0xFFFFFFFFFFFFFFFF:
        raise ProtocolError("TIME_SYNC host timestamp must be uint64")
    return TIME_SYNC.pack(host_t0_ns)


def unpack_time_sync(payload: bytes) -> int:
    if len(payload) != TIME_SYNC.size:
        raise ProtocolError("invalid TIME_SYNC payload length")
    return TIME_SYNC.unpack(payload)[0]


def pack_time_sync_ack(host_t0_ns: int, mcu_rx_us: int, mcu_tx_us: int) -> bytes:
    return TIME_SYNC_ACK.pack(host_t0_ns, mcu_rx_us, mcu_tx_us)


def unpack_time_sync_ack(payload: bytes) -> tuple[int, int, int]:
    if len(payload) != TIME_SYNC_ACK.size:
        raise ProtocolError("invalid TIME_SYNC_ACK payload length")
    return TIME_SYNC_ACK.unpack(payload)


def pack_can_diagnostics(value: CanDiagnostics) -> bytes:
    return CAN_DIAGNOSTICS.pack(
        value.format_version,
        value.payload_size,
        value.session_epoch,
        value.motor_marker_mask,
        value.window_flags,
        0,
        value.window_reset_count,
        value.window_start_us,
        value.window_duration_us,
        *value.target_tx_complete,
        *value.position_request,
        *value.position_response,
        *value.position_timeout,
        *value.temperature_request,
        *value.temperature_response,
        *value.temperature_timeout,
        *value.motor_tx_drop,
        *value.motor_rx_error,
        *value.motor_busoff,
        *value.main_can_busoff,
        *value.main_can_rx_overflow,
        *value.main_can_rx_high_water,
        value.unexpected_response_count,
        value.maintenance_response_count,
        value.query_target_overlap_count,
        value.target_retry_count,
        value.target_retry_exhausted_count,
        value.target_deadline_failure_count,
        *value.main_can_tx_abort,
        *value.main_can_tx_error,
        *value.main_can_tx_recovery,
        *value.main_can_completion_overflow,
        value.safety_preemption_count,
        value.max_safety_wait_us,
        value.max_fanout_us,
        value.max_rx_dispatch_latency_us,
        *value.main_can_rx_frame,
        *value.main_can_tx_busy,
        value.transition_failure_count,
        0,
        0,
        0,
    )


def unpack_can_diagnostics(payload: bytes) -> CanDiagnostics:
    if len(payload) != CAN_DIAGNOSTICS.size:
        raise ProtocolError("invalid CAN_DIAGNOSTICS payload length")
    values = CAN_DIAGNOSTICS.unpack(payload)
    if values[0] != CAN_DIAGNOSTICS_FORMAT_VERSION or values[1] != CAN_DIAGNOSTICS_PAYLOAD_SIZE:
        raise ProtocolError("unsupported CAN_DIAGNOSTICS format")
    return CanDiagnostics(
        format_version=values[0], payload_size=values[1],
        session_epoch=values[2], motor_marker_mask=values[3],
        window_flags=values[4], window_reset_count=values[6],
        window_start_us=values[7], window_duration_us=values[8],
        target_tx_complete=tuple(values[9:16]),
        position_request=tuple(values[16:23]),
        position_response=tuple(values[23:30]),
        position_timeout=tuple(values[30:37]),
        temperature_request=tuple(values[37:44]),
        temperature_response=tuple(values[44:51]),
        temperature_timeout=tuple(values[51:58]),
        motor_tx_drop=tuple(values[58:65]),
        motor_rx_error=tuple(values[65:72]),
        motor_busoff=tuple(values[72:79]),
        main_can_busoff=tuple(values[79:81]),
        main_can_rx_overflow=tuple(values[81:83]),
        main_can_rx_high_water=tuple(values[83:85]),
        unexpected_response_count=values[85],
        maintenance_response_count=values[86],
        query_target_overlap_count=values[87],
        target_retry_count=values[88],
        target_retry_exhausted_count=values[89],
        target_deadline_failure_count=values[90],
        main_can_tx_abort=tuple(values[91:93]),
        main_can_tx_error=tuple(values[93:95]),
        main_can_tx_recovery=tuple(values[95:97]),
        main_can_completion_overflow=tuple(values[97:99]),
        safety_preemption_count=values[99],
        max_safety_wait_us=values[100], max_fanout_us=values[101],
        max_rx_dispatch_latency_us=values[102],
        main_can_rx_frame=tuple(values[103:105]),
        main_can_tx_busy=tuple(values[105:107]),
        transition_failure_count=values[107],
    )


def pack_state(state: RobotState) -> bytes:
    validity = int(state.position_valid) | (int(state.velocity_valid) << 1) | (int(state.gripper_valid) << 2)
    hash_bytes = bytes.fromhex(state.config_hash)
    if state.position.shape != (7,) or state.velocity.shape != (7,) or len(hash_bytes) != 32:
        raise ProtocolError("invalid STATE values")
    progress = list(state.action_progress)
    if len(progress) > ACTION_PROGRESS_CAPACITY:
        raise ProtocolError("STATE action progress replay exceeds capacity")
    progress.extend(
        ActionProgressRecord(0, 0)
        for _ in range(ACTION_PROGRESS_CAPACITY - len(progress))
    )
    packed_progress: list[int] = []
    for record in progress:
        packed_progress.extend(
            (
                record.sequence,
                record.flags,
                record.can_queued_mcu_us & 0xFFFFFFFF,
                record.can_tx_complete_mcu_us & 0xFFFFFFFF,
                record.post_feedback_mcu_us & 0xFFFFFFFF,
                record.feedback_sweep_id,
            )
        )
    return STATE.pack(
        state.mcu_time_us,
        *state.position.astype(np.float32),
        *state.velocity.astype(np.float32),
        state.last_received_sequence,
        int(state.mode),
        validity,
        state.fault_bits,
        state.target_age_ms,
        hash_bytes,
        *state.following_error.astype(np.float32),
        *(int(value) for value in state.following_error_duration_ms),
        *(int(value) for value in state.feedback_age_ms),
        *(int(value) for value in state.feedback_loss_count),
        *(int(value) for value in state.consecutive_feedback_loss),
        *(int(value) for value in state.node_fault_bits),
        *(int(value) for value in state.node_validity),
        state.can_transport_status,
        state.hold_reason_bits,
        state.telemetry_validity,
        *(int(value) for value in state.feedback_sample_mcu_us),
        *(int(value) for value in state.feedback_sweep_id),
        state.coherent_sweep_id,
        state.feedback_max_skew_us,
        state.coherent_reference_mcu_us,
        int(state.state_repeated),
        len(state.action_progress),
        0,
        0,
        *packed_progress,
    )


def unpack_action_progress(payload: bytes) -> tuple[int, ActionProgressStage, int, int]:
    if len(payload) != ACTION_PROGRESS.size:
        raise ProtocolError("invalid ACTION_PROGRESS EVENT payload length")
    sequence, raw_stage, stage_time_us, sweep_id = ACTION_PROGRESS.unpack(payload)
    try:
        stage = ActionProgressStage(raw_stage)
    except ValueError as exc:
        raise ProtocolError("unknown ACTION_PROGRESS stage") from exc
    if sequence == 0:
        raise ProtocolError("ACTION_PROGRESS sequence must be non-zero")
    return sequence, stage, stage_time_us, sweep_id


def _extend_low_mcu_time(reference_us: int, low_us: int) -> int:
    if low_us == 0:
        return 0
    candidate = (reference_us & ~0xFFFFFFFF) | low_us
    if candidate > reference_us and candidate - reference_us > 0x80000000:
        candidate -= 1 << 32
    elif reference_us > candidate and reference_us - candidate > 0x80000000:
        candidate += 1 << 32
    return candidate


def unpack_state(payload: bytes, monotonic_ns: int) -> RobotState:
    if len(payload) != STATE.size:
        raise ProtocolError("invalid STATE payload length")
    values = STATE.unpack(payload)
    position = np.asarray(values[1:8], dtype=np.float32)
    velocity = np.asarray(values[8:15], dtype=np.float32)
    following_error = np.asarray(values[21:28], dtype=np.float32)
    if (
        not np.isfinite(position).all()
        or not np.isfinite(velocity).all()
        or not np.isfinite(following_error).all()
    ):
        raise ProtocolError("STATE contains NaN or Inf")
    validity = values[17]
    try:
        mode = ControlMode(values[16])
    except ValueError as exc:
        raise ProtocolError("STATE contains an invalid control mode") from exc
    mcu_time_us = values[0]
    # A CAN RX interrupt can publish a low 32-bit sample timestamp just after
    # firmware captured STATE.mcu_time_us. Older v2.1 firmware reconstructed
    # that small future offset with unsigned subtraction and emitted a value
    # near 2^64. A feedback sample cannot legitimately be newer than the STATE
    # carrying it, so clamp at the protocol boundary as a compatibility guard.
    feedback_sample_mcu_us = np.minimum(
        np.asarray(values[73:80], dtype=np.uint64),
        np.uint64(mcu_time_us),
    )
    coherent_reference_mcu_us = min(values[89], mcu_time_us)
    return RobotState(
        position=position,
        velocity=velocity,
        monotonic_ns=monotonic_ns,
        mcu_time_us=mcu_time_us,
        mode=mode,
        fault_bits=values[18],
        position_valid=bool(validity & 0x01),
        velocity_valid=bool(validity & 0x02),
        gripper_valid=bool(validity & 0x04),
        last_received_sequence=values[15],
        target_age_ms=values[19],
        config_hash=values[20].hex(),
        following_error=following_error,
        following_error_duration_ms=np.asarray(values[28:35], dtype=np.uint32),
        feedback_age_ms=np.asarray(values[35:42], dtype=np.uint32),
        feedback_loss_count=np.asarray(values[42:49], dtype=np.uint32),
        consecutive_feedback_loss=np.asarray(values[49:56], dtype=np.uint16),
        node_fault_bits=np.asarray(values[56:63], dtype=np.uint16),
        node_validity=np.asarray(values[63:70], dtype=np.uint8),
        can_transport_status=values[70],
        hold_reason_bits=values[71],
        telemetry_validity=values[72],
        feedback_sample_mcu_us=feedback_sample_mcu_us,
        feedback_sweep_id=np.asarray(values[80:87], dtype=np.uint32),
        coherent_sweep_id=values[87],
        feedback_max_skew_us=values[88],
        coherent_reference_mcu_us=coherent_reference_mcu_us,
        state_repeated=bool(values[90] & 0x01),
        action_progress=tuple(
            ActionProgressRecord(
                values[94 + index * 6],
                values[95 + index * 6],
                _extend_low_mcu_time(values[0], values[96 + index * 6]),
                _extend_low_mcu_time(values[0], values[97 + index * 6]),
                _extend_low_mcu_time(values[0], values[98 + index * 6]),
                values[99 + index * 6],
            )
            for index in range(min(values[91], ACTION_PROGRESS_CAPACITY))
            if values[94 + index * 6] != 0
        ),
    )


def monotonic_us(monotonic_ns: int) -> int:
    return monotonic_ns // 1_000
