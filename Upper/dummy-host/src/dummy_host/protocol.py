from __future__ import annotations

import enum
import math
import struct
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .schema import ControlMode, RobotState

MAGIC = 0x4459
PROTOCOL_VERSION = 1
MAX_DECODED_FRAME = 512
HEADER = struct.Struct("<HBBHHIIQ")
CRC = struct.Struct("<I")
HELLO = struct.Struct("<32sI")
HELLO_ACK = struct.Struct("<32sI32s")
ACQUIRE_CONTROL = struct.Struct("<I")
SET_MODE = struct.Struct("<B")
JOINT_TARGET = struct.Struct("<7f6fHH")
ACK = struct.Struct("<BBH")
STATE = struct.Struct("<Q7f7fIIBBHI32s")


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
    HELLO_ACK = 0x81
    STATE = 0x82
    ACK = 0x83
    NACK = 0x84
    FAULT = 0x85
    EVENT = 0x86


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
    def __init__(self, max_encoded_size: int = 600) -> None:
        self._buffer = bytearray()
        self.max_encoded_size = max_encoded_size
        self.dropped_frames = 0

    def feed(self, data: bytes) -> list[Packet]:
        packets: list[Packet] = []
        for byte in data:
            if byte == 0:
                if self._buffer:
                    try:
                        packets.append(decode_packet(bytes(self._buffer)))
                    except ProtocolError:
                        self.dropped_frames += 1
                    self._buffer.clear()
                continue
            if len(self._buffer) >= self.max_encoded_size:
                self._buffer.clear()
                self.dropped_frames += 1
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
    return JOINT_TARGET.pack(*action, *velocity, valid_for_ms, target_flags)


def unpack_joint_target(payload: bytes) -> tuple[np.ndarray, np.ndarray, int, int]:
    if len(payload) != JOINT_TARGET.size:
        raise ProtocolError("invalid SET_JOINT_TARGET payload length")
    values = JOINT_TARGET.unpack(payload)
    action = np.asarray(values[:7], dtype=np.float32)
    velocity = np.asarray(values[7:13], dtype=np.float32)
    if not np.isfinite(action).all() or not np.isfinite(velocity).all():
        raise ProtocolError("joint target contains NaN or Inf")
    return action, velocity, values[13], values[14]


def pack_state(state: RobotState) -> bytes:
    validity = int(state.position_valid) | (int(state.velocity_valid) << 1) | (int(state.gripper_valid) << 2)
    hash_bytes = bytes.fromhex(state.config_hash)
    if state.position.shape != (7,) or state.velocity.shape != (7,) or len(hash_bytes) != 32:
        raise ProtocolError("invalid STATE values")
    return STATE.pack(
        state.mcu_time_us,
        *state.position.astype(np.float32),
        *state.velocity.astype(np.float32),
        state.last_received_sequence,
        state.last_applied_sequence,
        int(state.mode),
        validity,
        state.fault_bits,
        state.target_age_ms,
        hash_bytes,
    )


def unpack_state(payload: bytes, monotonic_ns: int) -> RobotState:
    if len(payload) != STATE.size:
        raise ProtocolError("invalid STATE payload length")
    values = STATE.unpack(payload)
    position = np.asarray(values[1:8], dtype=np.float32)
    velocity = np.asarray(values[8:15], dtype=np.float32)
    if not np.isfinite(position).all() or not np.isfinite(velocity).all():
        raise ProtocolError("STATE contains NaN or Inf")
    validity = values[18]
    try:
        mode = ControlMode(values[17])
    except ValueError as exc:
        raise ProtocolError("STATE contains an invalid control mode") from exc
    return RobotState(
        position=position,
        velocity=velocity,
        monotonic_ns=monotonic_ns,
        mcu_time_us=values[0],
        mode=mode,
        fault_bits=values[19],
        position_valid=bool(validity & 0x01),
        velocity_valid=bool(validity & 0x02),
        gripper_valid=bool(validity & 0x04),
        last_received_sequence=values[15],
        last_applied_sequence=values[16],
        target_age_ms=values[20],
        config_hash=values[21].hex(),
    )


def monotonic_us(monotonic_ns: int) -> int:
    return monotonic_ns // 1_000
