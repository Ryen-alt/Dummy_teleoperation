from __future__ import annotations

import queue
import time

import numpy as np

from .protocol import (
    ACQUIRE_CONTROL,
    SET_MODE,
    MessageType,
    Packet,
    ResultCode,
    pack_ack,
    pack_hello_ack,
    pack_state,
    unpack_hello,
    unpack_joint_target,
)
from .schema import ControlMode, RobotConfig, RobotState
from .transport_serial import TransportClosed


class FakeMcuTransport:
    """In-memory firmware peer used before any motor is connected."""

    is_simulated = True

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self._rx: queue.Queue[Packet] = queue.Queue(128)
        self._open = False
        self._lease = False
        self._session = 0
        self._mode = ControlMode.HOLD
        self._last_received = 0
        self._last_applied = 0
        self._position = np.concatenate((config.initial_pose_rad, np.asarray([0.0], dtype=np.float32)))
        self._velocity = np.zeros(7, dtype=np.float32)

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    def receive(self, timeout: float | None = None) -> Packet | None:
        if not self._open:
            raise TransportClosed("fake MCU is closed")
        try:
            return self._rx.get(timeout=timeout)
        except queue.Empty:
            return None

    def send(self, packet: Packet) -> None:
        if not self._open:
            raise TransportClosed("fake MCU is closed")
        if packet.message_type == MessageType.HELLO:
            remote_hash, _ = unpack_hello(packet.payload)
            if remote_hash != self.config.config_hash_bytes:
                self._nack(packet, ResultCode.BAD_CONFIG)
                return
            self._rx.put(
                self._response(
                    packet,
                    MessageType.HELLO_ACK,
                    pack_hello_ack(self.config.config_hash_bytes, 0, "fake-mcu-v1"),
                )
            )
            return
        if packet.message_type == MessageType.ACQUIRE_CONTROL:
            if len(packet.payload) != ACQUIRE_CONTROL.size:
                self._nack(packet, ResultCode.BAD_LENGTH)
                return
            self._lease = True
            self._session = packet.session_id
            self._ack(packet)
            self._emit_state(packet.sequence)
            return
        if packet.session_id != self._session:
            self._nack(packet, ResultCode.BAD_SESSION)
            return
        if packet.message_type == MessageType.SET_MODE:
            if not self._lease or len(packet.payload) != SET_MODE.size:
                self._nack(packet, ResultCode.NO_LEASE)
                return
            try:
                self._mode = ControlMode(SET_MODE.unpack(packet.payload)[0])
            except ValueError:
                self._nack(packet, ResultCode.BAD_MODE)
                return
            self._ack(packet)
            self._emit_state(packet.sequence)
            return
        if packet.message_type == MessageType.SET_JOINT_TARGET:
            if not self._lease:
                self._nack(packet, ResultCode.NO_LEASE)
                return
            if self._mode not in (ControlMode.TELEOP, ControlMode.POLICY):
                self._nack(packet, ResultCode.BAD_MODE)
                return
            try:
                action, _, _, _ = unpack_joint_target(packet.payload)
            except ValueError:
                self._nack(packet, ResultCode.BAD_LENGTH)
                return
            if packet.sequence <= self._last_received:
                self._nack(packet, ResultCode.BAD_SEQUENCE)
                return
            self._last_received = packet.sequence
            self._last_applied = packet.sequence
            self._velocity = (action - self._position) * self.config.control_rate_hz
            self._position = action
            self._ack(packet)
            self._emit_state(packet.sequence)
            return
        if packet.message_type == MessageType.HEARTBEAT:
            self._ack(packet)
            return
        if packet.message_type == MessageType.HOLD:
            self._mode = ControlMode.HOLD
            self._ack(packet)
            self._emit_state(packet.sequence)
            return
        if packet.message_type == MessageType.RELEASE_CONTROL:
            self._mode = ControlMode.HOLD
            self._lease = False
            self._ack(packet)
            return
        if packet.message_type == MessageType.ESTOP:
            self._mode = ControlMode.FAULT
            self._lease = False
            self._ack(packet)
            return
        self._nack(packet, ResultCode.UNSUPPORTED)

    def _response(self, request: Packet, message_type: MessageType, payload: bytes) -> Packet:
        return Packet(message_type, request.session_id, request.sequence, time.monotonic_ns() // 1_000, payload)

    def _ack(self, request: Packet) -> None:
        self._rx.put(self._response(request, MessageType.ACK, pack_ack(request.message_type)))

    def _nack(self, request: Packet, result: ResultCode) -> None:
        self._rx.put(self._response(request, MessageType.NACK, pack_ack(request.message_type, result)))

    def _emit_state(self, sequence: int) -> None:
        state = RobotState(
            position=self._position.copy(),
            velocity=self._velocity.copy(),
            monotonic_ns=time.monotonic_ns(),
            mcu_time_us=time.monotonic_ns() // 1_000,
            mode=self._mode,
            fault_bits=1 if self._mode == ControlMode.FAULT else 0,
            position_valid=True,
            velocity_valid=True,
            gripper_valid=self.config.gripper_state_feedback,
            last_received_sequence=self._last_received,
            last_applied_sequence=self._last_applied,
            target_age_ms=0,
            config_hash=self.config.config_hash,
        )
        self._rx.put(Packet(MessageType.STATE, self._session, sequence, time.monotonic_ns() // 1_000, pack_state(state)))
