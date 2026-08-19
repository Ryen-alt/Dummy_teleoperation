from __future__ import annotations

import queue
import time
from collections.abc import Callable

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

    def __init__(
        self,
        config: RobotConfig,
        *,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.config = config
        self.clock_ns = clock_ns
        self._rx: queue.Queue[Packet] = queue.Queue(128)
        self._open = False
        self._lease = False
        self._session = 0
        self._mode = ControlMode.HOLD
        self._last_received = 0
        self._last_applied = 0
        self._position = np.concatenate((config.initial_pose_rad, np.asarray([0.0], dtype=np.float32)))
        self._velocity = np.zeros(7, dtype=np.float32)
        self._lease_duration_ms = 0
        self._lease_deadline_ns: int | None = None
        self._target_deadline_ns: int | None = None
        self._state_period_ns = max(1, int(1e9 / config.control_rate_hz))
        self._next_state_ns: int | None = None

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False
        self._next_state_ns = None

    def receive(self, timeout: float | None = None) -> Packet | None:
        if not self._open:
            raise TransportClosed("fake MCU is closed")
        self._expire_deadlines()
        self._emit_periodic_state_if_due()
        receive_timeout = self._receive_timeout(timeout)
        try:
            return self._rx.get(timeout=receive_timeout)
        except queue.Empty:
            self._expire_deadlines()
            self._emit_periodic_state_if_due()
            try:
                return self._rx.get_nowait()
            except queue.Empty:
                pass
            return None

    def send(self, packet: Packet) -> None:
        if not self._open:
            raise TransportClosed("fake MCU is closed")
        self._expire_deadlines()
        if packet.message_type == MessageType.HELLO:
            remote_hash, _ = unpack_hello(packet.payload)
            if remote_hash != self.config.config_hash_bytes:
                self._nack(packet, ResultCode.BAD_CONFIG)
                return
            # Firmware uses the validated HELLO session for telemetry before a
            # control lease is acquired. Mirror that behavior so read-only and
            # dead-man-released startup paths are testable offline.
            self._session = packet.session_id
            self._next_state_ns = self.clock_ns() + self._state_period_ns
            self._rx.put(
                self._response(
                    packet,
                    MessageType.HELLO_ACK,
                    pack_hello_ack(self.config.config_hash_bytes, 0, "fake-mcu-v1"),
                )
            )
            return
        if packet.message_type == MessageType.ESTOP:
            self._mode = ControlMode.FAULT
            self._lease = False
            self._lease_deadline_ns = None
            self._target_deadline_ns = None
            self._ack(packet)
            self._emit_state(packet.sequence)
            return
        if packet.message_type == MessageType.HOLD:
            self._mode = ControlMode.HOLD
            self._target_deadline_ns = None
            self._ack(packet)
            self._emit_state(packet.sequence)
            return
        if packet.message_type == MessageType.ACQUIRE_CONTROL:
            if len(packet.payload) != ACQUIRE_CONTROL.size:
                self._nack(packet, ResultCode.BAD_LENGTH)
                return
            lease_ms = ACQUIRE_CONTROL.unpack(packet.payload)[0]
            if lease_ms == 0 or lease_ms > self.config.lease_timeout_ms:
                self._nack(packet, ResultCode.OUT_OF_RANGE)
                return
            self._lease = True
            self._session = packet.session_id
            self._lease_duration_ms = lease_ms
            self._extend_lease()
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
                requested_mode = ControlMode(SET_MODE.unpack(packet.payload)[0])
            except ValueError:
                self._nack(packet, ResultCode.BAD_MODE)
                return
            if requested_mode not in (ControlMode.HOLD, ControlMode.TELEOP, ControlMode.POLICY):
                self._nack(packet, ResultCode.BAD_MODE)
                return
            self._mode = requested_mode
            self._target_deadline_ns = None
            self._extend_lease()
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
                action, velocity, valid_for_ms, _ = unpack_joint_target(packet.payload)
            except ValueError:
                self._nack(packet, ResultCode.BAD_LENGTH)
                return
            if packet.sequence <= self._last_received:
                self._nack(packet, ResultCode.BAD_SEQUENCE)
                return
            if valid_for_ms == 0 or valid_for_ms > self.config.target_ttl_ms:
                self._nack(packet, ResultCode.EXPIRED)
                return
            if (
                np.any(action[:6] < self.config.joint_limit_min_rad)
                or np.any(action[:6] > self.config.joint_limit_max_rad)
                or action[6] < self.config.gripper_range[0]
                or action[6] > self.config.gripper_range[1]
                or np.any(velocity <= 0)
                or np.any(velocity > self.config.joint_velocity_limit_rad_s)
            ):
                self._nack(packet, ResultCode.OUT_OF_RANGE)
                return
            self._last_received = packet.sequence
            self._last_applied = packet.sequence
            self._velocity = (action - self._position) * self.config.control_rate_hz
            self._position = action
            self._target_deadline_ns = self.clock_ns() + valid_for_ms * 1_000_000
            self._extend_lease()
            self._ack(packet)
            self._emit_state(packet.sequence)
            return
        if packet.message_type == MessageType.HEARTBEAT:
            self._extend_lease()
            self._ack(packet)
            return
        if packet.message_type == MessageType.RELEASE_CONTROL:
            self._mode = ControlMode.HOLD
            self._lease = False
            self._lease_deadline_ns = None
            self._target_deadline_ns = None
            self._ack(packet)
            return
        self._nack(packet, ResultCode.UNSUPPORTED)

    def _response(self, request: Packet, message_type: MessageType, payload: bytes) -> Packet:
        return Packet(message_type, request.session_id, request.sequence, self.clock_ns() // 1_000, payload)

    def _ack(self, request: Packet) -> None:
        self._rx.put(self._response(request, MessageType.ACK, pack_ack(request.message_type)))

    def _nack(self, request: Packet, result: ResultCode) -> None:
        self._rx.put(self._response(request, MessageType.NACK, pack_ack(request.message_type, result)))

    def _emit_state(self, sequence: int) -> None:
        now_ns = self.clock_ns()
        self._next_state_ns = now_ns + self._state_period_ns
        state = RobotState(
            position=self._position.copy(),
            velocity=self._velocity.copy(),
            monotonic_ns=now_ns,
            mcu_time_us=now_ns // 1_000,
            mode=self._mode,
            fault_bits=1 if self._mode == ControlMode.FAULT else 0,
            position_valid=True,
            velocity_valid=True,
            gripper_valid=self.config.gripper_state_feedback,
            last_received_sequence=self._last_received,
            last_applied_sequence=self._last_applied,
            target_age_ms=0
            if self._target_deadline_ns is None
            else max(
                0,
                self.config.target_ttl_ms
                - int((self._target_deadline_ns - now_ns) / 1e6),
            ),
            config_hash=self.config.config_hash,
        )
        self._rx.put(
            Packet(
                MessageType.STATE,
                self._session,
                sequence,
                now_ns // 1_000,
                pack_state(state),
            )
        )

    def _receive_timeout(self, timeout: float | None) -> float | None:
        if self._next_state_ns is None:
            return timeout
        until_state_s = max(0.0, (self._next_state_ns - self.clock_ns()) / 1e9)
        return until_state_s if timeout is None else min(timeout, until_state_s)

    def _emit_periodic_state_if_due(self) -> None:
        if (
            self._session != 0
            and self._next_state_ns is not None
            and self.clock_ns() >= self._next_state_ns
        ):
            self._emit_state(self._last_received)

    def _extend_lease(self) -> None:
        if self._lease:
            self._lease_deadline_ns = self.clock_ns() + self._lease_duration_ms * 1_000_000

    def _expire_deadlines(self) -> None:
        now_ns = self.clock_ns()
        if self._lease and self._lease_deadline_ns is not None and now_ns >= self._lease_deadline_ns:
            self._lease = False
            self._lease_deadline_ns = None
            self._target_deadline_ns = None
            self._mode = ControlMode.HOLD
            self._emit_state(self._last_received)
            return
        if self._target_deadline_ns is not None and now_ns >= self._target_deadline_ns:
            self._target_deadline_ns = None
            self._mode = ControlMode.HOLD
            self._emit_state(self._last_received)
