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
    StreamDecoder,
    pack_ack,
    pack_hello_ack,
    pack_state,
    unpack_hello,
    unpack_joint_target,
)
from .domain.models import (
    FaultBits,
    HoldReasonBits,
    NodeFaultBits,
    NodeValidityBits,
    TelemetryValidityBits,
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
        self._raw_decoder = StreamDecoder()
        self._open = False
        self._lease = False
        self._session = 0
        self._mode = ControlMode.HOLD
        self._fault_bits = 0
        self._hold_reason_bits = 0
        self._last_received = 0
        self._last_applied = 0
        self._position = np.concatenate((config.initial_pose_rad, np.asarray([0.0], dtype=np.float32)))
        self._velocity = np.zeros(7, dtype=np.float32)
        self._feedback_age_ms = np.zeros(7, dtype=np.uint32)
        self._feedback_loss_count = np.zeros(7, dtype=np.uint32)
        self._consecutive_feedback_loss = np.zeros(7, dtype=np.uint16)
        self._node_fault_bits = np.zeros(7, dtype=np.uint16)
        self._node_validity = np.full(
            7, int(NodeValidityBits.POSITION), dtype=np.uint8
        )
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

    def inject_feedback_interruption(self, *, severe: bool = False, node: int = 1) -> None:
        """Deterministically expose the firmware HOLD/FAULT telemetry contract."""
        if not 1 <= node <= 7:
            raise ValueError("node must be in 1..7")
        index = node - 1
        age = self.config.feedback_fault_ms if severe else self.config.feedback_hold_ms
        self._feedback_age_ms[index] = age
        self._feedback_loss_count[index] += 1
        self._consecutive_feedback_loss[index] += 1
        self._node_fault_bits[index] |= int(NodeFaultBits.FEEDBACK_STALE)
        self._node_validity[index] = int(self._node_validity[index]) & (
            0xFF ^ int(NodeValidityBits.POSITION)
        )
        self._mode = ControlMode.FAULT if severe else ControlMode.HOLD
        self._hold_reason_bits |= int(HoldReasonBits.FEEDBACK_STALE)
        if severe:
            self._fault_bits |= int(FaultBits.FEEDBACK_LOST)
            self._lease = False
            self._lease_deadline_ns = None
            self._target_deadline_ns = None
        if self._open:
            self._emit_state(self._last_received)

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
                    pack_hello_ack(self.config.config_hash_bytes, 0, "fake-mcu-v2"),
                )
            )
            return
        if packet.message_type == MessageType.ESTOP:
            self._mode = ControlMode.FAULT
            self._fault_bits |= int(FaultBits.EMERGENCY_STOP)
            self._lease = False
            self._lease_deadline_ns = None
            self._target_deadline_ns = None
            self._ack(packet)
            self._emit_state(packet.sequence)
            return
        if packet.message_type == MessageType.HOLD:
            self._mode = ControlMode.HOLD
            self._hold_reason_bits |= int(HoldReasonBits.OPERATOR)
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
            self._hold_reason_bits = 0
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
            self._hold_reason_bits = (
                int(HoldReasonBits.OPERATOR)
                if requested_mode == ControlMode.HOLD
                else 0
            )
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
            self._hold_reason_bits = 0
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
            self._hold_reason_bits |= int(HoldReasonBits.OPERATOR)
            self._lease = False
            self._lease_deadline_ns = None
            self._target_deadline_ns = None
            self._ack(packet)
            return
        self._nack(packet, ResultCode.UNSUPPORTED)

    def send_raw_frame_for_fault_injection(self, frame: bytes) -> int:
        """Pass one bounded wire frame through the Fake MCU receive boundary.

        The normal host path remains packet based. This deliberately narrow entry
        point exists only so the gated fault tool can prove that malformed wire
        frames are discarded before they reach the command dispatcher.
        """
        if not self._open:
            raise TransportClosed("fake MCU is closed")
        if not frame or not frame.endswith(b"\x00") or len(frame) > 600:
            raise ValueError("fault-injection frame must be bounded and zero-delimited")
        dropped_before = self._raw_decoder.dropped_frames
        for packet in self._raw_decoder.feed(frame):
            self.send(packet)
        return self._raw_decoder.dropped_frames - dropped_before

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
            fault_bits=self._fault_bits,
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
            following_error=np.zeros(7, dtype=np.float32),
            following_error_duration_ms=np.zeros(7, dtype=np.uint32),
            feedback_age_ms=self._feedback_age_ms.copy(),
            feedback_loss_count=self._feedback_loss_count.copy(),
            consecutive_feedback_loss=self._consecutive_feedback_loss.copy(),
            node_fault_bits=self._node_fault_bits.copy(),
            node_validity=self._node_validity.copy(),
            hold_reason_bits=self._hold_reason_bits,
            telemetry_validity=int(
                TelemetryValidityBits.FOLLOWING_ERROR
                | TelemetryValidityBits.CAN_FEEDBACK
            ),
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
            self._hold_reason_bits |= int(HoldReasonBits.LEASE_TIMEOUT)
            self._emit_state(self._last_received)
            return
        if self._target_deadline_ns is not None and now_ns >= self._target_deadline_ns:
            self._target_deadline_ns = None
            self._mode = ControlMode.HOLD
            self._hold_reason_bits |= int(HoldReasonBits.TARGET_TIMEOUT)
            self._emit_state(self._last_received)
