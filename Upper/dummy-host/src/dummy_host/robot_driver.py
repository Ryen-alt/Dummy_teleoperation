from __future__ import annotations

import queue
import secrets
import threading
import time
from collections.abc import Callable

import numpy as np

from .cameras import Camera, CameraManager
from .control import ActionGateway
from .domain import ActionProposal, ActionSpace, RobotHealth
from .protocol import (
    ACQUIRE_CONTROL,
    SET_MODE,
    MessageType,
    Packet,
    ResultCode,
    monotonic_us,
    pack_hello,
    pack_joint_target,
    unpack_ack,
    unpack_hello_ack,
    unpack_state,
)
from .safety import SafetyError, SafetyFilter
from .schema import AppliedAction, ConfigError, ControlMode, RobotConfig, RobotState
from .sync import ObservationSynchronizer
from .transport_serial import PacketTransport, TransportError


class RobotError(RuntimeError):
    pass


class CommandRejected(RobotError):
    pass


class DummyRobot:
    def __init__(
        self,
        config: RobotConfig,
        transport: PacketTransport,
        *,
        camera: Camera | None = None,
        camera_manager: CameraManager | None = None,
        allow_unverified_hardware: bool = False,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        response_timeout_s: float = 0.5,
        connect_timeout_s: float = 2.0,
    ) -> None:
        self.config = config
        self.transport = transport
        if camera is not None and camera_manager is not None:
            raise ValueError("provide camera or camera_manager, not both")
        self.camera = camera
        self.camera_manager = (
            camera_manager
            if camera_manager is not None
            else (None if camera is None else CameraManager({camera.role: camera}))
        )
        self.observation_synchronizer = (
            None if self.camera_manager is None else ObservationSynchronizer(self.camera_manager)
        )
        self.allow_unverified_hardware = allow_unverified_hardware
        self.clock_ns = clock_ns
        if response_timeout_s <= 0:
            raise ValueError("response_timeout_s must be positive")
        if connect_timeout_s <= 0:
            raise ValueError("connect_timeout_s must be positive")
        self.response_timeout_s = response_timeout_s
        self.connect_timeout_s = connect_timeout_s
        self.safety = SafetyFilter(config)
        self.action_gateway = ActionGateway(config, self.safety)
        self.session_id = secrets.randbits(32) or 1
        self._sequence = 0
        self._state: RobotState | None = None
        self._state_condition = threading.Condition()
        self._pending: dict[int, queue.Queue[Packet]] = {}
        self._pending_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader: threading.Thread | None = None
        self._connected = False
        self._control_acquired = False
        self._reader_error: BaseException | None = None
        self.firmware_version: str | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected and self._reader_error is None

    def connect(self) -> None:
        if self._connected:
            return
        with self._state_condition:
            self._state = None
        self.transport.open()
        self._stop.clear()
        self._reader_error = None
        self._reader = threading.Thread(target=self._read_loop, name="dummy-robot-rx", daemon=True)
        self._reader.start()
        deadline = time.monotonic() + self.connect_timeout_s
        try:
            response = self._hello_with_retry(deadline)
            if response.message_type != MessageType.HELLO_ACK:
                raise RobotError(f"expected HELLO_ACK, received {response.message_type.name}")
            remote_hash, _, self.firmware_version = unpack_hello_ack(response.payload)
            if remote_hash != self.config.config_hash_bytes:
                raise ConfigError(
                    f"firmware config hash {remote_hash.hex()} does not match host {self.config.config_hash}"
                )
            self._wait_for_first_state(deadline)
            self._connected = True
            if self.camera_manager is not None:
                self.camera_manager.start()
        except BaseException:
            self._connected = False
            self._stop_reader_and_transport()
            raise

    def disconnect(self) -> None:
        if self._control_acquired:
            try:
                self.hold()
                self.release_control()
            except BaseException:
                pass
        if self.camera_manager is not None:
            self.camera_manager.stop()
        self._stop_reader_and_transport()
        self._connected = False

    def acquire_control(self, mode: str | ControlMode) -> None:
        self._require_connected()
        target_mode = ControlMode[mode.upper()] if isinstance(mode, str) else ControlMode(mode)
        if target_mode not in (ControlMode.TELEOP, ControlMode.POLICY):
            raise RobotError("control can only be acquired in TELEOP or POLICY mode")
        if (
            (
                not self.config.hardware_parameters_verified
                or not self.config.external_target_execution_ready
            )
            and not self.transport.is_simulated
            and not self.allow_unverified_hardware
        ):
            raise ConfigError("real external target execution is not verified and ready")
        self._expect_ack(
            self._request(MessageType.ACQUIRE_CONTROL, ACQUIRE_CONTROL.pack(self.config.lease_timeout_ms)),
            MessageType.ACQUIRE_CONTROL,
        )
        self._expect_ack(
            self._request(MessageType.SET_MODE, SET_MODE.pack(int(target_mode))),
            MessageType.SET_MODE,
        )
        self._wait_for_mode(target_mode)
        self._control_acquired = True
        self.action_gateway.reset()

    def release_control(self) -> None:
        if not self._connected or not self._control_acquired:
            return
        self._expect_ack(self._request(MessageType.RELEASE_CONTROL), MessageType.RELEASE_CONTROL)
        self._control_acquired = False
        self.action_gateway.reset()

    def read_state(self, max_age_ms: int | None = None) -> RobotState:
        self._raise_reader_error()
        with self._state_condition:
            state = self._state
        if state is None:
            raise RobotError("no robot state has been received")
        max_age_ms = self.config.max_state_age_ms if max_age_ms is None else max_age_ms
        age_ms = (self.clock_ns() - state.monotonic_ns) / 1e6
        if age_ms < 0 or age_ms > max_age_ms:
            raise RobotError(f"robot state is stale ({age_ms:.1f} ms)")
        return state

    def get_observation(self) -> dict[str, object]:
        state = self.read_state()
        if self.observation_synchronizer is None:
            return {
                "observation.state": state.position.copy(),
                "timestamp_ns": state.monotonic_ns,
                "gripper_state_valid": state.gripper_valid,
            }
        return self.observation_synchronizer.build(state).as_policy_dict()

    def send_action(
        self,
        action: np.ndarray,
        *,
        max_velocity_rad_s: np.ndarray | None = None,
    ) -> AppliedAction:
        return self.apply_absolute_action(
            action,
            source="direct",
            max_velocity_rad_s=max_velocity_rad_s,
        )

    def apply_absolute_action(
        self,
        action: np.ndarray,
        *,
        source: str = "direct",
        max_velocity_rad_s: np.ndarray | None = None,
    ) -> AppliedAction:
        self._require_control()
        now_ns = self.clock_ns()
        try:
            proposal = ActionProposal(
                source=source,
                action_space=ActionSpace.JOINT_POSITION_ABSOLUTE,
                values=action,
                generated_at_ns=now_ns,
                valid_until_ns=now_ns + self.config.target_ttl_ms * 1_000_000,
            )
        except ValueError:
            self._best_effort_hold()
            raise
        return self.submit_action(proposal, max_velocity_rad_s=max_velocity_rad_s)

    def submit_action(
        self,
        proposal: ActionProposal,
        *,
        max_velocity_rad_s: np.ndarray | None = None,
    ) -> AppliedAction:
        self._require_control()
        now_ns = self.clock_ns()
        state = self.read_state()
        try:
            result = self.action_gateway.evaluate(
                proposal,
                state,
                now_ns,
                velocity_limit_rad_s=max_velocity_rad_s,
            )
        except SafetyError:
            self._best_effort_hold()
            raise
        sequence = self._next_sequence()
        packet = Packet(
            MessageType.SET_JOINT_TARGET,
            self.session_id,
            sequence,
            monotonic_us(now_ns),
            pack_joint_target(
                result.applied,
                self.config.joint_velocity_limit_rad_s
                if max_velocity_rad_s is None
                else max_velocity_rad_s,
                self.config.target_ttl_ms,
            ),
        )
        response = self._send_wait(packet)
        self._expect_ack(response, MessageType.SET_JOINT_TARGET)
        return AppliedAction(
            result.requested,
            result.applied,
            sequence,
            now_ns,
            result.clipped,
            result.reasons,
            canonical=result.applied,
            source=proposal.source,
        )

    def heartbeat(self) -> None:
        self._require_control()
        self._expect_ack(self._request(MessageType.HEARTBEAT), MessageType.HEARTBEAT)

    def hold(self) -> None:
        self._require_connected()
        self._expect_ack(self._request(MessageType.HOLD), MessageType.HOLD)
        self._wait_for_mode(ControlMode.HOLD)
        self.action_gateway.reset()

    def emergency_stop(self) -> None:
        self._require_connected()
        self._expect_ack(self._request(MessageType.ESTOP), MessageType.ESTOP)
        self._wait_for_mode(ControlMode.FAULT)
        self._control_acquired = False
        self.action_gateway.reset()

    def health(self) -> RobotHealth:
        now_ns = self.clock_ns()
        with self._state_condition:
            state = self._state
        if state is None:
            return RobotHealth(self.is_connected, False, None, 0, None)
        age_ms = (now_ns - state.monotonic_ns) / 1e6
        return RobotHealth(
            connected=self.is_connected,
            state_fresh=0 <= age_ms <= self.config.max_state_age_ms,
            mode=state.mode,
            fault_bits=state.fault_bits,
            state_age_ms=age_ms,
            details={
                "last_received_sequence": state.last_received_sequence,
                "last_applied_sequence": state.last_applied_sequence,
                "target_age_ms": state.target_age_ms,
                "hold_reason_bits": state.hold_reason_bits,
                "telemetry_validity": state.telemetry_validity,
                "following_error": state.following_error.tolist(),
                "feedback_age_ms": state.feedback_age_ms.tolist(),
                "feedback_loss_count": state.feedback_loss_count.tolist(),
                "node_fault_bits": state.node_fault_bits.tolist(),
            },
        )

    def __enter__(self) -> "DummyRobot":
        self.connect()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.disconnect()

    def _request(self, message_type: MessageType, payload: bytes = b"") -> Packet:
        now_ns = self.clock_ns()
        return self._send_wait(
            Packet(message_type, self.session_id, self._next_sequence(), monotonic_us(now_ns), payload)
        )

    def _hello_with_retry(self, deadline: float) -> Packet:
        payload = pack_hello(self.config.config_hash_bytes)
        last_timeout: RobotError | None = None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RobotError(
                    f"timeout waiting for HELLO response after {self.connect_timeout_s:.1f} s"
                ) from last_timeout
            now_ns = self.clock_ns()
            packet = Packet(
                MessageType.HELLO,
                self.session_id,
                self._next_sequence(),
                monotonic_us(now_ns),
                payload,
            )
            try:
                return self._send_wait(
                    packet,
                    timeout_s=min(self.response_timeout_s, remaining),
                )
            except RobotError as exc:
                if not isinstance(exc.__cause__, queue.Empty):
                    raise
                last_timeout = exc

    def _wait_for_first_state(self, deadline: float) -> None:
        with self._state_condition:
            while self._state is None:
                self._raise_reader_error()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RobotError(
                        "timeout waiting for first STATE after HELLO; "
                        "check that the firmware uses the latest HELLO session for telemetry"
                    )
                self._state_condition.wait(remaining)

    def _send_wait(self, packet: Packet, *, timeout_s: float | None = None) -> Packet:
        pending: queue.Queue[Packet] = queue.Queue(maxsize=1)
        with self._pending_lock:
            self._pending[packet.sequence] = pending
        try:
            self.transport.send(packet)
            try:
                response = pending.get(
                    timeout=self.response_timeout_s if timeout_s is None else timeout_s
                )
            except queue.Empty as exc:
                self._raise_reader_error()
                raise RobotError(f"timeout waiting for {packet.message_type.name} response") from exc
            if response.message_type == MessageType.NACK:
                ack = unpack_ack(response.payload)
                raise CommandRejected(f"{ack.request_type.name} rejected: {ack.result.name} detail={ack.detail}")
            if response.session_id != packet.session_id:
                raise RobotError("response belongs to a stale control session")
            return response
        finally:
            with self._pending_lock:
                self._pending.pop(packet.sequence, None)

    def _read_loop(self) -> None:
        try:
            while not self._stop.is_set():
                packet = self.transport.receive(timeout=0.1)
                if packet is None:
                    continue
                if packet.message_type == MessageType.STATE:
                    if packet.session_id != self.session_id:
                        continue
                    state = unpack_state(packet.payload, self.clock_ns())
                    if state.config_hash != self.config.config_hash:
                        raise ConfigError("STATE configuration hash mismatch")
                    with self._state_condition:
                        self._state = state
                        self._state_condition.notify_all()
                    continue
                with self._pending_lock:
                    pending = self._pending.get(packet.sequence)
                if pending is not None:
                    try:
                        pending.put_nowait(packet)
                    except queue.Full:
                        pass
        except BaseException as exc:
            if not self._stop.is_set():
                self._reader_error = exc
                self._stop.set()
                with self._state_condition:
                    self._state_condition.notify_all()

    def _expect_ack(self, packet: Packet, expected: MessageType) -> None:
        if packet.message_type != MessageType.ACK:
            raise RobotError(f"expected ACK for {expected.name}, received {packet.message_type.name}")
        ack = unpack_ack(packet.payload)
        if ack.request_type != expected or ack.result != ResultCode.OK:
            raise CommandRejected(f"unexpected ACK: {ack}")

    def _wait_for_mode(self, expected: ControlMode) -> None:
        deadline = time.monotonic() + self.response_timeout_s
        with self._state_condition:
            while self._state is None or self._state.mode != expected:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RobotError(f"timeout waiting for STATE mode {expected.name}")
                self._state_condition.wait(remaining)

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        if self._sequence == 0:
            self._sequence = 1
        return self._sequence

    def _best_effort_hold(self) -> None:
        try:
            self.hold()
        except BaseException:
            pass

    def _require_connected(self) -> None:
        self._raise_reader_error()
        if not self._connected:
            raise RobotError("robot is not connected")

    def _require_control(self) -> None:
        self._require_connected()
        if not self._control_acquired:
            raise RobotError("control lease has not been acquired")

    def _raise_reader_error(self) -> None:
        if self._reader_error is not None:
            raise TransportError(f"robot receive loop failed: {self._reader_error}") from self._reader_error

    def _stop_reader_and_transport(self) -> None:
        self._stop.set()
        self.transport.close()
        if self._reader is not None and self._reader is not threading.current_thread():
            self._reader.join(timeout=1.0)
        self._reader = None
