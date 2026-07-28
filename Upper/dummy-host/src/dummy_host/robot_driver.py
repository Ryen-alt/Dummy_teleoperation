from __future__ import annotations

import queue
import secrets
import threading
import time
from collections.abc import Callable

import numpy as np

from .cameras import D435Camera
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
        camera: D435Camera | None = None,
        allow_unverified_hardware: bool = False,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        response_timeout_s: float = 0.5,
    ) -> None:
        self.config = config
        self.transport = transport
        self.camera = camera
        self.allow_unverified_hardware = allow_unverified_hardware
        self.clock_ns = clock_ns
        self.response_timeout_s = response_timeout_s
        self.safety = SafetyFilter(config)
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
        self.transport.open()
        self._stop.clear()
        self._reader_error = None
        self._reader = threading.Thread(target=self._read_loop, name="dummy-robot-rx", daemon=True)
        self._reader.start()
        try:
            response = self._request(MessageType.HELLO, pack_hello(self.config.config_hash_bytes))
            if response.message_type != MessageType.HELLO_ACK:
                raise RobotError(f"expected HELLO_ACK, received {response.message_type.name}")
            remote_hash, _, self.firmware_version = unpack_hello_ack(response.payload)
            if remote_hash != self.config.config_hash_bytes:
                raise ConfigError(
                    f"firmware config hash {remote_hash.hex()} does not match host {self.config.config_hash}"
                )
            self._connected = True
            if self.camera is not None:
                self.camera.start()
        except BaseException:
            self._stop_reader_and_transport()
            raise

    def disconnect(self) -> None:
        if self._control_acquired:
            try:
                self.hold()
                self.release_control()
            except BaseException:
                pass
        if self.camera is not None:
            self.camera.stop()
        self._stop_reader_and_transport()
        self._connected = False

    def acquire_control(self, mode: str | ControlMode) -> None:
        self._require_connected()
        target_mode = ControlMode[mode.upper()] if isinstance(mode, str) else ControlMode(mode)
        if target_mode not in (ControlMode.TELEOP, ControlMode.POLICY):
            raise RobotError("control can only be acquired in TELEOP or POLICY mode")
        if (
            not self.config.hardware_parameters_verified
            and not self.transport.is_simulated
            and not self.allow_unverified_hardware
        ):
            raise ConfigError("hardware parameters are not verified; refusing real control acquisition")
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
        self.safety.reset()

    def release_control(self) -> None:
        if not self._connected or not self._control_acquired:
            return
        self._expect_ack(self._request(MessageType.RELEASE_CONTROL), MessageType.RELEASE_CONTROL)
        self._control_acquired = False
        self.safety.reset()

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
        if self.camera is None:
            return {
                "observation.state": state.position.copy(),
                "timestamp_ns": state.monotonic_ns,
                "gripper_state_valid": state.gripper_valid,
            }
        return ObservationSynchronizer(self.camera).build(state).as_policy_dict()

    def send_action(self, action: np.ndarray) -> AppliedAction:
        self._require_control()
        now_ns = self.clock_ns()
        state = self.read_state()
        try:
            result = self.safety.apply(action, state, now_ns)
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
                self.config.joint_velocity_limit_rad_s,
                self.config.target_ttl_ms,
            ),
        )
        response = self._send_wait(packet)
        self._expect_ack(response, MessageType.SET_JOINT_TARGET)
        return AppliedAction(result.requested, result.applied, sequence, now_ns, result.clipped, result.reasons)

    def heartbeat(self) -> None:
        self._require_control()
        self._expect_ack(self._request(MessageType.HEARTBEAT), MessageType.HEARTBEAT)

    def hold(self) -> None:
        self._require_connected()
        self._expect_ack(self._request(MessageType.HOLD), MessageType.HOLD)
        self.safety.reset()

    def emergency_stop(self) -> None:
        self._require_connected()
        self._expect_ack(self._request(MessageType.ESTOP), MessageType.ESTOP)
        self._control_acquired = False
        self.safety.reset()

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

    def _send_wait(self, packet: Packet) -> Packet:
        pending: queue.Queue[Packet] = queue.Queue(maxsize=1)
        with self._pending_lock:
            self._pending[packet.sequence] = pending
        try:
            self.transport.send(packet)
            try:
                response = pending.get(timeout=self.response_timeout_s)
            except queue.Empty as exc:
                self._raise_reader_error()
                raise RobotError(f"timeout waiting for {packet.message_type.name} response") from exc
            if response.message_type == MessageType.NACK:
                ack = unpack_ack(response.payload)
                raise CommandRejected(f"{ack.request_type.name} rejected: {ack.result.name} detail={ack.detail}")
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
