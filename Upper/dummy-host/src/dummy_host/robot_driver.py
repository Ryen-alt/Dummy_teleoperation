from __future__ import annotations

import queue
import secrets
import threading
import time
import heapq
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from .cameras import Camera, CameraManager
from .control import ActionGateway
from .domain import (
    ActionLifecycleUpdate,
    ActionProgressFlags,
    ActionProposal,
    ActionSpace,
    ActionStage,
    RobotHealth,
)
from .protocol import (
    CAPABILITY_MULTI_CHANNEL_SEQUENCE,
    CAPABILITY_TARGET_KEEPALIVE,
    CAPABILITY_CAN_TX_COMPLETE_EXACT,
    CAPABILITY_CONTROL_FRESHNESS_TOKEN,
    CAPABILITY_TIME_SYNC,
    CAPABILITY_CAN_DIAGNOSTICS,
    CAPABILITY_CAN_DIAGNOSTICS_V2,
    ACK_DETAIL_FEEDBACK_NOT_READY,
    ACQUIRE_CONTROL,
    ActionProgressStage,
    CanDiagnostics,
    SET_MODE,
    MessageType,
    Packet,
    ResultCode,
    monotonic_us,
    pack_hello,
    pack_joint_target,
    pack_target_keepalive,
    pack_time_sync,
    unpack_ack,
    unpack_action_progress,
    unpack_can_diagnostics,
    unpack_hello_ack,
    unpack_state,
    unpack_time_sync_ack,
)
from .safety import SafetyError, SafetyFilter
from .schema import AppliedAction, ConfigError, ControlMode, RobotConfig, RobotState
from .sync import ObservationSynchronizer
from .transport_serial import (
    PacketTransport,
    TransportError,
    TransportTxUpdate,
    TxOutcome,
)
from .time_sync import TimeSyncExchange


class RobotError(RuntimeError):
    pass


class CommandRejected(RobotError):
    def __init__(
        self,
        message: str,
        *,
        result: ResultCode | None = None,
        detail: int = 0,
    ) -> None:
        super().__init__(message)
        self.result = result
        self.detail = detail


class ActionCreditUnavailable(RobotError):
    """The preceding motion action has not reached exact CAN TX completion."""


@dataclass(frozen=True)
class ActionCredit:
    reservation_id: int
    control_tick_id: int
    reserved_ns: int


class DummyRobot:
    def __init__(
        self,
        config: RobotConfig,
        transport: PacketTransport,
        *,
        camera: Camera | None = None,
        camera_manager: CameraManager | None = None,
        allow_unverified_hardware: bool = False,
        acceptance_session: bool = False,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        response_timeout_s: float = 0.5,
        action_ack_timeout_s: float = 0.05,
        connect_timeout_s: float = 2.0,
        action_observation_timeout_s: float = 0.25,
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
        self.acceptance_session = acceptance_session
        self.clock_ns = clock_ns
        if response_timeout_s <= 0:
            raise ValueError("response_timeout_s must be positive")
        if action_ack_timeout_s <= 0:
            raise ValueError("action_ack_timeout_s must be positive")
        if connect_timeout_s <= 0:
            raise ValueError("connect_timeout_s must be positive")
        if action_observation_timeout_s <= 0:
            raise ValueError("action_observation_timeout_s must be positive")
        self.response_timeout_s = response_timeout_s
        self.action_ack_timeout_s = action_ack_timeout_s
        self.connect_timeout_s = connect_timeout_s
        self.action_observation_timeout_s = action_observation_timeout_s
        self.safety = SafetyFilter(config)
        self.action_gateway = ActionGateway(config, self.safety)
        self.session_id = secrets.randbits(32) or 1
        self._sequence = 0
        self._control_tick_id = 0
        self._sequence_lock = threading.Lock()
        self._state: RobotState | None = None
        self._state_condition = threading.Condition()
        self._pending: dict[int, queue.Queue[Packet]] = {}
        self._ack_deadlines: dict[int, int] = {}
        self._completion_deadlines: dict[int, int] = {}
        self._deadline_heap: list[tuple[int, int, int]] = []
        self._action_stages: dict[int, set[ActionStage]] = {}
        self._action_control_ticks: dict[int, int] = {}
        self._active_action_sequence: int | None = None
        self._action_credit_generation = 0
        self._action_credit: ActionCredit | None = None
        self._action_credit_sequence: int | None = None
        self._action_listener: Callable[[ActionLifecycleUpdate], None] | None = None
        self._pending_lock = threading.Lock()
        self._watchdog_condition = threading.Condition(self._pending_lock)
        self._stop = threading.Event()
        self._reader: threading.Thread | None = None
        self._watchdog: threading.Thread | None = None
        self._connected = False
        self._control_acquired = False
        self._reader_error: BaseException | None = None
        self.firmware_version: str | None = None
        self.firmware_capabilities = 0
        set_tx_observer = getattr(self.transport, "set_tx_observer", None)
        if callable(set_tx_observer):
            set_tx_observer(self._on_serial_tx)

    def set_action_lifecycle_listener(
        self, listener: Callable[[ActionLifecycleUpdate], None] | None
    ) -> None:
        with self._pending_lock:
            self._action_listener = listener

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
        self._watchdog = threading.Thread(
            target=self._action_watchdog_loop,
            name="dummy-action-watchdog",
            daemon=True,
        )
        self._reader.start()
        self._watchdog.start()
        deadline = time.monotonic() + self.connect_timeout_s
        try:
            response = self._hello_with_retry(deadline)
            if response.message_type != MessageType.HELLO_ACK:
                raise RobotError(f"expected HELLO_ACK, received {response.message_type.name}")
            (
                remote_hash,
                self.firmware_capabilities,
                self.firmware_version,
            ) = unpack_hello_ack(response.payload)
            if remote_hash != self.config.config_hash_bytes:
                raise ConfigError(
                    f"firmware config hash {remote_hash.hex()} does not match host {self.config.config_hash}"
                )
            if (
                not self.transport.is_simulated
                and self.firmware_version != "dummy-ref-v2.2.1"
            ):
                raise ConfigError(
                    "protocol v5 host requires firmware dummy-ref-v2.2.1 exactly; "
                    f"received {self.firmware_version!r}"
                )
            if (
                not self.transport.is_simulated
                and self.firmware_capabilities
                & (
                    CAPABILITY_MULTI_CHANNEL_SEQUENCE
                    | CAPABILITY_TARGET_KEEPALIVE
                    | CAPABILITY_CAN_TX_COMPLETE_EXACT
                    | CAPABILITY_CONTROL_FRESHNESS_TOKEN
                    | CAPABILITY_TIME_SYNC
                    | CAPABILITY_CAN_DIAGNOSTICS
                    | CAPABILITY_CAN_DIAGNOSTICS_V2
                )
                != (
                    CAPABILITY_MULTI_CHANNEL_SEQUENCE
                    | CAPABILITY_TARGET_KEEPALIVE
                    | CAPABILITY_CAN_TX_COMPLETE_EXACT
                    | CAPABILITY_CONTROL_FRESHNESS_TOKEN
                    | CAPABILITY_TIME_SYNC
                    | CAPABILITY_CAN_DIAGNOSTICS
                    | CAPABILITY_CAN_DIAGNOSTICS_V2
                )
            ):
                raise ConfigError(
                    "dummy-ref-v2.2.1 firmware is missing required protocol-v5 "
                    "execution-evidence capabilities; rebuild and reflash v2.2.1"
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
        if not self.transport.is_simulated and not self.allow_unverified_hardware:
            if not self.config.hardware_parameters_verified:
                raise ConfigError("real external target execution hardware is not verified")
            if target_mode is ControlMode.POLICY and not self.config.external_target_execution_ready:
                raise ConfigError("real POLICY execution is not production-ready")
            teleop_ready = self.config.external_target_execution_ready or (
                self.config.external_target_acceptance_ready
                and self.acceptance_session
            )
            if target_mode is ControlMode.TELEOP and not teleop_ready:
                raise ConfigError(
                    "real TELEOP execution requires the production gate or an "
                    "explicit acceptance session"
                )
        self.wait_for_feedback_ready()
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
        self._active_action_sequence = None
        self._clear_action_credit()
        self.action_gateway.reset()

    def wait_for_feedback_ready(
        self,
        *,
        consecutive_sweeps: int = 3,
        timeout_s: float | None = None,
        state_validator: Callable[[RobotState], bool] | None = None,
    ) -> RobotState:
        """Wait for distinct coherent feedback sweeps before ACQUIRE_CONTROL.

        When supplied, ``state_validator`` must accept every newly observed
        sweep. A rejected sweep resets the consecutive-ready window.
        """

        if consecutive_sweeps <= 0:
            raise ValueError("consecutive_sweeps must be positive")
        timeout_s = self.connect_timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout_s
        observed: list[int] = []
        with self._state_condition:
            while True:
                self._raise_reader_error()
                state = self._state
                if state is not None and state.coherent:
                    if not observed or observed[-1] != state.coherent_sweep_id:
                        if state_validator is None or state_validator(state):
                            observed.append(state.coherent_sweep_id)
                            observed = observed[-consecutive_sweeps:]
                        else:
                            observed.clear()
                    if (
                        len(observed) >= consecutive_sweeps
                        and state.can_transport_status & 0x40
                    ):
                        return state
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    status = 0 if state is None else state.can_transport_status
                    sweep = 0 if state is None else state.coherent_sweep_id
                    raise RobotError(
                        "timeout in WAIT_FEEDBACK_READY: "
                        f"coherent_sweeps={len(observed)}/{consecutive_sweeps} "
                        f"latest_sweep={sweep} can_status=0x{status:02x}"
                    )
                self._state_condition.wait(min(remaining, 0.1))

    def release_control(self) -> None:
        if not self._connected or not self._control_acquired:
            return
        self._expect_ack(self._request(MessageType.RELEASE_CONTROL), MessageType.RELEASE_CONTROL)
        self._control_acquired = False
        self._active_action_sequence = None
        self._clear_action_credit()
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

    def enqueue_absolute_action(
        self,
        action: np.ndarray,
        *,
        source: str = "direct",
        max_velocity_rad_s: np.ndarray | None = None,
        generated_at_ns: int | None = None,
        action_credit: ActionCredit | None = None,
    ) -> AppliedAction:
        """Safety-check and non-blockingly enqueue one target for serial TX.

        Completion is reported through ActionLifecycleUpdate; the caller never
        waits for the serial writer, ACK, CAN fan-out or motor observation.
        """

        self._require_control()
        received_ns = self.clock_ns()
        owned_credit = action_credit
        if owned_credit is None:
            control_tick_id = self.advance_control_tick()
            owned_credit = self.reserve_action_credit(
                control_tick_id, reserved_ns=received_ns
            )
            if owned_credit is None:
                raise ActionCreditUnavailable(
                    "previous action has not reached CAN_TX_COMPLETE_EXACT"
                )
        generated_ns = received_ns if generated_at_ns is None else generated_at_ns
        try:
            proposal = ActionProposal(
                source=source,
                action_space=ActionSpace.JOINT_POSITION_ABSOLUTE,
                values=action,
                generated_at_ns=generated_ns,
                valid_until_ns=generated_ns + self.config.target_ttl_ms * 1_000_000,
            )
            result = self.action_gateway.evaluate(
                proposal,
                self.read_state(),
                received_ns,
                velocity_limit_rad_s=max_velocity_rad_s,
            )
        except (ValueError, SafetyError):
            self.cancel_action_credit(owned_credit)
            raise
        accepted_ns = self.clock_ns()
        # Sequence allocation happens only after the candidate has passed the
        # safety gateway. Rejected desired snapshots therefore cannot consume
        # protocol sequence numbers or manufacture partial action lifecycles.
        sequence = self._next_sequence()
        self._prepare_action_sequence(sequence)
        self._bind_action_credit(owned_credit, sequence)
        self._emit_action_stage(sequence, ActionStage.RECEIVED, received_ns)
        self._emit_action_stage(sequence, ActionStage.SAFETY_ACCEPTED, accepted_ns)
        try:
            packet = Packet(
                MessageType.SET_JOINT_TARGET,
                self.session_id,
                sequence,
                monotonic_us(accepted_ns),
                pack_joint_target(
                    result.applied,
                    self.config.joint_velocity_limit_rad_s
                    if max_velocity_rad_s is None
                    else max_velocity_rad_s,
                    self.config.target_ttl_ms,
                    owned_credit.control_tick_id,
                ),
            )
        except BaseException as exc:
            self._emit_action_stage(
                sequence, ActionStage.FAILED, self.clock_ns(), detail=str(exc)
            )
            raise
        enqueued_ns = self.clock_ns()
        self._emit_action_stage(sequence, ActionStage.SEND_ENQUEUED, enqueued_ns)
        previous_active_sequence = self._active_action_sequence
        self._active_action_sequence = sequence
        try:
            self.transport.send(packet)
        except BaseException as exc:
            if self._active_action_sequence == sequence:
                self._active_action_sequence = previous_active_sequence
            self._emit_action_stage(
                sequence, ActionStage.FAILED, self.clock_ns(), detail=str(exc)
            )
            raise
        return AppliedAction(
            result.requested,
            result.applied,
            sequence,
            accepted_ns,
            result.clipped,
            result.reasons,
            canonical=result.applied,
            source=proposal.source,
            session_epoch=self.session_id,
            control_tick_id=owned_credit.control_tick_id,
        )

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
        control_tick_id = self._next_control_tick_id()
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
                control_tick_id,
            ),
        )
        response = self._send_wait(packet)
        self._expect_ack(response, MessageType.SET_JOINT_TARGET)
        self._active_action_sequence = sequence
        return AppliedAction(
            result.requested,
            result.applied,
            sequence,
            now_ns,
            result.clipped,
            result.reasons,
            canonical=result.applied,
            source=proposal.source,
            session_epoch=self.session_id,
            control_tick_id=control_tick_id,
        )

    def time_sync(self) -> TimeSyncExchange:
        """Perform one protocol-v5 four-timestamp clock exchange."""

        self._require_connected()
        if not self.firmware_capabilities & CAPABILITY_TIME_SYNC:
            raise RobotError("firmware does not advertise TIME_SYNC")
        host_t0_ns = self.clock_ns()
        response = self._request(MessageType.TIME_SYNC, pack_time_sync(host_t0_ns))
        host_t3_ns = self.clock_ns()
        if response.message_type != MessageType.TIME_SYNC_ACK:
            raise RobotError(
                f"expected TIME_SYNC_ACK, received {response.message_type.name}"
            )
        echoed_t0_ns, mcu_rx_us, mcu_tx_us = unpack_time_sync_ack(response.payload)
        if echoed_t0_ns != host_t0_ns:
            raise RobotError("TIME_SYNC_ACK does not echo this exchange")
        return TimeSyncExchange(host_t0_ns, mcu_rx_us, mcu_tx_us, host_t3_ns)

    def read_can_diagnostics(self) -> CanDiagnostics:
        self._require_connected()
        required = CAPABILITY_CAN_DIAGNOSTICS | CAPABILITY_CAN_DIAGNOSTICS_V2
        if self.firmware_capabilities & required != required:
            raise RobotError("firmware does not advertise CAN diagnostics v2")
        response = self._request(MessageType.GET_CAN_DIAGNOSTICS)
        if response.message_type != MessageType.CAN_DIAGNOSTICS:
            raise RobotError(
                f"expected CAN_DIAGNOSTICS, received {response.message_type.name}"
            )
        return unpack_can_diagnostics(response.payload)

    def heartbeat(self) -> None:
        self._require_control()
        self._expect_ack(self._request(MessageType.HEARTBEAT), MessageType.HEARTBEAT)

    def refresh_target(self, action_sequence: int, control_tick_id: int) -> None:
        """Refresh exactly one active motion target from a healthy control tick.

        This is deliberately separate from HEARTBEAT: a lease heartbeat must
        never make an old target live forever after the control loop stalls.
        The firmware accepts the refresh only while ``action_sequence`` is the
        currently active target.
        """

        self._require_control()
        if self._active_action_sequence != action_sequence:
            raise CommandRejected(
                "TARGET_KEEPALIVE rejected locally: BAD_SEQUENCE "
                f"active={self._active_action_sequence} requested={action_sequence}",
                result=ResultCode.BAD_SEQUENCE,
            )
        try:
            self._expect_ack(
                self._request(
                    MessageType.TARGET_KEEPALIVE,
                    pack_target_keepalive(action_sequence, control_tick_id),
                ),
                MessageType.TARGET_KEEPALIVE,
            )
        except CommandRejected as exc:
            # A refresh can already be inside the reliable serial channel when
            # the control thread observes exact fan-out and enqueues its next
            # target. If that newer target won the final wire-order race, the
            # old refresh is a safe no-op rather than a link failure. Every
            # other rejection remains fail-closed.
            if (
                exc.result is ResultCode.BAD_SEQUENCE
                and self._active_action_sequence != action_sequence
            ):
                return
            raise

    def hold(self) -> None:
        self._require_connected()
        self._expect_ack(self._request(MessageType.HOLD), MessageType.HOLD)
        self._wait_for_mode(ControlMode.HOLD)
        self._active_action_sequence = None
        self._clear_action_credit()
        self.action_gateway.reset()

    def request_priority_hold(self) -> None:
        """Queue HOLD immediately without blocking the caller for its ACK.

        Runtime failure paths use this before attempting recorder or Episode
        bookkeeping, both of which may themselves be the source of the
        exception.  SerialTransport gives HOLD safety priority and atomically
        preempts its pending motion mailbox.
        """

        self._require_connected()
        self._enqueue_priority_hold()

    def emergency_stop(self) -> None:
        self._require_connected()
        self._expect_ack(self._request(MessageType.ESTOP), MessageType.ESTOP)
        self._wait_for_mode(ControlMode.FAULT)
        self._control_acquired = False
        self._active_action_sequence = None
        self._clear_action_credit()
        self.action_gateway.reset()

    def health(self) -> RobotHealth:
        now_ns = self.clock_ns()
        with self._state_condition:
            state = self._state
        if state is None:
            return RobotHealth(self.is_connected, False, None, 0, None)
        age_ms = (now_ns - state.monotonic_ns) / 1e6
        transport_diagnostics = getattr(self.transport, "diagnostics", None)
        transport_details: dict[str, object] = {}
        if callable(transport_diagnostics):
            transport_details = vars(transport_diagnostics())
        return RobotHealth(
            connected=self.is_connected,
            state_fresh=0 <= age_ms <= self.config.max_state_age_ms,
            mode=state.mode,
            fault_bits=state.fault_bits,
            state_age_ms=age_ms,
            details={
                "last_received_sequence": state.last_received_sequence,
                "last_can_queued_exact_sequence": state.last_can_queued_exact_sequence,
                "target_age_ms": state.target_age_ms,
                "hold_reason_bits": state.hold_reason_bits,
                "telemetry_validity": state.telemetry_validity,
                "can_transport_status": state.can_transport_status,
                "following_error": state.following_error.tolist(),
                "feedback_age_ms": state.feedback_age_ms.tolist(),
                "feedback_loss_count": state.feedback_loss_count.tolist(),
                "node_fault_bits": state.node_fault_bits.tolist(),
                "coherent_sweep_id": state.coherent_sweep_id,
                "feedback_max_skew_us": state.feedback_max_skew_us,
                "coherent_reference_mcu_us": state.coherent_reference_mcu_us,
                "state_repeated": state.state_repeated,
                "transport": transport_details,
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
        payload = pack_hello(
            self.config.config_hash_bytes,
            CAPABILITY_MULTI_CHANNEL_SEQUENCE
            | CAPABILITY_TARGET_KEEPALIVE
            | CAPABILITY_CAN_TX_COMPLETE_EXACT
            | CAPABILITY_CONTROL_FRESHNESS_TOKEN
            | CAPABILITY_TIME_SYNC
            | CAPABILITY_CAN_DIAGNOSTICS
            | CAPABILITY_CAN_DIAGNOSTICS_V2,
        )
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
                if (
                    ack.request_type == MessageType.ACQUIRE_CONTROL
                    and ack.result == ResultCode.BAD_MODE
                    and ack.detail == ACK_DETAIL_FEEDBACK_NOT_READY
                ):
                    raise CommandRejected(
                        "ACQUIRE_CONTROL rejected: CAN feedback bootstrap is not ready "
                        f"(can_status=0x{self.read_state().can_transport_status:02x})",
                        result=ack.result,
                        detail=ack.detail,
                    )
                raise CommandRejected(
                    f"{ack.request_type.name} rejected: {ack.result.name} detail={ack.detail}",
                    result=ack.result,
                    detail=ack.detail,
                )
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
                    self._observe_action_progress(state)
                    continue
                if packet.message_type == MessageType.EVENT:
                    if packet.session_id != self.session_id:
                        continue
                    sequence, stage, stage_time_us, sweep_id = unpack_action_progress(
                        packet.payload
                    )
                    self._apply_action_progress(
                        sequence, stage, stage_time_us, sweep_id
                    )
                    continue
                with self._pending_lock:
                    pending = self._pending.get(packet.sequence)
                    is_action = packet.sequence in self._action_stages
                if pending is not None:
                    try:
                        pending.put_nowait(packet)
                    except queue.Full:
                        pass
                elif is_action:
                    if packet.message_type == MessageType.ACK:
                        ack = unpack_ack(packet.payload)
                        if (
                            ack.request_type == MessageType.SET_JOINT_TARGET
                            and ack.result == ResultCode.OK
                        ):
                            self._emit_action_stage(
                                packet.sequence,
                                ActionStage.ACKNOWLEDGED,
                                self.clock_ns(),
                                mcu_time_us=packet.sender_time_us,
                            )
                            with self._watchdog_condition:
                                self._ack_deadlines.pop(packet.sequence, None)
                                stages = self._action_stages.get(packet.sequence, set())
                                if not {
                                    ActionStage.CAN_QUEUED_EXACT,
                                    ActionStage.CAN_TX_COMPLETE_EXACT,
                                    ActionStage.POST_COMMAND_FEEDBACK,
                                }.issubset(stages):
                                    deadline_ns = self.clock_ns() + int(
                                        self.action_observation_timeout_s * 1e9
                                    )
                                    self._completion_deadlines[packet.sequence] = deadline_ns
                                    heapq.heappush(
                                        self._deadline_heap,
                                        (deadline_ns, 1, packet.sequence),
                                    )
                                self._watchdog_condition.notify_all()
                        else:
                            self._emit_action_stage(
                                packet.sequence,
                                ActionStage.REJECTED,
                                self.clock_ns(),
                                detail=f"unexpected ACK: {ack}",
                            )
                    else:
                        detail = packet.message_type.name
                        if packet.message_type == MessageType.NACK:
                            detail = str(unpack_ack(packet.payload))
                        self._emit_action_stage(
                            packet.sequence,
                            ActionStage.REJECTED
                            if packet.message_type == MessageType.NACK
                            else ActionStage.FAILED,
                            self.clock_ns(),
                            detail=detail,
                        )
        except BaseException as exc:
            if not self._stop.is_set():
                self._reader_error = exc
                self._stop.set()
                with self._state_condition:
                    self._state_condition.notify_all()

    def _observe_action_progress(self, state: RobotState) -> None:
        with self._pending_lock:
            known_sequences = frozenset(self._action_stages)
        for progress in state.action_progress:
            if progress.sequence not in known_sequences:
                continue
            if progress.flags & int(ActionProgressFlags.SUPERSEDED):
                emitted = self._emit_action_stage(
                    progress.sequence,
                    ActionStage.SUPERSEDED,
                    state.monotonic_ns,
                    mcu_time_us=progress.can_queued_mcu_us,
                    detail="firmware target was replaced before exact seven-node fan-out",
                )
                if emitted:
                    self._enqueue_priority_hold()
                continue
            if progress.flags & int(ActionProgressFlags.PREEMPTED_BY_SAFETY):
                self._emit_action_stage(
                    progress.sequence,
                    ActionStage.PREEMPTED_BY_SAFETY,
                    state.monotonic_ns,
                    detail="firmware safety transition preempted CAN fan-out",
                )
                continue
            if progress.flags & int(ActionProgressFlags.FAILED):
                emitted = self._emit_action_stage(
                    progress.sequence,
                    ActionStage.FAILED,
                    state.monotonic_ns,
                    detail="firmware CAN execution failed",
                )
                if emitted:
                    self._enqueue_priority_hold()
                continue
            if progress.flags & int(ActionProgressFlags.CAN_QUEUED_EXACT):
                self._emit_action_stage(
                    progress.sequence,
                    ActionStage.CAN_QUEUED_EXACT,
                    state.monotonic_ns,
                    mcu_time_us=progress.can_queued_mcu_us,
                )
            if progress.flags & int(ActionProgressFlags.CAN_TX_COMPLETE_EXACT):
                self._emit_action_stage(
                    progress.sequence,
                    ActionStage.CAN_TX_COMPLETE_EXACT,
                    state.monotonic_ns,
                    mcu_time_us=progress.can_tx_complete_mcu_us,
                )
            if progress.flags & int(ActionProgressFlags.POST_COMMAND_FEEDBACK):
                self._emit_action_stage(
                    progress.sequence,
                    ActionStage.POST_COMMAND_FEEDBACK,
                    state.monotonic_ns,
                    mcu_time_us=progress.post_feedback_mcu_us,
                )

    def _apply_action_progress(
        self,
        sequence: int,
        stage: ActionProgressStage,
        mcu_time_us: int,
        sweep_id: int,
    ) -> None:
        del sweep_id
        with self._pending_lock:
            if sequence not in self._action_stages:
                return
        mapping = {
            ActionProgressStage.CAN_QUEUED_EXACT: ActionStage.CAN_QUEUED_EXACT,
            ActionProgressStage.CAN_TX_COMPLETE_EXACT: ActionStage.CAN_TX_COMPLETE_EXACT,
            ActionProgressStage.POST_COMMAND_FEEDBACK: ActionStage.POST_COMMAND_FEEDBACK,
            ActionProgressStage.SUPERSEDED: ActionStage.SUPERSEDED,
            ActionProgressStage.PREEMPTED_BY_SAFETY: ActionStage.PREEMPTED_BY_SAFETY,
            ActionProgressStage.FAILED: ActionStage.FAILED,
        }
        emitted = self._emit_action_stage(
            sequence,
            mapping[stage],
            self.clock_ns(),
            mcu_time_us=mcu_time_us,
            detail=(
                "firmware target was replaced before exact seven-node fan-out"
                if stage is ActionProgressStage.SUPERSEDED
                else None
            ),
        )
        if emitted and stage in {
            ActionProgressStage.SUPERSEDED,
            ActionProgressStage.FAILED,
        }:
            self._enqueue_priority_hold()

    def _action_watchdog_loop(self) -> None:
        while not self._stop.is_set():
            now_ns = self.clock_ns()
            expired_ack: list[int] = []
            expired_completion: list[int] = []
            with self._watchdog_condition:
                while self._deadline_heap and self._deadline_heap[0][0] <= now_ns:
                    deadline_ns, deadline_kind, sequence = heapq.heappop(
                        self._deadline_heap
                    )
                    deadlines = (
                        self._ack_deadlines
                        if deadline_kind == 0
                        else self._completion_deadlines
                    )
                    if deadlines.get(sequence) != deadline_ns:
                        continue
                    deadlines.pop(sequence, None)
                    (expired_ack if deadline_kind == 0 else expired_completion).append(
                        sequence
                    )
                while self._deadline_heap:
                    deadline_ns, deadline_kind, sequence = self._deadline_heap[0]
                    deadlines = (
                        self._ack_deadlines
                        if deadline_kind == 0
                        else self._completion_deadlines
                    )
                    if deadlines.get(sequence) == deadline_ns:
                        break
                    heapq.heappop(self._deadline_heap)
                wait_s = 0.05
                if self._deadline_heap:
                    wait_s = max(
                        0.001,
                        min(wait_s, (self._deadline_heap[0][0] - now_ns) / 1e9),
                    )
                if not expired_ack and not expired_completion:
                    self._watchdog_condition.wait(wait_s)
                    continue
            for sequence in expired_ack:
                self._emit_action_stage(
                    sequence,
                    ActionStage.FAILED,
                    now_ns,
                    detail="timeout waiting for SET_JOINT_TARGET ACK",
                )
            for sequence in expired_completion:
                self._emit_action_stage(
                    sequence,
                    ActionStage.FAILED,
                    now_ns,
                    detail=(
                        "action did not reach CAN_TX_COMPLETE_EXACT and "
                        "POST_COMMAND_FEEDBACK within completion deadline"
                    ),
                )

    def _emit_action_stage(
        self,
        sequence: int,
        stage: ActionStage,
        host_time_ns: int,
        *,
        mcu_time_us: int = 0,
        detail: str | None = None,
    ) -> bool:
        with self._pending_lock:
            stages = self._action_stages.setdefault(sequence, set())
            if stage in stages:
                return False
            exact = ActionStage.CAN_TX_COMPLETE_EXACT in stages
            completed_before = {
                ActionStage.ACKNOWLEDGED,
                ActionStage.CAN_TX_COMPLETE_EXACT,
                ActionStage.POST_COMMAND_FEEDBACK,
            }.issubset(stages)
            if completed_before and stage in {
                ActionStage.SUPERSEDED,
                ActionStage.PREEMPTED_BY_SAFETY,
                ActionStage.REJECTED,
                ActionStage.FAILED,
            }:
                return False
            if exact and stage in {
                ActionStage.SUPERSEDED,
                ActionStage.PREEMPTED_BY_SAFETY,
            }:
                return False
            stages.add(stage)
            listener = self._action_listener
            control_tick_id = self._action_control_ticks.get(sequence, 0)
            completed = {
                ActionStage.ACKNOWLEDGED,
                ActionStage.CAN_QUEUED_EXACT,
                ActionStage.CAN_TX_COMPLETE_EXACT,
                ActionStage.POST_COMMAND_FEEDBACK,
            }.issubset(stages)
            terminal = completed or stage in (
                ActionStage.SUPERSEDED,
                ActionStage.PREEMPTED_BY_SAFETY,
                ActionStage.REJECTED,
                ActionStage.FAILED,
            )
            if terminal:
                self._ack_deadlines.pop(sequence, None)
                self._completion_deadlines.pop(sequence, None)
                self._watchdog_condition.notify_all()
                # Keep terminal records bounded while retaining a small audit
                # tail for delayed duplicate STATE frames.
                if len(self._action_stages) > 512:
                    oldest = next(iter(self._action_stages))
                    if oldest != sequence:
                        self._action_stages.pop(oldest, None)
                        self._action_control_ticks.pop(oldest, None)
            if stage is ActionStage.CAN_TX_COMPLETE_EXACT or terminal:
                self._release_action_credit_locked(sequence)
            if (
                stage
                in {
                    ActionStage.SUPERSEDED,
                    ActionStage.PREEMPTED_BY_SAFETY,
                    ActionStage.REJECTED,
                    ActionStage.FAILED,
                }
                and self._active_action_sequence == sequence
            ):
                self._active_action_sequence = None
        if listener is not None:
            listener(
                ActionLifecycleUpdate(
                    sequence,
                    stage,
                    host_time_ns,
                    mcu_time_us=mcu_time_us,
                    detail=detail,
                    session_epoch=self.session_id,
                    control_tick_id=control_tick_id,
                )
            )
        return True

    def _on_serial_tx(self, update: TransportTxUpdate) -> None:
        sequence = update.sequence
        with self._pending_lock:
            is_action = sequence in self._action_stages
        if not is_action:
            return
        if update.outcome is TxOutcome.SUPERSEDED:
            self._emit_action_stage(
                sequence,
                ActionStage.SUPERSEDED,
                update.finished_ns,
                detail="host latest-value mailbox replaced the unsent target",
            )
            self._enqueue_priority_hold()
            return
        if update.outcome is TxOutcome.PREEMPTED_BY_SAFETY:
            self._emit_action_stage(
                sequence,
                ActionStage.PREEMPTED_BY_SAFETY,
                update.finished_ns,
                detail="unsent target was cleared by a safety command",
            )
            return
        if update.started_ns:
            self._emit_action_stage(
                sequence, ActionStage.SERIAL_SEND_STARTED, update.started_ns
            )
        self._emit_action_stage(
            sequence,
            ActionStage.SERIAL_SEND_FINISHED
            if update.outcome is TxOutcome.SENT
            else ActionStage.FAILED,
            update.finished_ns,
            detail=update.detail,
        )
        if update.outcome is TxOutcome.SENT:
            with self._watchdog_condition:
                stages = self._action_stages.get(sequence, set())
                if ActionStage.ACKNOWLEDGED not in stages:
                    deadline_ns = update.finished_ns + int(
                        self.action_ack_timeout_s * 1e9
                    )
                    self._ack_deadlines[sequence] = deadline_ns
                    heapq.heappush(
                        self._deadline_heap,
                        (deadline_ns, 0, sequence),
                    )
                    self._watchdog_condition.notify_all()

    def _prepare_action_sequence(self, sequence: int) -> None:
        """Start an exact lifecycle even after the uint32 sequence wraps."""

        with self._watchdog_condition:
            self._action_stages.pop(sequence, None)
            self._action_control_ticks.pop(sequence, None)
            self._ack_deadlines.pop(sequence, None)
            self._completion_deadlines.pop(sequence, None)

    def _next_control_tick_id(self) -> int:
        with self._sequence_lock:
            self._control_tick_id = (self._control_tick_id + 1) & 0xFFFFFFFF
            if self._control_tick_id == 0:
                self._control_tick_id = 1
            return self._control_tick_id

    def advance_control_tick(self) -> int:
        """Publish one new control-health generation for target freshness."""

        self._require_control()
        return self._next_control_tick_id()

    def reserve_action_credit(
        self, control_tick_id: int, *, reserved_ns: int | None = None
    ) -> ActionCredit | None:
        """Atomically reserve the sole uncompleted-CAN motion slot."""

        self._require_control()
        if not 0 < control_tick_id <= 0xFFFFFFFF:
            raise ValueError("control_tick_id must be a non-zero uint32")
        with self._pending_lock:
            if self._action_credit is not None:
                return None
            self._action_credit_generation += 1
            credit = ActionCredit(
                self._action_credit_generation,
                control_tick_id,
                self.clock_ns() if reserved_ns is None else reserved_ns,
            )
            self._action_credit = credit
            self._action_credit_sequence = None
            return credit

    def cancel_action_credit(self, credit: ActionCredit) -> None:
        """Release an unbound reservation after candidate generation fails."""

        with self._pending_lock:
            if self._action_credit == credit and self._action_credit_sequence is None:
                self._action_credit = None

    def _bind_action_credit(self, credit: ActionCredit, sequence: int) -> None:
        with self._pending_lock:
            if self._action_credit != credit or self._action_credit_sequence is not None:
                raise ActionCreditUnavailable("action credit is stale or already consumed")
            self._action_credit_sequence = sequence
            self._action_control_ticks[sequence] = credit.control_tick_id

    def _release_action_credit_locked(self, sequence: int) -> None:
        if self._action_credit_sequence == sequence:
            self._action_credit = None
            self._action_credit_sequence = None

    def _clear_action_credit(self) -> None:
        with self._pending_lock:
            self._action_credit = None
            self._action_credit_sequence = None

    def _enqueue_priority_hold(self) -> None:
        """Non-blockingly preempt motion without waiting on the serial writer."""

        if self._stop.is_set():
            return
        now_ns = self.clock_ns()
        try:
            self.transport.send(
                Packet(
                    MessageType.HOLD,
                    self.session_id,
                    self._next_sequence(),
                    monotonic_us(now_ns),
                )
            )
        except BaseException as exc:
            self._reader_error = TransportError(f"cannot enqueue priority HOLD: {exc}")
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
        with self._sequence_lock:
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
        with self._watchdog_condition:
            self._watchdog_condition.notify_all()
        if self._watchdog is not None and self._watchdog is not threading.current_thread():
            self._watchdog.join(timeout=1.0)
        self._watchdog = None
        with self._pending_lock:
            self._ack_deadlines.clear()
            self._completion_deadlines.clear()
            self._deadline_heap.clear()
            self._pending.clear()
            self._action_credit = None
            self._action_credit_sequence = None
