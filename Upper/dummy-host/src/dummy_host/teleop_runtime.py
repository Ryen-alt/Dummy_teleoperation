from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, replace
from threading import Condition, Event, Lock, Thread
from typing import Callable, Protocol

import numpy as np

from .cameras import CameraError, CameraFrame
from .cartesian_teleop import CartesianPoseIntegrator, CartesianTeleopError
from .domain import ActionLifecycleUpdate, ActionStage, EpisodeError, EpisodeManager, EpisodeStatus
from .kinematics.calibration import CartesianCalibration
from .kinematics.contracts import KinematicsBackend, KinematicsError
from .recording import ControlTickTiming, RecorderBackpressure, SessionRecorder
from .robot_driver import ActionCredit, DummyRobot, RobotError
from .scheduler import FixedRateScheduler, ScheduledTick, SchedulerStats
from .schema import ControlMode, RobotState
from .teleop import (
    ControlTimingError,
    JointVelocityIntegrator,
    TeleopCommand,
    TeleopError,
    TeleopProfile,
)
from .time_sync import AffineTimeSyncEstimator
from .transport_serial import TransportError


class TeleopInput(Protocol):
    def poll(self, now_ns: int | None = None) -> TeleopCommand: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class TeleopRunStats:
    actions_sent: int
    hold_transitions: int
    episode_events: int
    final_mode: str
    final_received_sequence: int
    final_post_command_feedback_sequence: int
    scheduler: SchedulerStats
    cartesian_reanchors: int = 0
    ik_soft_overruns: int = 0
    coherent_sweep_skips: int = 0
    ik_hard_timeouts: int = 0
    action_credit_misses: int = 0


class _InputWorker:
    def __init__(
        self,
        source: TeleopInput,
        *,
        rate_hz: int,
        clock_ns: Callable[[], int],
    ) -> None:
        self.source = source
        self.period_s = 1.0 / rate_hz
        self.clock_ns = clock_ns
        self.stop = Event()
        self.condition = Condition()
        self.latest: TeleopCommand | None = None
        self.events: deque[TeleopCommand] = deque(maxlen=64)
        self.error: BaseException | None = None
        self.started = False
        self.thread = Thread(target=self._run, name="dummy-teleop-input", daemon=True)

    def start(self) -> None:
        self.started = True
        self.thread.start()

    def snapshot(self, *, wait_s: float = 0.0) -> TeleopCommand | None:
        with self.condition:
            if self.latest is None and self.error is None and wait_s > 0:
                self.condition.wait(wait_s)
            if self.error is not None:
                raise TeleopError(f"input thread failed: {self.error}") from self.error
            command = self.latest
            if command is None:
                return None
            command = replace(command, episode_event=None)
            if self.events:
                event = self.events.popleft()
                command = replace(command, episode_event=event.episode_event)
            return command

    def close(self) -> None:
        self.stop.set()
        if self.started:
            self.thread.join(timeout=1.0)
        self.source.close()

    def _run(self) -> None:
        try:
            while not self.stop.is_set():
                command = self.source.poll(self.clock_ns())
                with self.condition:
                    self.latest = command
                    if command.episode_event is not None:
                        self.events.append(command)
                    self.condition.notify_all()
                self.stop.wait(self.period_s)
        except BaseException as exc:
            self.error = exc
            with self.condition:
                self.condition.notify_all()


class _EvidenceTelemetryWorker:
    """Collect protocol-v5 clock and CAN evidence away from the 20 Hz loop."""

    def __init__(
        self,
        robot: DummyRobot,
        recorder: SessionRecorder,
        *,
        clock_ns: Callable[[], int],
    ) -> None:
        self.robot = robot
        self.recorder = recorder
        self.clock_ns = clock_ns
        self.estimator = AffineTimeSyncEstimator()
        self.stop = Event()
        self.lock = Lock()
        self.error: BaseException | None = None
        self.thread = Thread(
            target=self._run, name="dummy-evidence-telemetry", daemon=True
        )

    def start(self) -> None:
        self.thread.start()

    def snapshot_error(self) -> BaseException | None:
        with self.lock:
            return self.error

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=2.0)

    def _run(self) -> None:
        next_sync = 0.0
        next_diagnostics = 0.0
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                if now >= next_sync:
                    exchange = self.robot.time_sync()
                    self.recorder.record_time_sync(
                        exchange, self.estimator.observe(exchange)
                    )
                    next_sync = now + 0.5
                if now >= next_diagnostics:
                    self.recorder.record_can_diagnostics(
                        self.robot.read_can_diagnostics(),
                        host_time_ns=self.clock_ns(),
                    )
                    next_diagnostics = now + 1.0
                wait_s = max(
                    0.001,
                    min(next_sync, next_diagnostics) - time.monotonic(),
                )
                self.stop.wait(wait_s)
        except BaseException as exc:
            with self.lock:
                self.error = exc


class _LeaseCoordinator:
    """Own blocking lease traffic and control-bound target refreshes off control."""

    def __init__(
        self,
        robot: DummyRobot,
        *,
        clock_ns: Callable[[], int],
        event_callback: Callable[[str, int, dict[str, object]], None],
    ) -> None:
        self.robot = robot
        self.clock_ns = clock_ns
        self.event_callback = event_callback
        self.condition = Condition()
        self.desired = "hold"
        self.acquired = False
        self.error: BaseException | None = None
        self.stop = False
        self._refresh_generation = 0
        self._refresh_served_generation = 0
        self._refresh_request: tuple[int, int, int, int] | None = None
        self._last_control_tick_id: int | None = None
        self._last_control_tick_ns: int | None = None
        self.thread = Thread(target=self._run, name="dummy-lease-heartbeat", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def request(self, desired: str) -> None:
        if desired not in {"hold", "teleop", "estop"}:
            raise ValueError("invalid lease state")
        with self.condition:
            self.desired = desired
            if desired != "teleop":
                self._refresh_request = None
                self._last_control_tick_id = None
                self._last_control_tick_ns = None
            self.condition.notify_all()

    def note_control_tick(self, control_tick_id: int, control_tick_ns: int) -> None:
        """Publish one real control-thread health token to the coordinator."""

        if not 0 < control_tick_id <= 0xFFFFFFFF or control_tick_ns < 0:
            raise ValueError("invalid control health tick")
        with self.condition:
            if self.desired != "teleop" or not self.acquired:
                return
            previous = self._last_control_tick_id
            if previous is not None:
                delta = (control_tick_id - previous) & 0xFFFFFFFF
                if delta == 0 or delta >= 0x80000000:
                    raise ValueError("control health tick must advance monotonically")
            self._last_control_tick_id = control_tick_id
            self._last_control_tick_ns = control_tick_ns
            self.condition.notify_all()

    def request_target_refresh(
        self,
        action_sequence: int,
        control_tick_id: int,
        control_tick_ns: int,
    ) -> None:
        """Request one refresh backed by a recent control-thread tick.

        The request is replaceable and never repeats autonomously. If control
        stops ticking, only the independent lease remains alive; the motion
        target still expires and the firmware enters HOLD.
        """

        if (
            not 0 < action_sequence <= 0xFFFFFFFF
            or not 0 < control_tick_id <= 0xFFFFFFFF
            or control_tick_ns < 0
        ):
            raise ValueError("invalid target refresh request")
        with self.condition:
            if self.desired != "teleop":
                return
            self._refresh_generation += 1
            self._refresh_request = (
                self._refresh_generation,
                action_sequence,
                control_tick_id,
                control_tick_ns,
            )
            self.condition.notify_all()

    def cancel_target_refresh(self) -> None:
        with self.condition:
            self._refresh_request = None

    def snapshot(self) -> tuple[bool, BaseException | None]:
        with self.condition:
            return self.acquired, self.error

    def close(self) -> None:
        self.request("hold")
        deadline = time.monotonic() + 1.0
        with self.condition:
            while self.acquired and self.error is None and time.monotonic() < deadline:
                self.condition.wait(0.05)
            self.stop = True
            self.condition.notify_all()
        self.thread.join(timeout=1.0)

    def _run(self) -> None:
        heartbeat_s = max(0.02, self.robot.config.lease_timeout_ms / 3000.0)
        heartbeat_ns = int(heartbeat_s * 1e9)
        refresh_health_ns = 75_000_000
        refresh_period_ns = 40_000_000
        next_heartbeat_ns = 0
        next_refresh_check_ns = 0
        try:
            while True:
                with self.condition:
                    if self.stop:
                        return
                    desired = self.desired
                    acquired = self.acquired
                    refresh_request = self._refresh_request
                    last_control_tick_id = self._last_control_tick_id
                    last_control_tick_ns = self._last_control_tick_ns
                if desired == "estop":
                    self.robot.emergency_stop()
                    with self.condition:
                        self.acquired = False
                        self.desired = "hold"
                        self.condition.notify_all()
                    self.event_callback("operator_estop", self.clock_ns(), {})
                    continue
                if desired == "teleop" and not acquired:
                    started = self.clock_ns()
                    self.event_callback("wait_feedback_ready", started, {})
                    self.robot.acquire_control(ControlMode.TELEOP)
                    with self.condition:
                        self.acquired = True
                        # Allow one 75 ms interval for the control thread to
                        # publish its first real health token after ACQUIRE.
                        self._last_control_tick_id = None
                        self._last_control_tick_ns = self.clock_ns()
                        still_desired = self.desired
                        self.condition.notify_all()
                    next_heartbeat_ns = self.clock_ns() + heartbeat_ns
                    next_refresh_check_ns = self.clock_ns()
                    self.event_callback(
                        "deadman_acquired",
                        self.clock_ns(),
                        {"wait_started_ns": started},
                    )
                    if still_desired != "teleop":
                        continue
                elif desired == "hold" and acquired:
                    self.robot.hold()
                    self.robot.release_control()
                    with self.condition:
                        self.acquired = False
                        self.condition.notify_all()
                    self.event_callback("control_released", self.clock_ns(), {})
                    continue
                elif desired == "teleop" and acquired:
                    now_ns = self.clock_ns()
                    if (
                        last_control_tick_ns is not None
                        and now_ns - last_control_tick_ns > refresh_health_ns
                    ):
                        self.robot.request_priority_hold()
                        self.event_callback(
                            "control_health_timeout",
                            now_ns,
                            {
                                "control_tick_id": last_control_tick_id,
                                "control_tick_age_ns": now_ns - last_control_tick_ns,
                                "health_limit_ns": refresh_health_ns,
                            },
                        )
                        raise TeleopError(
                            "control freshness exceeded 75 ms; priority HOLD requested"
                        )
                    if (
                        refresh_request is not None
                        and refresh_request[0] != self._refresh_served_generation
                        and now_ns >= next_refresh_check_ns
                    ):
                        (
                            generation,
                            action_sequence,
                            control_tick_id,
                            control_tick_ns,
                        ) = refresh_request
                        age_ns = now_ns - control_tick_ns
                        if 0 <= age_ns <= refresh_health_ns:
                            self.robot.refresh_target(action_sequence, control_tick_id)
                        else:
                            self.robot.request_priority_hold()
                            self.event_callback(
                                "control_health_timeout",
                                self.clock_ns(),
                                {
                                    "action_sequence": action_sequence,
                                    "control_tick_age_ns": age_ns,
                                    "health_limit_ns": refresh_health_ns,
                                },
                            )
                            raise TeleopError(
                                "control freshness exceeded 75 ms; priority HOLD requested"
                            )
                        next_refresh_check_ns = self.clock_ns() + refresh_period_ns
                        with self.condition:
                            self._refresh_served_generation = generation
                            if (
                                self._refresh_request is not None
                                and self._refresh_request[0] == generation
                            ):
                                self._refresh_request = None
                        continue
                    if now_ns >= next_heartbeat_ns:
                        self.robot.heartbeat()
                        next_heartbeat_ns = self.clock_ns() + heartbeat_ns
                with self.condition:
                    wait_s = heartbeat_s
                    if desired == "teleop" and acquired:
                        wait_s = max(
                            0.001,
                            min(
                                heartbeat_s,
                                (next_heartbeat_ns - self.clock_ns()) / 1e9,
                            ),
                        )
                        if refresh_request is not None:
                            wait_s = max(
                                0.001,
                                min(
                                    wait_s,
                                    (next_refresh_check_ns - self.clock_ns()) / 1e9,
                                ),
                            )
                        if last_control_tick_ns is not None:
                            wait_s = max(
                                0.001,
                                min(
                                    wait_s,
                                    (
                                        last_control_tick_ns
                                        + refresh_health_ns
                                        - self.clock_ns()
                                    )
                                    / 1e9,
                                ),
                            )
                    self.condition.wait(wait_s)
        except BaseException as exc:
            with self.condition:
                self.error = exc
                self.acquired = False
                self.condition.notify_all()


def mask_teleop_command(
    command: TeleopCommand,
    *,
    allowed_joints: set[int] | None,
    allow_gripper: bool,
) -> TeleopCommand:
    if allowed_joints is not None and not allowed_joints.issubset(set(range(1, 7))):
        raise ValueError("allowed_joints must contain only values in [1, 6]")
    velocity = command.joint_velocity_rad_s.astype(np.float32, copy=True)
    if allowed_joints is not None:
        for index in range(6):
            if index + 1 not in allowed_joints:
                velocity[index] = 0.0
    gripper_velocity = command.gripper_velocity_per_s if allow_gripper else 0.0
    raw = dict(command.raw)
    raw["unmasked_joint_velocity_rad_s"] = command.joint_velocity_rad_s.tolist()
    raw["unmasked_cartesian_twist"] = command.cartesian_twist.tolist()
    raw["unmasked_gripper_velocity_per_s"] = command.gripper_velocity_per_s
    raw["allowed_joints"] = list(range(1, 7)) if allowed_joints is None else sorted(allowed_joints)
    raw["gripper_allowed"] = allow_gripper
    return TeleopCommand(
        monotonic_ns=command.monotonic_ns,
        source=command.source,
        joint_velocity_rad_s=velocity,
        gripper_velocity_per_s=gripper_velocity,
        deadman=command.deadman,
        hold_requested=command.hold_requested,
        estop_requested=command.estop_requested,
        episode_event=command.episode_event,
        connected=command.connected,
        raw=raw,
        teleop_mode=command.teleop_mode,
        cartesian_twist=command.cartesian_twist,
        event_ns=command.event_ns,
    )


def _cameras_for_state(
    robot: DummyRobot,
    state: RobotState,
) -> tuple[dict[str, CameraFrame], str | None]:
    if robot.camera_manager is None:
        return {}, None
    try:
        return robot.camera_manager.nearest_all(state.monotonic_ns), None
    except CameraError as exc:
        return {}, str(exc)


def run_teleop_collection(
    robot: DummyRobot,
    input_source: TeleopInput,
    recorder: SessionRecorder,
    profile: TeleopProfile,
    *,
    duration_s: float | None = None,
    require_camera: bool = False,
    allowed_joints: set[int] | None = None,
    allow_gripper: bool = False,
    task_id: str = "teleop_unspecified",
    task: str = "Unspecified teleoperation task",
    episode_manager: EpisodeManager | None = None,
    teleop_mode: str = "joint",
    kinematics: KinematicsBackend | None = None,
    cartesian_calibration: CartesianCalibration | None = None,
    clock_ns: Callable[[], int] = time.monotonic_ns,
) -> TeleopRunStats:
    """Run keyboard/gamepad collection over either FakeMcu or SerialTransport.

    Device-specific values stay in the recorder. Only the integrated absolute
    target goes through DummyRobot, so fake and real operation use the exact
    same safety filter and binary firmware protocol.
    """

    if duration_s is not None and duration_s <= 0:
        input_source.close()
        raise ValueError("duration_s must be positive")
    if require_camera and robot.camera_manager is None:
        input_source.close()
        raise ValueError("require_camera needs a configured camera")
    if teleop_mode not in {"joint", "cartesian"}:
        input_source.close()
        raise ValueError("teleop_mode must be 'joint' or 'cartesian'")
    if teleop_mode == "cartesian":
        if kinematics is None:
            input_source.close()
            raise ValueError("Cartesian teleoperation requires a kinematics backend")
        if allowed_joints not in (None, set(range(1, 7))):
            input_source.close()
            raise ValueError("Cartesian teleoperation requires all six joints to be allowed")
        if not robot.transport.is_simulated and (
            cartesian_calibration is None or not cartesian_calibration.validated
        ):
            input_source.close()
            raise ValueError(
                "real Cartesian teleoperation requires a validated Cartesian calibration"
            )
        if not robot.transport.is_simulated:
            assert cartesian_calibration is not None
            urdf_path = getattr(kinematics, "urdf_path", None)
            calibration_hash = getattr(kinematics, "calibration_hash", None)
            if urdf_path is None or calibration_hash != cartesian_calibration.file_hash:
                input_source.close()
                raise ValueError(
                    "real Cartesian kinematics backend does not match the selected calibration"
                )
            try:
                cartesian_calibration.validate_for(
                    robot.config,
                    urdf_path,
                    require_validated=True,
                )
            except KinematicsError:
                input_source.close()
                raise
            if kinematics.tip_link != cartesian_calibration.tip_frame:
                input_source.close()
                raise ValueError(
                    "real Cartesian kinematics tip frame does not match calibration"
                )
    episode_manager = EpisodeManager() if episode_manager is None else episode_manager
    integrator = (
        JointVelocityIntegrator(profile, robot.config)
        if teleop_mode == "joint"
        else CartesianPoseIntegrator(profile, robot.config, kinematics)
    )
    scheduler = FixedRateScheduler(robot.config.control_rate_hz, clock_ns=clock_ns)
    input_worker = _InputWorker(
        input_source, rate_hz=robot.config.control_rate_hz, clock_ns=clock_ns
    )
    stop = Event()
    action_failure = Event()
    action_failure_detail: list[str] = []
    action_stages: dict[int, set[ActionStage]] = {}
    action_stage_lock = Lock()
    episode_last_sequence: int | None = None
    latest_action_sequence: int | None = None
    episode_finalize_deadline_ns: int | None = None
    actions_sent = 0
    hold_transitions = 0
    episode_events = 0
    final_state: RobotState | None = None
    last_command: TeleopCommand | None = None
    scheduler_stats = SchedulerStats(0, 0, 0.0, 0.0, 0.0)
    deadline_ns: int | None = None
    lease: _LeaseCoordinator | None = None
    evidence_telemetry: _EvidenceTelemetryWorker | None = None
    control_had_lease = False
    hold_latched = False
    last_control_start_ns: int | None = None
    last_fresh_sweep_ns: int | None = None
    cartesian_reanchors = 0
    ik_soft_overruns = 0
    coherent_sweep_skips = 0
    ik_hard_timeouts = 0
    action_credit_misses = 0
    idle_input_timeout_active = False
    runtime_error: BaseException | None = None

    def runtime_event(name: str, when_ns: int, payload: dict[str, object]) -> None:
        recorder.record_event(name, monotonic_ns=when_ns, payload=payload)

    def lifecycle(update: ActionLifecycleUpdate) -> None:
        recorder.record_action_lifecycle(update)
        with action_stage_lock:
            action_stages.setdefault(update.sequence, set()).add(update.stage)
        if update.stage in {
            ActionStage.FAILED,
            ActionStage.REJECTED,
            ActionStage.SUPERSEDED,
        }:
            action_failure_detail.append(update.detail or "action lifecycle failure")
            action_failure.set()

    def fail_active_episode(reason: str, now_ns: int) -> None:
        nonlocal episode_events
        snapshot = episode_manager.snapshot
        if snapshot.status not in {
            EpisodeStatus.RECORDING,
            EpisodeStatus.FINALIZING,
        }:
            return
        failed = episode_manager.finish(
            EpisodeStatus.FAILED,
            now_ns=now_ns,
            failure_reason=reason,
        )
        recorder.record_event(
            "episode_failure",
            monotonic_ns=now_ns,
            payload={
                "episode_id": failed.episode_id,
                "status": failed.status.value,
                "reason": reason,
                "task_id": failed.task_id,
                "task": failed.task,
            },
        )
        episode_events += 1

    def record_control_sample(
        command: TeleopCommand,
        state: RobotState,
        tick: ScheduledTick,
        *,
        action: object | None = None,
        target_generated_ns: int | None = None,
        send_enqueued_ns: int | None = None,
        valid: bool = True,
        invalid_reason: str | None = None,
    ) -> None:
        frames, camera_error = _cameras_for_state(robot, state)
        reason = invalid_reason
        sample_valid = valid and state.coherent
        if not state.coherent and reason is None:
            reason = "robot state is not a coherent seven-node feedback sweep"
        if require_camera and camera_error is not None:
            sample_valid = False
            reason = camera_error
        end_ns = clock_ns()
        diagnostics = getattr(robot.transport, "diagnostics", None)
        transport_diagnostics = (
            vars(diagnostics()) if callable(diagnostics) else {}
        )
        recorder.record_sample(
            command,
            state,
            action=action,  # type: ignore[arg-type]
            camera_frames=frames,
            valid=sample_valid,
            invalid_reason=reason,
            timing=ControlTickTiming(
                raw_tick_index=tick.raw_tick_index,
                planned_ns=tick.planned_ns,
                actual_start_ns=tick.actual_start_ns,
                actual_end_ns=end_ns,
                target_generated_ns=target_generated_ns,
                send_enqueued_ns=send_enqueued_ns,
                missed_periods=tick.missed_periods,
                next_rebase_deadline_ns=tick.next_rebase_deadline_ns,
                transport_diagnostics=transport_diagnostics,
            ),
        )

    try:
        robot.connect()
        recorder.update_runtime_metadata(
            firmware_version=robot.firmware_version or "unknown",
            session_epoch=robot.session_id,
        )
        robot.set_action_lifecycle_listener(lifecycle)
        evidence_telemetry = _EvidenceTelemetryWorker(
            robot, recorder, clock_ns=clock_ns
        )
        evidence_telemetry.start()
        robot.hold()
        final_state = robot.read_state()
        if teleop_mode == "cartesian" and not robot.transport.is_simulated:
            assert cartesian_calibration is not None
            try:
                final_state = robot.wait_for_feedback_ready(
                    consecutive_sweeps=3,
                    state_validator=lambda state: cartesian_calibration.is_ready(
                        state.position[:6]
                    ),
                )
            except RobotError as exc:
                final_state = robot.read_state()
                error = cartesian_calibration.ready_error(final_state.position[:6])
                recorder.record_event(
                    "cartesian_ready_pose_rejected",
                    payload={
                        "measured_joint_rad": final_state.position[:6].tolist(),
                        "ready_pose_rad": cartesian_calibration.ready_pose_rad.tolist()
                        if cartesian_calibration.ready_pose_rad is not None
                        else None,
                        "ready_tolerance_rad": cartesian_calibration.ready_tolerance_rad.tolist()
                        if cartesian_calibration.ready_tolerance_rad is not None
                        else None,
                        "absolute_error_rad": error.tolist(),
                        "feedback_gate_error": str(exc),
                    },
                )
                raise TeleopError(
                    "robot did not remain within the validated Cartesian-ready pose "
                    "for three coherent sweeps"
                ) from exc
            recorder.record_event(
                "cartesian_ready_pose_accepted",
                payload={
                    "coherent_sweep_id": final_state.coherent_sweep_id,
                    "measured_joint_rad": final_state.position[:6].tolist(),
                    "calibration_id": cartesian_calibration.calibration_id,
                },
            )
        recorder.record_event(
            "robot_connected",
            payload={"firmware_version": robot.firmware_version or "unknown"},
        )
        lease = _LeaseCoordinator(robot, clock_ns=clock_ns, event_callback=runtime_event)
        lease.start()
        input_worker.start()
        if input_worker.snapshot(wait_s=0.5) is None:
            raise TeleopError("input thread did not produce an initial snapshot")

        collection_started_ns = clock_ns()
        deadline_ns = (
            None
            if duration_s is None
            else collection_started_ns + int(duration_s * 1e9)
        )
        recorder.record_event(
            "collection_started",
            monotonic_ns=collection_started_ns,
            payload={"requested_duration_s": duration_s},
        )

        def tick(scheduled: ScheduledTick) -> None:
            nonlocal actions_sent, episode_events, final_state, last_command
            nonlocal control_had_lease, hold_transitions, hold_latched
            nonlocal last_control_start_ns
            nonlocal last_fresh_sweep_ns, cartesian_reanchors, ik_soft_overruns
            nonlocal coherent_sweep_skips, ik_hard_timeouts
            nonlocal action_credit_misses
            nonlocal episode_last_sequence, episode_finalize_deadline_ns
            nonlocal latest_action_sequence
            nonlocal idle_input_timeout_active
            now_ns = scheduled.actual_start_ns
            if deadline_ns is not None and now_ns >= deadline_ns:
                stop.set()
                return
            assert lease is not None
            assert evidence_telemetry is not None
            telemetry_error = evidence_telemetry.snapshot_error()
            if telemetry_error is not None:
                fail_active_episode(
                    f"evidence_telemetry_failed: {telemetry_error}", now_ns
                )
                raise TeleopError(
                    f"time-sync/CAN evidence thread failed: {telemetry_error}"
                ) from telemetry_error
            acquired, lease_error = lease.snapshot()
            if lease_error is not None:
                fail_active_episode(f"lease_thread_failed: {lease_error}", now_ns)
                raise TeleopError(f"lease/heartbeat thread failed: {lease_error}") from lease_error
            command_raw = input_worker.snapshot()
            if command_raw is None:
                raise TeleopError("input snapshot is unavailable")
            command = mask_teleop_command(
                command_raw,
                allowed_joints=allowed_joints,
                allow_gripper=allow_gripper,
            )
            last_command = command
            final_state = robot.read_state()

            interval_s = None if last_control_start_ns is None else (
                now_ns - last_control_start_ns
            ) / 1e9
            last_control_start_ns = now_ns
            budget_s = 1.5 / robot.config.control_rate_hz
            if acquired and interval_s is not None and interval_s > budget_s:
                hold_latched = True
                lease.request("hold")
                integrator.reset()
                hold_transitions += 1
                reason = (
                    f"control interval {interval_s * 1000:.1f} ms exceeds "
                    f"{budget_s * 1000:.1f} ms budget"
                )
                fail_active_episode("control_timing_overrun", now_ns)
                recorder.record_event(
                    "control_timing_overrun",
                    monotonic_ns=now_ns,
                    payload={"measured_dt_s": interval_s, "budget_s": budget_s},
                )
                record_control_sample(command, final_state, scheduled, valid=False, invalid_reason=reason)
                return

            if action_failure.is_set():
                hold_latched = True
                lease.request("hold")
                integrator.reset()
                hold_transitions += 1
                reason = action_failure_detail[-1] if action_failure_detail else "action failed"
                fail_active_episode("action_lifecycle_failed", now_ns)
                record_control_sample(command, final_state, scheduled, valid=False, invalid_reason=reason)
                action_failure.clear()
                latest_action_sequence = None
                return

            if command.teleop_mode != teleop_mode:
                lease.request("hold")
                fail_active_episode("teleop_mode_mismatch", now_ns)
                record_control_sample(
                    command,
                    final_state,
                    scheduled,
                    valid=False,
                    invalid_reason=(
                        f"input emitted {command.teleop_mode!r} while runtime is {teleop_mode!r}"
                    ),
                )
                raise TeleopError("teleoperation input mode does not match runtime mode")
            age_ms = (now_ns - command.monotonic_ns) / 1e6
            if age_ms < 0 or age_ms > profile.input_timeout_ms:
                lease.request("hold")
                active_control = acquired or control_had_lease or command.deadman
                if active_control or age_ms < 0:
                    fail_active_episode("input_timeout", now_ns)
                if not idle_input_timeout_active or active_control or age_ms < 0:
                    recorder.record_event(
                        "input_timeout",
                        monotonic_ns=now_ns,
                        payload={
                            "input_age_ms": age_ms,
                            "active_control": active_control,
                        },
                    )
                idle_input_timeout_active = True
                record_control_sample(
                    command,
                    final_state,
                    scheduled,
                    valid=False,
                    invalid_reason=f"input command stale: {age_ms:.1f} ms",
                )
                if active_control or age_ms < 0:
                    raise TeleopError(f"input command is stale ({age_ms:.1f} ms)")
                return
            if idle_input_timeout_active:
                recorder.record_event(
                    "input_recovered",
                    monotonic_ns=now_ns,
                    payload={"input_age_ms": age_ms},
                )
                idle_input_timeout_active = False

            if bool(command.raw.get("input_sync_lost", False)):
                lease.request("hold")
                hold_latched = True
                integrator.reset()
                hold_transitions += 1
                fail_active_episode("input_time_chain_interrupted", now_ns)
                recorder.record_event(
                    "input_sync_lost",
                    monotonic_ns=now_ns,
                    payload={
                        "input_event_ns": command.event_ns,
                        "input_snapshot_ns": command.monotonic_ns,
                    },
                )
                record_control_sample(
                    command,
                    final_state,
                    scheduled,
                    valid=False,
                    invalid_reason="evdev reported SYN_DROPPED; input state was resynchronized",
                )
                return

            if command.episode_event is not None:
                requested_episode_event = command.episode_event
                episode_status = episode_manager.snapshot.status
                episode_active = episode_status in {
                    EpisodeStatus.RECORDING,
                    EpisodeStatus.FINALIZING,
                }
                transition_allowed = (
                    (requested_episode_event == "start" and not episode_active)
                    or (
                        requested_episode_event == "success"
                        and episode_status is EpisodeStatus.RECORDING
                    )
                    or (
                        requested_episode_event in {"failure", "cancel"}
                        and episode_active
                    )
                )
                if not transition_allowed:
                    # Episode buttons are metadata controls, not robot safety
                    # controls.  A stray stick click while no Episode is active
                    # must not terminate an otherwise healthy teleop session.
                    recorder.record_event(
                        "episode_transition_ignored",
                        monotonic_ns=now_ns,
                        payload={
                            "event": requested_episode_event,
                            "current_status": episode_status.value,
                        },
                    )
                    command = replace(command, episode_event=None)
                    last_command = command
                else:
                    try:
                        if requested_episode_event == "start":
                            episode = episode_manager.begin(
                                task_id=task_id,
                                task=task,
                                now_ns=now_ns,
                                metadata={"source": command.source},
                            )
                            episode_last_sequence = None
                            episode_finalize_deadline_ns = None
                        elif requested_episode_event == "success":
                            if episode_last_sequence is None:
                                fail_active_episode("episode_has_no_action", now_ns)
                                lease.request("hold")
                                record_control_sample(
                                    command,
                                    final_state,
                                    scheduled,
                                    valid=False,
                                    invalid_reason="an Episode without any action cannot be accepted",
                                )
                                return
                            episode = episode_manager.begin_finalizing(now_ns=now_ns)
                            episode_finalize_deadline_ns = now_ns + 250_000_000
                        else:
                            outcome = {
                                "failure": EpisodeStatus.FAILED,
                                "cancel": EpisodeStatus.CANCELLED,
                            }.get(requested_episode_event)
                            if outcome is None:
                                raise EpisodeError(
                                    f"unknown Episode event {requested_episode_event!r}"
                                )
                            episode = episode_manager.finish(outcome, now_ns=now_ns)
                    except EpisodeError as exc:
                        lease.request("hold")
                        hold_latched = True
                        integrator.reset()
                        fail_active_episode("episode_transition_rejected", now_ns)
                        recorder.record_event(
                            "episode_transition_rejected",
                            monotonic_ns=now_ns,
                            payload={
                                "event": requested_episode_event,
                                "error": str(exc),
                            },
                        )
                        record_control_sample(
                            command,
                            final_state,
                            scheduled,
                            valid=False,
                            invalid_reason=f"Episode transition rejected: {exc}",
                        )
                        return
                    recorder.record_event(
                        "episode_finalizing"
                        if requested_episode_event == "success"
                        else f"episode_{requested_episode_event}",
                        monotonic_ns=now_ns,
                        payload={
                            "source": command.source,
                            "episode_id": episode.episode_id,
                            "status": episode.status.value,
                            "task_id": episode.task_id,
                            "task": episode.task,
                        },
                    )
                    episode_events += 1
                    if requested_episode_event in {"failure", "cancel"}:
                        lease.request("hold")
                        integrator.reset()
                        control_had_lease = False
                        hold_transitions += 1
                        record_control_sample(command, final_state, scheduled)
                        return

            if not command.connected:
                lease.request("hold")
                fail_active_episode("input_disconnected", now_ns)
                record_control_sample(
                    command, final_state, scheduled, valid=False,
                    invalid_reason="input device disconnected",
                )
                raise TeleopError("input device disconnected")
            if command.estop_requested:
                lease.request("estop")
                integrator.reset()
                fail_active_episode("operator_estop", now_ns)
                record_control_sample(
                    command, final_state, scheduled, valid=False,
                    invalid_reason="operator emergency stop",
                )
                stop.set()
                return

            # Every healthy acquired control iteration publishes exactly one
            # freshness generation. The lease thread may consume it once for
            # the active target; it can never synthesize fresh generations.
            control_tick_id = robot.advance_control_tick() if acquired else None
            if control_tick_id is not None:
                lease.note_control_tick(control_tick_id, now_ns)

            episode_snapshot = episode_manager.snapshot
            if episode_snapshot.status is EpisodeStatus.FINALIZING:
                assert episode_last_sequence is not None
                assert episode_finalize_deadline_ns is not None
                with action_stage_lock:
                    completed_stages = set(
                        action_stages.get(episode_last_sequence, set())
                    )
                required_stages = {
                    ActionStage.ACKNOWLEDGED,
                    ActionStage.CAN_QUEUED_EXACT,
                    ActionStage.CAN_TX_COMPLETE_EXACT,
                    ActionStage.POST_COMMAND_FEEDBACK,
                }
                if (
                    ActionStage.ACKNOWLEDGED in completed_stages
                    and control_tick_id is not None
                ):
                    lease.request_target_refresh(
                        episode_last_sequence, control_tick_id, now_ns
                    )
                if required_stages.issubset(completed_stages):
                    accepted = episode_manager.finish(
                        EpisodeStatus.ACCEPTED,
                        now_ns=now_ns,
                    )
                    lease.request("hold")
                    integrator.reset()
                    control_had_lease = False
                    hold_transitions += 1
                    recorder.record_event(
                        "episode_success",
                        monotonic_ns=now_ns,
                        payload={
                            "episode_id": accepted.episode_id,
                            "status": accepted.status.value,
                            "task_id": accepted.task_id,
                            "task": accepted.task,
                            "last_action_sequence": episode_last_sequence,
                            "completion_stages": sorted(
                                stage.value for stage in completed_stages
                            ),
                        },
                    )
                    episode_events += 1
                    episode_last_sequence = None
                    episode_finalize_deadline_ns = None
                    latest_action_sequence = None
                    record_control_sample(command, final_state, scheduled)
                    return
                if now_ns >= episode_finalize_deadline_ns:
                    missing = sorted(
                        stage.value for stage in required_stages - completed_stages
                    )
                    fail_active_episode("episode_action_completion_timeout", now_ns)
                    lease.request("hold")
                    integrator.reset()
                    control_had_lease = False
                    hold_latched = True
                    hold_transitions += 1
                    recorder.record_event(
                        "episode_finalize_timeout",
                        monotonic_ns=now_ns,
                        payload={
                            "last_action_sequence": episode_last_sequence,
                            "missing_stages": missing,
                        },
                    )
                    episode_last_sequence = None
                    episode_finalize_deadline_ns = None
                    record_control_sample(
                        command,
                        final_state,
                        scheduled,
                        valid=False,
                        invalid_reason=(
                            "last action did not reach exact completion watermarks: "
                            + ", ".join(missing)
                        ),
                    )
                    return
                record_control_sample(
                    command,
                    final_state,
                    scheduled,
                    valid=False,
                    invalid_reason="Episode is waiting for its final action watermarks",
                )
                return

            if command.hold_requested or not command.deadman or hold_latched:
                lease.request("hold")
                if not command.deadman:
                    hold_latched = False
                if control_had_lease:
                    integrator.reset()
                    control_had_lease = False
                    latest_action_sequence = None
                    hold_transitions += 1
                    hold_event = (
                        "operator_hold"
                        if command.hold_requested
                        else "deadman_released"
                        if not command.deadman
                        else "safety_hold_latched"
                    )
                    recorder.record_event(
                        hold_event,
                        monotonic_ns=now_ns,
                    )
                record_control_sample(command, final_state, scheduled)
                return

            lease.request("teleop")
            if not acquired:
                record_control_sample(
                    command,
                    final_state,
                    scheduled,
                    valid=False,
                    invalid_reason="WAIT_FEEDBACK_READY or ACQUIRE_CONTROL in progress",
                )
                return
            if not control_had_lease:
                if not final_state.coherent or not final_state.gripper_valid:
                    lease.request("hold")
                    hold_latched = True
                    record_control_sample(
                        command,
                        final_state,
                        scheduled,
                        valid=False,
                        invalid_reason="acquired without coherent joint and gripper feedback",
                    )
                    return
                integrator.reset(final_state, now_ns=now_ns)
                control_had_lease = True
                latest_action_sequence = None
                last_fresh_sweep_ns = now_ns if teleop_mode == "cartesian" else None
                record_control_sample(command, final_state, scheduled)
                return

            if teleop_mode == "cartesian":
                assert isinstance(integrator, CartesianPoseIntegrator)
                if not integrator.has_fresh_coherent_sweep(final_state):
                    coherent_sweep_skips += 1
                    if last_fresh_sweep_ns is None:
                        last_fresh_sweep_ns = now_ns
                    stall_ns = max(0, now_ns - last_fresh_sweep_ns)
                    reason_code = (
                        "duplicate_coherent_sweep"
                        if final_state.coherent
                        else "incoherent_feedback_sweep"
                    )
                    raw = dict(command.raw)
                    raw["cartesian"] = {
                        "stage": "feedback_coherence",
                        "status": "skipped_without_motion",
                        "reason": reason_code,
                        "coherent_sweep_id": final_state.coherent_sweep_id,
                        "last_consumed_sweep_id": integrator.last_sweep_id,
                        "stall_duration_ns": stall_ns,
                        "stall_limit_ns": robot.config.target_ttl_ms * 1_000_000,
                    }
                    command = replace(command, raw=raw)
                    last_command = command
                    integrator.advance_without_motion(now_ns)
                    if latest_action_sequence is not None:
                        with action_stage_lock:
                            refresh_stages = set(
                                action_stages.get(latest_action_sequence, set())
                            )
                        if ActionStage.ACKNOWLEDGED in refresh_stages:
                            lease.request_target_refresh(
                                latest_action_sequence,
                                control_tick_id,
                                now_ns,
                            )
                    if stall_ns >= robot.config.target_ttl_ms * 1_000_000:
                        hold_latched = True
                        lease.request("hold")
                        integrator.reset()
                        hold_transitions += 1
                        fail_active_episode("coherent_sweep_stalled", now_ns)
                        recorder.record_event(
                            "coherent_sweep_stalled",
                            monotonic_ns=now_ns,
                            payload={
                                "reason": reason_code,
                                "stall_duration_ns": stall_ns,
                                "coherent_sweep_id": final_state.coherent_sweep_id,
                            },
                        )
                        record_control_sample(
                            command,
                            final_state,
                            scheduled,
                            valid=False,
                            invalid_reason="coherent feedback sweep stalled beyond target TTL",
                        )
                        return
                    record_control_sample(
                        command,
                        final_state,
                        scheduled,
                        valid=False,
                        invalid_reason=reason_code,
                    )
                    return

            assert control_tick_id is not None
            action_credit: ActionCredit | None = robot.reserve_action_credit(
                control_tick_id, reserved_ns=now_ns
            )
            if action_credit is None:
                action_credit_misses += 1
                hold_latched = True
                robot.request_priority_hold()
                lease.request("hold")
                integrator.reset()
                hold_transitions += 1
                fail_active_episode("action_credit_miss", now_ns)
                recorder.record_event(
                    "action_credit_miss",
                    monotonic_ns=now_ns,
                    payload={
                        "latest_action_sequence": latest_action_sequence,
                        "control_tick_id": control_tick_id,
                    },
                )
                record_control_sample(
                    command,
                    final_state,
                    scheduled,
                    valid=False,
                    invalid_reason=(
                        "previous action did not reach CAN_TX_COMPLETE_EXACT "
                        "before the next 20 Hz control tick"
                    ),
                )
                return

            try:
                if teleop_mode == "cartesian":
                    assert isinstance(integrator, CartesianPoseIntegrator)
                    proposal = integrator.propose(command, final_state, now_ns)
                    latest_after_ik = robot.read_state()
                    if (
                        not latest_after_ik.coherent
                        or latest_after_ik.coherent_sweep_id
                        != proposal.source_sweep_id
                    ):
                        raise CartesianTeleopError(
                            "coherent feedback sweep changed while IK was solving",
                            metadata={
                                **proposal.metadata,
                                "stage": "ik_result_freshness",
                                "source_sweep_id": proposal.source_sweep_id,
                                "latest_sweep_id": latest_after_ik.coherent_sweep_id,
                            },
                        )
                    control_tick_deadline_ns = now_ns + int(
                        1e9 / robot.config.control_rate_hz
                    )
                    if clock_ns() > control_tick_deadline_ns:
                        raise CartesianTeleopError(
                            "IK result exceeded the current control tick deadline",
                            metadata={
                                **proposal.metadata,
                                "stage": "ik_result_freshness",
                                "control_tick_deadline_ns": control_tick_deadline_ns,
                            },
                        )
                    requested = proposal.action
                    raw = dict(command.raw)
                    raw["cartesian"] = dict(proposal.metadata)
                    command = replace(command, raw=raw)
                    last_command = command
                    budget = proposal.metadata.get("solve_budget", {})
                    if isinstance(budget, dict) and budget.get("soft_budget_exceeded"):
                        ik_soft_overruns += 1
                else:
                    assert isinstance(integrator, JointVelocityIntegrator)
                    requested = integrator.step(command, final_state, now_ns)
            except (ControlTimingError, CartesianTeleopError) as exc:
                robot.cancel_action_credit(action_credit)
                lease.request("hold")
                hold_latched = True
                integrator.reset()
                hold_transitions += 1
                fail_active_episode("control_target_generation_failed", now_ns)
                recorder.record_event(
                    "cartesian_target_invalid"
                    if isinstance(exc, CartesianTeleopError)
                    else "control_timing_invalid",
                    monotonic_ns=now_ns,
                    payload={
                        "error": str(exc),
                        "metadata": exc.metadata
                        if isinstance(exc, CartesianTeleopError)
                        else {},
                    },
                )
                if isinstance(exc, CartesianTeleopError):
                    raw = dict(command.raw)
                    raw["cartesian"] = dict(exc.metadata)
                    command = replace(command, raw=raw)
                    last_command = command
                    ik = exc.metadata.get("ik")
                    if isinstance(ik, dict) and ik.get("timed_out"):
                        ik_hard_timeouts += 1
                record_control_sample(
                    command, final_state, scheduled, valid=False, invalid_reason=str(exc)
                )
                return
            target_generated_ns = clock_ns()
            lease.cancel_target_refresh()
            try:
                action = robot.enqueue_absolute_action(
                    requested,
                    source=command.source,
                    max_velocity_rad_s=profile.joint_speed_rad_s,
                    generated_at_ns=target_generated_ns,
                    action_credit=action_credit,
                )
            except Exception as exc:
                robot.cancel_action_credit(action_credit)
                lease.request("hold")
                hold_latched = True
                integrator.reset()
                hold_transitions += 1
                fail_active_episode("action_enqueue_failed", now_ns)
                recorder.record_event(
                    "action_enqueue_failed",
                    monotonic_ns=now_ns,
                    payload={"error": str(exc)},
                )
                record_control_sample(
                    command,
                    final_state,
                    scheduled,
                    valid=False,
                    invalid_reason=f"action enqueue failed: {exc}",
                )
                raise TeleopError(f"action enqueue failed: {exc}") from exc
            if teleop_mode == "cartesian":
                assert isinstance(integrator, CartesianPoseIntegrator)
                try:
                    commit = integrator.commit(proposal, action)
                except CartesianTeleopError as exc:
                    lease.request("hold")
                    hold_latched = True
                    integrator.reset()
                    hold_transitions += 1
                    fail_active_episode("cartesian_commit_failed", now_ns)
                    recorder.record_event(
                        "cartesian_commit_failed",
                        monotonic_ns=now_ns,
                        payload={"error": str(exc), "metadata": exc.metadata},
                    )
                    record_control_sample(
                        command,
                        final_state,
                        scheduled,
                        action=action,
                        target_generated_ns=target_generated_ns,
                        valid=False,
                        invalid_reason=str(exc),
                    )
                    raise
                raw = dict(command.raw)
                cartesian_raw = dict(raw.get("cartesian", {}))
                cartesian_raw["commit"] = commit.as_dict()
                cartesian_raw["applied_action"] = action.applied.tolist()
                cartesian_raw["requested_action"] = action.requested.tolist()
                cartesian_raw["action_clipped"] = action.clipped
                cartesian_raw["action_reasons"] = list(action.reasons)
                raw["cartesian"] = cartesian_raw
                command = replace(command, raw=raw)
                last_command = command
                if commit.reanchored:
                    cartesian_reanchors += 1
                last_fresh_sweep_ns = now_ns
            send_enqueued_ns = clock_ns()
            actions_sent += 1
            latest_action_sequence = action.sequence
            if episode_manager.snapshot.status is EpisodeStatus.RECORDING:
                episode_last_sequence = action.sequence
            record_control_sample(
                command,
                final_state,
                scheduled,
                action=action,
                target_generated_ns=target_generated_ns,
                send_enqueued_ns=send_enqueued_ns,
            )

        scheduler_stats = scheduler.run_timed(tick, stop)
        recorder.record_event(
            "collection_stopped",
            monotonic_ns=clock_ns(),
            payload={"requested_duration_s": duration_s},
        )
    except BaseException as exc:
        runtime_error = exc
        # Safety must not depend on Episode/recorder bookkeeping succeeding.
        # Queue the transport-priority HOLD first; the synchronous HOLD in the
        # cleanup block below remains the acknowledgement/mode confirmation.
        if robot.is_connected:
            try:
                robot.request_priority_hold()
            except BaseException:
                pass
        if lease is not None:
            lease.request("hold")
        try:
            fail_active_episode(f"runtime_error: {exc}", clock_ns())
        except BaseException:
            pass
        raise
    finally:
        try:
            snapshot = episode_manager.snapshot
            if snapshot.status in {
                EpisodeStatus.RECORDING,
                EpisodeStatus.FINALIZING,
            }:
                outcome = (
                    EpisodeStatus.FAILED
                    if runtime_error is not None
                    else EpisodeStatus.CANCELLED
                )
                terminal = episode_manager.finish(
                    outcome,
                    now_ns=clock_ns(),
                    failure_reason=(
                        f"runtime_error: {runtime_error}"
                        if runtime_error is not None
                        else "session_exit"
                    ),
                )
                recorder.record_event(
                    "episode_failure"
                    if outcome is EpisodeStatus.FAILED
                    else "episode_cancel",
                    payload={
                        "episode_id": terminal.episode_id,
                        "status": terminal.status.value,
                        "reason": terminal.failure_reason,
                    },
                )
            if lease is not None:
                lease.close()
            if evidence_telemetry is not None:
                evidence_telemetry.close()
            robot.set_action_lifecycle_listener(None)
            if robot.is_connected:
                try:
                    robot.hold()
                    final_state = robot.read_state()
                except BaseException as exc:
                    try:
                        recorder.record_event("final_hold_failed", payload={"error": str(exc)})
                    except BaseException:
                        pass
                robot.disconnect()
        finally:
            input_worker.close()

    if final_state is None:
        final_mode = "UNKNOWN"
        final_received = 0
        final_applied = 0
    else:
        final_mode = final_state.mode.name
        final_received = final_state.last_received_sequence
        final_applied = final_state.last_post_command_feedback_sequence
    return TeleopRunStats(
        actions_sent=actions_sent,
        hold_transitions=hold_transitions,
        episode_events=episode_events,
        final_mode=final_mode,
        final_received_sequence=final_received,
        final_post_command_feedback_sequence=final_applied,
        scheduler=scheduler_stats,
        cartesian_reanchors=cartesian_reanchors,
        ik_soft_overruns=ik_soft_overruns,
        coherent_sweep_skips=coherent_sweep_skips,
        ik_hard_timeouts=ik_hard_timeouts,
        action_credit_misses=action_credit_misses,
    )
