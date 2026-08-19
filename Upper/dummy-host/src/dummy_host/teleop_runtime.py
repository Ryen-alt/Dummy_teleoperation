from __future__ import annotations

import time
from dataclasses import dataclass
from threading import Event
from typing import Callable, Protocol

import numpy as np

from .cameras import CameraError, CameraFrame
from .domain import EpisodeError, EpisodeManager, EpisodeStatus
from .recording import SessionRecorder
from .robot_driver import DummyRobot
from .scheduler import FixedRateScheduler, SchedulerStats
from .schema import ControlMode, RobotState
from .teleop import JointVelocityIntegrator, TeleopCommand, TeleopError, TeleopProfile


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
    final_applied_sequence: int
    scheduler: SchedulerStats


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
    episode_manager = EpisodeManager() if episode_manager is None else episode_manager
    integrator = JointVelocityIntegrator(profile, robot.config)
    scheduler = FixedRateScheduler(robot.config.control_rate_hz, clock_ns=clock_ns)
    stop = Event()
    acquired = False
    actions_sent = 0
    hold_transitions = 0
    episode_events = 0
    final_state: RobotState | None = None
    last_command: TeleopCommand | None = None
    deadline_ns = None if duration_s is None else clock_ns() + int(duration_s * 1e9)

    def release_to_hold(reason: str, now_ns: int) -> RobotState:
        nonlocal acquired, hold_transitions
        if acquired:
            robot.hold()
            robot.release_control()
            acquired = False
            integrator.reset()
            hold_transitions += 1
            recorder.record_event(reason, monotonic_ns=now_ns)
        return robot.read_state()

    try:
        robot.connect()
        recorder.update_runtime_metadata(
            firmware_version=robot.firmware_version or "unknown"
        )
        # Establish a known safe mode and obtain the first valid STATE before
        # the operator presses the dead-man. This also makes fake and real
        # startup semantics identical.
        robot.hold()
        final_state = robot.read_state()
        recorder.record_event(
            "robot_connected",
            payload={"firmware_version": robot.firmware_version or "unknown"},
        )

        def tick(now_ns: int) -> None:
            nonlocal acquired, actions_sent, episode_events, final_state, last_command
            if deadline_ns is not None and now_ns >= deadline_ns:
                stop.set()
                return
            command = mask_teleop_command(
                input_source.poll(now_ns),
                allowed_joints=allowed_joints,
                allow_gripper=allow_gripper,
            )
            last_command = command
            age_ms = (now_ns - command.monotonic_ns) / 1e6
            if age_ms < 0 or age_ms > profile.input_timeout_ms:
                final_state = release_to_hold("input_timeout", now_ns)
                recorder.record_sample(
                    command,
                    final_state,
                    valid=False,
                    invalid_reason=f"input command stale: {age_ms:.1f} ms",
                )
                raise TeleopError(f"input command is stale ({age_ms:.1f} ms)")
            if command.episode_event is not None:
                try:
                    if command.episode_event == "start":
                        episode = episode_manager.begin(
                            task_id=task_id,
                            task=task,
                            now_ns=now_ns,
                            metadata={"source": command.source},
                        )
                    else:
                        outcome = {
                            "success": EpisodeStatus.ACCEPTED,
                            "failure": EpisodeStatus.FAILED,
                            "cancel": EpisodeStatus.CANCELLED,
                        }.get(command.episode_event)
                        if outcome is None:
                            raise EpisodeError(f"unknown Episode event {command.episode_event!r}")
                        episode = episode_manager.finish(outcome, now_ns=now_ns)
                except EpisodeError as exc:
                    final_state = release_to_hold("episode_transition_rejected", now_ns)
                    recorder.record_event(
                        "episode_transition_rejected",
                        monotonic_ns=now_ns,
                        payload={"event": command.episode_event, "error": str(exc)},
                    )
                    raise TeleopError(str(exc)) from exc
                recorder.record_event(
                    f"episode_{command.episode_event}",
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
            if not command.connected:
                final_state = release_to_hold("input_disconnected", now_ns)
                recorder.record_sample(
                    command,
                    final_state,
                    valid=False,
                    invalid_reason="input device disconnected",
                )
                raise TeleopError("input device disconnected")
            if command.estop_requested:
                robot.emergency_stop()
                acquired = False
                integrator.reset()
                final_state = robot.read_state()
                recorder.record_event("operator_estop", monotonic_ns=now_ns)
                recorder.record_sample(
                    command,
                    final_state,
                    valid=False,
                    invalid_reason="operator emergency stop",
                )
                stop.set()
                return
            if command.hold_requested or not command.deadman:
                reason = "operator_hold" if command.hold_requested else "deadman_released"
                final_state = release_to_hold(reason, now_ns)
                frames, camera_error = _cameras_for_state(robot, final_state)
                recorder.record_sample(
                    command,
                    final_state,
                    camera_frames=frames,
                    valid=not require_camera or camera_error is None,
                    invalid_reason=camera_error if require_camera else None,
                )
                if require_camera and camera_error is not None:
                    raise TeleopError(f"required camera observation is invalid: {camera_error}")
                return

            if not acquired:
                robot.acquire_control(ControlMode.TELEOP)
                acquired = True
                initial_state = robot.read_state()
                if not initial_state.position_valid or not initial_state.gripper_valid:
                    raise TeleopError(
                        "valid joint and gripper feedback are required before teleoperation"
                    )
                integrator.reset(initial_state)
                recorder.record_event(
                    "deadman_acquired", monotonic_ns=now_ns, payload={"source": command.source}
                )

            state_before = robot.read_state()
            requested = integrator.step(command, state_before, now_ns)
            action = robot.apply_absolute_action(
                requested,
                source=command.source,
                max_velocity_rad_s=profile.joint_speed_rad_s,
            )
            actions_sent += 1
            final_state = robot.read_state()
            frames, camera_error = _cameras_for_state(robot, final_state)
            state_for_record = final_state
            if require_camera and camera_error is not None:
                final_state = release_to_hold("camera_invalid", now_ns)
            recorder.record_sample(
                command,
                state_for_record,
                action=action,
                camera_frames=frames,
                valid=not require_camera or camera_error is None,
                invalid_reason=camera_error if require_camera else None,
            )
            if require_camera and camera_error is not None:
                raise TeleopError(f"required camera observation is invalid: {camera_error}")

        scheduler_stats = scheduler.run(tick, stop)
    finally:
        try:
            episode = episode_manager.snapshot
            if episode.status is EpisodeStatus.RECORDING:
                cancelled = episode_manager.finish(
                    EpisodeStatus.CANCELLED,
                    now_ns=clock_ns(),
                    failure_reason="session_exit",
                )
                recorder.record_event(
                    "episode_cancel",
                    payload={
                        "episode_id": cancelled.episode_id,
                        "status": cancelled.status.value,
                        "reason": "session_exit",
                    },
                )
            if robot.is_connected:
                try:
                    if acquired:
                        final_state = release_to_hold("session_exit", clock_ns())
                    else:
                        robot.hold()
                        final_state = robot.read_state()
                except BaseException as exc:
                    try:
                        recorder.record_event("final_hold_failed", payload={"error": str(exc)})
                    except BaseException:
                        pass
                robot.disconnect()
        finally:
            input_source.close()

    if final_state is not None and last_command is not None:
        final_raw = dict(last_command.raw)
        final_raw["final_state_snapshot"] = True
        recorder.record_sample(
            TeleopCommand(
                monotonic_ns=clock_ns(),
                source=last_command.source,
                joint_velocity_rad_s=np.zeros(6, dtype=np.float32),
                gripper_velocity_per_s=0.0,
                deadman=False,
                hold_requested=True,
                estop_requested=False,
                episode_event=None,
                connected=last_command.connected,
                raw=final_raw,
            ),
            final_state,
            valid=final_state.mode == ControlMode.HOLD,
            invalid_reason=None if final_state.mode == ControlMode.HOLD else "session did not end in HOLD",
        )

    if final_state is None:
        final_mode = "UNKNOWN"
        final_received = 0
        final_applied = 0
    else:
        final_mode = final_state.mode.name
        final_received = final_state.last_received_sequence
        final_applied = final_state.last_applied_sequence
    return TeleopRunStats(
        actions_sent=actions_sent,
        hold_transitions=hold_transitions,
        episode_events=episode_events,
        final_mode=final_mode,
        final_received_sequence=final_received,
        final_applied_sequence=final_applied,
        scheduler=scheduler_stats,
    )
