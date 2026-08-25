from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Protocol

import numpy as np

from .fake_mcu import FakeMcuTransport
from .robot_driver import DummyRobot
from .scheduler import FixedRateScheduler, SchedulerStats
from .schema import ControlMode, RobotConfig, load_robot_config
from .transport_serial import SerialTransport


class BringupError(RuntimeError):
    pass


class Deadman(Protocol):
    def is_pressed(self) -> bool: ...
    def close(self) -> None: ...


class EvdevDeadman:
    def __init__(self, device_path: str, key_name: str = "KEY_SPACE") -> None:
        try:
            from evdev import InputDevice, ecodes
        except ImportError as exc:
            raise BringupError("install dummy-host[bringup] to use an evdev dead-man") from exc
        try:
            self._device = InputDevice(device_path)
        except OSError as exc:
            raise BringupError(f"cannot open dead-man device {device_path}: {exc}") from exc
        try:
            self._key_code = int(ecodes.ecodes[key_name])
        except (KeyError, TypeError, ValueError) as exc:
            self._device.close()
            raise BringupError(f"unknown evdev key {key_name}") from exc
        self.last_error: str | None = None

    def is_pressed(self) -> bool:
        try:
            return self._key_code in self._device.active_keys()
        except (OSError, SystemError) as exc:
            self.last_error = str(exc)
            return False

    def close(self) -> None:
        self._device.close()


@dataclass(frozen=True)
class JointBringupPlan:
    joint: int
    delta_deg: float
    delta_rad: float
    max_velocity_rad_s: float
    duration_s: float
    control_rate_hz: int
    start_action: tuple[float, ...]
    final_requested_action: tuple[float, ...]


@dataclass(frozen=True)
class BringupRunResult:
    actions_sent: int
    last_sequence: int
    last_post_command_feedback_sequence: int
    final_mode: str
    scheduler: SchedulerStats


def make_joint_bringup_plan(
    config: RobotConfig,
    *,
    joint: int,
    delta_deg: float = 0.2,
    max_velocity_rad_s: float = 0.02,
    duration_s: float = 1.0,
    max_abs_delta_deg: float = 1.0,
    validate_initial_pose_limits: bool = True,
) -> JointBringupPlan:
    if joint not in range(1, 7):
        raise BringupError("joint must be in [1, 6]")
    if not math.isfinite(delta_deg) or abs(delta_deg) > max_abs_delta_deg or delta_deg == 0:
        raise BringupError(f"delta_deg must be finite, non-zero and within ±{max_abs_delta_deg}")
    if not math.isfinite(max_velocity_rad_s) or max_velocity_rad_s <= 0:
        raise BringupError("max_velocity_rad_s must be finite and positive")
    if max_velocity_rad_s > float(config.joint_velocity_limit_rad_s[joint - 1]):
        raise BringupError("requested velocity exceeds the configured joint limit")
    if not math.isfinite(duration_s) or duration_s <= 0 or duration_s > 10:
        raise BringupError("duration_s must be finite and in (0, 10]")

    start = np.concatenate(
        (config.initial_pose_rad.astype(np.float32, copy=True), np.asarray([0.0], dtype=np.float32))
    )
    final = start.copy()
    final[joint - 1] += np.float32(math.radians(delta_deg))
    if validate_initial_pose_limits and (
        final[joint - 1] < config.joint_limit_min_rad[joint - 1]
        or final[joint - 1] > config.joint_limit_max_rad[joint - 1]
    ):
        raise BringupError("planned target is outside the configured soft limit")
    return JointBringupPlan(
        joint=joint,
        delta_deg=delta_deg,
        delta_rad=math.radians(delta_deg),
        max_velocity_rad_s=max_velocity_rad_s,
        duration_s=duration_s,
        control_rate_hz=config.control_rate_hz,
        start_action=tuple(float(value) for value in start),
        final_requested_action=tuple(float(value) for value in final),
    )


def run_simulated_bringup(config: RobotConfig, plan: JointBringupPlan) -> BringupRunResult:
    transport = FakeMcuTransport(config)
    robot = DummyRobot(config, transport)
    stop = threading.Event()
    scheduler = FixedRateScheduler(plan.control_rate_hz)
    actions_sent = 0
    last_sequence = 0
    stats = SchedulerStats(0, 0, 0.0, 0.0)
    final_state = None
    target = np.asarray(plan.final_requested_action, dtype=np.float32)
    velocity_limit = config.joint_velocity_limit_rad_s.copy()
    velocity_limit[plan.joint - 1] = np.float32(plan.max_velocity_rad_s)

    try:
        robot.connect()
        robot.acquire_control(ControlMode.TELEOP)
        # WAIT_FEEDBACK_READY and ACQUIRE are preparation, not part of the
        # requested motion duration.
        deadline_ns = time.monotonic_ns() + int(plan.duration_s * 1e9)

        def tick(now_ns: int) -> None:
            nonlocal actions_sent, last_sequence
            if now_ns >= deadline_ns:
                stop.set()
                return
            applied = robot.send_action(target, max_velocity_rad_s=velocity_limit)
            actions_sent += 1
            last_sequence = applied.sequence

        stats = scheduler.run(tick, stop)
        final_state = robot.read_state()
    finally:
        try:
            if robot.is_connected:
                robot.hold()
                final_state = robot.read_state()
                robot.release_control()
        finally:
            robot.disconnect()

    return BringupRunResult(
        actions_sent=actions_sent,
        last_sequence=last_sequence,
        last_post_command_feedback_sequence=(
            0 if final_state is None else final_state.last_post_command_feedback_sequence
        ),
        final_mode="UNKNOWN" if final_state is None else final_state.mode.name,
        scheduler=stats,
    )


def run_real_bringup(
    config: RobotConfig,
    plan: JointBringupPlan,
    *,
    port: str,
    deadman: Deadman,
    log_jsonl: str | Path,
    wait_for_deadman_s: float = 30.0,
) -> BringupRunResult:
    if not config.hardware_parameters_verified:
        deadman.close()
        raise BringupError("real bring-up is blocked until hardware_parameters_verified is true")
    log_path = Path(log_jsonl)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    if log_path.exists():
        deadman.close()
        raise BringupError(f"refusing to overwrite existing bring-up log: {log_path}")
    robot = DummyRobot(config, SerialTransport(port))
    stop = threading.Event()
    scheduler = FixedRateScheduler(plan.control_rate_hz)
    velocity_limit = config.joint_velocity_limit_rad_s.copy()
    velocity_limit[plan.joint - 1] = np.float32(plan.max_velocity_rad_s)
    actions_sent = 0
    last_sequence = 0
    final_state = None
    stats = SchedulerStats(0, 0, 0.0, 0.0)
    acquired = False
    primary_error: BaseException | None = None

    try:
        robot.connect()
        wait_deadline = time.monotonic() + wait_for_deadman_s
        while not deadman.is_pressed():
            if time.monotonic() >= wait_deadline:
                raise BringupError("dead-man was not pressed before the wait timeout")
            time.sleep(0.02)
        robot.acquire_control(ControlMode.TELEOP)
        acquired = True
        if not deadman.is_pressed():
            raise BringupError("dead-man was released during control acquisition")

        initial_state = robot.read_state()
        if not initial_state.position_valid or not initial_state.gripper_valid:
            raise BringupError(
                "real bring-up requires valid joint and gripper feedback so untouched targets can be held"
            )
        target = initial_state.position.astype(np.float32, copy=True)
        target[plan.joint - 1] += np.float32(plan.delta_rad)
        if (
            target[plan.joint - 1] < config.joint_limit_min_rad[plan.joint - 1]
            or target[plan.joint - 1] > config.joint_limit_max_rad[plan.joint - 1]
        ):
            raise BringupError("target rebased from the live state is outside the soft limit")
        deadline_ns = time.monotonic_ns() + int(plan.duration_s * 1e9)

        with log_path.open("x", encoding="utf-8", newline="\n") as log:

            def tick(now_ns: int) -> None:
                nonlocal actions_sent, last_sequence, final_state
                if now_ns >= deadline_ns or not deadman.is_pressed():
                    stop.set()
                    return
                state_before = robot.read_state()
                applied = robot.send_action(target, max_velocity_rad_s=velocity_limit)
                actions_sent += 1
                last_sequence = applied.sequence
                final_state = robot.read_state()
                record = {
                    "monotonic_ns": now_ns,
                    "requested": applied.requested.tolist(),
                    "applied": applied.applied.tolist(),
                    "sequence": applied.sequence,
                    "last_received_sequence": final_state.last_received_sequence,
                    "last_post_command_feedback_sequence": (
                        final_state.last_post_command_feedback_sequence
                    ),
                    "state_position": final_state.position.tolist(),
                    "state_velocity": final_state.velocity.tolist(),
                    "following_error": final_state.following_error.tolist(),
                    "following_error_duration_ms": (
                        final_state.following_error_duration_ms.tolist()
                    ),
                    "feedback_age_ms": final_state.feedback_age_ms.tolist(),
                    "feedback_loss_count": final_state.feedback_loss_count.tolist(),
                    "consecutive_feedback_loss": (
                        final_state.consecutive_feedback_loss.tolist()
                    ),
                    "node_fault_bits": final_state.node_fault_bits.tolist(),
                    "node_validity": final_state.node_validity.tolist(),
                    "position_valid": final_state.position_valid,
                    "velocity_valid": final_state.velocity_valid,
                    "gripper_valid": final_state.gripper_valid,
                    "state_age_ms": (time.monotonic_ns() - state_before.monotonic_ns) / 1e6,
                    "fault_bits": final_state.fault_bits,
                    "hold_reason_bits": final_state.hold_reason_bits,
                    "telemetry_validity": final_state.telemetry_validity,
                    "mode": final_state.mode.name,
                    "firmware_version": robot.firmware_version,
                    "reasons": list(applied.reasons),
                }
                log.write(json.dumps(record, separators=(",", ":")) + "\n")
                log.flush()

            stats = scheduler.run(tick, stop)
    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        cleanup_error: BaseException | None = None
        try:
            if robot.is_connected:
                robot.hold()
                final_state = robot.read_state()
                if acquired:
                    robot.release_control()
        except BaseException as exc:
            cleanup_error = exc
        try:
            robot.disconnect()
        except BaseException as exc:
            if cleanup_error is None:
                cleanup_error = exc
        try:
            deadman.close()
        except BaseException as exc:
            if cleanup_error is None:
                cleanup_error = exc
        if primary_error is None and cleanup_error is not None:
            raise cleanup_error

    return BringupRunResult(
        actions_sent=actions_sent,
        last_sequence=last_sequence,
        last_post_command_feedback_sequence=(
            0 if final_state is None else final_state.last_post_command_feedback_sequence
        ),
        final_mode="UNKNOWN" if final_state is None else final_state.mode.name,
        scheduler=stats,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plan, simulate, or dead-man control a bounded single-joint bring-up"
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--joint", type=int, required=True)
    parser.add_argument("--delta-deg", type=float, default=0.2)
    parser.add_argument("--max-velocity", type=float, default=0.02)
    parser.add_argument("--duration", type=float, default=1.0)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--simulate", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--deadman-device")
    parser.add_argument("--deadman-key", default="KEY_SPACE")
    parser.add_argument("--log-jsonl")
    args = parser.parse_args()

    config = load_robot_config(args.config)
    plan = make_joint_bringup_plan(
        config,
        joint=args.joint,
        delta_deg=args.delta_deg,
        max_velocity_rad_s=args.max_velocity,
        duration_s=args.duration,
        # Dry-run/simulation use the configured initial pose. Real execution
        # rebases from fresh feedback and validates the live target in run_real_bringup().
        validate_initial_pose_limits=not args.execute,
    )
    if args.dry_run:
        print(json.dumps(asdict(plan), indent=2, ensure_ascii=False))
        return
    if args.simulate:
        result = run_simulated_bringup(config, plan)
        print(json.dumps(asdict(result), indent=2, ensure_ascii=False))
        return
    missing = [
        option
        for option, value in (
            ("--port", args.port),
            ("--deadman-device", args.deadman_device),
            ("--log-jsonl", args.log_jsonl),
        )
        if not value
    ]
    if missing:
        parser.error(f"--execute missing required value(s): {', '.join(missing)}")
    result = run_real_bringup(
        config,
        plan,
        port=args.port,
        deadman=EvdevDeadman(args.deadman_device, args.deadman_key),
        log_jsonl=args.log_jsonl,
    )
    print(json.dumps(asdict(result), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
