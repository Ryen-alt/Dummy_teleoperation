from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass, replace

import numpy as np

from dummy_host.gamepad import GamepadState
from dummy_host.input_evdev import InputDeviceError, create_gamepad_source
from dummy_host.schema import ControlMode, RobotConfig, RobotState, load_robot_config
from dummy_host.teleop import (
    GamepadMapper,
    JointVelocityIntegrator,
    TeleopCommand,
    TeleopProfile,
    load_teleop_profile,
    validate_profile_for_robot,
)


AXIS_ALIASES = {
    "lx": "left_x",
    "ly": "left_y",
    "rx": "right_x",
    "ry": "right_y",
    "lt": "left_trigger",
    "rt": "right_trigger",
    "dx": "dpad_x",
    "dy": "dpad_y",
}
BUTTON_ALIASES = {
    "a": "a",
    "b": "b",
    "x": "x",
    "y": "y",
    "lb": "lb",
    "rb": "rb",
    "view": "view",
    "back": "view",
    "menu": "menu",
    "start": "menu",
    "xbox": "xbox",
    "ls": "left_stick",
    "rs": "right_stick",
}


@dataclass(frozen=True)
class VirtualMappingResult:
    status: str
    command: TeleopCommand
    target: np.ndarray
    action: str = "tick"
    steps: int = 1
    target_delta: np.ndarray | None = None

    def as_dict(self) -> dict[str, object]:
        target_delta = (
            np.zeros_like(self.target)
            if self.target_delta is None
            else self.target_delta
        )
        return {
            "action": self.action,
            "steps": self.steps,
            "status": self.status,
            "connected": self.command.connected,
            "deadman": self.command.deadman,
            "hold": self.command.hold_requested,
            "estop": self.command.estop_requested,
            "episode_event": self.command.episode_event,
            "joint_velocity_rad_s": self.command.joint_velocity_rad_s.tolist(),
            "gripper_velocity_per_s": self.command.gripper_velocity_per_s,
            "virtual_target_rad": self.target.tolist(),
            "virtual_target_deg": np.rad2deg(self.target[:6]).tolist(),
            "virtual_target_delta_rad": target_delta[:6].tolist(),
            "virtual_target_delta_deg": np.rad2deg(target_delta[:6]).tolist(),
            "virtual_gripper": float(self.target[6]),
            "virtual_gripper_delta": float(target_delta[6]),
            "raw": dict(self.command.raw),
        }


class VirtualMappingSession:
    """Integrate teleop commands into a local pose without opening robot transport."""

    def __init__(self, profile: TeleopProfile, config: RobotConfig) -> None:
        validate_profile_for_robot(profile, config)
        self.profile = profile
        self.config = config
        self.integrator = JointVelocityIntegrator(profile, config)
        self.target = np.concatenate(
            (config.initial_pose_rad, np.asarray([0.5], dtype=np.float32))
        ).astype(np.float32)
        self.integrator.reset(self._state(0))

    def apply(self, command: TeleopCommand, now_ns: int) -> VirtualMappingResult:
        state = self._state(now_ns)
        if not command.connected:
            status = "DISCONNECTED/HOLD"
        elif command.estop_requested:
            status = "ESTOP_REQUESTED"
        elif command.hold_requested:
            status = "HOLD_REQUESTED"
        elif not command.deadman:
            status = "DEADMAN_RELEASED/HOLD"
        else:
            status = "ACTIVE_VIRTUAL"

        if status == "ACTIVE_VIRTUAL":
            self.target = self.integrator.step(command, state, now_ns)
        else:
            self.integrator.reset(state, now_ns)
        return VirtualMappingResult(status, command, self.target.copy())

    def _state(self, now_ns: int) -> RobotState:
        return RobotState(
            position=self.target.copy(),
            velocity=np.zeros(7, dtype=np.float32),
            monotonic_ns=now_ns,
            mcu_time_us=now_ns // 1_000,
            mode=ControlMode.TELEOP,
            fault_bits=0,
            position_valid=True,
            velocity_valid=True,
            gripper_valid=True,
            last_received_sequence=0,
            target_age_ms=0,
            config_hash=self.config.config_hash,
        )


class VirtualXboxInput:
    def __init__(self, profile: TeleopProfile) -> None:
        self.profile = profile
        self.mapper = GamepadMapper(profile)
        self.axes = {name: 0.0 for name in profile.gamepad.protocol.axes}
        self.pressed: set[str] = set()

    def poll(self, now_ns: int) -> TeleopCommand:
        state = GamepadState(
            monotonic_ns=now_ns,
            axes=self.axes,
            pressed=frozenset(self.pressed),
            protocol_id=self.profile.gamepad.protocol.protocol_id,
            raw={"transport": "virtual"},
        )
        return self.mapper.map_state(state)

    def clear(self) -> None:
        self.axes = {name: 0.0 for name in self.profile.gamepad.protocol.axes}
        self.pressed.clear()


def _advance(
    controller: VirtualXboxInput,
    session: VirtualMappingSession,
    *,
    now_ns: int,
    period_ns: int,
    count: int,
    action: str,
) -> tuple[int, VirtualMappingResult]:
    """Advance a virtual action and retain its cumulative, user-visible change."""
    if count <= 0:
        raise ValueError("count must be greater than zero")

    target_before = session.target.copy()
    result: VirtualMappingResult | None = None
    for _ in range(count):
        now_ns += period_ns
        result = session.apply(controller.poll(now_ns), now_ns)

    assert result is not None
    return now_ns, replace(
        result,
        action=action,
        steps=count,
        target_delta=result.target - target_before,
    )


def demo_results(profile: TeleopProfile, config: RobotConfig) -> list[VirtualMappingResult]:
    controller = VirtualXboxInput(profile)
    session = VirtualMappingSession(profile, config)
    period_ns = int(1e9 / config.control_rate_hz)
    now_ns = 1_000_000_000
    scenarios = (
        ("idle / deadman released", {}, set(), 1),
        ("J1 / left_x +0.8", {"left_x": 0.8}, {"lb"}, 20),
        ("J2 / left_y -0.8", {"left_y": -0.8}, {"lb"}, 20),
        ("J3 / right_y +0.8", {"right_y": 0.8}, {"lb"}, 20),
        ("J4 / right_x +0.8", {"right_x": 0.8}, {"lb"}, 20),
        ("J5 / dpad_x +1.0", {"dpad_x": 1.0}, {"lb"}, 20),
        ("J6 / dpad_y -1.0", {"dpad_y": -1.0}, {"lb"}, 20),
        ("gripper open / X", {}, {"lb", "x"}, 10),
        ("gripper close / B", {}, {"lb", "b"}, 10),
        ("hold / View", {}, {"view"}, 1),
        ("estop / LB+RB+Menu", {}, {"lb", "rb", "menu"}, 1),
    )
    output: list[VirtualMappingResult] = []
    for action, axes, pressed, steps in scenarios:
        controller.clear()
        if output:
            # Reset the acceleration limiter between independent demo cases so
            # one joint's deceleration is not mistaken for cross-axis mapping.
            now_ns += period_ns
            session.apply(controller.poll(now_ns), now_ns)
        controller.axes.update(axes)
        controller.pressed.update(pressed)
        now_ns, result = _advance(
            controller,
            session,
            now_ns=now_ns,
            period_ns=period_ns,
            count=steps,
            action=action,
        )
        output.append(result)
    return output


def _render(result: VirtualMappingResult, *, json_output: bool) -> None:
    payload = result.as_dict()
    if json_output:
        print(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
            flush=True,
        )
        return
    target_delta = (
        np.zeros_like(result.target)
        if result.target_delta is None
        else result.target_delta
    )
    velocity = ", ".join(f"{value:+.4f}" for value in result.command.joint_velocity_rad_s)
    delta = ", ".join(f"{value:+.3f}" for value in np.rad2deg(target_delta[:6]))
    target = ", ".join(f"{value:+.3f}" for value in np.rad2deg(result.target[:6]))
    raw_axes = result.command.raw.get("axes", {})
    active_axes = " ".join(
        f"{name}={float(value):+.2f}"
        for name, value in raw_axes.items()
        if abs(float(value)) > 1e-6
    )
    pressed = ",".join(str(value) for value in result.command.raw.get("pressed", []))
    print(
        f"[{result.action}] steps={result.steps} {result.status:23s} "
        f"connected={str(result.command.connected).lower()} "
        f"deadman={int(result.command.deadman)} "
        f"hold={int(result.command.hold_requested)} estop={int(result.command.estop_requested)} "
        f"axes={active_axes or '-'} buttons={pressed or '-'}\n"
        f"  qdot=[{velocity}] delta_deg=[{delta}] target_deg=[{target}] "
        f"gripper_delta={target_delta[6]:+.3f} gripper={result.target[6]:.3f} "
        f"episode={result.command.episode_event or '-'}",
        flush=True,
    )


def _canonical_axis(name: str, profile: TeleopProfile) -> str:
    logical = AXIS_ALIASES.get(name.lower(), name)
    if logical not in profile.gamepad.protocol.axes:
        raise ValueError(f"unknown axis {name!r}")
    return logical


def _canonical_button(name: str, profile: TeleopProfile) -> str:
    logical = BUTTON_ALIASES.get(name.lower(), name)
    if logical not in profile.gamepad.protocol.buttons:
        raise ValueError(f"unknown button {name!r}")
    return logical


def _run_interactive(profile: TeleopProfile, config: RobotConfig, *, json_output: bool) -> None:
    controller = VirtualXboxInput(profile)
    session = VirtualMappingSession(profile, config)
    period_ns = int(1e9 / config.control_rate_hz)
    now_ns = 1_000_000_000
    print(
        "Virtual Xbox mapping only: no serial port, control lease, camera, or robot is opened.\n"
        "Commands: axis <lx|ly|rx|ry|dx|dy> <-1..1>, press/release/tap <button>,\n"
        "          step [count], clear, show, demo, help, quit"
    )

    def tick(count: int = 1, *, action: str = "tick") -> VirtualMappingResult:
        nonlocal now_ns
        now_ns, result = _advance(
            controller,
            session,
            now_ns=now_ns,
            period_ns=period_ns,
            count=count,
            action=action,
        )
        return result

    _render(tick(action="initial"), json_output=json_output)
    while True:
        try:
            parts = input("xbox-map> ").strip().split()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not parts:
            continue
        command = parts[0].lower()
        try:
            if command in {"quit", "exit", "q"}:
                return
            if command == "axis" and len(parts) == 3:
                axis = _canonical_axis(parts[1], profile)
                value = float(parts[2])
                if not -1.0 <= value <= 1.0:
                    raise ValueError("axis value must be in [-1, 1]")
                controller.axes[axis] = value
                _render(
                    tick(action=f"axis {axis}={value:+.3f}"),
                    json_output=json_output,
                )
            elif command in {"press", "release", "tap"} and len(parts) == 2:
                button = _canonical_button(parts[1], profile)
                if command == "release":
                    controller.pressed.discard(button)
                    _render(
                        tick(action=f"release {button}"),
                        json_output=json_output,
                    )
                else:
                    controller.pressed.add(button)
                    _render(
                        tick(action=f"{command} {button}"),
                        json_output=json_output,
                    )
                    if command == "tap":
                        controller.pressed.discard(button)
                        tick(action=f"release {button}")
            elif command == "step" and len(parts) <= 2:
                count = 1 if len(parts) == 1 else int(parts[1])
                if count <= 0 or count > 10_000:
                    raise ValueError("step count must be in [1, 10000]")
                _render(
                    tick(count, action=f"step {count}"),
                    json_output=json_output,
                )
            elif command == "clear" and len(parts) == 1:
                controller.clear()
                _render(tick(action="clear"), json_output=json_output)
            elif command == "show" and len(parts) == 1:
                _render(tick(action="show"), json_output=json_output)
            elif command == "demo" and len(parts) == 1:
                for result in demo_results(profile, config):
                    _render(result, json_output=json_output)
            elif command == "help":
                print("axes: lx ly rx ry lt rt dx dy; buttons: a b x y lb rb view menu xbox ls rs")
            else:
                print("invalid command; type help")
        except (TypeError, ValueError) as exc:
            print(f"error: {exc}")


def _run_device(
    endpoint: str,
    profile: TeleopProfile,
    config: RobotConfig,
    *,
    duration_s: float | None,
    print_hz: float,
    json_output: bool,
) -> None:
    source = create_gamepad_source(endpoint, profile)
    session = VirtualMappingSession(profile, config)
    period_s = 1.0 / config.control_rate_hz
    print_period_s = 1.0 / print_hz
    started = time.monotonic()
    last_print = 0.0
    last_render_target = session.target.copy()
    steps_since_print = 0
    print(
        "Live controller -> virtual robot only; no serial port or robot is opened.",
        flush=True,
    )
    try:
        while duration_s is None or time.monotonic() - started < duration_s:
            tick_started = time.monotonic()
            now_ns = time.monotonic_ns()
            result = session.apply(source.poll(now_ns), now_ns)
            steps_since_print += 1
            if tick_started - last_print >= print_period_s:
                result = replace(
                    result,
                    action="live",
                    steps=steps_since_print,
                    target_delta=result.target - last_render_target,
                )
                _render(result, json_output=json_output)
                last_render_target = result.target.copy()
                steps_since_print = 0
                last_print = tick_started
            time.sleep(max(0.0, period_s - (time.monotonic() - tick_started)))
    except KeyboardInterrupt:
        pass
    finally:
        source.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test Xbox/custom gamepad mappings against a virtual robot pose only"
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--input-config", required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--simulate", action="store_true", help="interactive virtual Xbox controls")
    source.add_argument(
        "--device",
        help="live Linux evdev controller path, or 'auto'; still without a robot",
    )
    parser.add_argument("--demo", action="store_true", help="run a built-in virtual mapping demo and exit")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    if args.demo and not args.simulate:
        parser.error("--demo requires --simulate")
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    if args.print_hz <= 0:
        parser.error("--print-hz must be positive")

    config = load_robot_config(args.config)
    profile = load_teleop_profile(args.input_config)
    validate_profile_for_robot(profile, config)
    print(
        f"protocol={profile.gamepad.protocol.protocol_id} "
        f"transport={profile.gamepad.protocol.transport} mapping_hash={profile.config_hash}",
        flush=True,
    )
    if args.demo:
        for result in demo_results(profile, config):
            _render(result, json_output=args.json)
    elif args.simulate:
        _run_interactive(profile, config, json_output=args.json)
    else:
        assert args.device is not None
        try:
            _run_device(
                args.device,
                profile,
                config,
                duration_s=args.duration,
                print_hz=args.print_hz,
                json_output=args.json,
            )
        except InputDeviceError as exc:
            parser.exit(
                2,
                f"error: {exc}\n"
                "hint: use --device auto, or name one existing "
                "/dev/input/by-id/*-event-joystick link. eventX/eventY are "
                "documentation placeholders.\n",
            )


if __name__ == "__main__":
    main()
