from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .gamepad import (
    GamepadProtocolConfig,
    GamepadProtocolError,
    GamepadState,
    PhysicalAxisBinding,
)
from .schema import RobotConfig, RobotState


class TeleopConfigError(ValueError):
    pass


class TeleopError(RuntimeError):
    pass


EPISODE_EVENTS = ("start", "success", "failure", "cancel")


@dataclass(frozen=True)
class GamepadAxisBinding:
    axis: str
    joint: int
    invert: bool


@dataclass(frozen=True)
class KeyboardMapping:
    joint_positive: tuple[str, ...]
    joint_negative: tuple[str, ...]
    gripper_open: str
    gripper_close: str
    deadman: str
    hold: str
    estop: str
    episode_buttons: Mapping[str, str]


@dataclass(frozen=True)
class GamepadMapping:
    protocol: GamepadProtocolConfig
    joint_axes: tuple[GamepadAxisBinding, ...]
    gripper_open: str
    gripper_close: str
    deadman: str
    hold: str
    estop_chord: tuple[str, ...]
    episode_buttons: Mapping[str, str]
    deadzone: float
    response_exponent: float


@dataclass(frozen=True)
class TeleopProfile:
    version: int
    joint_speed_rad_s: np.ndarray
    joint_acceleration_rad_s2: np.ndarray
    gripper_speed_per_s: float
    input_timeout_ms: int
    keyboard: KeyboardMapping
    gamepad: GamepadMapping
    config_hash: str


@dataclass(frozen=True)
class TeleopCommand:
    monotonic_ns: int
    source: str
    joint_velocity_rad_s: np.ndarray
    gripper_velocity_per_s: float
    deadman: bool
    hold_requested: bool
    estop_requested: bool
    episode_event: str | None
    connected: bool
    raw: Mapping[str, object]

    def __post_init__(self) -> None:
        velocity = np.asarray(self.joint_velocity_rad_s)
        if velocity.dtype != np.float32 or velocity.shape != (6,):
            raise TeleopError("joint_velocity_rad_s must be float32[6]")
        if not np.isfinite(velocity).all() or not math.isfinite(self.gripper_velocity_per_s):
            raise TeleopError("teleop velocity contains NaN or Inf")
        if self.monotonic_ns < 0:
            raise TeleopError("monotonic_ns must be non-negative")
        if not self.source:
            raise TeleopError("teleop source must be non-empty")
        if self.episode_event is not None and self.episode_event not in EPISODE_EVENTS:
            raise TeleopError(f"unknown episode event {self.episode_event}")
        copied = velocity.astype(np.float32, copy=True)
        copied.setflags(write=False)
        object.__setattr__(self, "joint_velocity_rad_s", copied)


def _canonical_hash(raw: Mapping[str, Any]) -> str:
    payload = json.dumps(raw, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _float_array(raw: Mapping[str, Any], key: str) -> np.ndarray:
    value = np.asarray(raw.get(key), dtype=np.float32)
    if value.shape != (6,) or not np.isfinite(value).all() or np.any(value <= 0):
        raise TeleopConfigError(f"{key} must contain six positive finite values")
    value.setflags(write=False)
    return value


def _required_key(mapping: Mapping[str, Any], key: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value:
        raise TeleopConfigError(f"{key} must be a non-empty evdev code name")
    return value


def _episode_buttons(raw: object, section: str) -> dict[str, str]:
    if not isinstance(raw, dict) or set(raw) != set(EPISODE_EVENTS):
        raise TeleopConfigError(f"{section}.episode_buttons must define {list(EPISODE_EVENTS)}")
    return {event: _required_key(raw, event) for event in EPISODE_EVENTS}


def _load_gamepad_protocol(raw: Mapping[str, Any]) -> GamepadProtocolConfig:
    protocol_raw = raw.get("protocol")
    if protocol_raw is None:
        # Version-1 compatibility: logical and evdev code names were identical.
        axes_raw = raw.get("joint_axes", ())
        axis_names = [item.get("axis") for item in axes_raw if isinstance(item, dict)]
        episode_raw = raw.get("episode_buttons")
        episode_values = episode_raw.values() if isinstance(episode_raw, dict) else ()
        estop_raw = raw.get("estop_chord")
        estop_values = estop_raw if isinstance(estop_raw, (list, tuple)) else ()
        buttons = {
            value: value
            for value in (
                raw.get("gripper_open"),
                raw.get("gripper_close"),
                raw.get("deadman"),
                raw.get("hold"),
                *estop_values,
                *episode_values,
            )
            if isinstance(value, str) and value
        }
        return GamepadProtocolConfig(
            protocol_id="legacy_evdev_v1",
            transport="evdev",
            axes={name: PhysicalAxisBinding(name) for name in axis_names if isinstance(name, str)},
            buttons=buttons,
        )
    if not isinstance(protocol_raw, dict):
        raise TeleopConfigError("gamepad.protocol must be a mapping")
    protocol_id = protocol_raw.get("id")
    transport = protocol_raw.get("transport")
    if not isinstance(protocol_id, str) or not protocol_id:
        raise TeleopConfigError("gamepad.protocol.id must be non-empty")
    if not isinstance(transport, str) or not transport:
        raise TeleopConfigError("gamepad.protocol.transport must be non-empty")
    axes_raw = protocol_raw.get("axes")
    buttons_raw = protocol_raw.get("buttons")
    if not isinstance(axes_raw, dict) or not isinstance(buttons_raw, dict):
        raise TeleopConfigError("gamepad.protocol axes/buttons must be mappings")
    axes: dict[str, PhysicalAxisBinding] = {}
    for logical_name, binding_raw in axes_raw.items():
        if not isinstance(logical_name, str) or not logical_name:
            raise TeleopConfigError("gamepad protocol axis names must be non-empty")
        if isinstance(binding_raw, str):
            axes[logical_name] = PhysicalAxisBinding(binding_raw)
            continue
        if not isinstance(binding_raw, dict):
            raise TeleopConfigError(f"gamepad protocol axis {logical_name} must be a string or mapping")
        code = _required_key(binding_raw, "code")
        invert = binding_raw.get("invert", False)
        if not isinstance(invert, bool):
            raise TeleopConfigError(f"gamepad protocol axis {logical_name} invert must be boolean")
        axes[logical_name] = PhysicalAxisBinding(code, invert)
    buttons = {
        logical_name: str(code)
        for logical_name, code in buttons_raw.items()
        if isinstance(logical_name, str) and logical_name and isinstance(code, str) and code
    }
    if len(buttons) != len(buttons_raw):
        raise TeleopConfigError("gamepad protocol button names/codes must be non-empty strings")
    return GamepadProtocolConfig(protocol_id, transport, axes, buttons)


def load_teleop_profile(path: str | Path) -> TeleopProfile:
    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise TeleopConfigError(f"cannot load {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise TeleopConfigError("teleop configuration root must be a mapping")
    version = raw.get("version")
    if not isinstance(version, int) or isinstance(version, bool) or version <= 0:
        raise TeleopConfigError("version must be a positive integer")

    control = raw.get("control")
    if not isinstance(control, dict):
        raise TeleopConfigError("control must be a mapping")
    joint_speed = _float_array(control, "joint_speed_rad_s")
    joint_acceleration = _float_array(control, "joint_acceleration_rad_s2")
    gripper_speed = control.get("gripper_speed_per_s")
    timeout_ms = control.get("input_timeout_ms")
    if not isinstance(gripper_speed, (int, float)) or not math.isfinite(gripper_speed) or gripper_speed <= 0:
        raise TeleopConfigError("gripper_speed_per_s must be positive and finite")
    if not isinstance(timeout_ms, int) or isinstance(timeout_ms, bool) or timeout_ms <= 0:
        raise TeleopConfigError("input_timeout_ms must be a positive integer")

    keyboard_raw = raw.get("keyboard")
    if not isinstance(keyboard_raw, dict):
        raise TeleopConfigError("keyboard must be a mapping")
    positive = tuple(keyboard_raw.get("joint_positive", ()))
    negative = tuple(keyboard_raw.get("joint_negative", ()))
    if (
        len(positive) != 6
        or len(negative) != 6
        or not all(isinstance(item, str) and item for item in positive + negative)
    ):
        raise TeleopConfigError("keyboard joint_positive/joint_negative must each contain six keys")
    motion_keys = positive + negative
    if len(set(motion_keys)) != len(motion_keys):
        raise TeleopConfigError("keyboard joint motion keys must be unique")
    keyboard = KeyboardMapping(
        joint_positive=positive,
        joint_negative=negative,
        gripper_open=_required_key(keyboard_raw, "gripper_open"),
        gripper_close=_required_key(keyboard_raw, "gripper_close"),
        deadman=_required_key(keyboard_raw, "deadman"),
        hold=_required_key(keyboard_raw, "hold"),
        estop=_required_key(keyboard_raw, "estop"),
        episode_buttons=_episode_buttons(keyboard_raw.get("episode_buttons"), "keyboard"),
    )
    keyboard_controls = (
        keyboard.gripper_open,
        keyboard.gripper_close,
        keyboard.deadman,
        keyboard.hold,
        keyboard.estop,
        *keyboard.episode_buttons.values(),
    )
    if len(set(keyboard_controls)) != len(keyboard_controls):
        raise TeleopConfigError("keyboard safety/gripper/episode keys must be unique")
    if set(motion_keys) & set(keyboard_controls):
        raise TeleopConfigError("keyboard motion keys overlap a safety/gripper/episode key")

    gamepad_raw = raw.get("gamepad")
    if not isinstance(gamepad_raw, dict):
        raise TeleopConfigError("gamepad must be a mapping")
    try:
        protocol = _load_gamepad_protocol(gamepad_raw)
    except GamepadProtocolError as exc:
        raise TeleopConfigError(f"invalid gamepad protocol: {exc}") from exc
    mapping_raw = gamepad_raw.get("mapping", gamepad_raw)
    if not isinstance(mapping_raw, dict):
        raise TeleopConfigError("gamepad.mapping must be a mapping")
    axes_raw = mapping_raw.get("joint_axes")
    if not isinstance(axes_raw, list) or len(axes_raw) != 6:
        raise TeleopConfigError("gamepad.joint_axes must contain six bindings")
    bindings: list[GamepadAxisBinding] = []
    for item in axes_raw:
        if not isinstance(item, dict):
            raise TeleopConfigError("each gamepad axis binding must be a mapping")
        axis = _required_key(item, "axis")
        joint = item.get("joint")
        invert = item.get("invert", False)
        if not isinstance(joint, int) or isinstance(joint, bool) or joint not in range(1, 7):
            raise TeleopConfigError("gamepad axis joint must be in [1, 6]")
        if not isinstance(invert, bool):
            raise TeleopConfigError("gamepad axis invert must be boolean")
        bindings.append(GamepadAxisBinding(axis, joint, invert))
    if {item.joint for item in bindings} != set(range(1, 7)):
        raise TeleopConfigError("gamepad axes must map every joint exactly once")
    if len({item.axis for item in bindings}) != 6:
        raise TeleopConfigError("gamepad axis names must be unique")
    deadzone = mapping_raw.get("deadzone")
    exponent = mapping_raw.get("response_exponent")
    if isinstance(deadzone, bool) or not isinstance(deadzone, (int, float)) or not 0 <= deadzone < 1:
        raise TeleopConfigError("gamepad.deadzone must be in [0, 1)")
    if (
        isinstance(exponent, bool)
        or not isinstance(exponent, (int, float))
        or not math.isfinite(exponent)
        or exponent < 1
    ):
        raise TeleopConfigError("gamepad.response_exponent must be finite and >= 1")
    chord_raw = mapping_raw.get("estop_chord", ())
    if not isinstance(chord_raw, (list, tuple)):
        raise TeleopConfigError("gamepad.estop_chord must be a list")
    chord = tuple(chord_raw)
    if (
        len(chord) < 2
        or len(set(chord)) != len(chord)
        or not all(isinstance(item, str) and item for item in chord)
    ):
        raise TeleopConfigError("gamepad.estop_chord must contain at least two buttons")
    gamepad = GamepadMapping(
        protocol=protocol,
        joint_axes=tuple(bindings),
        gripper_open=_required_key(mapping_raw, "gripper_open"),
        gripper_close=_required_key(mapping_raw, "gripper_close"),
        deadman=_required_key(mapping_raw, "deadman"),
        hold=_required_key(mapping_raw, "hold"),
        estop_chord=chord,
        episode_buttons=_episode_buttons(mapping_raw.get("episode_buttons"), "gamepad"),
        deadzone=float(deadzone),
        response_exponent=float(exponent),
    )
    gamepad_controls = (
        gamepad.gripper_open,
        gamepad.gripper_close,
        gamepad.deadman,
        gamepad.hold,
        *gamepad.episode_buttons.values(),
    )
    if len(set(gamepad_controls)) != len(gamepad_controls):
        raise TeleopConfigError("gamepad primary controls must be unique")
    used_axes = {binding.axis for binding in gamepad.joint_axes}
    used_buttons = {
        gamepad.gripper_open,
        gamepad.gripper_close,
        gamepad.deadman,
        gamepad.hold,
        *gamepad.estop_chord,
        *gamepad.episode_buttons.values(),
    }
    missing_axes = used_axes - set(protocol.axes)
    missing_buttons = used_buttons - set(protocol.buttons)
    if missing_axes or missing_buttons:
        raise TeleopConfigError(
            f"gamepad mapping references controls missing from protocol: "
            f"axes={sorted(missing_axes)} buttons={sorted(missing_buttons)}"
        )

    return TeleopProfile(
        version=version,
        joint_speed_rad_s=joint_speed,
        joint_acceleration_rad_s2=joint_acceleration,
        gripper_speed_per_s=float(gripper_speed),
        input_timeout_ms=timeout_ms,
        keyboard=keyboard,
        gamepad=gamepad,
        config_hash=_canonical_hash(raw),
    )


def validate_profile_for_robot(profile: TeleopProfile, config: RobotConfig) -> None:
    if np.any(profile.joint_speed_rad_s > config.joint_velocity_limit_rad_s):
        raise TeleopConfigError("teleop joint speed exceeds robot_config.yaml limits")
    if np.any(profile.joint_acceleration_rad_s2 > config.joint_acceleration_limit_rad_s2):
        raise TeleopConfigError("teleop joint acceleration exceeds robot_config.yaml limits")


def _episode_edge(
    pressed: set[str], previous: set[str], buttons: Mapping[str, str]
) -> str | None:
    for event in EPISODE_EVENTS:
        key = buttons[event]
        if key in pressed and key not in previous:
            return event
    return None


class KeyboardMapper:
    def __init__(self, profile: TeleopProfile) -> None:
        self.profile = profile
        self._previous_pressed: set[str] = set()

    def map(self, pressed: set[str], now_ns: int, *, connected: bool = True) -> TeleopCommand:
        mapping = self.profile.keyboard
        velocity = np.zeros(6, dtype=np.float32)
        for index, (positive, negative) in enumerate(
            zip(mapping.joint_positive, mapping.joint_negative)
        ):
            direction = int(positive in pressed) - int(negative in pressed)
            velocity[index] = np.float32(direction * self.profile.joint_speed_rad_s[index])
        gripper_direction = int(mapping.gripper_close in pressed) - int(
            mapping.gripper_open in pressed
        )
        episode = _episode_edge(pressed, self._previous_pressed, mapping.episode_buttons)
        self._previous_pressed = set(pressed)
        return TeleopCommand(
            monotonic_ns=now_ns,
            source="keyboard",
            joint_velocity_rad_s=velocity,
            gripper_velocity_per_s=gripper_direction * self.profile.gripper_speed_per_s,
            deadman=connected and mapping.deadman in pressed,
            hold_requested=mapping.hold in pressed,
            estop_requested=mapping.estop in pressed,
            episode_event=episode,
            connected=connected,
            raw={"pressed": sorted(pressed)},
        )


def shape_axis(value: float, deadzone: float, exponent: float) -> float:
    if not math.isfinite(value):
        raise TeleopError("gamepad axis contains NaN or Inf")
    value = max(-1.0, min(1.0, value))
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return math.copysign(scaled**exponent, value)


class GamepadMapper:
    def __init__(self, profile: TeleopProfile) -> None:
        self.profile = profile
        self._previous_pressed: set[str] = set()

    def map(
        self,
        axes: Mapping[str, float],
        pressed: set[str],
        now_ns: int,
        *,
        connected: bool = True,
        protocol_id: str | None = None,
        transport_raw: Mapping[str, object] | None = None,
    ) -> TeleopCommand:
        mapping = self.profile.gamepad
        velocity = np.zeros(6, dtype=np.float32)
        shaped_axes: dict[str, float] = {}
        for binding in mapping.joint_axes:
            raw_value = float(axes.get(binding.axis, 0.0))
            shaped = shape_axis(raw_value, mapping.deadzone, mapping.response_exponent)
            if binding.invert:
                shaped = -shaped
            shaped_axes[binding.axis] = shaped
            velocity[binding.joint - 1] = np.float32(
                shaped * self.profile.joint_speed_rad_s[binding.joint - 1]
            )
        gripper_direction = int(mapping.gripper_close in pressed) - int(
            mapping.gripper_open in pressed
        )
        episode = _episode_edge(pressed, self._previous_pressed, mapping.episode_buttons)
        self._previous_pressed = set(pressed)
        return TeleopCommand(
            monotonic_ns=now_ns,
            source="gamepad",
            joint_velocity_rad_s=velocity,
            gripper_velocity_per_s=gripper_direction * self.profile.gripper_speed_per_s,
            deadman=connected and mapping.deadman in pressed,
            hold_requested=mapping.hold in pressed,
            estop_requested=set(mapping.estop_chord).issubset(pressed),
            episode_event=episode,
            connected=connected,
            raw={
                "protocol_id": protocol_id or mapping.protocol.protocol_id,
                "axes": dict(sorted(axes.items())),
                "shaped_axes": shaped_axes,
                "pressed": sorted(pressed),
                "transport": {} if transport_raw is None else dict(transport_raw),
            },
        )

    def map_state(self, state: GamepadState) -> TeleopCommand:
        return self.map(
            state.axes,
            set(state.pressed),
            state.monotonic_ns,
            connected=state.connected,
            protocol_id=state.protocol_id,
            transport_raw=state.raw,
        )


class JointVelocityIntegrator:
    def __init__(self, profile: TeleopProfile, config: RobotConfig) -> None:
        validate_profile_for_robot(profile, config)
        self.profile = profile
        self.config = config
        self._target: np.ndarray | None = None
        self._velocity = np.zeros(6, dtype=np.float32)
        self._last_time_ns: int | None = None

    def reset(self, state: RobotState | None = None, now_ns: int | None = None) -> None:
        self._velocity.fill(0)
        self._target = None if state is None else state.position.astype(np.float32, copy=True)
        self._last_time_ns = now_ns

    def step(self, command: TeleopCommand, state: RobotState, now_ns: int) -> np.ndarray:
        if not command.connected or not command.deadman or command.hold_requested or command.estop_requested:
            raise TeleopError("cannot integrate a command without an active dead-man")
        age_ms = (now_ns - command.monotonic_ns) / 1e6
        if age_ms < 0 or age_ms > self.profile.input_timeout_ms:
            raise TeleopError(f"input command is stale ({age_ms:.1f} ms)")
        if not state.position_valid or state.position.shape != (7,):
            raise TeleopError("valid float32[7] robot position is required")
        if not state.gripper_valid:
            raise TeleopError("valid gripper feedback is required before teleoperation")

        period_s = 1.0 / self.config.control_rate_hz
        if self._target is None:
            self._target = state.position.astype(np.float32, copy=True)
        if self._last_time_ns is None:
            dt = period_s
        else:
            measured_dt = (now_ns - self._last_time_ns) / 1e9
            if measured_dt <= 0 or measured_dt > self.profile.input_timeout_ms / 1000:
                raise TeleopError("teleop control period is invalid or timed out")
            dt = max(period_s * 0.5, min(period_s * 1.5, measured_dt))

        desired = np.clip(
            command.joint_velocity_rad_s,
            -self.profile.joint_speed_rad_s,
            self.profile.joint_speed_rad_s,
        )
        max_delta = self.profile.joint_acceleration_rad_s2 * dt
        self._velocity = np.clip(desired, self._velocity - max_delta, self._velocity + max_delta)
        self._target[:6] += self._velocity * dt
        self._target[:6] = np.clip(
            self._target[:6], self.config.joint_limit_min_rad, self.config.joint_limit_max_rad
        )
        gripper_velocity = float(
            np.clip(
                command.gripper_velocity_per_s,
                -self.profile.gripper_speed_per_s,
                self.profile.gripper_speed_per_s,
            )
        )
        self._target[6] = np.float32(
            np.clip(
                self._target[6] + gripper_velocity * dt,
                self.config.gripper_range[0],
                self.config.gripper_range[1],
            )
        )
        self._last_time_ns = now_ns
        return self._target.astype(np.float32, copy=True)
