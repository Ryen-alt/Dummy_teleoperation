from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml


class ConfigError(ValueError):
    pass


class ControlMode(IntEnum):
    DISABLED = 1
    HOLD = 2
    TELEOP = 3
    POLICY = 4
    GRAVITY = 5
    FAULT = 6


@dataclass(frozen=True)
class CameraConfig:
    name: str
    model: str
    device_serial: str
    width: int
    height: int
    fps: int
    color_format: str
    depth_format: str
    align_depth_to_color: bool
    max_frame_age_ms: int
    max_sync_skew_ms: int
    calibration_version: str
    color_exposure: float | None = None
    color_white_balance: float | None = None
    depth_exposure: float | None = None


@dataclass(frozen=True)
class RobotConfig:
    robot_id: str
    config_version: int
    hardware_parameters_verified: bool
    joint_order: tuple[str, ...]
    joint_unit: str
    action_semantics: str
    control_rate_hz: int
    firmware_loop_hz: int
    joint_zero_offset_rad: np.ndarray
    joint_sign: np.ndarray
    joint_reduction: np.ndarray
    joint_limit_min_rad: np.ndarray
    joint_limit_max_rad: np.ndarray
    joint_velocity_limit_rad_s: np.ndarray
    joint_acceleration_limit_rad_s2: np.ndarray
    initial_pose_rad: np.ndarray
    gripper_range: tuple[float, float]
    gripper_state_feedback: bool
    max_state_age_ms: int
    target_ttl_ms: int
    lease_timeout_ms: int
    max_target_overshoot_rad: float
    cameras: Mapping[str, CameraConfig]
    config_hash: str = field(compare=True)

    @property
    def config_hash_bytes(self) -> bytes:
        return bytes.fromhex(self.config_hash)


@dataclass(frozen=True)
class RobotState:
    position: np.ndarray
    velocity: np.ndarray
    monotonic_ns: int
    mcu_time_us: int
    mode: ControlMode
    fault_bits: int
    position_valid: bool
    velocity_valid: bool
    gripper_valid: bool
    last_received_sequence: int
    last_applied_sequence: int
    target_age_ms: int
    config_hash: str


@dataclass(frozen=True)
class AppliedAction:
    requested: np.ndarray
    applied: np.ndarray
    sequence: int
    monotonic_ns: int
    clipped: bool
    reasons: tuple[str, ...]


def _array(raw: Mapping[str, Any], key: str, length: int, *, dtype: Any = np.float32) -> np.ndarray:
    if key not in raw:
        raise ConfigError(f"missing required field: {key}")
    value = np.asarray(raw[key], dtype=dtype)
    if value.shape != (length,):
        raise ConfigError(f"{key} must contain {length} values")
    if not np.all(np.isfinite(value)):
        raise ConfigError(f"{key} contains NaN or Inf")
    value.setflags(write=False)
    return value


def _positive_int(raw: Mapping[str, Any], key: str) -> int:
    value = raw.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ConfigError(f"{key} must be a positive integer")
    return value


def _canonical_hash(raw: Mapping[str, Any]) -> str:
    encoded = json.dumps(raw, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_robot_config(path: str | Path) -> RobotConfig:
    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"cannot load {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError("configuration root must be a mapping")

    order = tuple(raw.get("joint_order", ()))
    expected_order = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")
    if order != expected_order or len(set(order)) != 7:
        raise ConfigError(f"joint_order must be exactly {list(expected_order)}")
    if raw.get("joint_unit") != "rad":
        raise ConfigError("joint_unit must be rad")
    if raw.get("action_semantics") != "absolute_joint_position":
        raise ConfigError("only absolute_joint_position actions are supported")

    mins = _array(raw, "joint_limit_min_rad", 6)
    maxs = _array(raw, "joint_limit_max_rad", 6)
    if np.any(mins >= maxs):
        raise ConfigError("each joint minimum must be lower than its maximum")
    signs = _array(raw, "joint_sign", 6)
    if not np.all(np.isin(signs, (-1.0, 1.0))):
        raise ConfigError("joint_sign values must be -1 or 1")
    velocity = _array(raw, "joint_velocity_limit_rad_s", 6)
    acceleration = _array(raw, "joint_acceleration_limit_rad_s2", 6)
    reduction = _array(raw, "joint_reduction", 6)
    if np.any(velocity <= 0) or np.any(acceleration <= 0) or np.any(reduction <= 0):
        raise ConfigError("joint velocity, acceleration and reduction must be positive")

    initial = _array(raw, "initial_pose_rad", 6)
    if np.any(initial < mins) or np.any(initial > maxs):
        raise ConfigError("initial_pose_rad is outside joint limits")

    gripper = tuple(float(v) for v in raw.get("gripper_range", ()))
    if len(gripper) != 2 or not np.isfinite(gripper).all() or gripper[0] >= gripper[1]:
        raise ConfigError("gripper_range must contain an increasing finite pair")

    cameras_raw = raw.get("cameras")
    if not isinstance(cameras_raw, dict) or set(cameras_raw) != {"wrist"}:
        raise ConfigError("this deployment requires exactly one camera named wrist")
    cam_raw = cameras_raw["wrist"]
    if not isinstance(cam_raw, dict) or str(cam_raw.get("model", "")).upper() != "D435":
        raise ConfigError("wrist camera model must be D435")
    camera = CameraConfig(
        name="wrist",
        model="D435",
        device_serial=str(cam_raw.get("device_serial", "")),
        width=_positive_int(cam_raw, "width"),
        height=_positive_int(cam_raw, "height"),
        fps=_positive_int(cam_raw, "fps"),
        color_format=str(cam_raw.get("color_format", "")),
        depth_format=str(cam_raw.get("depth_format", "")),
        align_depth_to_color=bool(cam_raw.get("align_depth_to_color", True)),
        max_frame_age_ms=_positive_int(cam_raw, "max_frame_age_ms"),
        max_sync_skew_ms=_positive_int(cam_raw, "max_sync_skew_ms"),
        calibration_version=str(cam_raw.get("calibration_version", "")),
        color_exposure=cam_raw.get("color_exposure"),
        color_white_balance=cam_raw.get("color_white_balance"),
        depth_exposure=cam_raw.get("depth_exposure"),
    )
    if camera.color_format != "bgr8" or camera.depth_format != "z16":
        raise ConfigError("D435 formats must be bgr8 color and z16 depth")
    if not camera.calibration_version:
        raise ConfigError("camera calibration_version is required")

    verified = raw.get("hardware_parameters_verified")
    if not isinstance(verified, bool):
        raise ConfigError("hardware_parameters_verified must be boolean")

    return RobotConfig(
        robot_id=str(raw.get("robot_id", "")),
        config_version=_positive_int(raw, "config_version"),
        hardware_parameters_verified=verified,
        joint_order=order,
        joint_unit="rad",
        action_semantics="absolute_joint_position",
        control_rate_hz=_positive_int(raw, "control_rate_hz"),
        firmware_loop_hz=_positive_int(raw, "firmware_loop_hz"),
        joint_zero_offset_rad=_array(raw, "joint_zero_offset_rad", 6),
        joint_sign=signs,
        joint_reduction=reduction,
        joint_limit_min_rad=mins,
        joint_limit_max_rad=maxs,
        joint_velocity_limit_rad_s=velocity,
        joint_acceleration_limit_rad_s2=acceleration,
        initial_pose_rad=initial,
        gripper_range=(gripper[0], gripper[1]),
        gripper_state_feedback=bool(raw.get("gripper_state_feedback", False)),
        max_state_age_ms=_positive_int(raw, "max_state_age_ms"),
        target_ttl_ms=_positive_int(raw, "target_ttl_ms"),
        lease_timeout_ms=_positive_int(raw, "lease_timeout_ms"),
        max_target_overshoot_rad=float(raw.get("max_target_overshoot_rad", 0.0)),
        cameras={"wrist": camera},
        config_hash=_canonical_hash(raw),
    )
