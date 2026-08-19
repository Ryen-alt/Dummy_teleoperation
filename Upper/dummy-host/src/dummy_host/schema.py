from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .domain.models import AppliedAction, ControlMode, RobotState


class ConfigError(ValueError):
    pass


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
    driver: str = "realsense"
    enabled: bool = True
    required: bool = True


@dataclass(frozen=True)
class CameraRigConfig:
    rig_id: str
    version: int
    cameras: Mapping[str, CameraConfig]
    config_hash: str


@dataclass(frozen=True)
class RobotConfig:
    robot_id: str
    config_version: int
    robot_calibration_id: str
    hardware_parameters_verified: bool
    external_target_execution_ready: bool
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
    camera_rig: CameraRigConfig
    config_hash: str = field(compare=True)

    @property
    def config_hash_bytes(self) -> bytes:
        return bytes.fromhex(self.config_hash)

    @property
    def cameras(self) -> Mapping[str, CameraConfig]:
        """Compatibility view; new code should consume camera_rig explicitly."""
        return self.camera_rig.cameras


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


def _robot_config_hash(raw: Mapping[str, Any]) -> str:
    # Cameras have their own version/hash and must not force a firmware rebuild.
    camera_keys = {"camera_rig_id", "camera_rig_version", "cameras"}
    return _canonical_hash({key: value for key, value in raw.items() if key not in camera_keys})


def _read_yaml_mapping(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"cannot load {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError("configuration root must be a mapping")
    return raw


def _parse_camera_rig(raw: Mapping[str, Any]) -> CameraRigConfig:
    rig_id = raw.get("camera_rig_id")
    if not isinstance(rig_id, str) or not rig_id.strip():
        raise ConfigError("camera_rig_id must be a non-empty string")
    rig_version = _positive_int(raw, "camera_rig_version")
    cameras_raw = raw.get("cameras")
    if not isinstance(cameras_raw, dict) or not cameras_raw:
        raise ConfigError("cameras must be a non-empty mapping of logical roles")
    cameras: dict[str, CameraConfig] = {}
    for role, cam_raw in cameras_raw.items():
        if not isinstance(role, str) or not role or not role.replace("_", "").isalnum():
            raise ConfigError(f"invalid camera role {role!r}")
        if not isinstance(cam_raw, dict):
            raise ConfigError(f"camera {role} must be a mapping")
        driver = str(cam_raw.get("driver", "")).lower()
        if driver not in {"realsense", "opencv", "fake", "replay"}:
            raise ConfigError(f"camera {role} has unsupported driver {driver!r}")
        model = str(cam_raw.get("model", ""))
        if not model:
            raise ConfigError(f"camera {role} model must be non-empty")
        enabled = cam_raw.get("enabled", True)
        required = cam_raw.get("required", True)
        align_depth = cam_raw.get("align_depth_to_color", False)
        if (
            not isinstance(enabled, bool)
            or not isinstance(required, bool)
            or not isinstance(align_depth, bool)
        ):
            raise ConfigError(
                f"camera {role} enabled/required/align_depth_to_color flags must be boolean"
            )
        if required and not enabled:
            raise ConfigError(f"required camera {role} cannot be disabled")
        camera = CameraConfig(
            name=role,
            model=model,
            device_serial=str(cam_raw.get("device_serial", "")),
            width=_positive_int(cam_raw, "width"),
            height=_positive_int(cam_raw, "height"),
            fps=_positive_int(cam_raw, "fps"),
            color_format=str(cam_raw.get("color_format", "")),
            depth_format=str(cam_raw.get("depth_format", "none")),
            align_depth_to_color=align_depth,
            max_frame_age_ms=_positive_int(cam_raw, "max_frame_age_ms"),
            max_sync_skew_ms=_positive_int(cam_raw, "max_sync_skew_ms"),
            calibration_version=str(cam_raw.get("calibration_version", "")),
            color_exposure=cam_raw.get("color_exposure"),
            color_white_balance=cam_raw.get("color_white_balance"),
            depth_exposure=cam_raw.get("depth_exposure"),
            driver=driver,
            enabled=enabled,
            required=required,
        )
        if camera.color_format != "rgb8":
            raise ConfigError(f"camera {role} output color_format must be rgb8")
        if driver == "realsense" and camera.depth_format != "z16":
            raise ConfigError(f"RealSense camera {role} depth_format must be z16")
        if driver == "opencv" and camera.depth_format != "none":
            raise ConfigError(f"OpenCV camera {role} depth_format must be none")
        if not camera.calibration_version:
            raise ConfigError(f"camera {role} calibration_version is required")
        for option_name in ("color_exposure", "color_white_balance", "depth_exposure"):
            option_value = getattr(camera, option_name)
            if option_value is not None and (
                isinstance(option_value, bool)
                or not isinstance(option_value, (int, float))
                or not np.isfinite(option_value)
            ):
                raise ConfigError(f"camera {role} {option_name} must be null or finite")
        cameras[role] = camera
    camera_rig_raw = {
        "camera_rig_id": rig_id,
        "camera_rig_version": rig_version,
        "cameras": cameras_raw,
    }
    return CameraRigConfig(rig_id, rig_version, cameras, _canonical_hash(camera_rig_raw))


def load_camera_rig_config(path: str | Path) -> CameraRigConfig:
    """Load an independently versioned camera rig without changing robot safety identity."""
    return _parse_camera_rig(_read_yaml_mapping(path))


def load_robot_config(
    path: str | Path,
    *,
    camera_rig_path: str | Path | None = None,
) -> RobotConfig:
    raw = _read_yaml_mapping(path)
    robot_id = raw.get("robot_id")
    if not isinstance(robot_id, str) or not robot_id.strip():
        raise ConfigError("robot_id must be a non-empty string")

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

    camera_rig = (
        load_camera_rig_config(camera_rig_path)
        if camera_rig_path is not None
        else _parse_camera_rig(raw)
    )

    verified = raw.get("hardware_parameters_verified")
    if not isinstance(verified, bool):
        raise ConfigError("hardware_parameters_verified must be boolean")
    execution_ready = raw.get("external_target_execution_ready")
    if not isinstance(execution_ready, bool):
        raise ConfigError("external_target_execution_ready must be boolean")
    if execution_ready and not verified:
        raise ConfigError(
            "external_target_execution_ready requires hardware_parameters_verified"
        )
    robot_calibration_id = raw.get("robot_calibration_id")
    if not isinstance(robot_calibration_id, str) or not robot_calibration_id.strip():
        raise ConfigError("robot_calibration_id must be a non-empty string")

    max_overshoot = float(raw.get("max_target_overshoot_rad", 0.0))
    if not np.isfinite(max_overshoot) or max_overshoot < 0:
        raise ConfigError("max_target_overshoot_rad must be finite and non-negative")

    gripper_feedback = raw.get("gripper_state_feedback", False)
    if not isinstance(gripper_feedback, bool):
        raise ConfigError("gripper_state_feedback must be boolean")

    return RobotConfig(
        robot_id=robot_id,
        config_version=_positive_int(raw, "config_version"),
        robot_calibration_id=robot_calibration_id,
        hardware_parameters_verified=verified,
        external_target_execution_ready=execution_ready,
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
        gripper_state_feedback=gripper_feedback,
        max_state_age_ms=_positive_int(raw, "max_state_age_ms"),
        target_ttl_ms=_positive_int(raw, "target_ttl_ms"),
        lease_timeout_ms=_positive_int(raw, "lease_timeout_ms"),
        max_target_overshoot_rad=max_overshoot,
        camera_rig=camera_rig,
        config_hash=_robot_config_hash(raw),
    )
