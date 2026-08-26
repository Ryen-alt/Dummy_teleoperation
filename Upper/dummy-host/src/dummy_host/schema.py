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
class CameraCalibration:
    schema_version: int
    calibration_id: str
    calibrated_utc: str
    camera_model: str
    device_serial: str
    width: int
    height: int
    intrinsic_matrix: np.ndarray
    distortion_model: str
    distortion_coefficients: np.ndarray
    parent_frame: str
    translation_m: np.ndarray
    rotation_xyzw: np.ndarray
    source_path: str
    file_hash: str


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
    calibration_file: str | None = None
    calibration_hash: str | None = None
    driver: str = "realsense"
    enabled: bool = True
    required: bool = True


@dataclass(frozen=True)
class CameraRigConfig:
    rig_id: str
    version: int
    cameras: Mapping[str, CameraConfig]
    calibrations: Mapping[str, CameraCalibration] = field(compare=False)
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
    can_scheduler_watchdog_hz: int
    can_target_hz_per_node: int
    can_position_hz_per_node: int
    can_temperature_hz_per_node: int
    coherent_max_skew_ms: int
    can_node_quiet_us: int
    can_response_timeout_us: int
    can_tx_abort_timeout_us: int
    can_target_fanout_timeout_us: int
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
    gripper_velocity_limit_per_s: float
    gripper_acceleration_limit_per_s2: float
    joint_following_error_limit_rad: np.ndarray
    gripper_following_error_limit: float
    following_error_hold_ms: int
    feedback_hold_ms: int
    feedback_fault_ms: int
    temperature_max_age_ms: int
    temperature_fault_c: float
    temperature_fault_ms: int
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


def _positive_float(raw: Mapping[str, Any], key: str) -> float:
    value = raw.get(key)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not np.isfinite(value)
        or value <= 0
    ):
        raise ConfigError(f"{key} must be a positive finite number")
    return float(value)


def _canonical_hash(raw: Mapping[str, Any]) -> str:
    encoded = json.dumps(raw, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise ConfigError(f"cannot read calibration file {path}: {exc}") from exc
    return digest.hexdigest()


def _finite_vector(raw: Mapping[str, Any], key: str, length: int) -> np.ndarray:
    value = _array(raw, key, length, dtype=np.float64)
    return value


def load_camera_calibration(path: str | Path) -> CameraCalibration:
    path = Path(path)
    raw = _read_yaml_mapping(path)
    if raw.get("schema_version") != 1:
        raise ConfigError("camera calibration schema_version must be 1")
    calibration_id = raw.get("calibration_id")
    calibrated_utc = raw.get("calibrated_utc")
    camera = raw.get("camera")
    intrinsics = raw.get("intrinsics")
    extrinsics = raw.get("extrinsics")
    if not isinstance(calibration_id, str) or not calibration_id.strip():
        raise ConfigError("camera calibration_id must be a non-empty string")
    if not isinstance(calibrated_utc, str) or not calibrated_utc.strip():
        raise ConfigError("camera calibrated_utc must be a non-empty string")
    if not isinstance(camera, dict):
        raise ConfigError("camera calibration camera must be a mapping")
    if not isinstance(intrinsics, dict):
        raise ConfigError("camera calibration intrinsics must be a mapping")
    if not isinstance(extrinsics, dict):
        raise ConfigError("camera calibration extrinsics must be a mapping")
    model = camera.get("model")
    device_serial = camera.get("device_serial")
    if not isinstance(model, str) or not model.strip():
        raise ConfigError("camera calibration model must be a non-empty string")
    if not isinstance(device_serial, str) or not device_serial.strip():
        raise ConfigError("camera calibration device_serial must be a non-empty string")
    width = _positive_int(camera, "width")
    height = _positive_int(camera, "height")
    matrix = _finite_vector(intrinsics, "matrix", 9).reshape(3, 3)
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or abs(matrix[2, 2] - 1.0) > 1e-6:
        raise ConfigError("camera intrinsic matrix has invalid focal length or homogeneous row")
    distortion_model = intrinsics.get("distortion_model")
    coefficients_raw = intrinsics.get("coefficients")
    if not isinstance(distortion_model, str) or not distortion_model.strip():
        raise ConfigError("camera distortion_model must be a non-empty string")
    if not isinstance(coefficients_raw, list) or not coefficients_raw:
        raise ConfigError("camera distortion coefficients must be a non-empty list")
    coefficients = np.asarray(coefficients_raw, dtype=np.float64)
    if coefficients.ndim != 1 or not np.isfinite(coefficients).all():
        raise ConfigError("camera distortion coefficients must be a finite vector")
    coefficients.setflags(write=False)
    parent_frame = extrinsics.get("parent_frame")
    if not isinstance(parent_frame, str) or not parent_frame.strip():
        raise ConfigError("camera extrinsics parent_frame must be a non-empty string")
    translation = _finite_vector(extrinsics, "translation_m", 3)
    rotation = _finite_vector(extrinsics, "rotation_xyzw", 4)
    rotation_norm = float(np.linalg.norm(rotation))
    if abs(rotation_norm - 1.0) > 1e-3:
        raise ConfigError("camera extrinsic rotation_xyzw must be a unit quaternion")
    return CameraCalibration(
        schema_version=1,
        calibration_id=calibration_id,
        calibrated_utc=calibrated_utc,
        camera_model=model,
        device_serial=device_serial,
        width=width,
        height=height,
        intrinsic_matrix=matrix,
        distortion_model=distortion_model,
        distortion_coefficients=coefficients,
        parent_frame=parent_frame,
        translation_m=translation,
        rotation_xyzw=rotation,
        source_path=str(path.resolve()),
        file_hash=_file_hash(path),
    )


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


def _parse_camera_rig(raw: Mapping[str, Any], *, base_dir: Path) -> CameraRigConfig:
    rig_id = raw.get("camera_rig_id")
    if not isinstance(rig_id, str) or not rig_id.strip():
        raise ConfigError("camera_rig_id must be a non-empty string")
    rig_version = _positive_int(raw, "camera_rig_version")
    cameras_raw = raw.get("cameras")
    if not isinstance(cameras_raw, dict) or not cameras_raw:
        raise ConfigError("cameras must be a non-empty mapping of logical roles")
    cameras: dict[str, CameraConfig] = {}
    calibrations: dict[str, CameraCalibration] = {}
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
        calibration_file_raw = cam_raw.get("calibration_file")
        if calibration_file_raw is not None and (
            not isinstance(calibration_file_raw, str) or not calibration_file_raw.strip()
        ):
            raise ConfigError(f"camera {role} calibration_file must be null or a path")
        calibration: CameraCalibration | None = None
        calibration_path: Path | None = None
        if isinstance(calibration_file_raw, str):
            calibration_path = Path(calibration_file_raw)
            if not calibration_path.is_absolute():
                calibration_path = base_dir / calibration_path
            calibration = load_camera_calibration(calibration_path)
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
            calibration_file=None if calibration_path is None else str(calibration_path.resolve()),
            calibration_hash=None if calibration is None else calibration.file_hash,
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
        if calibration is not None:
            if calibration.calibration_id != camera.calibration_version:
                raise ConfigError(
                    f"camera {role} calibration ID does not match calibration_version"
                )
            if calibration.camera_model != camera.model:
                raise ConfigError(f"camera {role} calibration model does not match rig")
            if calibration.device_serial != camera.device_serial:
                raise ConfigError(f"camera {role} calibration device_serial does not match rig")
            if (calibration.width, calibration.height) != (camera.width, camera.height):
                raise ConfigError(f"camera {role} calibration resolution does not match rig")
        for option_name in ("color_exposure", "color_white_balance", "depth_exposure"):
            option_value = getattr(camera, option_name)
            if option_value is not None and (
                isinstance(option_value, bool)
                or not isinstance(option_value, (int, float))
                or not np.isfinite(option_value)
            ):
                raise ConfigError(f"camera {role} {option_name} must be null or finite")
        cameras[role] = camera
        if calibration is not None:
            calibrations[role] = calibration
    camera_rig_raw = {
        "camera_rig_id": rig_id,
        "camera_rig_version": rig_version,
        "cameras": cameras_raw,
        "calibration_sha256": {
            role: calibration.file_hash for role, calibration in sorted(calibrations.items())
        },
    }
    return CameraRigConfig(
        rig_id,
        rig_version,
        cameras,
        calibrations,
        _canonical_hash(camera_rig_raw),
    )


def load_camera_rig_config(path: str | Path) -> CameraRigConfig:
    """Load an independently versioned camera rig without changing robot safety identity."""
    path = Path(path)
    return _parse_camera_rig(_read_yaml_mapping(path), base_dir=path.parent)


def validate_camera_rig_for_formal_collection(rig: CameraRigConfig) -> None:
    """Reject smoke-test camera metadata at the formal Raw Session boundary."""
    errors: list[str] = []
    for role, camera in rig.cameras.items():
        if not camera.enabled or not camera.required:
            continue
        if camera.calibration_version.lower().startswith("uncalibrated"):
            errors.append(f"{role}: calibration_version is uncalibrated")
        if camera.calibration_version.lower().startswith("example-"):
            errors.append(f"{role}: example calibration must be replaced with measured data")
        if camera.calibration_file is None or camera.calibration_hash is None:
            errors.append(f"{role}: versioned calibration_file is required")
        if camera.color_exposure is None:
            errors.append(f"{role}: fixed color_exposure is required")
        if camera.color_white_balance is None:
            errors.append(f"{role}: fixed color_white_balance is required")
        if camera.driver == "realsense" and camera.depth_exposure is None:
            errors.append(f"{role}: fixed depth_exposure is required")
    if errors:
        raise ConfigError(
            "camera rig is not ready for formal collection: " + "; ".join(errors)
        )


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
        else _parse_camera_rig(raw, base_dir=Path(path).parent)
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

    following_limits = _array(raw, "joint_following_error_limit_rad", 6)
    if np.any(following_limits <= 0):
        raise ConfigError("joint_following_error_limit_rad must be positive")
    feedback_hold_ms = _positive_int(raw, "feedback_hold_ms")
    feedback_fault_ms = _positive_int(raw, "feedback_fault_ms")
    if feedback_fault_ms <= feedback_hold_ms:
        raise ConfigError("feedback_fault_ms must be greater than feedback_hold_ms")
    can_scheduler_watchdog_hz = _positive_int(
        raw, "can_scheduler_watchdog_hz"
    )
    can_target_hz = _positive_int(raw, "can_target_hz_per_node")
    can_position_hz = _positive_int(raw, "can_position_hz_per_node")
    can_temperature_hz = _positive_int(raw, "can_temperature_hz_per_node")
    coherent_max_skew_ms = _positive_int(raw, "coherent_max_skew_ms")
    can_node_quiet_us = _positive_int(raw, "can_node_quiet_us")
    can_response_timeout_us = _positive_int(raw, "can_response_timeout_us")
    can_tx_abort_timeout_us = _positive_int(raw, "can_tx_abort_timeout_us")
    can_target_fanout_timeout_us = _positive_int(
        raw, "can_target_fanout_timeout_us"
    )
    if not 100 <= can_scheduler_watchdog_hz <= 5000:
        raise ConfigError(
            "can_scheduler_watchdog_hz must be between 100 and 5000 Hz"
        )
    if coherent_max_skew_ms * can_position_hz < 1000:
        raise ConfigError(
            "coherent_max_skew_ms is shorter than one configured position period"
        )
    if can_response_timeout_us > can_node_quiet_us:
        raise ConfigError(
            "can_response_timeout_us must not exceed can_node_quiet_us"
        )
    if can_target_fanout_timeout_us < can_tx_abort_timeout_us:
        raise ConfigError(
            "can_target_fanout_timeout_us must cover can_tx_abort_timeout_us"
        )

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
        can_scheduler_watchdog_hz=can_scheduler_watchdog_hz,
        can_target_hz_per_node=can_target_hz,
        can_position_hz_per_node=can_position_hz,
        can_temperature_hz_per_node=can_temperature_hz,
        coherent_max_skew_ms=coherent_max_skew_ms,
        can_node_quiet_us=can_node_quiet_us,
        can_response_timeout_us=can_response_timeout_us,
        can_tx_abort_timeout_us=can_tx_abort_timeout_us,
        can_target_fanout_timeout_us=can_target_fanout_timeout_us,
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
        gripper_velocity_limit_per_s=_positive_float(
            raw, "gripper_velocity_limit_per_s"
        ),
        gripper_acceleration_limit_per_s2=_positive_float(
            raw, "gripper_acceleration_limit_per_s2"
        ),
        joint_following_error_limit_rad=following_limits,
        gripper_following_error_limit=_positive_float(
            raw, "gripper_following_error_limit"
        ),
        following_error_hold_ms=_positive_int(raw, "following_error_hold_ms"),
        feedback_hold_ms=feedback_hold_ms,
        feedback_fault_ms=feedback_fault_ms,
        temperature_max_age_ms=_positive_int(raw, "temperature_max_age_ms"),
        temperature_fault_c=_positive_float(raw, "temperature_fault_c"),
        temperature_fault_ms=_positive_int(raw, "temperature_fault_ms"),
        max_state_age_ms=_positive_int(raw, "max_state_age_ms"),
        target_ttl_ms=_positive_int(raw, "target_ttl_ms"),
        lease_timeout_ms=_positive_int(raw, "lease_timeout_ms"),
        max_target_overshoot_rad=max_overshoot,
        camera_rig=camera_rig,
        config_hash=_robot_config_hash(raw),
    )
