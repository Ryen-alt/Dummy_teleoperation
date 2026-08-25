from __future__ import annotations

import hashlib
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Mapping

import numpy as np
import yaml

from ..calibration.geometry import (
    make_transform,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
)
from ..schema import RobotConfig
from .contracts import KinematicsError


def _sha256_file(path: Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError as exc:
        raise KinematicsError(f"cannot hash {path}: {exc}") from exc


def _sha256_value(value: object, name: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdefABCDEF" for character in value)
    ):
        raise KinematicsError(f"{name} must be a SHA-256 hex digest")
    return value.lower()


def _vector(value: object, length: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise KinematicsError(f"{name} must contain {length} finite values")
    result = result.copy()
    result.setflags(write=False)
    return result


def _optional_string(value: object, name: str, *, required: bool) -> str | None:
    if value is None and not required:
        return None
    if not isinstance(value, str) or not value.strip():
        raise KinematicsError(f"{name} must be a non-empty string")
    return value.strip()


@dataclass(frozen=True)
class CartesianCalibration:
    version: int
    calibration_id: str
    robot_id: str
    validated: bool
    validated_utc: str | None
    urdf_sha256: str
    ready_pose_rad: np.ndarray | None
    ready_tolerance_rad: np.ndarray | None
    tip_frame: str
    tool0_T_tcp: np.ndarray | None
    evidence_session: str | None
    source_path: str
    file_hash: str

    def as_dict(self) -> dict[str, object]:
        return {
            "version": self.version,
            "calibration_id": self.calibration_id,
            "robot_id": self.robot_id,
            "validated": self.validated,
            "validated_utc": self.validated_utc,
            "urdf_sha256": self.urdf_sha256,
            "ready_pose_rad": (
                None if self.ready_pose_rad is None else self.ready_pose_rad.tolist()
            ),
            "ready_tolerance_rad": (
                None
                if self.ready_tolerance_rad is None
                else self.ready_tolerance_rad.tolist()
            ),
            "tip_frame": self.tip_frame,
            "tool0_T_tcp": (
                None
                if self.tool0_T_tcp is None
                else {
                    "translation_m": self.tool0_T_tcp[:3, 3].tolist(),
                    "rotation_xyzw": matrix_to_quaternion_xyzw(
                        self.tool0_T_tcp[:3, :3]
                    ).tolist(),
                }
            ),
            "evidence_session": self.evidence_session,
            "source_path": self.source_path,
            "sha256": self.file_hash,
        }

    def validate_for(
        self,
        robot_config: RobotConfig,
        urdf_path: str | Path,
        *,
        require_validated: bool,
    ) -> None:
        if self.robot_id != robot_config.robot_id:
            raise KinematicsError(
                f"Cartesian calibration robot_id {self.robot_id!r} does not match "
                f"{robot_config.robot_id!r}"
            )
        actual_urdf_hash = _sha256_file(Path(urdf_path))
        if self.urdf_sha256 != actual_urdf_hash:
            raise KinematicsError(
                "Cartesian calibration URDF hash does not match the selected model"
            )
        if require_validated and not self.validated:
            raise KinematicsError("real Cartesian execution requires a validated calibration")
        if not self.validated:
            return
        assert self.ready_pose_rad is not None
        assert self.ready_tolerance_rad is not None
        assert self.tool0_T_tcp is not None
        lower = robot_config.joint_limit_min_rad
        upper = robot_config.joint_limit_max_rad
        if np.any(self.ready_pose_rad < lower) or np.any(self.ready_pose_rad > upper):
            raise KinematicsError("Cartesian-ready pose is outside configured joint limits")
        if np.any(self.ready_pose_rad - self.ready_tolerance_rad < lower) or np.any(
            self.ready_pose_rad + self.ready_tolerance_rad > upper
        ):
            raise KinematicsError(
                "Cartesian-ready tolerance band extends outside configured joint limits"
            )

    def ready_error(self, joint_position_rad: np.ndarray) -> np.ndarray:
        if not self.validated or self.ready_pose_rad is None:
            raise KinematicsError("Cartesian calibration is not ready for execution")
        joints = _vector(joint_position_rad, 6, "measured joint position")
        return np.abs(joints - self.ready_pose_rad)

    def is_ready(self, joint_position_rad: np.ndarray) -> bool:
        if self.ready_tolerance_rad is None:
            return False
        return bool(np.all(self.ready_error(joint_position_rad) <= self.ready_tolerance_rad))


def load_cartesian_calibration(path: str | Path) -> CartesianCalibration:
    source = Path(path)
    try:
        raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise KinematicsError(f"cannot load Cartesian calibration {source}: {exc}") from exc
    if not isinstance(raw, Mapping):
        raise KinematicsError("Cartesian calibration root must be a mapping")

    version = raw.get("version")
    if not isinstance(version, int) or isinstance(version, bool) or version != 1:
        raise KinematicsError("Cartesian calibration version must be 1")
    calibration_id = _optional_string(raw.get("calibration_id"), "calibration_id", required=True)
    robot_id = _optional_string(raw.get("robot_id"), "robot_id", required=True)
    validated = raw.get("validated")
    if not isinstance(validated, bool):
        raise KinematicsError("validated must be boolean")
    validated_utc = _optional_string(
        raw.get("validated_utc"), "validated_utc", required=validated
    )
    if validated_utc is not None:
        try:
            parsed_time = datetime.fromisoformat(
                validated_utc.replace("Z", "+00:00")
            )
        except ValueError as exc:
            raise KinematicsError("validated_utc must be an ISO-8601 timestamp") from exc
        if parsed_time.tzinfo is None:
            raise KinematicsError("validated_utc must include a timezone")
    if not validated and validated_utc is not None:
        raise KinematicsError("unvalidated Cartesian calibration must have null validated_utc")
    urdf_sha256 = _sha256_value(raw.get("urdf_sha256"), "urdf_sha256")
    tip_frame = _optional_string(raw.get("tip_frame"), "tip_frame", required=True)
    if tip_frame != "tcp":
        raise KinematicsError("validated Cartesian calibration tip_frame must be 'tcp'")

    ready_pose: np.ndarray | None = None
    ready_tolerance: np.ndarray | None = None
    tool0_T_tcp: np.ndarray | None = None
    evidence = _optional_string(
        raw.get("evidence_session"), "evidence_session", required=validated
    )
    if not validated and evidence is not None:
        raise KinematicsError("unvalidated Cartesian calibration must have null evidence_session")
    if validated:
        ready_pose = _vector(raw.get("ready_pose_rad"), 6, "ready_pose_rad")
        ready_tolerance = _vector(
            raw.get("ready_tolerance_rad"), 6, "ready_tolerance_rad"
        )
        if np.any(ready_tolerance <= 0):
            raise KinematicsError("ready_tolerance_rad must be positive")
        transform_raw = raw.get("tool0_T_tcp")
        if not isinstance(transform_raw, Mapping):
            raise KinematicsError("tool0_T_tcp must be a mapping")
        translation = _vector(
            transform_raw.get("translation_m"), 3, "tool0_T_tcp.translation_m"
        )
        quaternion = _vector(
            transform_raw.get("rotation_xyzw"), 4, "tool0_T_tcp.rotation_xyzw"
        )
        if not np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-6, rtol=0.0):
            raise KinematicsError("tool0_T_tcp.rotation_xyzw must be a unit quaternion")
        tool0_T_tcp = make_transform(
            quaternion_xyzw_to_matrix(quaternion), translation
        )
        tool0_T_tcp.setflags(write=False)
    else:
        for name in ("ready_pose_rad", "ready_tolerance_rad", "tool0_T_tcp"):
            if raw.get(name) is not None:
                raise KinematicsError(
                    f"unvalidated Cartesian calibration must leave {name} null"
                )

    assert calibration_id is not None
    assert robot_id is not None
    assert tip_frame is not None
    return CartesianCalibration(
        version=version,
        calibration_id=calibration_id,
        robot_id=robot_id,
        validated=validated,
        validated_utc=validated_utc,
        urdf_sha256=urdf_sha256,
        ready_pose_rad=ready_pose,
        ready_tolerance_rad=ready_tolerance,
        tip_frame=tip_frame,
        tool0_T_tcp=tool0_T_tcp,
        evidence_session=evidence,
        source_path=str(source.resolve()),
        file_hash=_sha256_file(source),
    )
