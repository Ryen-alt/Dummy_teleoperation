from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from ..calibration.geometry import matrix_to_quaternion_xyzw, project_rotation


class KinematicsError(RuntimeError):
    pass


def _finite_json_float(value: float) -> float | None:
    return float(value) if np.isfinite(value) else None


@dataclass(frozen=True)
class CartesianPose:
    """A framed Cartesian pose expressed in SI units."""

    position_m: np.ndarray
    rotation: np.ndarray
    base_frame: str = "base_link"
    tip_frame: str = "tool0"

    def __post_init__(self) -> None:
        position = np.asarray(self.position_m, dtype=np.float64)
        rotation = np.asarray(self.rotation, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise KinematicsError("Cartesian position must be a finite 3-vector in metres")
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            raise KinematicsError("Cartesian rotation must be a finite 3x3 matrix")
        if not self.base_frame or not self.tip_frame:
            raise KinematicsError("Cartesian base/tip frames must be non-empty")
        projected = project_rotation(rotation)
        if not np.allclose(rotation, projected, atol=1e-6):
            raise KinematicsError("Cartesian rotation must be orthonormal")
        position = position.copy()
        projected = projected.copy()
        position.setflags(write=False)
        projected.setflags(write=False)
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "rotation", projected)

    @property
    def quaternion_xyzw(self) -> np.ndarray:
        return matrix_to_quaternion_xyzw(self.rotation)

    def as_dict(self) -> dict[str, object]:
        return {
            "frame": self.base_frame,
            "tip": self.tip_frame,
            "position_m": self.position_m.tolist(),
            "quaternion_xyzw": self.quaternion_xyzw.tolist(),
        }


@dataclass(frozen=True)
class IKResult:
    success: bool
    joint_position_rad: np.ndarray | None
    branch_id: str | None
    position_error_m: float
    orientation_error_rad: float
    joint_limit_margin_rad: np.ndarray | None
    minimum_singular_value: float
    singularity_flags: tuple[str, ...]
    clipped: bool
    reasons: tuple[str, ...]
    iterations: int
    failure_reason: str | None
    solver: str
    solver_version: str
    model_hash: str
    solve_duration_ns: int = 0
    timed_out: bool = False
    timeout_stage: str | None = None

    def __post_init__(self) -> None:
        if self.joint_position_rad is not None:
            joints = np.asarray(self.joint_position_rad)
            if joints.dtype != np.float32 or joints.shape != (6,):
                raise KinematicsError("IK joint result must be float32[6]")
            if not np.isfinite(joints).all():
                raise KinematicsError("IK joint result contains NaN or Inf")
            joints = joints.copy()
            joints.setflags(write=False)
            object.__setattr__(self, "joint_position_rad", joints)
        if self.success != (self.joint_position_rad is not None):
            raise KinematicsError("successful IK must have joints and failed IK must not")
        if self.joint_limit_margin_rad is not None:
            margin = np.asarray(self.joint_limit_margin_rad, dtype=np.float64)
            if margin.shape != (6,) or not np.isfinite(margin).all():
                raise KinematicsError("IK joint-limit margin must be a finite 6-vector")
            margin = margin.copy()
            margin.setflags(write=False)
            object.__setattr__(self, "joint_limit_margin_rad", margin)
        if self.success != (self.joint_limit_margin_rad is not None):
            raise KinematicsError("successful IK must include per-joint limit margins")
        if self.solve_duration_ns < 0:
            raise KinematicsError("IK solve duration must be non-negative")
        if self.timed_out != (self.timeout_stage is not None):
            raise KinematicsError("timed-out IK must identify its timeout stage")
        if self.timed_out and self.success:
            raise KinematicsError("timed-out IK cannot be successful")

    @property
    def minimum_joint_margin_rad(self) -> float:
        if self.joint_limit_margin_rad is None:
            return float("nan")
        return float(np.min(self.joint_limit_margin_rad))

    def as_dict(self) -> dict[str, object]:
        return {
            "success": self.success,
            "joint_position_rad": (
                None if self.joint_position_rad is None else self.joint_position_rad.tolist()
            ),
            "branch_id": self.branch_id,
            "position_residual_m": _finite_json_float(self.position_error_m),
            "orientation_residual_rad": _finite_json_float(
                self.orientation_error_rad
            ),
            "joint_limit_margin_rad": (
                None
                if self.joint_limit_margin_rad is None
                else self.joint_limit_margin_rad.tolist()
            ),
            "minimum_joint_margin_rad": _finite_json_float(
                self.minimum_joint_margin_rad
            ),
            "minimum_singular_value": _finite_json_float(
                self.minimum_singular_value
            ),
            "singularity_flags": list(self.singularity_flags),
            "clipped": self.clipped,
            "reasons": list(self.reasons),
            "iterations": self.iterations,
            "failure_reason": self.failure_reason,
            "solver": self.solver,
            "solver_version": self.solver_version,
            "model_hash": self.model_hash,
            "solve_duration_ns": self.solve_duration_ns,
            "timed_out": self.timed_out,
            "timeout_stage": self.timeout_stage,
        }


class KinematicsBackend(Protocol):
    base_link: str
    tip_link: str
    model_hash: str

    def forward(self, joint_position_rad: np.ndarray) -> CartesianPose: ...

    def inverse(
        self,
        target: CartesianPose,
        measured_joint_rad: np.ndarray,
        previous_joint_rad: np.ndarray | None = None,
        *,
        hard_budget_ns: int | None = None,
    ) -> IKResult: ...

    def describe(self) -> dict[str, object]: ...
