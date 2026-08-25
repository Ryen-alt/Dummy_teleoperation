from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path
from typing import Callable

import numpy as np

from ..calibration.geometry import rotation_vector, validate_transform
from ..calibration.urdf import UrdfError, UrdfKinematics
from .contracts import CartesianPose, IKResult, KinematicsError


class _SolveBudgetExceeded(RuntimeError):
    def __init__(self, stage: str) -> None:
        super().__init__(stage)
        self.stage = stage
        self.position_error = float("inf")
        self.orientation_error = float("inf")
        self.minimum_singular_value = float("nan")
        self.iterations = 0


class DummyUrdfKinematics:
    """Deterministic local DLS IK over the repository's canonical Dummy URDF.

    This backend deliberately returns only one continuous local solution.  It
    seeds from the preceding accepted target and measured joints, then chooses
    the converged candidate nearest the measured state.  Absolute targets still
    pass through ``ActionGateway`` after this solver.
    """

    SOLVER = "dummy_urdf_damped_least_squares"
    SOLVER_VERSION = "1"

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        joint_min_rad: np.ndarray,
        joint_max_rad: np.ndarray,
        joint_limit_margin_rad: float,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        max_iterations: int,
        damping: float,
        finite_difference_rad: float,
        max_solver_step_rad: float,
        max_solution_step_rad: float,
        translation_scale_m: float,
        tool0_T_tip: np.ndarray | None = None,
        tip_frame: str = "tool0",
        calibration_hash: str | None = None,
        clock_ns: Callable[[], int] = time.perf_counter_ns,
    ) -> None:
        self.urdf_path = Path(urdf_path)
        self._urdf = UrdfKinematics(self.urdf_path, base_link="base_link", tip_link="tool0")
        self.base_link = self._urdf.base_link
        if not tip_frame:
            raise KinematicsError("tip_frame must be non-empty")
        self.tip_link = tip_frame
        self._clock_ns = clock_ns
        transform = np.eye(4, dtype=np.float64) if tool0_T_tip is None else tool0_T_tip
        self.tool0_T_tip = validate_transform(transform, name="tool0_T_tip").copy()
        self.tool0_T_tip.setflags(write=False)
        try:
            self.urdf_hash = hashlib.sha256(self.urdf_path.read_bytes()).hexdigest()
        except OSError as exc:
            raise KinematicsError(f"cannot hash URDF {self.urdf_path}: {exc}") from exc
        identity = np.allclose(self.tool0_T_tip, np.eye(4), atol=1e-12, rtol=0.0)
        if calibration_hash is not None:
            try:
                if len(bytes.fromhex(calibration_hash)) != 32:
                    raise ValueError
            except ValueError as exc:
                raise KinematicsError("calibration_hash must be a SHA-256 hex digest") from exc
        model_identity = {
            "urdf_sha256": self.urdf_hash,
            "tip_frame": self.tip_link,
            "tool0_T_tip": self.tool0_T_tip.tolist(),
            "calibration_sha256": calibration_hash,
        }
        self.model_hash = hashlib.sha256(
            json.dumps(model_identity, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        self.calibration_hash = calibration_hash
        self._identity_tip = identity

        self.joint_min_rad = self._validated_joints(joint_min_rad, "joint_min_rad")
        self.joint_max_rad = self._validated_joints(joint_max_rad, "joint_max_rad")
        if np.any(self.joint_min_rad >= self.joint_max_rad):
            raise KinematicsError("joint minimums must be below maximums")
        if any(
            joint.lower is None or joint.upper is None
            for joint in self._urdf.movable_joints
        ):
            raise KinematicsError("every Cartesian-chain URDF joint needs finite limits")
        urdf_min = np.asarray(
            [joint.lower for joint in self._urdf.movable_joints], dtype=np.float64
        )
        urdf_max = np.asarray(
            [joint.upper for joint in self._urdf.movable_joints], dtype=np.float64
        )
        if (
            urdf_min.shape != (6,)
            or urdf_max.shape != (6,)
            or not np.isfinite(urdf_min).all()
            or not np.isfinite(urdf_max).all()
            or not np.allclose(self.joint_min_rad, urdf_min, atol=1e-6, rtol=0.0)
            or not np.allclose(self.joint_max_rad, urdf_max, atol=1e-6, rtol=0.0)
        ):
            raise KinematicsError("robot configuration joint limits do not match the URDF")
        if not math.isfinite(joint_limit_margin_rad) or joint_limit_margin_rad < 0:
            raise KinematicsError("joint_limit_margin_rad must be finite and non-negative")
        self.lower = self.joint_min_rad + joint_limit_margin_rad
        self.upper = self.joint_max_rad - joint_limit_margin_rad
        if np.any(self.lower >= self.upper):
            raise KinematicsError("joint limit margin leaves an empty range")
        if max_iterations <= 0:
            raise KinematicsError("max_iterations must be positive")
        for name, value in (
            ("position_tolerance_m", position_tolerance_m),
            ("orientation_tolerance_rad", orientation_tolerance_rad),
            ("damping", damping),
            ("finite_difference_rad", finite_difference_rad),
            ("max_solver_step_rad", max_solver_step_rad),
            ("max_solution_step_rad", max_solution_step_rad),
            ("translation_scale_m", translation_scale_m),
        ):
            if not math.isfinite(value) or value <= 0:
                raise KinematicsError(f"{name} must be positive and finite")
        self.position_tolerance_m = float(position_tolerance_m)
        self.orientation_tolerance_rad = float(orientation_tolerance_rad)
        self.max_iterations = int(max_iterations)
        self.damping = float(damping)
        self.finite_difference_rad = float(finite_difference_rad)
        self.max_solver_step_rad = float(max_solver_step_rad)
        self.max_solution_step_rad = float(max_solution_step_rad)
        self.translation_scale_m = float(translation_scale_m)

    @staticmethod
    def _validated_joints(value: np.ndarray, name: str) -> np.ndarray:
        result = np.asarray(value, dtype=np.float64)
        if result.shape != (6,) or not np.isfinite(result).all():
            raise KinematicsError(f"{name} must be a finite 6-vector")
        return result.copy()

    def forward(self, joint_position_rad: np.ndarray) -> CartesianPose:
        joints = self._validated_joints(joint_position_rad, "joint_position_rad")
        try:
            transform = self._urdf.base_T_tool0(joints, check_limits=False) @ self.tool0_T_tip
        except UrdfError as exc:
            raise KinematicsError(str(exc)) from exc
        return CartesianPose(
            transform[:3, 3],
            transform[:3, :3],
            base_frame=self.base_link,
            tip_frame=self.tip_link,
        )

    @staticmethod
    def _pose_error(
        target: CartesianPose,
        actual: CartesianPose,
    ) -> tuple[np.ndarray, float, float]:
        translation = target.position_m - actual.position_m
        angular = rotation_vector(target.rotation @ actual.rotation.T)
        return translation, float(np.linalg.norm(translation)), float(np.linalg.norm(angular))

    def _check_budget(self, deadline_ns: int | None, stage: str) -> None:
        if deadline_ns is not None and self._clock_ns() > deadline_ns:
            raise _SolveBudgetExceeded(stage)

    def _jacobian(
        self,
        joints: np.ndarray,
        pose: CartesianPose,
        deadline_ns: int | None,
    ) -> np.ndarray:
        jacobian = np.empty((6, 6), dtype=np.float64)
        for index in range(6):
            self._check_budget(deadline_ns, f"jacobian_column_{index}")
            perturbed = joints.copy()
            delta = self.finite_difference_rad
            if perturbed[index] + delta > self.upper[index]:
                delta = -delta
            perturbed[index] += delta
            sample = self.forward(perturbed)
            jacobian[:3, index] = (sample.position_m - pose.position_m) / delta
            jacobian[3:, index] = rotation_vector(
                sample.rotation @ pose.rotation.T
            ) / delta
        return jacobian

    def _solve_seed(
        self,
        target: CartesianPose,
        seed: np.ndarray,
        deadline_ns: int | None,
    ) -> tuple[np.ndarray | None, float, float, float, int]:
        joints = np.clip(seed, self.lower, self.upper)
        minimum_singular = float("nan")
        position_error = float("inf")
        orientation_error = float("inf")
        iteration = 0
        try:
            for iteration in range(self.max_iterations + 1):
                self._check_budget(deadline_ns, f"iteration_{iteration}_fk")
                pose = self.forward(joints)
                translation, position_error, orientation_error = self._pose_error(
                    target, pose
                )
                if (
                    position_error <= self.position_tolerance_m
                    and orientation_error <= self.orientation_tolerance_rad
                ):
                    jacobian = self._jacobian(joints, pose, deadline_ns)
                    minimum_singular = float(
                        np.linalg.svd(jacobian, compute_uv=False)[-1]
                    )
                    return (
                        joints,
                        position_error,
                        orientation_error,
                        minimum_singular,
                        iteration,
                    )
                if iteration == self.max_iterations:
                    break
                jacobian = self._jacobian(joints, pose, deadline_ns)
                minimum_singular = float(np.linalg.svd(jacobian, compute_uv=False)[-1])
                weighted_jacobian = jacobian.copy()
                weighted_jacobian[:3] /= self.translation_scale_m
                error = np.concatenate(
                    (
                        translation / self.translation_scale_m,
                        rotation_vector(target.rotation @ pose.rotation.T),
                    )
                )
                normal = weighted_jacobian @ weighted_jacobian.T
                normal += np.eye(6, dtype=np.float64) * self.damping**2
                try:
                    step = weighted_jacobian.T @ np.linalg.solve(normal, error)
                except np.linalg.LinAlgError:
                    return (
                        None,
                        position_error,
                        orientation_error,
                        minimum_singular,
                        iteration,
                    )
                step_norm = float(np.linalg.norm(step))
                if not math.isfinite(step_norm):
                    return (
                        None,
                        position_error,
                        orientation_error,
                        minimum_singular,
                        iteration,
                    )
                if step_norm > self.max_solver_step_rad:
                    step *= self.max_solver_step_rad / step_norm
                updated = np.clip(joints + step, self.lower, self.upper)
                if np.allclose(updated, joints, atol=1e-12):
                    return (
                        None,
                        position_error,
                        orientation_error,
                        minimum_singular,
                        iteration,
                    )
                joints = updated
        except _SolveBudgetExceeded as exc:
            exc.position_error = position_error
            exc.orientation_error = orientation_error
            exc.minimum_singular_value = minimum_singular
            exc.iterations = iteration
            raise
        return None, position_error, orientation_error, minimum_singular, self.max_iterations

    def inverse(
        self,
        target: CartesianPose,
        measured_joint_rad: np.ndarray,
        previous_joint_rad: np.ndarray | None = None,
        *,
        hard_budget_ns: int | None = None,
    ) -> IKResult:
        started_ns = self._clock_ns()
        if hard_budget_ns is not None and hard_budget_ns <= 0:
            raise KinematicsError("hard_budget_ns must be positive")
        deadline_ns = None if hard_budget_ns is None else started_ns + hard_budget_ns
        if target.base_frame != self.base_link or target.tip_frame != self.tip_link:
            raise KinematicsError(
                f"target frames must be {self.base_link}->{self.tip_link}, received "
                f"{target.base_frame}->{target.tip_frame}"
            )
        measured = self._validated_joints(measured_joint_rad, "measured_joint_rad")
        seeds: list[tuple[str, np.ndarray]] = []
        previous: np.ndarray | None = None
        if previous_joint_rad is not None:
            previous = self._validated_joints(previous_joint_rad, "previous_joint_rad")
            seeds.append(("previous", previous))
        seeds.append(("measured", measured))

        candidates: list[tuple[float, str, np.ndarray, float, float, float, int]] = []
        best_failure = (float("inf"), float("inf"), float("nan"), 0)
        continuity_rejected = False
        seen: list[np.ndarray] = []
        try:
            for branch, seed in seeds:
                self._check_budget(deadline_ns, f"seed_{branch}")
                if any(np.allclose(seed, item, atol=1e-10) for item in seen):
                    continue
                seen.append(seed)
                solved, position_error, orientation_error, singular, iterations = self._solve_seed(
                    target, seed, deadline_ns
                )
                if position_error + orientation_error < best_failure[0] + best_failure[1]:
                    best_failure = (position_error, orientation_error, singular, iterations)
                if solved is not None:
                    continuity_reference = previous if previous is not None else measured
                    if (
                        float(np.max(np.abs(solved - continuity_reference)))
                        > self.max_solution_step_rad
                    ):
                        continuity_rejected = True
                        continue
                    measured_cost = float(np.linalg.norm(solved - measured))
                    continuity_cost = (
                        0.0
                        if previous is None
                        else float(np.linalg.norm(solved - previous))
                    )
                    cost = measured_cost + 0.25 * continuity_cost
                    candidates.append(
                        (cost, branch, solved, position_error, orientation_error, singular, iterations)
                    )
            self._check_budget(deadline_ns, "finalize")
        except _SolveBudgetExceeded as exc:
            position_error, orientation_error, singular, iterations = best_failure
            if (
                exc.position_error + exc.orientation_error
                < position_error + orientation_error
            ):
                position_error = exc.position_error
                orientation_error = exc.orientation_error
                singular = exc.minimum_singular_value
                iterations = exc.iterations
            return IKResult(
                success=False,
                joint_position_rad=None,
                branch_id=None,
                position_error_m=position_error,
                orientation_error_rad=orientation_error,
                joint_limit_margin_rad=None,
                minimum_singular_value=singular,
                singularity_flags=(),
                clipped=False,
                reasons=("solve_budget_exceeded",),
                iterations=iterations,
                failure_reason="solve_budget_exceeded",
                solver=self.SOLVER,
                solver_version=self.SOLVER_VERSION,
                model_hash=self.model_hash,
                solve_duration_ns=max(0, self._clock_ns() - started_ns),
                timed_out=True,
                timeout_stage=exc.stage,
            )

        if not candidates:
            position_error, orientation_error, singular, iterations = best_failure
            failure_reason = (
                "continuity_step_exceeded"
                if continuity_rejected
                else "no_converged_solution"
            )
            return IKResult(
                success=False,
                joint_position_rad=None,
                branch_id=None,
                position_error_m=position_error,
                orientation_error_rad=orientation_error,
                joint_limit_margin_rad=None,
                minimum_singular_value=singular,
                singularity_flags=(
                    ("low_numeric_jacobian_rank",)
                    if math.isfinite(singular) and singular < 1e-5
                    else ()
                ),
                clipped=False,
                reasons=(failure_reason,),
                iterations=iterations,
                failure_reason=failure_reason,
                solver=self.SOLVER,
                solver_version=self.SOLVER_VERSION,
                model_hash=self.model_hash,
                solve_duration_ns=max(0, self._clock_ns() - started_ns),
            )

        _, branch, solved, position_error, orientation_error, singular, iterations = min(
            candidates, key=lambda item: item[0]
        )
        margin = np.minimum(solved - self.joint_min_rad, self.joint_max_rad - solved)
        return IKResult(
            success=True,
            joint_position_rad=solved.astype(np.float32),
            branch_id=branch,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            joint_limit_margin_rad=margin,
            minimum_singular_value=singular,
            singularity_flags=(
                ("low_numeric_jacobian_rank",)
                if math.isfinite(singular) and singular < 1e-5
                else ()
            ),
            clipped=False,
            reasons=(),
            iterations=iterations,
            failure_reason=None,
            solver=self.SOLVER,
            solver_version=self.SOLVER_VERSION,
            model_hash=self.model_hash,
            solve_duration_ns=max(0, self._clock_ns() - started_ns),
        )

    def describe(self) -> dict[str, object]:
        return {
            **self._urdf.describe(),
            "urdf_tip_link": self._urdf.tip_link,
            "tip_link": self.tip_link,
            "urdf_hash": self.urdf_hash,
            "model_hash": self.model_hash,
            "cartesian_calibration_hash": self.calibration_hash,
            "tool0_T_tip": self.tool0_T_tip.tolist(),
            "identity_tip": self._identity_tip,
            "solver": self.SOLVER,
            "solver_version": self.SOLVER_VERSION,
            "joint_limit_margin_rad": (self.lower - self.joint_min_rad).tolist(),
            "position_tolerance_m": self.position_tolerance_m,
            "orientation_tolerance_rad": self.orientation_tolerance_rad,
            "max_solution_step_rad": self.max_solution_step_rad,
        }
