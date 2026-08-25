from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np

from .calibration.geometry import axis_angle_rotation, rotation_vector
from .domain import AppliedAction
from .gamepad import GamepadState
from .kinematics.contracts import CartesianPose, IKResult, KinematicsBackend
from .schema import RobotConfig, RobotState
from .teleop import (
    TeleopCommand,
    TeleopError,
    TeleopProfile,
    _episode_edge,
    integration_substeps,
    shape_axis,
    validate_profile_for_robot,
)


class CartesianTeleopError(TeleopError):
    def __init__(self, message: str, *, metadata: Mapping[str, object]) -> None:
        super().__init__(message)
        self.metadata = dict(metadata)


@dataclass(frozen=True)
class CartesianProposal:
    proposal_id: str
    revision: int
    source_sweep_id: int
    generated_at_ns: int
    dt_s: float
    action: np.ndarray
    candidate_pose: CartesianPose
    candidate_velocity: np.ndarray
    ik_result: IKResult
    metadata: Mapping[str, object]

    def __post_init__(self) -> None:
        if not self.proposal_id or self.revision < 0 or self.source_sweep_id <= 0:
            raise CartesianTeleopError(
                "Cartesian proposal identity is invalid",
                metadata={"stage": "result_validation"},
            )
        if self.generated_at_ns < 0 or not np.isfinite(self.dt_s) or self.dt_s <= 0:
            raise CartesianTeleopError(
                "Cartesian proposal timing is invalid",
                metadata={"stage": "result_validation"},
            )
        action = np.asarray(self.action)
        if action.dtype != np.float32 or action.shape != (7,) or not np.isfinite(action).all():
            raise CartesianTeleopError(
                "Cartesian proposal must contain finite float32[7]",
                metadata={"stage": "result_validation"},
            )
        velocity = np.asarray(self.candidate_velocity, dtype=np.float64)
        if velocity.shape != (6,) or not np.isfinite(velocity).all():
            raise CartesianTeleopError(
                "Cartesian proposal velocity must be a finite 6-vector",
                metadata={"stage": "result_validation"},
            )
        action = action.copy()
        action.setflags(write=False)
        velocity = velocity.copy()
        velocity.setflags(write=False)
        object.__setattr__(self, "action", action)
        object.__setattr__(self, "candidate_velocity", velocity)
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class CartesianCommit:
    proposal_id: str
    action_sequence: int
    applied_pose: CartesianPose
    arm_clipped: bool
    reanchored: bool
    reanchor_reason: str | None
    realized_velocity: np.ndarray
    candidate_to_applied_position_error_m: float
    candidate_to_applied_orientation_error_rad: float

    def __post_init__(self) -> None:
        velocity = np.asarray(self.realized_velocity, dtype=np.float64)
        if velocity.shape != (6,) or not np.isfinite(velocity).all():
            raise CartesianTeleopError(
                "Cartesian committed velocity must be a finite 6-vector",
                metadata={"stage": "commit_validation"},
            )
        if (
            not self.proposal_id
            or self.action_sequence <= 0
            or self.candidate_to_applied_position_error_m < 0
            or self.candidate_to_applied_orientation_error_rad < 0
            or self.reanchored != (self.reanchor_reason is not None)
        ):
            raise CartesianTeleopError(
                "Cartesian commit metadata is invalid",
                metadata={"stage": "commit_validation"},
            )
        velocity = velocity.copy()
        velocity.setflags(write=False)
        object.__setattr__(self, "realized_velocity", velocity)

    def as_dict(self) -> dict[str, object]:
        return {
            "proposal_id": self.proposal_id,
            "action_sequence": self.action_sequence,
            "applied_pose": self.applied_pose.as_dict(),
            "arm_clipped": self.arm_clipped,
            "reanchored": self.reanchored,
            "reanchor_reason": self.reanchor_reason,
            "commit_strategy": "applied_fk_reanchor" if self.reanchored else "candidate_pose",
            "realized_velocity": self.realized_velocity.tolist(),
            "candidate_to_applied_position_error_m": (
                self.candidate_to_applied_position_error_m
            ),
            "candidate_to_applied_orientation_error_rad": (
                self.candidate_to_applied_orientation_error_rad
            ),
        }


class CartesianGamepadMapper:
    """Map a conventional Xbox-style controller to a base-frame twist.

    Orientation is commanded as angular velocity from ordinary axes/buttons;
    no IMU or gyroscope is required.
    """

    def __init__(self, profile: TeleopProfile) -> None:
        if profile.cartesian is None:
            raise TeleopError("teleop profile does not define a Cartesian mapping")
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
        cartesian = self.profile.cartesian
        assert cartesian is not None
        mapping = self.profile.gamepad
        normalized = np.zeros(6, dtype=np.float64)
        shaped_axes: dict[str, float] = {}
        for binding in cartesian.axes:
            default = -1.0 if binding.unipolar else 0.0
            raw_value = float(axes.get(binding.axis, default))
            if binding.unipolar:
                raw_value = (max(-1.0, min(1.0, raw_value)) + 1.0) * 0.5
            shaped = shape_axis(raw_value, mapping.deadzone, mapping.response_exponent)
            if binding.invert:
                shaped = -shaped
            shaped_axes[binding.axis] = shaped
            normalized[binding.component] += shaped
        normalized = np.clip(normalized, -1.0, 1.0)
        twist = (normalized * cartesian.speed).astype(np.float32)
        gripper_direction = int(mapping.gripper_close in pressed) - int(
            mapping.gripper_open in pressed
        )
        episode = _episode_edge(pressed, self._previous_pressed, mapping.episode_buttons)
        self._previous_pressed = set(pressed)
        return TeleopCommand(
            monotonic_ns=now_ns,
            source="gamepad",
            joint_velocity_rad_s=np.zeros(6, dtype=np.float32),
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
                "normalized_twist": normalized.tolist(),
                "pressed": sorted(pressed),
                "transport": {} if transport_raw is None else dict(transport_raw),
            },
            teleop_mode="cartesian",
            cartesian_twist=twist,
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


class CartesianPoseIntegrator:
    def __init__(
        self,
        profile: TeleopProfile,
        config: RobotConfig,
        kinematics: KinematicsBackend,
    ) -> None:
        validate_profile_for_robot(profile, config)
        if profile.cartesian is None:
            raise TeleopError("teleop profile does not define Cartesian controls")
        if kinematics.base_link != "base_link" or not kinematics.tip_link:
            raise TeleopError("Cartesian kinematics must use base_link and a named tip frame")
        self.profile = profile
        self.cartesian = profile.cartesian
        self.config = config
        self.kinematics = kinematics
        self._target_pose: CartesianPose | None = None
        self._commanded_pose: CartesianPose | None = None
        self._previous_joint: np.ndarray | None = None
        self._velocity = np.zeros(6, dtype=np.float64)
        self._gripper_target: float | None = None
        self._last_time_ns: int | None = None
        self._last_sweep_id: int | None = None
        self._revision = 0

    def reset(self, state: RobotState | None = None, now_ns: int | None = None) -> None:
        self._velocity.fill(0.0)
        self._target_pose = None
        self._commanded_pose = None
        self._previous_joint = None
        self._gripper_target = None
        self._last_time_ns = now_ns
        self._last_sweep_id = None
        self._revision += 1
        if state is None:
            return
        self._validate_state(state)
        joints = state.position[:6].astype(np.float64)
        self._target_pose = self.kinematics.forward(joints)
        self._commanded_pose = self._target_pose
        self._previous_joint = joints
        self._gripper_target = float(state.position[6])
        if state.coherent:
            self._last_sweep_id = state.coherent_sweep_id

    @property
    def revision(self) -> int:
        return self._revision

    @property
    def last_sweep_id(self) -> int | None:
        return self._last_sweep_id

    @property
    def target_pose(self) -> CartesianPose | None:
        return self._target_pose

    @property
    def commanded_pose(self) -> CartesianPose | None:
        return self._commanded_pose

    def has_fresh_coherent_sweep(self, state: RobotState) -> bool:
        return state.coherent and state.coherent_sweep_id != self._last_sweep_id

    def advance_without_motion(self, now_ns: int) -> None:
        if now_ns < 0 or (self._last_time_ns is not None and now_ns < self._last_time_ns):
            raise CartesianTeleopError(
                "cannot move Cartesian timing anchor backwards",
                metadata={"stage": "timing"},
            )
        self._last_time_ns = now_ns
        self._revision += 1

    @staticmethod
    def _validate_state(state: RobotState) -> None:
        if not state.position_valid or state.position.shape != (7,):
            raise TeleopError("valid float32[7] robot position is required")
        if not state.gripper_valid:
            raise TeleopError("valid gripper feedback is required before teleoperation")

    def _periods(self, now_ns: int) -> tuple[float, ...]:
        nominal = 1.0 / self.config.control_rate_hz
        if self._last_time_ns is None:
            return (nominal,)
        measured = (now_ns - self._last_time_ns) / 1e9
        try:
            return integration_substeps(measured, nominal)
        except TeleopError as exc:
            raise CartesianTeleopError(
                str(exc),
                metadata={"stage": "timing", "measured_dt_s": measured},
            ) from exc

    def _check_workspace(self, pose: CartesianPose) -> str | None:
        position = pose.position_m
        if np.any(position < self.cartesian.workspace_min_m) or np.any(
            position > self.cartesian.workspace_max_m
        ):
            return "target_outside_workspace_box"
        if float(np.linalg.norm(position[:2])) < self.cartesian.min_base_radius_m:
            return "target_inside_base_exclusion_radius"
        return None

    def propose(
        self,
        command: TeleopCommand,
        state: RobotState,
        now_ns: int,
    ) -> CartesianProposal:
        if command.teleop_mode != "cartesian":
            raise CartesianTeleopError(
                "Cartesian integrator received a joint-mode command",
                metadata={"stage": "command_validation"},
            )
        if (
            not command.connected
            or not command.deadman
            or command.hold_requested
            or command.estop_requested
        ):
            raise CartesianTeleopError(
                "cannot integrate a Cartesian command without an active dead-man",
                metadata={"stage": "command_validation"},
            )
        age_ms = (now_ns - command.monotonic_ns) / 1e6
        if age_ms < 0 or age_ms > self.profile.input_timeout_ms:
            raise CartesianTeleopError(
                f"input command is stale ({age_ms:.1f} ms)",
                metadata={"stage": "timing", "input_age_ms": age_ms},
            )
        self._validate_state(state)
        if not state.coherent:
            raise CartesianTeleopError(
                "Cartesian proposal requires a coherent feedback sweep",
                metadata={"stage": "feedback_coherence"},
            )
        if state.coherent_sweep_id == self._last_sweep_id:
            raise CartesianTeleopError(
                "Cartesian proposal requires a new coherent feedback sweep",
                metadata={
                    "stage": "feedback_coherence",
                    "coherent_sweep_id": state.coherent_sweep_id,
                },
            )
        if (
            self._target_pose is None
            or self._commanded_pose is None
            or self._gripper_target is None
        ):
            raise CartesianTeleopError(
                "Cartesian integrator must be reset from coherent feedback before proposing",
                metadata={"stage": "lifecycle"},
            )
        assert self._target_pose is not None
        assert self._commanded_pose is not None
        assert self._gripper_target is not None

        periods = self._periods(now_ns)
        desired = np.clip(
            command.cartesian_twist.astype(np.float64),
            -self.cartesian.speed,
            self.cartesian.speed,
        )
        position = self._target_pose.position_m.copy()
        rotation = self._target_pose.rotation.copy()
        velocity = self._velocity.copy()
        for dt in periods:
            max_delta = self.cartesian.acceleration * dt
            velocity = np.clip(desired, velocity - max_delta, velocity + max_delta)
            position = position + velocity[:3] * dt
            angular_step = velocity[3:] * dt
            angle = float(np.linalg.norm(angular_step))
            if angle > 1e-12:
                rotation = axis_angle_rotation(angular_step / angle, angle) @ rotation
        candidate_pose = CartesianPose(
            position,
            rotation,
            base_frame=self.kinematics.base_link,
            tip_frame=self.kinematics.tip_link,
        )
        measured_pose = self.kinematics.forward(state.position[:6])
        base_metadata: dict[str, object] = {
            "teleop_semantics_version": 1,
            "control_frame": self.kinematics.base_link,
            "tip_frame": self.kinematics.tip_link,
            "source_sweep_id": state.coherent_sweep_id,
            "integrator_revision": self._revision,
            "dt_s": float(sum(periods)),
            "integration_substeps_s": list(periods),
            "requested_twist": command.cartesian_twist.tolist(),
            "limited_twist": velocity.tolist(),
            "candidate_velocity": velocity.tolist(),
            "target_pose": candidate_pose.as_dict(),
            "measured_joint_rad": state.position[:6].tolist(),
            "measured_pose": measured_pose.as_dict(),
            "workspace_validation": {"success": True, "reason": None},
        }
        workspace_error = self._check_workspace(candidate_pose)
        if workspace_error is not None:
            base_metadata["failure_reason"] = workspace_error
            base_metadata["stage"] = "workspace"
            base_metadata["workspace_validation"] = {
                "success": False,
                "reason": workspace_error,
            }
            raise CartesianTeleopError(workspace_error, metadata=base_metadata)

        result = self.kinematics.inverse(
            candidate_pose,
            state.position[:6],
            self._previous_joint,
            hard_budget_ns=int(self.cartesian.hard_budget_ms * 1_000_000),
        )
        base_metadata["ik"] = result.as_dict()
        base_metadata["solve_budget"] = {
            "soft_budget_ns": int(self.cartesian.soft_budget_ms * 1_000_000),
            "hard_budget_ns": int(self.cartesian.hard_budget_ms * 1_000_000),
            "soft_budget_exceeded": (
                result.solve_duration_ns > int(self.cartesian.soft_budget_ms * 1_000_000)
            ),
            "hard_budget_exceeded": result.timed_out,
            "timeout_stage": result.timeout_stage,
        }
        if not result.success or result.joint_position_rad is None:
            base_metadata["stage"] = "inverse_kinematics"
            raise CartesianTeleopError(
                f"Cartesian IK failed: {result.failure_reason or 'invalid solution'}",
                metadata=base_metadata,
            )

        gripper_velocity = float(
            np.clip(
                command.gripper_velocity_per_s,
                -self.profile.gripper_speed_per_s,
                self.profile.gripper_speed_per_s,
            )
        )
        gripper_target = float(
            np.clip(
                self._gripper_target + gripper_velocity * float(sum(periods)),
                self.config.gripper_range[0],
                self.config.gripper_range[1],
            )
        )
        action = np.concatenate(
            (result.joint_position_rad, np.asarray([gripper_target], dtype=np.float32))
        ).astype(np.float32)
        proposal_id = f"cartesian:{self._revision}:{state.coherent_sweep_id}:{now_ns}"
        base_metadata["proposal_id"] = proposal_id
        return CartesianProposal(
            proposal_id=proposal_id,
            revision=self._revision,
            source_sweep_id=state.coherent_sweep_id,
            generated_at_ns=now_ns,
            dt_s=float(sum(periods)),
            action=action,
            candidate_pose=candidate_pose,
            candidate_velocity=velocity,
            ik_result=result,
            metadata=base_metadata,
        )

    def commit(
        self,
        proposal: CartesianProposal,
        action: AppliedAction,
    ) -> CartesianCommit:
        if proposal.revision != self._revision:
            raise CartesianTeleopError(
                "Cartesian proposal revision is stale",
                metadata={"stage": "commit_validation", "proposal_id": proposal.proposal_id},
            )
        if proposal.source_sweep_id == self._last_sweep_id:
            raise CartesianTeleopError(
                "Cartesian proposal sweep was already committed",
                metadata={"stage": "commit_validation", "proposal_id": proposal.proposal_id},
            )
        if not np.array_equal(action.requested, proposal.action):
            raise CartesianTeleopError(
                "AppliedAction.requested does not match Cartesian proposal",
                metadata={"stage": "commit_validation", "proposal_id": proposal.proposal_id},
            )
        if self._commanded_pose is None:
            raise CartesianTeleopError(
                "Cartesian integrator has no commanded pose to commit from",
                metadata={"stage": "commit_validation", "proposal_id": proposal.proposal_id},
            )

        applied_pose = self.kinematics.forward(action.applied[:6])
        arm_clipped = not np.allclose(
            action.requested[:6], action.applied[:6], rtol=0.0, atol=1e-7
        )
        position_error = float(
            np.linalg.norm(proposal.candidate_pose.position_m - applied_pose.position_m)
        )
        orientation_error = float(
            np.linalg.norm(
                rotation_vector(
                    proposal.candidate_pose.rotation @ applied_pose.rotation.T
                )
            )
        )
        if arm_clipped:
            linear = (
                applied_pose.position_m - self._commanded_pose.position_m
            ) / proposal.dt_s
            angular = rotation_vector(
                applied_pose.rotation @ self._commanded_pose.rotation.T
            ) / proposal.dt_s
            realized_velocity = np.clip(
                np.concatenate((linear, angular)),
                -self.cartesian.speed,
                self.cartesian.speed,
            )
            committed_target = applied_pose
        else:
            realized_velocity = proposal.candidate_velocity.copy()
            committed_target = proposal.candidate_pose

        self._target_pose = committed_target
        self._commanded_pose = applied_pose
        self._previous_joint = action.applied[:6].astype(np.float64)
        self._velocity = realized_velocity.astype(np.float64, copy=True)
        self._gripper_target = float(action.applied[6])
        self._last_time_ns = proposal.generated_at_ns
        self._last_sweep_id = proposal.source_sweep_id
        self._revision += 1
        return CartesianCommit(
            proposal_id=proposal.proposal_id,
            action_sequence=action.sequence,
            applied_pose=applied_pose,
            arm_clipped=arm_clipped,
            reanchored=arm_clipped,
            reanchor_reason="arm_action_clipped" if arm_clipped else None,
            realized_velocity=realized_velocity,
            candidate_to_applied_position_error_m=position_error,
            candidate_to_applied_orientation_error_rad=orientation_error,
        )
