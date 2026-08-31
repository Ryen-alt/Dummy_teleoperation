from __future__ import annotations

import json
import sqlite3
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from dummy_host.calibration.geometry import rotation_vector
from dummy_host.cartesian_teleop import (
    CartesianGamepadMapper,
    CartesianPoseIntegrator,
    CartesianTeleopError,
)
from dummy_host.domain import AppliedAction
from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.kinematics import CartesianPose, DummyUrdfKinematics, IKResult
from dummy_host.recording import SessionRecorder
from dummy_host.robot_driver import DummyRobot
from dummy_host.schema import ControlMode, RobotConfig, RobotState
from dummy_host.teleop import TeleopCommand, TeleopProfile, load_teleop_profile
from dummy_host.teleop_runtime import run_teleop_collection


PROJECT = Path(__file__).parents[1]
URDF = PROJECT.parents[1] / "Dummy_URDF" / "dummy.urdf"


def _profile() -> TeleopProfile:
    return load_teleop_profile(PROJECT / "configs" / "teleop_inputs.yaml")


def _kinematics(
    config: RobotConfig,
    profile: TeleopProfile,
    **backend_options,
) -> DummyUrdfKinematics:
    cartesian = profile.cartesian
    assert cartesian is not None
    return DummyUrdfKinematics(
        URDF,
        joint_min_rad=config.joint_limit_min_rad,
        joint_max_rad=config.joint_limit_max_rad,
        joint_limit_margin_rad=cartesian.joint_limit_margin_rad,
        position_tolerance_m=cartesian.position_tolerance_m,
        orientation_tolerance_rad=cartesian.orientation_tolerance_rad,
        max_iterations=cartesian.max_iterations,
        sigma_warn=cartesian.sigma_warn,
        sigma_hard=cartesian.sigma_hard,
        damping_min=cartesian.damping_min,
        damping_max=cartesian.damping_max,
        task_trust_region=cartesian.task_trust_region,
        soft_limit_zone_rad=cartesian.soft_limit_zone_rad,
        max_solver_step_rad=cartesian.max_solver_step_rad,
        max_solution_step_rad=cartesian.max_solution_step_rad,
        translation_scale_m=cartesian.translation_scale_m,
        **backend_options,
    )


def _state(
    config: RobotConfig,
    joints: np.ndarray,
    now_ns: int,
    *,
    sweep_id: int = 1,
) -> RobotState:
    return RobotState(
        position=np.concatenate((joints, np.asarray([0.5]))).astype(np.float32),
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
        config_hash=config.config_hash,
        can_transport_status=0x40,
        feedback_sample_mcu_us=np.full(7, now_ns // 1_000, dtype=np.uint64),
        feedback_sweep_id=np.full(7, sweep_id, dtype=np.uint32),
        coherent_sweep_id=sweep_id,
        coherent_reference_mcu_us=now_ns // 1_000,
    )


def _command(now_ns: int, twist: list[float], *, gripper: float = 0.0) -> TeleopCommand:
    return TeleopCommand(
        monotonic_ns=now_ns,
        source="gamepad",
        joint_velocity_rad_s=np.zeros(6, dtype=np.float32),
        gripper_velocity_per_s=gripper,
        deadman=True,
        hold_requested=False,
        estop_requested=False,
        episode_event=None,
        connected=True,
        raw={},
        teleop_mode="cartesian",
        cartesian_twist=np.asarray(twist, dtype=np.float32),
    )


def _applied(
    requested: np.ndarray,
    applied: np.ndarray | None = None,
    *,
    sequence: int = 1,
    reasons: tuple[str, ...] = (),
) -> AppliedAction:
    actual = requested if applied is None else applied
    return AppliedAction(
        requested=requested.astype(np.float32),
        applied=actual.astype(np.float32),
        sequence=sequence,
        monotonic_ns=sequence,
        clipped=not np.array_equal(requested, actual),
        reasons=reasons,
        source="gamepad",
    )


class _LinearKinematics:
    base_link = "base_link"
    tip_link = "tool0"
    model_hash = "1" * 64

    def __init__(
        self,
        *,
        tolerance_m: float = 0.0,
        solve_duration_ns: int = 1_000_000,
        timed_out: bool = False,
    ) -> None:
        self.tolerance_m = tolerance_m
        self.solve_duration_ns = solve_duration_ns
        self.timed_out = timed_out

    def forward(self, joint_position_rad: np.ndarray) -> CartesianPose:
        joints = np.asarray(joint_position_rad, dtype=np.float64)
        return CartesianPose(
            np.asarray(
                [0.2 + 0.05 * joints[0], 0.05 * joints[1], 0.3 + 0.05 * joints[2]]
            ),
            np.eye(3),
        )

    def inverse(
        self,
        target: CartesianPose,
        measured_joint_rad: np.ndarray,
        previous_joint_rad: np.ndarray | None = None,
        *,
        hard_budget_ns: int | None = None,
    ) -> IKResult:
        seed = np.asarray(
            measured_joint_rad if previous_joint_rad is None else previous_joint_rad,
            dtype=np.float64,
        )
        seed_pose = self.forward(seed)
        error = float(np.linalg.norm(target.position_m - seed_pose.position_m))
        timed_out = self.timed_out or (
            hard_budget_ns is not None and self.solve_duration_ns > hard_budget_ns
        )
        if timed_out:
            return IKResult(
                success=False,
                joint_position_rad=None,
                branch_id=None,
                position_error_m=error,
                orientation_error_rad=0.0,
                joint_limit_margin_rad=None,
                minimum_singular_value=1.0,
                singularity_flags=(),
                clipped=False,
                reasons=("solve_budget_exceeded",),
                iterations=1,
                failure_reason="solve_budget_exceeded",
                solver="linear_test",
                solver_version="1",
                model_hash=self.model_hash,
                solve_duration_ns=self.solve_duration_ns,
                timed_out=True,
                timeout_stage="iteration_1_fk",
            )
        solved = seed.copy()
        if error > self.tolerance_m:
            solved[:3] = (
                target.position_m - np.asarray([0.2, 0.0, 0.3])
            ) / 0.05
        return IKResult(
            success=True,
            joint_position_rad=solved.astype(np.float32),
            branch_id="previous",
            position_error_m=0.0 if error > self.tolerance_m else error,
            orientation_error_rad=0.0,
            joint_limit_margin_rad=np.ones(6),
            minimum_singular_value=1.0,
            singularity_flags=(),
            clipped=False,
            reasons=(),
            iterations=1,
            failure_reason=None,
            solver="linear_test",
            solver_version="1",
            model_hash=self.model_hash,
            solve_duration_ns=self.solve_duration_ns,
        )

    def describe(self) -> dict[str, object]:
        return {"model_hash": self.model_hash, "tip_link": self.tip_link}


def test_xbox_cartesian_mapping_uses_axes_without_an_imu() -> None:
    profile = _profile()
    mapper = CartesianGamepadMapper(profile)
    # Linux evdev maps an untouched 0..max trigger to -1.  Both untouched
    # triggers must cancel to zero rather than creating an accidental Z drift.
    axes = {
        "left_y": -0.8,
        "left_x": 0.4,
        "right_trigger": -1.0,
        "left_trigger": -1.0,
        "right_y": 0.5,
        "right_x": -0.5,
        "dpad_x": 1.0,
    }
    command = mapper.map(axes, {"lb"}, 1_000)
    assert command.teleop_mode == "cartesian"
    assert command.deadman
    assert np.all(command.joint_velocity_rad_s == 0)
    assert command.cartesian_twist[0] > 0
    assert command.cartesian_twist[1] > 0
    assert command.cartesian_twist[2] == 0
    assert command.cartesian_twist[3] > 0
    assert command.cartesian_twist[4] < 0
    assert command.cartesian_twist[5] < 0


def test_urdf_fk_ik_round_trip_is_continuous(config: RobotConfig) -> None:
    profile = _profile()
    kinematics = _kinematics(config, profile)
    target_joints = np.asarray([0.32, 1.02, -1.05, 0.43, 0.24, 0.54])
    measured = target_joints + np.asarray([0.01, -0.01, 0.01, -0.01, 0.01, -0.01])
    target_pose = kinematics.forward(target_joints)
    result = kinematics.inverse(target_pose, measured, measured)
    assert result.success
    assert result.joint_position_rad is not None
    solved_pose = kinematics.forward(result.joint_position_rad)
    assert np.linalg.norm(solved_pose.position_m - target_pose.position_m) <= 3e-4
    assert result.orientation_error_rad <= 3e-3
    assert np.linalg.norm(result.joint_position_rad - measured) < 0.2
    assert result.model_hash == kinematics.model_hash


def test_cartesian_integrator_fails_closed_outside_workspace(config: RobotConfig) -> None:
    profile = _profile()
    cartesian = profile.cartesian
    assert cartesian is not None
    kinematics = _kinematics(config, profile)
    joints = np.asarray([0.3, 1.0, -1.0, 0.4, 0.2, 0.5], dtype=np.float32)
    state = _state(config, joints, 1_000_000_000)
    pose = kinematics.forward(joints)
    narrow_cartesian = replace(
        cartesian,
        workspace_max_m=np.asarray(
            [pose.position_m[0], 0.35, 0.48], dtype=np.float64
        ),
    )
    narrow_profile = replace(profile, cartesian=narrow_cartesian)
    integrator = CartesianPoseIntegrator(narrow_profile, config, kinematics)
    integrator.reset(state)
    command = _command(1_000_000_000, [0.01, 0, 0, 0, 0, 0])
    with pytest.raises(CartesianTeleopError, match="workspace") as captured:
        integrator.propose(command, _state(config, joints, 1_000_000_000, sweep_id=2), 1_000_000_000)
    assert captured.value.metadata["stage"] == "workspace"


def test_cartesian_propose_is_pure_and_uncommitted_failure_changes_nothing(
    config: RobotConfig,
) -> None:
    profile = _profile()
    kinematics = _LinearKinematics()
    joints = config.initial_pose_rad.astype(np.float32)
    integrator = CartesianPoseIntegrator(profile, config, kinematics)
    integrator.reset(_state(config, joints, 1_000_000_000, sweep_id=1), now_ns=1_000_000_000)
    target_before = integrator.target_pose
    assert target_before is not None
    position_before = target_before.position_m.copy()
    previous_before = integrator._previous_joint.copy()
    velocity_before = integrator._velocity.copy()
    gripper_before = integrator._gripper_target
    revision_before = integrator.revision

    proposal = integrator.propose(
        _command(1_050_000_000, [0.01, 0, 0, 0, 0, 0]),
        _state(config, joints, 1_050_000_000, sweep_id=2),
        1_050_000_000,
    )

    # A gateway rejection or transport exception leaves this proposal
    # uncommitted, so every integrator state variable remains unchanged.
    assert proposal.revision == revision_before
    assert integrator.revision == revision_before
    assert integrator.last_sweep_id == 1
    assert np.array_equal(integrator.target_pose.position_m, position_before)
    assert np.array_equal(integrator._previous_joint, previous_before)
    assert np.array_equal(integrator._velocity, velocity_before)
    assert integrator._gripper_target == gripper_before


def test_arm_clipping_reanchors_to_applied_tcp_fk_without_windup(
    config: RobotConfig,
) -> None:
    profile = _profile()
    kinematics = _LinearKinematics()
    joints = config.initial_pose_rad.astype(np.float32)
    integrator = CartesianPoseIntegrator(profile, config, kinematics)
    now_ns = 1_000_000_000
    integrator.reset(_state(config, joints, now_ns, sweep_id=1), now_ns=now_ns)
    maximum_error = 0.0

    for index in range(100):
        now_ns += 50_000_000
        state = _state(config, joints, now_ns, sweep_id=index + 2)
        proposal = integrator.propose(
            _command(now_ns, [0.03, 0, 0, 0, 0, 0]), state, now_ns
        )
        actual = proposal.action.copy()
        actual[:6] = joints
        commit = integrator.commit(
            proposal,
            _applied(
                proposal.action,
                actual,
                sequence=index + 1,
                reasons=("velocity_limited",),
            ),
        )
        expected = kinematics.forward(joints)
        assert commit.arm_clipped and commit.reanchored
        assert np.allclose(integrator.target_pose.position_m, expected.position_m)
        maximum_error = max(
            maximum_error, commit.candidate_to_applied_position_error_m
        )

    assert maximum_error < 0.002
    assert np.allclose(
        integrator.target_pose.position_m,
        kinematics.forward(joints).position_m,
    )


def test_unclipped_sub_tolerance_motion_accumulates_until_ik_moves(
    config: RobotConfig,
) -> None:
    profile = _profile()
    kinematics = _LinearKinematics(tolerance_m=0.001)
    joints = config.initial_pose_rad.astype(np.float32)
    original = joints.copy()
    integrator = CartesianPoseIntegrator(profile, config, kinematics)
    now_ns = 1_000_000_000
    integrator.reset(_state(config, joints, now_ns, sweep_id=1), now_ns=now_ns)
    requested_joint_0: list[float] = []

    for index in range(8):
        now_ns += 50_000_000
        proposal = integrator.propose(
            _command(now_ns, [0.004, 0, 0, 0, 0, 0]),
            _state(config, joints, now_ns, sweep_id=index + 2),
            now_ns,
        )
        action = _applied(proposal.action, sequence=index + 1)
        commit = integrator.commit(proposal, action)
        assert not commit.reanchored
        joints = action.applied[:6].copy()
        requested_joint_0.append(float(joints[0]))

    assert requested_joint_0[0] == pytest.approx(float(original[0]))
    assert requested_joint_0[-1] > float(original[0])
    assert integrator.target_pose.position_m[0] > kinematics.forward(original).position_m[0]


def test_gripper_only_clipping_does_not_reanchor_arm(config: RobotConfig) -> None:
    profile = _profile()
    kinematics = _LinearKinematics()
    joints = config.initial_pose_rad.astype(np.float32)
    integrator = CartesianPoseIntegrator(profile, config, kinematics)
    integrator.reset(_state(config, joints, 1_000_000_000, sweep_id=1), now_ns=1_000_000_000)
    proposal = integrator.propose(
        _command(1_050_000_000, [0.01, 0, 0, 0, 0, 0], gripper=0.5),
        _state(config, joints, 1_050_000_000, sweep_id=2),
        1_050_000_000,
    )
    actual = proposal.action.copy()
    actual[6] = 0.5
    commit = integrator.commit(
        proposal,
        _applied(proposal.action, actual, reasons=("gripper_range",)),
    )
    assert not commit.arm_clipped
    assert not commit.reanchored
    assert np.allclose(
        integrator.target_pose.position_m, proposal.candidate_pose.position_m
    )
    assert integrator._gripper_target == pytest.approx(0.5)


def test_reset_invalidates_an_outstanding_proposal(config: RobotConfig) -> None:
    profile = _profile()
    kinematics = _LinearKinematics()
    joints = config.initial_pose_rad.astype(np.float32)
    integrator = CartesianPoseIntegrator(profile, config, kinematics)
    state = _state(config, joints, 1_000_000_000, sweep_id=1)
    integrator.reset(state, now_ns=1_000_000_000)
    proposal = integrator.propose(
        _command(1_050_000_000, [0.01, 0, 0, 0, 0, 0]),
        _state(config, joints, 1_050_000_000, sweep_id=2),
        1_050_000_000,
    )
    integrator.reset(state, now_ns=1_050_000_000)
    with pytest.raises(CartesianTeleopError, match="revision is stale"):
        integrator.commit(proposal, _applied(proposal.action))


def test_ik_soft_overrun_is_committable_and_hard_timeout_is_not(
    config: RobotConfig,
) -> None:
    profile = _profile()
    joints = config.initial_pose_rad.astype(np.float32)
    soft_backend = _LinearKinematics(solve_duration_ns=13_000_000)
    soft = CartesianPoseIntegrator(profile, config, soft_backend)
    soft.reset(_state(config, joints, 1_000_000_000, sweep_id=1), now_ns=1_000_000_000)
    proposal = soft.propose(
        _command(1_050_000_000, [0.01, 0, 0, 0, 0, 0]),
        _state(config, joints, 1_050_000_000, sweep_id=2),
        1_050_000_000,
    )
    assert proposal.metadata["solve_budget"]["soft_budget_exceeded"] is True
    assert soft.commit(proposal, _applied(proposal.action)).reanchored is False

    hard_backend = _LinearKinematics(solve_duration_ns=21_000_000)
    hard = CartesianPoseIntegrator(profile, config, hard_backend)
    hard.reset(_state(config, joints, 2_000_000_000, sweep_id=1), now_ns=2_000_000_000)
    revision = hard.revision
    with pytest.raises(CartesianTeleopError, match="solve_budget_exceeded") as captured:
        hard.propose(
            _command(2_050_000_000, [0.01, 0, 0, 0, 0, 0]),
            _state(config, joints, 2_050_000_000, sweep_id=2),
            2_050_000_000,
        )
    assert captured.value.metadata["ik"]["timed_out"] is True
    assert hard.revision == revision
    assert hard.last_sweep_id == 1


def test_real_urdf_solver_checks_the_hard_deadline_inside_solve(
    config: RobotConfig,
) -> None:
    class StepClock:
        def __init__(self) -> None:
            self.value = 0

        def __call__(self) -> int:
            current = self.value
            self.value += 10_000_000
            return current

    profile = _profile()
    backend = _kinematics(config, profile, clock_ns=StepClock())
    joints = np.asarray([0.32, 1.02, -1.05, 0.43, 0.24, 0.54])
    result = backend.inverse(
        backend.forward(joints),
        joints,
        joints,
        hard_budget_ns=20_000_000,
    )
    assert not result.success
    assert result.timed_out
    assert result.failure_reason == "solve_budget_exceeded"
    assert result.timeout_stage is not None
    assert result.solve_duration_ns >= 20_000_000
    json.dumps(result.as_dict(), allow_nan=False)


def test_geometric_jacobian_matches_central_difference(config: RobotConfig) -> None:
    profile = _profile()
    tool0_T_tip = np.eye(4, dtype=np.float64)
    tool0_T_tip[:3, 3] = [0.04, -0.02, 0.08]
    backend = _kinematics(
        config,
        profile,
        tool0_T_tip=tool0_T_tip,
        tip_frame="test_tcp",
    )
    joints = np.asarray([0.32, 1.02, -1.05, 0.43, 0.24, 0.54])
    pose = backend.forward(joints)
    analytic = backend._jacobian(joints, pose, None)
    numeric = np.empty((6, 6), dtype=np.float64)
    delta = 1e-6
    for index in range(6):
        lower = joints.copy()
        upper = joints.copy()
        lower[index] -= delta
        upper[index] += delta
        lower_pose = backend.forward(lower)
        upper_pose = backend.forward(upper)
        numeric[:3, index] = (
            upper_pose.position_m - lower_pose.position_m
        ) / (2.0 * delta)
        numeric[3:, index] = rotation_vector(
            upper_pose.rotation @ lower_pose.rotation.T
        ) / (2.0 * delta)
    np.testing.assert_allclose(analytic, numeric, atol=2e-6, rtol=2e-5)


def test_adaptive_ik_suppresses_hard_singular_direction_and_limit_motion(
    config: RobotConfig,
) -> None:
    backend = _kinematics(config, _profile())
    jacobian = np.eye(6, dtype=np.float64)
    # After translation weighting this direction has sigma=0.002, below
    # sigma_hard=0.004, so motion along it must be suppressed.
    jacobian[0, 0] = backend.translation_scale_m * 0.002
    middle = (backend.lower + backend.upper) * 0.5
    singular_step, singular = backend._adaptive_task_step(
        jacobian,
        np.asarray([0.01, 0.0, 0.0]),
        np.zeros(3),
        middle,
        None,
        "test_singular",
    )
    assert singular == pytest.approx(0.002)
    assert singular_step[0] == pytest.approx(0.0, abs=1e-12)
    assert backend._singularity_flags(singular) == ("singularity_hard",)

    regular = np.eye(6, dtype=np.float64)
    near_lower = middle.copy()
    near_lower[0] = backend.lower[0] + backend.soft_limit_zone_rad * 0.5
    blocked, _ = backend._adaptive_task_step(
        regular,
        np.asarray([-0.01, 0.0, 0.0]),
        np.zeros(3),
        near_lower,
        None,
        "test_lower_blocked",
    )
    recovery, _ = backend._adaptive_task_step(
        regular,
        np.asarray([0.01, 0.0, 0.0]),
        np.zeros(3),
        near_lower,
        None,
        "test_lower_recovery",
    )
    assert blocked[0] == pytest.approx(0.0, abs=1e-12)
    assert recovery[0] > 0.0


class _ScriptedCartesianGamepad:
    def __init__(
        self,
        mapper: CartesianGamepadMapper,
        *,
        start_episode: bool = False,
        deadman_until_poll: int = 9,
    ) -> None:
        self.mapper = mapper
        self.start_episode = start_episode
        self.deadman_until_poll = deadman_until_poll
        self.polls = 0
        self.closed = False

    def poll(self, now_ns: int | None = None) -> TeleopCommand:
        assert now_ns is not None
        self.polls += 1
        pressed = {"lb"} if 2 <= self.polls <= self.deadman_until_poll else set()
        if self.start_episode and self.polls == 2:
            pressed.add("y")
        return self.mapper.map(
            {
                "left_x": 0.6,
                "right_trigger": -1.0,
                "left_trigger": -1.0,
            },
            pressed,
            now_ns,
        )

    def close(self) -> None:
        self.closed = True


def test_cartesian_fake_mcu_reuses_joint_gateway_and_records_semantics(
    config: RobotConfig,
    tmp_path: Path,
) -> None:
    profile = _profile()
    kinematics = _kinematics(config, profile)
    # Keep the synthetic dead-man asserted long enough for the asynchronous
    # lease thread to acquire control even under a loaded full-suite runner.
    source = _ScriptedCartesianGamepad(
        CartesianGamepadMapper(profile), deadman_until_poll=40
    )
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="gamepad",
        session_name="session_cartesian_runtime",
        extra_manifest={
            "teleop_mode": "cartesian",
            "kinematics": kinematics.describe(),
        },
    )
    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=config.target_ttl_ms / 1000.0 + 0.46,
        teleop_mode="cartesian",
        kinematics=kinematics,
    )
    recorder.close()
    assert result.actions_sent >= 1
    assert result.final_mode == "HOLD"
    assert source.closed
    with sqlite3.connect(recorder.db_path) as connection:
        rows = connection.execute(
            "SELECT teleop_mode, length(cartesian_twist), raw_input_json, "
            "length(requested_action), length(applied_action) "
            "FROM samples WHERE requested_action IS NOT NULL"
        ).fetchall()
    assert len(rows) == result.actions_sent
    assert all(row[0] == "cartesian" and row[1] == 24 for row in rows)
    assert all(row[3:] == (28, 28) for row in rows)
    cartesian_rows = [json.loads(row[2])["cartesian"] for row in rows]
    assert all("ik" in value and "solve_budget" in value for value in cartesian_rows)
    assert all("proposal_id" in value and "source_sweep_id" in value for value in cartesian_rows)
    assert all("commit" in value for value in cartesian_rows)
    assert all(
        "applied_pose" in value["commit"]
        and "candidate_to_applied_position_error_m" in value["commit"]
        for value in cartesian_rows
    )
    with sqlite3.connect(recorder.db_path) as connection:
        lifecycle = connection.execute(
            "SELECT COUNT(*), COUNT(post_command_feedback_host_ns) "
            "FROM action_lifecycle"
        ).fetchone()
    assert lifecycle is not None
    assert lifecycle[0] >= 1 and lifecycle[1] >= 1


def test_runtime_workspace_failure_sends_no_target_and_ends_in_hold(
    config: RobotConfig,
    tmp_path: Path,
) -> None:
    profile = _profile()
    cartesian = profile.cartesian
    assert cartesian is not None
    kinematics = _kinematics(config, profile)
    initial_pose = kinematics.forward(config.initial_pose_rad)
    narrow_profile = replace(
        profile,
        cartesian=replace(
            cartesian,
            workspace_max_m=np.asarray(
                [initial_pose.position_m[0], 0.35, 0.48], dtype=np.float64
            ),
        ),
    )
    source = _ScriptedCartesianGamepad(CartesianGamepadMapper(narrow_profile))
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        narrow_profile,
        source="gamepad",
        session_name="session_cartesian_workspace_failure",
    )
    result = run_teleop_collection(
        robot,
        source,
        recorder,
        narrow_profile,
        duration_s=0.56,
        teleop_mode="cartesian",
        kinematics=kinematics,
    )
    recorder.close(clean_shutdown=False)
    assert result.actions_sent == 0
    assert source.closed
    assert not robot.is_connected
    with sqlite3.connect(recorder.db_path) as connection:
        sent, invalid = connection.execute(
            "SELECT COUNT(requested_action), SUM(sample_valid = 0) FROM samples"
        ).fetchone()
    assert sent == 0
    assert invalid >= 1
    events = recorder.events_path.read_text(encoding="utf-8")
    assert '"event":"cartesian_target_invalid"' in events


def test_duplicate_sweeps_advance_time_without_integrating_old_feedback(
    config: RobotConfig,
) -> None:
    profile = _profile()
    kinematics = _LinearKinematics()
    joints = config.initial_pose_rad.astype(np.float32)
    integrator = CartesianPoseIntegrator(profile, config, kinematics)
    integrator.reset(
        _state(config, joints, 1_000_000_000, sweep_id=10),
        now_ns=1_000_000_000,
    )
    duplicate = _state(config, joints, 1_050_000_000, sweep_id=10)
    assert not integrator.has_fresh_coherent_sweep(duplicate)
    integrator.advance_without_motion(1_050_000_000)
    integrator.advance_without_motion(1_100_000_000)

    proposal = integrator.propose(
        _command(1_150_000_000, [0.01, 0, 0, 0, 0, 0]),
        _state(config, joints, 1_150_000_000, sweep_id=11),
        1_150_000_000,
    )
    assert proposal.dt_s == pytest.approx(0.05)
    assert proposal.source_sweep_id == 11


class _FreezingSweepFakeMcu(FakeMcuTransport):
    def __init__(self, config: RobotConfig, *, freeze_after: int = 8) -> None:
        super().__init__(config)
        self.freeze_after = freeze_after

    def _emit_state(self, sequence: int) -> None:
        if self._sweep_id >= self.freeze_after:
            self._sweep_id -= 1
        super()._emit_state(sequence)


def test_runtime_stalled_coherent_sweep_holds_and_fails_episode(
    config: RobotConfig,
    tmp_path: Path,
) -> None:
    profile = _profile()
    source = _ScriptedCartesianGamepad(
        CartesianGamepadMapper(profile),
        start_episode=True,
        deadman_until_poll=20,
    )
    robot = DummyRobot(config, _FreezingSweepFakeMcu(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="gamepad",
        session_name="session_cartesian_stalled_sweep",
    )
    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=0.76,
        teleop_mode="cartesian",
        kinematics=_LinearKinematics(),
    )
    recorder.close()
    events = recorder.events_path.read_text(encoding="utf-8")
    with sqlite3.connect(recorder.db_path) as connection:
        raw_rows = connection.execute(
            "SELECT raw_input_json FROM samples WHERE action_sequence IS NOT NULL"
        ).fetchall()
    source_sweeps = [
        json.loads(row[0])["cartesian"]["source_sweep_id"] for row in raw_rows
    ]
    assert result.coherent_sweep_skips >= 2
    assert len(source_sweeps) == len(set(source_sweeps))
    assert result.final_mode == "HOLD"
    assert '"event":"coherent_sweep_stalled"' in events
    assert '"reason":"coherent_sweep_stalled"' in events


def test_runtime_hard_ik_timeout_sends_no_action_and_holds(
    config: RobotConfig,
    tmp_path: Path,
) -> None:
    profile = _profile()
    source = _ScriptedCartesianGamepad(
        CartesianGamepadMapper(profile), start_episode=True
    )
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="gamepad",
        session_name="session_cartesian_hard_timeout",
    )
    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=0.46,
        teleop_mode="cartesian",
        kinematics=_LinearKinematics(solve_duration_ns=21_000_000),
    )
    recorder.close()
    events = recorder.events_path.read_text(encoding="utf-8")
    assert result.actions_sent == 0
    assert result.ik_hard_timeouts == 1
    assert result.final_mode == "HOLD"
    assert "solve_budget_exceeded" in events
    assert '"reason":"control_target_generation_failed"' in events
