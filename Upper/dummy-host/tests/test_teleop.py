from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import yaml

from dummy_host.apps.migrate_teleop_v5 import migrate_file
from dummy_host.schema import ControlMode, RobotConfig, RobotState
from dummy_host.teleop import (
    GamepadMapper,
    ControlTimingError,
    JointVelocityIntegrator,
    KeyboardMapper,
    TeleopConfigError,
    load_teleop_profile,
    shape_axis,
    validate_profile_for_robot,
)
from dummy_host.teleop_runtime import mask_teleop_command


def _profile():
    return load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")


def test_v4_profile_requires_explicit_migration_and_v5_policy_is_fixed(
    tmp_path: Path,
) -> None:
    source = Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["version"] = 4
    solver = raw["cartesian"]["solver"]
    for key in (
        "sigma_warn",
        "sigma_hard",
        "damping_min",
        "damping_max",
        "task_trust_region",
        "soft_limit_zone_rad",
    ):
        solver.pop(key)
    solver["damping"] = 0.02
    solver["finite_difference_rad"] = 0.0001
    legacy = tmp_path / "teleop_v4.yaml"
    legacy.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")

    with pytest.raises(TeleopConfigError, match="explicit adaptive-IK migration"):
        load_teleop_profile(legacy)
    migrated_path = migrate_file(legacy, tmp_path / "teleop_v5.yaml")
    migrated = load_teleop_profile(migrated_path)
    assert migrated.version == 5
    assert migrated.cartesian is not None
    assert migrated.cartesian.sigma_warn == pytest.approx(0.04)
    assert migrated.cartesian.sigma_hard == pytest.approx(0.004)
    assert migrated.cartesian.damping_min == pytest.approx(0.01)
    assert migrated.cartesian.damping_max == pytest.approx(0.08)

    changed = yaml.safe_load(migrated_path.read_text(encoding="utf-8"))
    changed["cartesian"]["solver"]["sigma_warn"] = 0.05
    changed_path = tmp_path / "teleop_v5_changed.yaml"
    changed_path.write_text(yaml.safe_dump(changed, sort_keys=False), encoding="utf-8")
    with pytest.raises(TeleopConfigError, match="sigma_warn is fixed at 0.04"):
        load_teleop_profile(changed_path)


def _state(config: RobotConfig, now_ns: int) -> RobotState:
    return RobotState(
        position=np.concatenate(
            (config.initial_pose_rad, np.asarray([0.5], dtype=np.float32))
        ).astype(np.float32),
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
    )


def test_keyboard_mapper_emits_physical_velocity_and_episode_edges(config: RobotConfig) -> None:
    profile = _profile()
    validate_profile_for_robot(profile, config)
    mapper = KeyboardMapper(profile)
    pressed = {"KEY_SPACE", "KEY_Q", "KEY_L", "KEY_F5"}
    first = mapper.map(pressed, 1_000)
    second = mapper.map(pressed, 2_000)
    assert first.deadman
    assert first.joint_velocity_rad_s[0] == profile.joint_speed_rad_s[0]
    assert first.gripper_velocity_per_s > 0
    assert first.episode_event == "start"
    assert second.episode_event is None


def test_gamepad_mapper_applies_deadzone_inversion_and_estop_chord() -> None:
    profile = _profile()
    mapper = GamepadMapper(profile)
    axes = {binding.axis: 0.0 for binding in profile.gamepad.joint_axes}
    axes["left_x"] = 0.6
    axes["left_y"] = 0.6
    command = mapper.map(
        axes,
        {"lb", "rb", "menu"},
        100,
    )
    assert command.deadman
    assert command.estop_requested
    assert command.joint_velocity_rad_s[0] > 0
    assert command.joint_velocity_rad_s[1] < 0
    assert command.raw["protocol_id"] == "flydigi_vader5_linux_evdev_v1"
    assert shape_axis(0.05, profile.gamepad.deadzone, profile.gamepad.response_exponent) == 0


def test_joint_velocity_integrator_accelerates_and_stays_bounded(config: RobotConfig) -> None:
    profile = _profile()
    mapper = KeyboardMapper(profile)
    integrator = JointVelocityIntegrator(profile, config)
    now_ns = 1_000_000_000
    state = _state(config, now_ns)
    integrator.reset(state)
    command = mapper.map({"KEY_SPACE", "KEY_Q"}, now_ns)
    target = integrator.step(command, state, now_ns)
    maximum_first_step = profile.joint_acceleration_rad_s2[0] / config.control_rate_hz**2
    assert 0 < target[0] - state.position[0] <= maximum_first_step + 1e-7
    assert np.all(target[:6] >= config.joint_limit_min_rad)
    assert np.all(target[:6] <= config.joint_limit_max_rad)


def test_joint_integrator_uses_real_jitter_and_rejects_over_budget_dt(
    config: RobotConfig,
) -> None:
    profile = _profile()
    mapper = KeyboardMapper(profile)
    state = _state(config, 1_000_000_000)
    integrator = JointVelocityIntegrator(profile, config)
    integrator.reset(state, now_ns=1_000_000_000)
    jittered_ns = 1_070_000_000
    jittered = integrator.step(
        mapper.map({"KEY_SPACE", "KEY_Q"}, jittered_ns), state, jittered_ns
    )
    assert jittered[0] > state.position[0]
    with pytest.raises(ControlTimingError, match="exceeds"):
        integrator.step(
            mapper.map({"KEY_SPACE", "KEY_Q"}, 1_150_000_000),
            state,
            1_150_000_000,
        )


def test_real_bringup_mask_blocks_unapproved_joints_and_gripper() -> None:
    profile = _profile()
    mapper = KeyboardMapper(profile)
    command = mapper.map({"KEY_SPACE", "KEY_Q", "KEY_W", "KEY_L"}, 1_000)
    command = replace(command, event_ns=900)
    masked = mask_teleop_command(command, allowed_joints={2}, allow_gripper=False)
    assert masked.joint_velocity_rad_s[0] == 0
    assert masked.joint_velocity_rad_s[1] != 0
    assert masked.gripper_velocity_per_s == 0
    assert masked.event_ns == 900
    assert masked.raw["allowed_joints"] == [2]
