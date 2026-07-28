from __future__ import annotations

import time

import numpy as np
import pytest

from dummy_host.safety import SafetyError, SafetyFilter
from dummy_host.schema import ControlMode, RobotState


def make_state(config, now_ns: int) -> RobotState:
    return RobotState(
        position=np.concatenate((config.initial_pose_rad, np.asarray([0.0], dtype=np.float32))),
        velocity=np.zeros(7, dtype=np.float32),
        monotonic_ns=now_ns,
        mcu_time_us=0,
        mode=ControlMode.TELEOP,
        fault_bits=0,
        position_valid=True,
        velocity_valid=True,
        gripper_valid=True,
        last_received_sequence=0,
        last_applied_sequence=0,
        target_age_ms=0,
        config_hash=config.config_hash,
    )


def test_wrong_dtype_and_nan_are_rejected(config) -> None:
    safety = SafetyFilter(config)
    now = time.monotonic_ns()
    state = make_state(config, now)
    with pytest.raises(SafetyError, match="dtype"):
        safety.apply(state.position.astype(np.float64), state, now)
    action = state.position.copy()
    action[2] = np.nan
    with pytest.raises(SafetyError, match="NaN"):
        safety.apply(action, state, now)


def test_velocity_and_acceleration_are_bounded(config) -> None:
    safety = SafetyFilter(config)
    now = time.monotonic_ns()
    state = make_state(config, now)
    requested = state.position.copy()
    requested[0] += 0.05
    result = safety.apply(requested, state, now)
    assert result.clipped
    assert "velocity_or_acceleration_limit" in result.reasons
    max_first_step = config.joint_acceleration_limit_rad_s2[0] / config.control_rate_hz**2
    assert result.applied[0] - state.position[0] <= max_first_step + 1e-7


def test_stale_state_causes_hold_condition(config) -> None:
    safety = SafetyFilter(config)
    now = time.monotonic_ns()
    state = make_state(config, now - (config.max_state_age_ms + 1) * 1_000_000)
    with pytest.raises(SafetyError, match="stale"):
        safety.apply(state.position.copy(), state, now)

