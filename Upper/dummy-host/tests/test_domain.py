from __future__ import annotations

import numpy as np
import pytest

from dummy_host.control import ActionGateway
from dummy_host.domain import ActionProposal, ActionSpace, ControlMode, RobotBackend, RobotState
from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.robot_driver import DummyRobot
from dummy_host.safety import SafetyError


def _state(config, now_ns: int) -> RobotState:
    position = np.concatenate((config.initial_pose_rad, np.asarray([0.5], dtype=np.float32)))
    return RobotState(
        position=position.astype(np.float32),
        velocity=np.zeros(7, dtype=np.float32),
        monotonic_ns=now_ns,
        mcu_time_us=now_ns // 1000,
        mode=ControlMode.TELEOP,
        fault_bits=0,
        position_valid=True,
        velocity_valid=False,
        gripper_valid=True,
        last_received_sequence=0,
        target_age_ms=0,
        config_hash=config.config_hash,
    )


def test_dummy_robot_satisfies_backend_contract(config) -> None:
    assert isinstance(DummyRobot(config, FakeMcuTransport(config)), RobotBackend)


def test_action_gateway_rejects_stale_and_non_absolute_proposals(config) -> None:
    gateway = ActionGateway(config)
    now_ns = 1_000_000_000
    values = np.concatenate((config.initial_pose_rad, np.asarray([0.5], dtype=np.float32)))
    stale = ActionProposal(
        source="test",
        action_space=ActionSpace.JOINT_POSITION_ABSOLUTE,
        values=values.astype(np.float32),
        generated_at_ns=now_ns - 2_000_000,
        valid_until_ns=now_ns - 1,
    )
    with pytest.raises(SafetyError, match="not valid"):
        gateway.evaluate(stale, _state(config, now_ns), now_ns)

    velocity = ActionProposal(
        source="test",
        action_space=ActionSpace.JOINT_VELOCITY,
        values=np.zeros(7, dtype=np.float32),
        generated_at_ns=now_ns,
        valid_until_ns=now_ns + 1_000_000,
    )
    with pytest.raises(SafetyError, match="requires absolute"):
        gateway.evaluate(velocity, _state(config, now_ns), now_ns)


def test_applied_action_retains_source_and_canonical_value(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    robot.connect()
    try:
        robot.acquire_control(ControlMode.TELEOP)
        state = robot.read_state()
        now_ns = robot.clock_ns()
        proposal = ActionProposal(
            source="contract-test",
            action_space=ActionSpace.JOINT_POSITION_ABSOLUTE,
            values=state.position.astype(np.float32, copy=True),
            generated_at_ns=now_ns,
            valid_until_ns=now_ns + config.target_ttl_ms * 1_000_000,
        )
        result = robot.submit_action(proposal)
        assert result.source == "contract-test"
        np.testing.assert_array_equal(result.canonical, result.applied)
        assert not result.applied.flags.writeable
    finally:
        robot.disconnect()
