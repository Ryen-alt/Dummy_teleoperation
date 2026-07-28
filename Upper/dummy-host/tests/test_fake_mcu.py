from __future__ import annotations

import time

import numpy as np

from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.robot_driver import DummyRobot
from dummy_host.schema import ControlMode


def test_dummy_robot_fake_mcu_closed_loop(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    with robot:
        assert robot.firmware_version == "fake-mcu-v1"
        robot.acquire_control(ControlMode.TELEOP)
        deadline = time.monotonic() + 0.5
        while robot.read_state().mode != ControlMode.TELEOP and time.monotonic() < deadline:
            time.sleep(0.005)
        state = robot.read_state()
        action = state.position.copy().astype(np.float32)
        action[0] += 0.01
        applied = robot.send_action(action)
        assert applied.sequence > 0
        assert applied.applied[0] > state.position[0]
        deadline = time.monotonic() + 0.5
        while robot.read_state().last_applied_sequence != applied.sequence and time.monotonic() < deadline:
            time.sleep(0.005)
        assert robot.read_state().last_applied_sequence == applied.sequence
        robot.heartbeat()
        robot.hold()
        deadline = time.monotonic() + 0.5
        while robot.read_state().mode != ControlMode.HOLD and time.monotonic() < deadline:
            time.sleep(0.005)
        assert robot.read_state().mode == ControlMode.HOLD

