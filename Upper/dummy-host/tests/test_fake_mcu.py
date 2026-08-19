from __future__ import annotations

import time

import numpy as np
import pytest

from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.protocol import MessageType, Packet, pack_hello
from dummy_host.robot_driver import DummyRobot, RobotError
from dummy_host.schema import ControlMode


class DropFirstHelloTransport(FakeMcuTransport):
    def __init__(self, config) -> None:
        super().__init__(config)
        self.hello_attempts = 0

    def send(self, packet: Packet) -> None:
        if packet.message_type == MessageType.HELLO:
            self.hello_attempts += 1
            if self.hello_attempts == 1:
                return
        super().send(packet)


class HelloAckWithoutStateTransport(FakeMcuTransport):
    def send(self, packet: Packet) -> None:
        super().send(packet)
        if packet.message_type == MessageType.HELLO:
            self._next_state_ns = None


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


def test_connect_retries_a_lost_hello_and_waits_for_state(config) -> None:
    transport = DropFirstHelloTransport(config)
    robot = DummyRobot(
        config,
        transport,
        response_timeout_s=0.02,
        connect_timeout_s=0.5,
    )
    try:
        robot.connect()
        assert transport.hello_attempts == 2
        assert robot.read_state().config_hash == config.config_hash
    finally:
        robot.disconnect()


def test_connect_rejects_hello_without_a_matching_state(config) -> None:
    robot = DummyRobot(
        config,
        HelloAckWithoutStateTransport(config),
        response_timeout_s=0.02,
        connect_timeout_s=0.08,
    )
    with pytest.raises(RobotError, match="first STATE after HELLO"):
        robot.connect()


def test_fake_mcu_target_timeout_enters_hold(config) -> None:
    now = [1_000_000_000]
    transport = FakeMcuTransport(config, clock_ns=lambda: now[0])
    transport.open()
    transport._lease = True
    transport._session = 7
    transport._mode = ControlMode.TELEOP
    transport._lease_duration_ms = config.lease_timeout_ms
    transport._extend_lease()
    transport._target_deadline_ns = now[0] + config.target_ttl_ms * 1_000_000
    now[0] += config.target_ttl_ms * 1_000_000
    transport.receive(timeout=0)
    assert transport._mode == ControlMode.HOLD


def test_fake_mcu_emits_periodic_state_while_idle(config) -> None:
    now = [1_000_000_000]
    transport = FakeMcuTransport(config, clock_ns=lambda: now[0])
    transport.open()
    transport.send(
        Packet(
            MessageType.HELLO,
            7,
            1,
            now[0] // 1_000,
            pack_hello(config.config_hash_bytes),
        )
    )
    assert transport.receive(timeout=0).message_type == MessageType.HELLO_ACK

    now[0] += int(1e9 / config.control_rate_hz)
    state = transport.receive(timeout=0)
    assert state is not None
    assert state.message_type == MessageType.STATE
    assert state.session_id == 7

