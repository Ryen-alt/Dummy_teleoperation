from __future__ import annotations

import time

import numpy as np
import pytest

from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.protocol import MessageType, Packet, pack_hello
from dummy_host.robot_driver import DummyRobot, RobotError
from dummy_host.schema import ConfigError, ControlMode
from dummy_host.domain.models import ActionStage, FaultBits, HoldReasonBits


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


class ContinuousStateWithoutTargetAckTransport(FakeMcuTransport):
    def _ack(self, request: Packet) -> None:
        if request.message_type != MessageType.SET_JOINT_TARGET:
            super()._ack(request)


class ExactCanWithoutPostFeedbackTransport(FakeMcuTransport):
    def _emit_state(self, sequence: int) -> None:
        applied = self._last_applied
        self._last_applied = 0
        try:
            super()._emit_state(sequence)
        finally:
            self._last_applied = applied


class OldV21WithoutMultiChannelSequence(FakeMcuTransport):
    is_simulated = False
    firmware_version = "dummy-ref-v2.1"
    firmware_capabilities = 0


def test_dummy_robot_fake_mcu_closed_loop(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    with robot:
        assert robot.firmware_version == "fake-mcu-v2.1"
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
        while (
            robot.read_state().last_post_command_feedback_sequence != applied.sequence
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)
        assert robot.read_state().last_post_command_feedback_sequence == applied.sequence
        robot.heartbeat()
        robot.hold()
        deadline = time.monotonic() + 0.5
        while robot.read_state().mode != ControlMode.HOLD and time.monotonic() < deadline:
            time.sleep(0.005)
        assert robot.read_state().mode == ControlMode.HOLD


def test_real_v21_without_multi_channel_sequence_capability_is_rejected(config) -> None:
    robot = DummyRobot(config, OldV21WithoutMultiChannelSequence(config))
    with pytest.raises(ConfigError, match="multi-channel sequence"):
        robot.connect()
    assert not robot.is_connected


def test_nonblocking_action_reports_exact_can_queue_and_post_feedback(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    stages = []
    robot.set_action_lifecycle_listener(stages.append)
    with robot:
        robot.acquire_control(ControlMode.TELEOP)
        state = robot.read_state()
        action = state.position.copy()
        action[0] += np.float32(0.005)
        applied = robot.enqueue_absolute_action(action, source="test")
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            observed = {
                update.stage for update in stages if update.sequence == applied.sequence
            }
            if ActionStage.POST_COMMAND_FEEDBACK in observed:
                break
            time.sleep(0.005)
        observed = [
            update.stage for update in stages if update.sequence == applied.sequence
        ]
        assert ActionStage.RECEIVED in observed
        assert ActionStage.SAFETY_ACCEPTED in observed
        assert ActionStage.SEND_ENQUEUED in observed
        assert ActionStage.SERIAL_SEND_STARTED in observed
        assert ActionStage.SERIAL_SEND_FINISHED in observed
        assert ActionStage.ACKNOWLEDGED in observed
        assert ActionStage.CAN_QUEUED_EXACT in observed
        assert ActionStage.POST_COMMAND_FEEDBACK in observed


def test_action_watchdog_expires_lost_ack_while_state_continues(config) -> None:
    robot = DummyRobot(
        config,
        ContinuousStateWithoutTargetAckTransport(config),
        response_timeout_s=0.03,
        action_observation_timeout_s=0.05,
    )
    stages = []
    robot.set_action_lifecycle_listener(stages.append)
    with robot:
        robot.acquire_control(ControlMode.TELEOP)
        target = robot.read_state().position.copy()
        target[0] += np.float32(0.005)
        applied = robot.enqueue_absolute_action(target, source="lost-ack")
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            if any(
                update.sequence == applied.sequence
                and update.stage is ActionStage.FAILED
                for update in stages
            ):
                break
            # Prove the periodic STATE stream remains healthy during timeout.
            robot.read_state()
            time.sleep(0.005)
        failure = next(
            update
            for update in stages
            if update.sequence == applied.sequence
            and update.stage is ActionStage.FAILED
        )
        assert "ACK" in (failure.detail or "")


def test_action_watchdog_expires_when_post_feedback_never_arrives(config) -> None:
    robot = DummyRobot(
        config,
        ExactCanWithoutPostFeedbackTransport(config),
        response_timeout_s=0.03,
        action_observation_timeout_s=0.04,
    )
    stages = []
    robot.set_action_lifecycle_listener(stages.append)
    with robot:
        robot.acquire_control(ControlMode.TELEOP)
        target = robot.read_state().position.copy()
        target[0] += np.float32(0.005)
        applied = robot.enqueue_absolute_action(target, source="missing-feedback")
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline and not any(
            update.sequence == applied.sequence
            and update.stage is ActionStage.FAILED
            for update in stages
        ):
            time.sleep(0.005)
        observed = {
            update.stage for update in stages if update.sequence == applied.sequence
        }
        assert ActionStage.ACKNOWLEDGED in observed
        assert ActionStage.CAN_QUEUED_EXACT in observed
        assert ActionStage.POST_COMMAND_FEEDBACK not in observed
        assert ActionStage.FAILED in observed


def test_action_lifecycle_is_reinitialized_after_sequence_wrap(config) -> None:
    transport = FakeMcuTransport(config)
    robot = DummyRobot(config, transport)
    stages = []
    robot.set_action_lifecycle_listener(stages.append)
    with robot:
        robot.acquire_control(ControlMode.TELEOP)
        robot._sequence = 0
        target = robot.read_state().position.copy()
        target[0] += np.float32(0.002)
        first = robot.enqueue_absolute_action(target, source="before-wrap")
        assert first.sequence == 1
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline and not any(
            update.sequence == 1
            and update.stage is ActionStage.POST_COMMAND_FEEDBACK
            for update in stages
        ):
            time.sleep(0.005)

        robot._sequence = 0xFFFFFFFF
        target[0] += np.float32(0.002)
        wrapped = robot.enqueue_absolute_action(target, source="after-wrap")
        assert wrapped.sequence == 1
        assert sum(
            update.sequence == 1 and update.stage is ActionStage.RECEIVED
            for update in stages
        ) == 2


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


def test_fake_mcu_feedback_interruption_is_visible_to_host(config) -> None:
    transport = FakeMcuTransport(config)
    robot = DummyRobot(config, transport)
    with robot:
        robot.acquire_control(ControlMode.TELEOP)
        transport.inject_feedback_interruption(severe=False, node=4)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            state = robot.read_state()
            if state.hold_reason_bits & int(HoldReasonBits.FEEDBACK_STALE):
                break
            time.sleep(0.005)
        assert state.mode == ControlMode.HOLD
        assert state.feedback_loss_count[3] == 1
        assert state.node_fault_bits[3] != 0

        transport.inject_feedback_interruption(severe=True, node=4)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            state = robot.read_state()
            if state.fault_bits & int(FaultBits.FEEDBACK_LOST):
                break
            time.sleep(0.005)
        assert state.mode == ControlMode.FAULT
        assert state.fault_bits & int(FaultBits.FEEDBACK_LOST)
