from __future__ import annotations

import time
from dataclasses import replace
from threading import Event, Lock, Thread

import numpy as np
import pytest

from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.protocol import (
    ACTION_PROGRESS,
    CAPABILITY_CAN_DIAGNOSTICS_V2,
    CAPABILITY_CAN_TIMING_PROFILE,
    CAN_DIAGNOSTICS_WINDOW_VALID,
    ActionProgressStage,
    MessageType,
    Packet,
    pack_hello,
)
from dummy_host.robot_driver import CommandRejected, DummyRobot, RobotError
from dummy_host.schema import ConfigError, ControlMode
from dummy_host.domain.models import ActionStage, FaultBits, HoldReasonBits
from dummy_host.safety import SafetyError


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


class RuntimeHoldOnSetModeTransport(FakeMcuTransport):
    def send(self, packet: Packet) -> None:
        if packet.message_type is MessageType.SET_MODE:
            self._mode = ControlMode.HOLD
            self._hold_reason_bits |= int(HoldReasonBits.RUNTIME_LIMIT)
            self._extend_lease()
            self._ack(packet)
            self._emit_state(packet.sequence)
            return
        super().send(packet)


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


class AckWithoutExactCanTransport(FakeMcuTransport):
    def _emit_state(self, sequence: int) -> None:
        progress = self._progress
        applied = self._last_applied
        self._progress = []
        self._last_applied = 0
        try:
            super()._emit_state(sequence)
        finally:
            self._progress = progress
            self._last_applied = applied


class StateReplayOnlyTransport(FakeMcuTransport):
    """Drop every EVENT so action progress must be recovered from STATE."""

    def receive(self, timeout: float | None = None) -> Packet | None:
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            remaining = (
                None if deadline is None else max(0.0, deadline - time.monotonic())
            )
            packet = super().receive(timeout=remaining)
            if packet is None or packet.message_type != MessageType.EVENT:
                return packet
            if deadline is not None and time.monotonic() >= deadline:
                return None


class OldV21WithoutMultiChannelSequence(FakeMcuTransport):
    is_simulated = False
    firmware_version = "dummy-ref-v2.1"
    firmware_capabilities = 0


class OldV22WithProtocolV5(FakeMcuTransport):
    is_simulated = False
    firmware_version = "dummy-ref-v2.2"


class V222WithoutCanDiagnosticsV2(FakeMcuTransport):
    is_simulated = False
    firmware_version = "dummy-ref-v2.2.2"
    firmware_capabilities = (
        FakeMcuTransport.firmware_capabilities
        & ~CAPABILITY_CAN_DIAGNOSTICS_V2
    )


class CurrentV222RealTransport(FakeMcuTransport):
    is_simulated = False
    firmware_version = "dummy-ref-v2.2.2"


class V222WithoutCanTimingProfile(FakeMcuTransport):
    is_simulated = False
    firmware_version = "dummy-ref-v2.2.2"
    firmware_capabilities = (
        FakeMcuTransport.firmware_capabilities
        & ~CAPABILITY_CAN_TIMING_PROFILE
    )


def test_dummy_robot_fake_mcu_closed_loop(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    with robot:
        assert robot.firmware_version == "fake-mcu-v2.2"
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


def test_control_acquire_rejects_feedback_outside_hard_limits(config) -> None:
    transport = FakeMcuTransport(config)
    transport._position[1] = np.float32(config.joint_limit_min_rad[1] - 0.01)
    robot = DummyRobot(config, transport, connect_timeout_s=0.1)

    with robot:
        with pytest.raises(RobotError, match=r"joint2=.*outside"):
            robot.acquire_control(ControlMode.TELEOP)
        assert robot.read_state().mode == ControlMode.HOLD
        assert not transport._lease


def test_control_acquire_waits_for_current_can_stream_window(config) -> None:
    transport = FakeMcuTransport(config)
    robot = DummyRobot(config, transport)

    with robot:
        original_read = robot.read_can_diagnostics
        reads = 0

        def delayed_window():
            nonlocal reads
            reads += 1
            current = original_read()
            if reads < 3:
                return replace(
                    current,
                    session_epoch=0,
                    motor_marker_mask=0,
                    window_flags=0,
                )
            return replace(
                current,
                session_epoch=robot.session_id,
                motor_marker_mask=0x7F,
                window_flags=CAN_DIAGNOSTICS_WINDOW_VALID,
            )

        robot.read_can_diagnostics = delayed_window  # type: ignore[method-assign]
        robot.acquire_control(ControlMode.TELEOP)
        assert reads >= 3


def test_failed_can_stream_transition_holds_and_releases_lease(config) -> None:
    transport = FakeMcuTransport(config)
    robot = DummyRobot(config, transport, connect_timeout_s=0.2)

    with robot:
        original_read = robot.read_can_diagnostics

        def failed_window():
            current = original_read()
            transport._mode = ControlMode.HOLD
            transport._hold_reason_bits |= int(HoldReasonBits.RUNTIME_LIMIT)
            transport._emit_state(0)
            return replace(
                current,
                session_epoch=0,
                motor_marker_mask=0,
                window_flags=0,
            )

        robot.read_can_diagnostics = failed_window  # type: ignore[method-assign]
        with pytest.raises(RobotError, match="CAN stream transition"):
            robot.acquire_control(ControlMode.TELEOP)
        assert robot.read_state().mode == ControlMode.HOLD
        assert not transport._lease


def test_runtime_hold_during_set_mode_fails_early_with_can_markers(config) -> None:
    transport = RuntimeHoldOnSetModeTransport(config)
    robot = DummyRobot(config, transport, response_timeout_s=0.5)

    with robot:
        started = time.monotonic()
        with pytest.raises(
            RobotError,
            match=r"CAN stream transition failed before TELEOP.*motor_marker_mask=0x7f",
        ):
            robot.acquire_control(ControlMode.TELEOP)
        assert time.monotonic() - started < 0.3
        assert robot.read_state().mode == ControlMode.HOLD
        assert not transport._lease


def test_protocol_v5_time_sync_and_can_diagnostics(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    with robot:
        exchange = robot.time_sync()
        assert exchange.host_t3_ns >= exchange.host_t0_ns
        assert exchange.mcu_tx_us >= exchange.mcu_rx_us
        assert exchange.rtt_ns >= 0
        diagnostics = robot.read_can_diagnostics()
        assert len(diagnostics.target_tx_complete) == 7
        assert diagnostics.window_duration_us >= 0


def test_can_diagnostics_requests_are_single_flight(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))

    with robot:
        original_request = robot._request
        first_entered = Event()
        release_first = Event()
        metric_lock = Lock()
        active = 0
        max_active = 0
        errors: list[BaseException] = []

        def monitored_request(message_type, payload=b""):
            nonlocal active, max_active
            if message_type is not MessageType.GET_CAN_DIAGNOSTICS:
                return original_request(message_type, payload)
            with metric_lock:
                active += 1
                max_active = max(max_active, active)
                is_first = active == 1
            if is_first:
                first_entered.set()
                release_first.wait(0.1)
            else:
                release_first.set()
            try:
                return original_request(message_type, payload)
            finally:
                with metric_lock:
                    active -= 1

        robot._request = monitored_request  # type: ignore[method-assign]

        def read_diagnostics() -> None:
            try:
                robot.read_can_diagnostics()
            except BaseException as exc:
                errors.append(exc)

        first = Thread(target=read_diagnostics)
        second = Thread(target=read_diagnostics)
        first.start()
        assert first_entered.wait(0.5)
        second.start()
        first.join(timeout=1.0)
        second.join(timeout=1.0)

        assert not first.is_alive() and not second.is_alive()
        assert errors == []
        assert max_active == 1


def test_target_keepalive_is_exact_and_heartbeat_does_not_refresh_target(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    with robot:
        robot.acquire_control(ControlMode.TELEOP)
        target = robot.read_state().position.copy()
        target[0] += np.float32(0.002)
        action = robot.send_action(target)

        time.sleep(0.06)
        robot.refresh_target(action.sequence, robot.advance_control_tick())
        time.sleep(0.06)
        assert robot.read_state().mode == ControlMode.TELEOP
        with pytest.raises(CommandRejected, match="BAD_SEQUENCE"):
            robot.refresh_target(
                (action.sequence + 1) & 0xFFFFFFFF or 1,
                robot.advance_control_tick(),
            )

        # A lease heartbeat is intentionally unable to keep that motion target
        # alive. Without another control-bound refresh, the configured target TTL
        # still fails closed into HOLD.
        robot.heartbeat()
        deadline = time.monotonic() + 0.3
        while robot.read_state().mode != ControlMode.HOLD and time.monotonic() < deadline:
            time.sleep(0.01)
        state = robot.read_state()
        assert state.mode == ControlMode.HOLD
        assert state.hold_reason_bits & int(HoldReasonBits.TARGET_TIMEOUT)


def test_freshness_token_rejects_duplicate_and_backward_values(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    with robot:
        robot.acquire_control(ControlMode.TELEOP)
        target = robot.read_state().position.copy()
        target[0] += np.float32(0.002)
        action = robot.enqueue_absolute_action(target, source="freshness-token")
        refresh_tick = robot.advance_control_tick()
        robot.refresh_target(action.sequence, refresh_tick)
        with pytest.raises(CommandRejected, match="BAD_SEQUENCE"):
            robot.refresh_target(action.sequence, refresh_tick)
        with pytest.raises(CommandRejected, match="BAD_SEQUENCE"):
            robot.refresh_target(action.sequence, (refresh_tick - 1) or 0xFFFFFFFF)


def test_action_credit_is_released_only_by_exact_can_completion(config) -> None:
    robot = DummyRobot(config, AckWithoutExactCanTransport(config))
    with robot:
        robot.acquire_control(ControlMode.TELEOP)
        target = robot.read_state().position.copy()
        target[0] += np.float32(0.002)
        action = robot.enqueue_absolute_action(target, source="credit")
        assert robot.reserve_action_credit(robot.advance_control_tick()) is None

        robot._emit_action_stage(
            action.sequence,
            ActionStage.CAN_TX_COMPLETE_EXACT,
            robot.clock_ns(),
        )
        credit = robot.reserve_action_credit(robot.advance_control_tick())
        assert credit is not None
        robot.cancel_action_credit(credit)


def test_rejected_candidate_consumes_neither_sequence_nor_credit(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    with robot:
        robot.acquire_control(ControlMode.TELEOP)
        robot._sequence = 41
        credit = robot.reserve_action_credit(robot.advance_control_tick())
        assert credit is not None
        with pytest.raises(SafetyError, match="not valid"):
            robot.enqueue_absolute_action(
                robot.read_state().position.copy(),
                source="expired",
                generated_at_ns=0,
                action_credit=credit,
            )
        assert robot._sequence == 41
        replacement = robot.reserve_action_credit(robot.advance_control_tick())
        assert replacement is not None
        robot.cancel_action_credit(replacement)


def test_protocol_v4_firmware_is_rejected_by_v5_host(config) -> None:
    robot = DummyRobot(config, OldV21WithoutMultiChannelSequence(config))
    with pytest.raises(ConfigError, match="protocol v5"):
        robot.connect()
    assert not robot.is_connected


def test_v22_firmware_is_rejected_by_v222_host(config) -> None:
    robot = DummyRobot(config, OldV22WithProtocolV5(config))
    with pytest.raises(ConfigError, match="dummy-ref-v2.2.2 exactly"):
        robot.connect()
    assert not robot.is_connected


def test_v222_firmware_without_diagnostics_v2_is_rejected(config) -> None:
    robot = DummyRobot(config, V222WithoutCanDiagnosticsV2(config))
    with pytest.raises(ConfigError, match="missing required protocol-v5"):
        robot.connect()
    assert not robot.is_connected


def test_v222_firmware_without_can_timing_profile_is_rejected(config) -> None:
    robot = DummyRobot(config, V222WithoutCanTimingProfile(config))
    with pytest.raises(ConfigError, match="missing required protocol-v5"):
        robot.connect()
    assert not robot.is_connected


def test_acceptance_gate_requires_explicit_session_and_blocks_policy(config) -> None:
    robot = DummyRobot(config, CurrentV222RealTransport(config))
    with robot:
        with pytest.raises(ConfigError, match="explicit acceptance session"):
            robot.acquire_control(ControlMode.TELEOP)

    robot = DummyRobot(
        config,
        CurrentV222RealTransport(config),
        acceptance_session=True,
    )
    with robot:
        robot.acquire_control(ControlMode.TELEOP)

    robot = DummyRobot(
        config,
        CurrentV222RealTransport(config),
        acceptance_session=True,
    )
    with robot:
        with pytest.raises(ConfigError, match="POLICY execution is not production-ready"):
            robot.acquire_control(ControlMode.POLICY)


def test_production_gate_allows_real_policy(config) -> None:
    production = replace(
        config,
        external_target_execution_ready=True,
        external_target_acceptance_ready=False,
    )
    robot = DummyRobot(production, CurrentV222RealTransport(production))
    with robot:
        robot.acquire_control(ControlMode.POLICY)


def test_state_replay_recovers_complete_action_when_every_event_is_lost(config) -> None:
    robot = DummyRobot(config, StateReplayOnlyTransport(config))
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
        assert ActionStage.CAN_TX_COMPLETE_EXACT in observed
        assert ActionStage.POST_COMMAND_FEEDBACK in observed


def test_action_progress_from_an_old_session_epoch_is_ignored(config) -> None:
    transport = AckWithoutExactCanTransport(config)
    robot = DummyRobot(config, transport)
    stages = []
    robot.set_action_lifecycle_listener(stages.append)
    with robot:
        robot.acquire_control(ControlMode.TELEOP)
        target = robot.read_state().position.copy()
        target[0] += np.float32(0.005)
        applied = robot.enqueue_absolute_action(target, source="old-epoch")
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline and not any(
            update.sequence == applied.sequence
            and update.stage is ActionStage.ACKNOWLEDGED
            for update in stages
        ):
            time.sleep(0.005)

        stale_epoch = robot.session_id % 0xFFFFFFFF + 1
        transport._rx.put(
            Packet(
                MessageType.EVENT,
                stale_epoch,
                applied.sequence,
                robot.clock_ns() // 1_000,
                ACTION_PROGRESS.pack(
                    applied.sequence,
                    int(ActionProgressStage.CAN_TX_COMPLETE_EXACT),
                    robot.clock_ns() // 1_000,
                    0,
                ),
            )
        )
        time.sleep(0.02)
        observed = {
            update.stage for update in stages if update.sequence == applied.sequence
        }
        assert ActionStage.ACKNOWLEDGED in observed
        assert ActionStage.CAN_TX_COMPLETE_EXACT not in observed


def test_exact_progress_exposes_firmware_fanout_measurement(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    updates = []
    robot.set_action_lifecycle_listener(updates.append)
    sequence = 123
    robot._prepare_action_sequence(sequence)
    robot._emit_action_stage(sequence, ActionStage.RECEIVED, robot.clock_ns())
    updates.clear()

    robot._apply_action_progress(
        sequence,
        ActionProgressStage.CAN_TX_COMPLETE_EXACT,
        1000,
        4321,
    )

    assert updates[-1].stage is ActionStage.CAN_TX_COMPLETE_EXACT
    assert updates[-1].measurement_us == 4321


def test_late_reliable_exact_event_enriches_state_replay_measurement(config) -> None:
    robot = DummyRobot(config, FakeMcuTransport(config))
    updates = []
    robot.set_action_lifecycle_listener(updates.append)
    sequence = 124
    robot._prepare_action_sequence(sequence)
    robot._emit_action_stage(sequence, ActionStage.RECEIVED, robot.clock_ns())
    updates.clear()
    robot._apply_action_progress(
        sequence, ActionProgressStage.CAN_TX_COMPLETE_EXACT, 1000, 0
    )
    robot._apply_action_progress(
        sequence, ActionProgressStage.CAN_TX_COMPLETE_EXACT, 1000, 987
    )
    robot._apply_action_progress(
        sequence, ActionProgressStage.CAN_TX_COMPLETE_EXACT, 1000, 987
    )

    assert len(updates) == 2
    assert updates[0].measurement_us == 0
    assert updates[1].measurement_us == 987


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
