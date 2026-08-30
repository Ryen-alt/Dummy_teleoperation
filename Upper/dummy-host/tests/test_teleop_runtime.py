from __future__ import annotations

from pathlib import Path
import sqlite3
import time
from types import SimpleNamespace

import numpy as np
import pytest

from dummy_host.domain import EpisodeManager, EpisodeStatus
from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.protocol import MessageType, unpack_joint_target
from dummy_host.recording import RecorderBackpressure, SessionRecorder
from dummy_host.robot_driver import DummyRobot
from dummy_host.scheduler import ScheduledTick
from dummy_host.schema import RobotConfig
from dummy_host.teleop import (
    JointVelocityIntegrator,
    KeyboardMapper,
    TeleopCommand,
    TeleopError,
    load_teleop_profile,
)
from dummy_host.teleop_runtime import _LeaseCoordinator, run_teleop_collection


class ScriptedKeyboard:
    def __init__(self, mapper: KeyboardMapper) -> None:
        self.mapper = mapper
        self.polls = 0
        self.closed = False

    def poll(self, now_ns: int | None = None) -> TeleopCommand:
        assert now_ns is not None
        self.polls += 1
        if 2 <= self.polls <= 9:
            keys = {"KEY_SPACE", "KEY_Q"}
            if self.polls == 2:
                keys.add("KEY_F5")
        else:
            keys = set()
        return self.mapper.map(keys, now_ns)

    def close(self) -> None:
        self.closed = True


class IdleKeyboard(ScriptedKeyboard):
    def poll(self, now_ns: int | None = None) -> TeleopCommand:
        assert now_ns is not None
        self.polls += 1
        return self.mapper.map(set(), now_ns)


class StrayEpisodeFailureKeyboard(ScriptedKeyboard):
    def poll(self, now_ns: int | None = None) -> TeleopCommand:
        assert now_ns is not None
        self.polls += 1
        keys: set[str] = set()
        if 2 <= self.polls <= 10:
            keys = {"KEY_SPACE", "KEY_Q"}
        if self.polls == 5:
            keys.add("KEY_F7")
        return self.mapper.map(keys, now_ns)


class SuccessfulEpisodeKeyboard(ScriptedKeyboard):
    def poll(self, now_ns: int | None = None) -> TeleopCommand:
        assert now_ns is not None
        self.polls += 1
        keys: set[str] = set()
        if 2 <= self.polls <= 10:
            keys = {"KEY_SPACE", "KEY_Q"}
        if self.polls == 2:
            keys.add("KEY_F5")
        if self.polls == 9:
            keys.add("KEY_F6")
        return self.mapper.map(keys, now_ns)


class FrozenTimestampKeyboard(ScriptedKeyboard):
    def __init__(self, mapper: KeyboardMapper) -> None:
        super().__init__(mapper)
        self.command: TeleopCommand | None = None

    def poll(self, now_ns: int | None = None) -> TeleopCommand:
        assert now_ns is not None
        self.polls += 1
        if self.command is None:
            self.command = self.mapper.map({"KEY_SPACE", "KEY_Q"}, now_ns)
        return self.command


class FrozenIdleTimestampKeyboard(FrozenTimestampKeyboard):
    def poll(self, now_ns: int | None = None) -> TeleopCommand:
        assert now_ns is not None
        self.polls += 1
        if self.command is None:
            self.command = self.mapper.map(set(), now_ns)
        return self.command


class AdvancingClock:
    def __init__(self, *, step_ns: int = 50_000_000) -> None:
        self.value = 0
        self.step_ns = step_ns

    def __call__(self) -> int:
        current = self.value
        self.value += self.step_ns
        return current


class DelayedExactFanoutTransport(FakeMcuTransport):
    """Hide exact fan-out progress long enough to require target refreshes."""

    def __init__(
        self,
        config: RobotConfig,
        *,
        hidden_states_per_target: int = 3,
        delayed_target_count: int = 1,
    ) -> None:
        super().__init__(config)
        self.hidden_states_per_target = hidden_states_per_target
        self.delayed_targets_remaining = delayed_target_count
        self.hidden_sequence: int | None = None
        self.hidden_states_remaining = 0
        self.target_keepalives = 0
        self.lease_heartbeats = 0
        self.target_overlap = False
        self.target_ttls_ms: list[int] = []

    def send(self, packet) -> None:
        if packet.message_type is MessageType.SET_JOINT_TARGET:
            if self.hidden_sequence is not None:
                self.target_overlap = True
            if self.delayed_targets_remaining > 0:
                self.hidden_sequence = packet.sequence
                self.hidden_states_remaining = self.hidden_states_per_target
                self.delayed_targets_remaining -= 1
            _, _, ttl_ms, _, _ = unpack_joint_target(packet.payload)
            self.target_ttls_ms.append(ttl_ms)
        elif packet.message_type is MessageType.TARGET_KEEPALIVE:
            self.target_keepalives += 1
        elif packet.message_type is MessageType.HEARTBEAT:
            self.lease_heartbeats += 1
        super().send(packet)

    def _emit_state(self, sequence: int) -> None:
        hidden = self.hidden_sequence
        if hidden is not None and self.hidden_states_remaining > 0:
            saved_progress = self._progress
            saved_applied = self._last_applied
            self._progress = [
                record for record in saved_progress if record.sequence != hidden
            ]
            self._last_applied = 0
            try:
                super()._emit_state(sequence)
            finally:
                self._progress = saved_progress
                self._last_applied = saved_applied
                self.hidden_states_remaining -= 1
            return
        super()._emit_state(sequence)
        if hidden is not None:
            self.hidden_sequence = None


class RecordingStartupOrderTransport(FakeMcuTransport):
    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)
        self.sent_types: list[MessageType] = []

    def send(self, packet) -> None:
        self.sent_types.append(packet.message_type)
        super().send(packet)


def test_lease_coordinator_holds_within_75_ms_when_control_ticks_stop() -> None:
    class Robot:
        config = SimpleNamespace(lease_timeout_ms=500)

        def __init__(self) -> None:
            self.priority_holds = 0

        def acquire_control(self, _mode) -> None:
            pass

        def heartbeat(self) -> None:
            pass

        def refresh_target(self, _sequence: int, _tick: int) -> None:
            pass

        def request_priority_hold(self) -> None:
            self.priority_holds += 1

        def hold(self) -> None:
            pass

        def release_control(self) -> None:
            pass

        def emergency_stop(self) -> None:
            pass

    robot = Robot()
    events: list[str] = []
    coordinator = _LeaseCoordinator(
        robot,  # type: ignore[arg-type]
        clock_ns=time.monotonic_ns,
        event_callback=lambda event, _now, _payload: events.append(event),
    )
    coordinator.start()
    coordinator.request("teleop")
    deadline = time.monotonic() + 0.3
    while not coordinator.snapshot()[0] and time.monotonic() < deadline:
        time.sleep(0.002)
    assert coordinator.snapshot()[0]
    coordinator.note_control_tick(1, time.monotonic_ns())

    deadline = time.monotonic() + 0.3
    while robot.priority_holds == 0 and time.monotonic() < deadline:
        time.sleep(0.002)
    acquired, error = coordinator.snapshot()
    coordinator.close()

    assert not acquired
    assert isinstance(error, TeleopError)
    assert robot.priority_holds == 1
    assert "control_health_timeout" in events


def test_keyboard_fake_mcu_collection_closes_in_hold(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")
    source = ScriptedKeyboard(KeyboardMapper(profile))
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_runtime",
    )
    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=0.56,
    )
    recorder_stats = recorder.close()
    # WAIT_FEEDBACK_READY and ACQUIRE run on the lease thread; the 20 Hz
    # control thread remains responsive and only emits targets afterwards.
    assert result.actions_sent >= 1
    assert result.hold_transitions >= 1
    assert result.episode_events == 1
    assert result.final_mode == "HOLD"
    assert (
        result.final_post_command_feedback_sequence
        == result.final_received_sequence
    )
    assert recorder_stats.samples >= 4
    with sqlite3.connect(recorder.db_path) as connection:
        lifecycle = connection.execute(
            "SELECT COUNT(*), COUNT(post_command_feedback_host_ns) "
            "FROM action_lifecycle"
        ).fetchone()
    assert lifecycle is not None
    assert lifecycle[0] >= 1
    assert lifecycle[1] >= 1
    assert source.closed
    assert not robot.is_connected


def test_runtime_single_delayed_exact_fanout_defers_without_ending_session(
    config: RobotConfig, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    source = ScriptedKeyboard(KeyboardMapper(profile))
    transport = DelayedExactFanoutTransport(config, hidden_states_per_target=1)
    deferred_integrator_calls = 0
    real_advance = JointVelocityIntegrator.advance_without_motion

    def record_deferred_integrator(self, now_ns: int) -> None:
        nonlocal deferred_integrator_calls
        deferred_integrator_calls += 1
        real_advance(self, now_ns)

    monkeypatch.setattr(
        JointVelocityIntegrator,
        "advance_without_motion",
        record_deferred_integrator,
    )
    robot = DummyRobot(config, transport)
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_delayed_exact",
    )

    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=0.7,
    )
    recorder.close()

    assert result.actions_sent >= 2
    assert result.action_credit_misses == 1
    assert transport.target_keepalives == 1
    assert transport.lease_heartbeats <= 5
    assert not transport.target_overlap
    assert deferred_integrator_calls == 1
    assert transport.target_ttls_ms
    assert set(transport.target_ttls_ms) == {200}
    assert config.lease_timeout_ms == 500
    assert result.final_mode == "HOLD"
    events = recorder.events_path.read_text(encoding="utf-8")
    assert '"event":"action_credit_deferred"' in events
    assert '"reason":"action_credit_miss"' not in events
    with sqlite3.connect(recorder.db_path) as connection:
        superseded = connection.execute(
            "SELECT COUNT(*) FROM action_lifecycle WHERE terminal_stage = 'superseded'"
        ).fetchone()
    assert superseded == (0,)


def test_runtime_sustained_exact_fanout_delay_holds_after_second_miss(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    source = ScriptedKeyboard(KeyboardMapper(profile))
    transport = DelayedExactFanoutTransport(
        config, hidden_states_per_target=100
    )
    robot = DummyRobot(config, transport)
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_sustained_delayed_exact",
    )

    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=0.8,
    )
    recorder.close()

    assert result.actions_sent == 1
    assert result.action_credit_misses == 2
    assert not transport.target_overlap
    events = recorder.events_path.read_text(encoding="utf-8")
    assert events.count('"event":"action_credit_deferred"') == 1
    assert events.count('"event":"action_credit_hold"') == 1
    assert '"reason":"action_credit_timeout"' in events
    with sqlite3.connect(recorder.db_path) as connection:
        fabricated_exact = connection.execute(
            "SELECT COUNT(*) FROM action_lifecycle "
            "WHERE can_tx_complete_exact_host_ns IS NOT NULL"
        ).fetchone()
        superseded = connection.execute(
            "SELECT COUNT(*) FROM action_lifecycle "
            "WHERE terminal_stage = 'superseded'"
        ).fetchone()
    assert fabricated_exact == (0,)
    assert superseded == (0,)


def test_control_overrun_drops_one_frame_then_holds_if_it_persists(
    config: RobotConfig,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    source = ScriptedKeyboard(KeyboardMapper(profile))
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_control_overrun_degradation",
    )

    def scripted_run(scheduler, callback, stop):
        # Two ticks establish the lease/integrator. A transient missed period
        # recovers; the final two consecutive misses must escalate to HOLD.
        for index, missed_periods in enumerate((0, 0, 0, 1, 0, 1, 1)):
            now_ns = time.monotonic_ns()
            callback(
                ScheduledTick(
                    index,
                    now_ns,
                    now_ns,
                    missed_periods,
                    now_ns + scheduler.period_ns,
                )
            )
            time.sleep(0.055)
        return scheduler.stats()

    monkeypatch.setattr(
        "dummy_host.teleop_runtime.FixedRateScheduler.run_timed",
        scripted_run,
    )
    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=0.8,
    )
    recorder.close()

    events = recorder.events_path.read_text(encoding="utf-8")
    assert events.count('"event":"control_timing_deferred"') == 2
    assert events.count('"event":"control_timing_recovered"') == 1
    assert events.count('"event":"control_timing_overrun"') == 1
    assert result.hold_transitions >= 1


def test_idle_fake_mcu_collection_keeps_state_fresh(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")
    source = IdleKeyboard(KeyboardMapper(profile))
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_idle_runtime",
    )
    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=0.31,
    )
    recorder.close()
    assert result.actions_sent == 0
    assert result.final_mode == "HOLD"
    assert source.closed
    assert not robot.is_connected
    with sqlite3.connect(recorder.db_path) as connection:
        exchanges = connection.execute(
            "SELECT COUNT(*) FROM time_sync_exchanges"
        ).fetchone()
        diagnostics = connection.execute(
            "SELECT COUNT(*) FROM can_diagnostics"
        ).fetchone()
    assert exchanges is not None and exchanges[0] >= 1
    assert diagnostics is not None and diagnostics[0] >= 1


def test_runtime_enters_hold_before_starting_evidence_traffic(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    source = IdleKeyboard(KeyboardMapper(profile))
    transport = RecordingStartupOrderTransport(config)
    robot = DummyRobot(config, transport)
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_startup_order",
    )

    run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=0.31,
    )
    recorder.close()

    initial_hold = transport.sent_types.index(MessageType.HOLD)
    assert initial_hold < transport.sent_types.index(MessageType.TIME_SYNC)
    assert initial_hold < transport.sent_types.index(MessageType.GET_CAN_DIAGNOSTICS)


def test_stray_episode_failure_while_idle_does_not_abort_teleoperation(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    source = StrayEpisodeFailureKeyboard(KeyboardMapper(profile))
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_stray_episode_failure",
    )

    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=0.7,
    )
    recorder.close()

    events = recorder.events_path.read_text(encoding="utf-8")
    assert result.actions_sent >= 1
    assert '"event":"episode_transition_ignored"' in events
    assert '"event":"episode_transition_rejected"' not in events
    assert result.final_mode == "HOLD"


def test_required_camera_cannot_be_silently_omitted(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")
    source = ScriptedKeyboard(KeyboardMapper(profile))
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_missing_camera",
    )
    with pytest.raises(ValueError, match="configured camera"):
        run_teleop_collection(
            robot,
            source,
            recorder,
            profile,
            require_camera=True,
        )
    recorder.close(clean_shutdown=False)
    assert source.closed


def test_frozen_input_timestamp_deterministically_holds_after_150_ms(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")
    assert profile.input_timeout_ms == 150
    source = FrozenTimestampKeyboard(KeyboardMapper(profile))
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_frozen_input",
    )
    with pytest.raises(TeleopError, match="input command is stale"):
        run_teleop_collection(
            robot,
            source,
            recorder,
            profile,
            duration_s=2.0,
            clock_ns=AdvancingClock(),
        )
    recorder.close(clean_shutdown=False)
    assert source.closed
    assert not robot.is_connected
    assert '"event":"input_timeout"' in recorder.events_path.read_text(encoding="utf-8")


def test_frozen_idle_input_is_invalid_but_does_not_abort_collection(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    source = FrozenIdleTimestampKeyboard(KeyboardMapper(profile))
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_frozen_idle_input",
    )

    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=1.0,
        clock_ns=AdvancingClock(),
    )
    recorder.close()

    assert result.actions_sent == 0
    assert result.final_mode == "HOLD"
    events = recorder.events_path.read_text(encoding="utf-8")
    assert events.count('"event":"input_timeout"') == 1
    with sqlite3.connect(recorder.db_path) as connection:
        invalid = connection.execute(
            "SELECT COUNT(*) FROM samples WHERE sample_valid = 0 "
            "AND invalid_reason LIKE 'input command stale:%'"
        ).fetchone()
    assert invalid is not None and invalid[0] >= 1


def test_episode_success_waits_for_exact_final_action_watermarks(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    source = SuccessfulEpisodeKeyboard(KeyboardMapper(profile))
    robot = DummyRobot(config, FakeMcuTransport(config))
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_finalize_success",
    )

    result = run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=0.8,
    )
    recorder.close()

    events = recorder.events_path.read_text(encoding="utf-8")
    assert '"event":"episode_finalizing"' in events
    assert '"event":"episode_success"' in events
    assert events.index('"event":"episode_finalizing"') < events.index(
        '"event":"episode_success"'
    )
    assert result.episode_events >= 3


def test_recorder_backpressure_holds_and_fails_active_episode(
    config: RobotConfig, tmp_path: Path
) -> None:
    class AuditedFakeMcu(FakeMcuTransport):
        def __init__(self, robot_config: RobotConfig) -> None:
            super().__init__(robot_config)
            self.sent_types: list[MessageType] = []

        def send(self, packet) -> None:
            self.sent_types.append(packet.message_type)
            super().send(packet)

    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    source = ScriptedKeyboard(KeyboardMapper(profile))
    transport = AuditedFakeMcu(config)
    robot = DummyRobot(config, transport)
    episodes = EpisodeManager(id_factory=lambda: "backpressure-episode")
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_runtime_backpressure",
    )
    real_record_sample = recorder.record_sample
    calls = 0

    def fail_after_episode_starts(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 6:
            raise RecorderBackpressure("injected recorder saturation")
        return real_record_sample(*args, **kwargs)

    recorder.record_sample = fail_after_episode_starts  # type: ignore[method-assign]
    with pytest.raises(RecorderBackpressure, match="injected recorder saturation"):
        run_teleop_collection(
            robot,
            source,
            recorder,
            profile,
            duration_s=2.0,
            episode_manager=episodes,
        )
    recorder.close(clean_shutdown=False)

    assert episodes.snapshot.status is EpisodeStatus.FAILED
    assert "runtime_error" in (episodes.snapshot.failure_reason or "")
    assert MessageType.HOLD in transport.sent_types
    events = recorder.events_path.read_text(encoding="utf-8")
    assert '"event":"episode_failure"' in events
    assert '"event":"episode_cancel"' not in events


def test_episode_finalizing_times_out_without_post_command_feedback(
    config: RobotConfig, tmp_path: Path
) -> None:
    class ExactCanWithoutPostFeedback(FakeMcuTransport):
        def _emit_state(self, sequence: int) -> None:
            applied = self._last_applied
            self._last_applied = 0
            try:
                super()._emit_state(sequence)
            finally:
                self._last_applied = applied

    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    source = SuccessfulEpisodeKeyboard(KeyboardMapper(profile))
    robot = DummyRobot(
        config,
        ExactCanWithoutPostFeedback(config),
        action_observation_timeout_s=1.0,
    )
    episodes = EpisodeManager(id_factory=lambda: "finalize-timeout-episode")
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_finalize_timeout",
    )

    run_teleop_collection(
        robot,
        source,
        recorder,
        profile,
        duration_s=1.0,
        episode_manager=episodes,
    )
    recorder.close()

    assert episodes.snapshot.status is EpisodeStatus.FAILED
    assert episodes.snapshot.failure_reason == "episode_action_completion_timeout"
    events = recorder.events_path.read_text(encoding="utf-8")
    assert '"event":"episode_finalizing"' in events
    assert '"event":"episode_finalize_timeout"' in events
    assert '"event":"episode_success"' not in events
