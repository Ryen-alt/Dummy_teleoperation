from __future__ import annotations

from pathlib import Path
import sqlite3

import numpy as np
import pytest

from dummy_host.domain import EpisodeManager, EpisodeStatus
from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.protocol import MessageType
from dummy_host.recording import RecorderBackpressure, SessionRecorder
from dummy_host.robot_driver import DummyRobot
from dummy_host.schema import RobotConfig
from dummy_host.teleop import KeyboardMapper, TeleopCommand, TeleopError, load_teleop_profile
from dummy_host.teleop_runtime import run_teleop_collection


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


class AdvancingClock:
    def __init__(self, *, step_ns: int = 50_000_000) -> None:
        self.value = 0
        self.step_ns = step_ns

    def __call__(self) -> int:
        current = self.value
        self.value += self.step_ns
        return current


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
