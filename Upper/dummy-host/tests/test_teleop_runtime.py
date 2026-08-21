from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.recording import SessionRecorder
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
        if 2 <= self.polls <= 4:
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
        duration_s=0.31,
    )
    recorder_stats = recorder.close()
    assert result.actions_sent >= 2
    assert result.hold_transitions >= 1
    assert result.episode_events == 1
    assert result.final_mode == "HOLD"
    assert result.final_applied_sequence == result.final_received_sequence
    assert recorder_stats.samples >= 4
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
