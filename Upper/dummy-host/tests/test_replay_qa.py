from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np

from dummy_host.apps.session_qa import analyze_session, render_html
from dummy_host.cameras import CameraFrame, ReplayCamera
from dummy_host.recording import SessionRecorder
from dummy_host.schema import AppliedAction, ControlMode, RobotConfig, RobotState
from dummy_host.teleop import KeyboardMapper, load_teleop_profile


def _record_replay_source(config: RobotConfig, root: Path) -> Path:
    profile = load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")
    start_ns = 9_000_000_000
    position = np.concatenate((config.initial_pose_rad, np.asarray([0.5], dtype=np.float32)))
    recorder = SessionRecorder(
        root,
        config,
        profile,
        source="keyboard",
        session_name="replay_source",
        extra_manifest={
            "camera_required": True,
            "camera_roles": ["wrist"],
        },
    )
    recorder.update_runtime_metadata(
        firmware_version="fake-mcu-v2.2", session_epoch=99
    )
    recorder.record_event(
        "episode_start",
        monotonic_ns=start_ns,
        payload={"episode_id": "episode-qa", "task_id": "qa", "task": "QA"},
    )
    mapper = KeyboardMapper(profile)
    for index in range(3):
        tick_ns = start_ns + (index + 1) * 50_000_000
        state_position = position.copy()
        state_position[0] += np.float32(index * 0.01)
        state = RobotState(
            position=state_position,
            velocity=np.zeros(7, dtype=np.float32),
            monotonic_ns=tick_ns,
            mcu_time_us=tick_ns // 1000,
            mode=ControlMode.TELEOP,
            fault_bits=0,
            position_valid=True,
            velocity_valid=False,
            gripper_valid=True,
            last_received_sequence=index + 1,
            target_age_ms=1,
            config_hash=config.config_hash,
        )
        command = mapper.map({"KEY_SPACE"}, tick_ns)
        action = AppliedAction(
            state_position.copy(), state_position.copy(), index + 1, tick_ns, False, (),
            session_epoch=99, control_tick_id=index + 1,
        )
        frame = CameraFrame(
            color=np.full((2, 3, 3), index, dtype=np.uint8),
            depth=np.full((2, 3), index, dtype=np.uint16),
            capture_time_ns=tick_ns,
            arrival_time_ns=tick_ns + 1_000_000,
            device_timestamp_ms=float(index * 50),
            frame_number=10 + index,
            depth_device_timestamp_ms=float(index * 50),
            depth_frame_number=10 + index,
            color_depth_skew_ms=0.0,
            role="wrist",
            calibration_version="uncalibrated-v0",
            depth_scale=0.001,
            timestamp_source="hardware_exposure",
        )
        recorder.record_sample(command, state, action=action, camera_frames={"wrist": frame})
    recorder.record_event(
        "episode_success",
        monotonic_ns=start_ns + 200_000_000,
        payload={"episode_id": "episode-qa", "status": "accepted"},
    )
    recorder.close()
    return recorder.session_dir


def test_replay_camera_rebases_raw_session_frames(config: RobotConfig, tmp_path: Path) -> None:
    session = _record_replay_source(config, tmp_path)
    now = [20_000_000_000]
    replay_config = replace(
        config.cameras["wrist"],
        driver="replay",
        model="raw-session-v2",
        device_serial=str(session),
        width=3,
        height=2,
        max_frame_age_ms=100,
        max_sync_skew_ms=30,
    )
    camera = ReplayCamera(replay_config, clock_ns=lambda: now[0])
    camera.start()
    first = camera.latest()
    assert first.frame_number == 10
    assert first.capture_time_ns == now[0]
    now[0] += 50_000_000
    second = camera.latest()
    assert second.frame_number == 11
    assert second.color[0, 0, 0] == 1
    assert camera.nearest(now[0]).frame_number == 11
    stats = camera.stats()
    assert stats.frames == 2
    assert stats.dropped_frames == 0
    camera.stop()


def test_session_qa_reports_export_gate_and_renders_svg(
    config: RobotConfig, tmp_path: Path
) -> None:
    session = _record_replay_source(config, tmp_path)
    report, states = analyze_session(session)
    assert report.ok
    assert not report.export_ready
    assert report.qualified_action_samples == 0
    assert report.time_sync_models == 0
    assert report.data_classification == "legacy_unspecified"
    assert not report.offline_training_only
    assert not report.real_policy_execution_allowed
    assert report.measured_sample_hz == 20.0
    assert report.episode_outcomes["accepted"] == 1
    assert report.cameras[0].role == "wrist"
    assert report.cameras[0].unique_frames == 3
    rendered = render_html(report, states)
    assert "<svg" in rendered
    assert "State trajectory" in rendered
