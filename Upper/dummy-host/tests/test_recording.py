from __future__ import annotations

import json
import sqlite3
from dataclasses import replace
from pathlib import Path

import numpy as np

from dummy_host.cameras import CameraFrame
from dummy_host.apps.session_check import check_session
from dummy_host.recording import SessionRecorder
from dummy_host.schema import (
    AppliedAction,
    ControlMode,
    RobotConfig,
    RobotState,
    load_camera_rig_config,
)
from dummy_host.teleop import KeyboardMapper, load_teleop_profile


def test_session_recorder_writes_recoverable_control_and_camera_data(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")
    now_ns = 5_000_000_000
    state = RobotState(
        position=np.concatenate((config.initial_pose_rad, np.asarray([0.5], dtype=np.float32))),
        velocity=np.zeros(7, dtype=np.float32),
        monotonic_ns=now_ns,
        mcu_time_us=now_ns // 1_000,
        mode=ControlMode.TELEOP,
        fault_bits=0,
        position_valid=True,
        velocity_valid=True,
        gripper_valid=True,
        last_received_sequence=7,
        last_applied_sequence=7,
        target_age_ms=5,
        config_hash=config.config_hash,
    )
    command = KeyboardMapper(profile).map({"KEY_SPACE", "KEY_Q"}, now_ns)
    action = AppliedAction(
        requested=state.position.copy(),
        applied=state.position.copy(),
        sequence=7,
        monotonic_ns=now_ns,
        clipped=False,
        reasons=(),
    )
    frame = CameraFrame(
        color=np.zeros((2, 3, 3), dtype=np.uint8),
        depth=np.ones((2, 3), dtype=np.uint16),
        capture_time_ns=now_ns - 1_000_000,
        arrival_time_ns=now_ns,
        device_timestamp_ms=12.0,
        frame_number=42,
        depth_device_timestamp_ms=12.1,
        depth_frame_number=43,
        color_depth_skew_ms=0.1,
    )
    global_frame = CameraFrame(
        color=np.zeros((2, 3, 3), dtype=np.uint8),
        depth=None,
        capture_time_ns=now_ns - 2_000_000,
        arrival_time_ns=now_ns,
        device_timestamp_ms=float("nan"),
        frame_number=9,
        depth_device_timestamp_ms=float("nan"),
        depth_frame_number=0,
        color_depth_skew_ms=0.0,
        role="global",
        calibration_version="global-test-v1",
    )
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_test",
        queue_size=8,
    )
    recorder.update_runtime_metadata(firmware_version="fake-mcu-v1")
    recorder.record_event("episode_start", monotonic_ns=now_ns)
    frames = {"wrist": frame, "global": global_frame}
    recorder.record_sample(command, state, action=action, camera_frames=frames)
    recorder.record_sample(command, state, action=action, camera_frames=frames)
    stats = recorder.close()

    assert stats.samples == 2
    assert stats.events == 1
    assert stats.camera_frames == 2
    with sqlite3.connect(recorder.db_path) as connection:
        row = connection.execute(
            "SELECT COUNT(*), action_sequence, length(applied_action), last_applied_sequence, "
            "length(state_following_error), length(feedback_age_ms), "
            "length(node_fault_bits), state_hold_reason_bits "
            "FROM samples"
        ).fetchone()
        camera_rows = connection.execute(
            "SELECT role, COUNT(*), COUNT(DISTINCT frame_path) "
            "FROM camera_samples GROUP BY role ORDER BY role"
        ).fetchall()
    assert row == (2, 7, 28, 7, 28, 28, 14, 0)
    assert camera_rows == [("global", 2, 1), ("wrist", 2, 1)]
    manifest = json.loads(recorder.manifest_path.read_text(encoding="utf-8"))
    assert manifest["clean_shutdown"] is True
    assert manifest["firmware_version"] == "fake-mcu-v1"
    assert manifest["robot_config_hash"] == config.config_hash
    assert manifest["schema_version"] == 2
    assert manifest["state_telemetry_version"] == 2
    assert manifest["camera_rig_hash"] == config.camera_rig.config_hash
    frame_files = list((recorder.session_dir / "frames").rglob("*.npz"))
    assert len(frame_files) == 2
    checksums = json.loads(recorder.checksums_path.read_text(encoding="utf-8"))
    assert "samples.sqlite" in checksums["files"]
    for frame_file in frame_files:
        assert frame_file.relative_to(recorder.session_dir).as_posix() in checksums["files"]
    report = check_session(recorder.session_dir)
    assert report.ok
    assert report.integrity == "ok"
    assert report.samples == 2
    assert report.camera_files == 2
    assert report.camera_frames_referenced == 2


def test_session_archives_camera_calibration_files(config: RobotConfig, tmp_path: Path) -> None:
    project = Path(__file__).parents[1]
    profile = load_teleop_profile(project / "configs" / "teleop_inputs.yaml")
    rig = load_camera_rig_config(project / "configs" / "camera_rig_dual.example.yaml")
    recorder = SessionRecorder(
        tmp_path,
        replace(config, camera_rig=rig),
        profile,
        source="keyboard",
        session_name="session_calibrations",
    )
    recorder.close()
    manifest = json.loads(recorder.manifest_path.read_text(encoding="utf-8"))
    assert set(manifest["camera_calibrations"]) == {"wrist", "global"}
    for role, record in manifest["camera_calibrations"].items():
        archived = recorder.session_dir / record["archive_path"]
        assert archived.is_file(), role
        assert record["sha256"] == rig.calibrations[role].file_hash
    assert check_session(recorder.session_dir).ok
