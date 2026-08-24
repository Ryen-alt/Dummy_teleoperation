from __future__ import annotations

import json
import sqlite3
import zipfile
from dataclasses import replace
from pathlib import Path
from threading import Event
from types import SimpleNamespace

import numpy as np
import pytest

from dummy_host.cameras import CameraFrame
from dummy_host.apps.session_check import check_session
from dummy_host.frame_archive import NpzFrameArchive
from dummy_host.recording import (
    RecorderBackpressure,
    SessionRecorder,
    estimate_camera_archive_bytes,
)
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
    for frame_file in frame_files:
        with zipfile.ZipFile(frame_file) as archive:
            assert archive.infolist()
            assert all(entry.compress_type == zipfile.ZIP_STORED for entry in archive.infolist())
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


def test_camera_archive_estimate_covers_uncompressed_arrays(config: RobotConfig) -> None:
    duration_s = 10.0
    estimate = estimate_camera_archive_bytes(config, duration_s)
    raw_bytes_per_sample = 0
    for camera in config.camera_rig.cameras.values():
        if not camera.enabled:
            continue
        pixels = camera.width * camera.height
        raw_bytes_per_sample += pixels * 3
        if camera.depth_format.strip().lower() not in {"", "none"}:
            raw_bytes_per_sample += pixels * 2
    assert estimate > raw_bytes_per_sample * config.control_rate_hz * duration_s
    with pytest.raises(ValueError, match="positive and finite"):
        estimate_camera_archive_bytes(config, 0.0)


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


def test_frame_archive_removes_partial_file_after_failed_atomic_write(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    frame = CameraFrame(
        color=np.zeros((2, 3, 3), dtype=np.uint8),
        depth=np.ones((2, 3), dtype=np.uint16),
        capture_time_ns=1,
        arrival_time_ns=2,
        device_timestamp_ms=3.0,
        frame_number=4,
        depth_device_timestamp_ms=3.1,
        depth_frame_number=5,
        color_depth_skew_ms=0.1,
    )
    archive = NpzFrameArchive(tmp_path, sync_files=False)

    def fail_after_partial_write(stream: object, **payload: object) -> None:
        del payload
        stream.write(b"partial")  # type: ignore[attr-defined]
        raise OSError("injected archive failure")

    monkeypatch.setattr("dummy_host.frame_archive.np.savez", fail_after_partial_write)
    with pytest.raises(OSError, match="injected archive failure"):
        archive.write(frame)

    assert archive.unique_frames == 0
    assert not list((tmp_path / "frames").rglob("*.npz"))
    assert not list((tmp_path / "frames").rglob("*.partial"))


def test_frame_archive_refuses_to_consume_disk_reserve(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    frame = CameraFrame(
        color=np.zeros((2, 3, 3), dtype=np.uint8),
        depth=None,
        capture_time_ns=1,
        arrival_time_ns=2,
        device_timestamp_ms=3.0,
        frame_number=4,
        depth_device_timestamp_ms=float("nan"),
        depth_frame_number=0,
        color_depth_skew_ms=0.0,
    )
    archive = NpzFrameArchive(tmp_path, minimum_free_bytes=100, sync_files=False)
    monkeypatch.setattr(
        "dummy_host.frame_archive.shutil.disk_usage",
        lambda path: SimpleNamespace(total=1_000, used=950, free=50),
    )

    with pytest.raises(OSError, match="free-space guard triggered"):
        archive.write(frame)

    assert archive.unique_frames == 0
    assert not list((tmp_path / "frames").rglob("*.npz"))


def test_sample_backpressure_preserves_capacity_for_critical_events(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")
    now_ns = 7_000_000_000
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
        last_received_sequence=1,
        last_applied_sequence=1,
        target_age_ms=1,
        config_hash=config.config_hash,
    )
    command = KeyboardMapper(profile).map({"KEY_SPACE"}, now_ns)
    frame = CameraFrame(
        color=np.zeros((2, 3, 3), dtype=np.uint8),
        depth=None,
        capture_time_ns=now_ns,
        arrival_time_ns=now_ns,
        device_timestamp_ms=0.0,
        frame_number=1,
        depth_device_timestamp_ms=float("nan"),
        depth_frame_number=0,
        color_depth_skew_ms=0.0,
    )
    writer_entered = Event()
    release_writer = Event()

    class BlockingArchive:
        unique_frames = 0

        def write(self, camera_frame: CameraFrame) -> str:
            del camera_frame
            writer_entered.set()
            assert release_writer.wait(timeout=2.0)
            return "frames/test.npz"

        def close(self) -> None:
            return

    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_backpressure_reserve",
        queue_size=6,
        frame_archive_factory=lambda session_dir: BlockingArchive(),
    )
    recorder.record_sample(command, state, camera_frame=frame)
    assert writer_entered.wait(timeout=2.0)
    recorder.record_sample(command, state)
    recorder.record_sample(command, state)
    with pytest.raises(RecorderBackpressure, match="4 slots reserved"):
        recorder.record_sample(command, state)

    for index in range(4):
        recorder.record_event(f"critical_{index}")
    with pytest.raises(RecorderBackpressure, match="queue is full"):
        recorder.record_event("critical_overflow")

    release_writer.set()
    stats = recorder.close(clean_shutdown=False)
    assert stats.samples == 3
    assert stats.events == 4
    assert stats.queue_high_watermark == 6
