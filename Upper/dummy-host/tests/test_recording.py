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
from dummy_host.domain import ActionProgressFlags, ActionProgressRecord
from dummy_host.recording import (
    RecorderBackpressure,
    SessionRecorder,
    estimate_camera_archive_bytes,
)
from dummy_host.domain import ActionLifecycleUpdate, ActionStage
from dummy_host.protocol import (
    CAN_DIAGNOSTICS_FORMAT_VERSION,
    CAN_DIAGNOSTICS_PAYLOAD_SIZE,
    CAN_DIAGNOSTICS_WINDOW_VALID,
    CanDiagnostics,
)
from dummy_host.time_sync import TimeSyncExchange, TimeSyncModel
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
        target_age_ms=5,
        config_hash=config.config_hash,
        can_transport_status=0x6F,
        coherent_reference_mcu_us=now_ns // 1_000,
        action_progress=(
            ActionProgressRecord(
                sequence=7,
                flags=int(ActionProgressFlags.CAN_QUEUED_EXACT)
                | int(ActionProgressFlags.POST_COMMAND_FEEDBACK),
                can_queued_mcu_us=now_ns // 1_000,
                post_feedback_mcu_us=now_ns // 1_000 + 1_000,
                feedback_sweep_id=1,
            ),
        ),
    )
    command = KeyboardMapper(profile).map({"KEY_SPACE", "KEY_Q"}, now_ns)
    action = AppliedAction(
        requested=state.position.copy(),
        applied=state.position.copy(),
        sequence=7,
        monotonic_ns=now_ns,
        clipped=False,
        reasons=(),
        session_epoch=123,
        control_tick_id=77,
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
        timestamp_source="hardware_exposure",
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
        timestamp_source="arrival",
    )
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="session_test",
        queue_size=8,
    )
    recorder.update_runtime_metadata(
        firmware_version="fake-mcu-v1", session_epoch=123
    )
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
            "SELECT COUNT(*), action_sequence, length(applied_action), "
            "length(state_following_error), length(feedback_age_ms), "
            "length(node_fault_bits), state_hold_reason_bits, state_can_transport_status, "
            "json_array_length(action_progress_json) "
            "FROM samples"
        ).fetchone()
        camera_rows = connection.execute(
            "SELECT role, COUNT(*), COUNT(DISTINCT frame_path), timestamp_source "
            "FROM camera_samples GROUP BY role ORDER BY role"
        ).fetchall()
    assert row == (2, 7, 28, 28, 28, 14, 0, 0x6F, 1)
    assert camera_rows == [
        ("global", 2, 1, "arrival"),
        ("wrist", 2, 1, "hardware_exposure"),
    ]
    manifest = json.loads(recorder.manifest_path.read_text(encoding="utf-8"))
    assert manifest["clean_shutdown"] is True
    assert manifest["firmware_version"] == "fake-mcu-v1"
    assert manifest["robot_config_hash"] == config.config_hash
    assert manifest["schema_version"] == 6
    assert manifest["can_diagnostics_format_version"] == 2
    assert manifest["binary_protocol_version"] == 5
    assert manifest["state_telemetry_version"] == 4
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


def test_v6_evidence_tables_store_exact_latency_clock_and_can_diagnostics(
    config: RobotConfig, tmp_path: Path
) -> None:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    recorder = SessionRecorder(
        tmp_path, config, profile, source="test", session_name="v5_evidence"
    )
    recorder.update_runtime_metadata(
        firmware_version="dummy-ref-v2.2.2", session_epoch=9
    )
    exchange = TimeSyncExchange(1_000_000, 1_000, 1_010, 1_020_000)
    model = TimeSyncModel(1, 1, 1000.0, 0.0, 10_000, 25.0, 3, 1_020_000)
    recorder.record_time_sync(exchange, model)
    recorder.record_can_diagnostics(
        CanDiagnostics(
            format_version=CAN_DIAGNOSTICS_FORMAT_VERSION,
            payload_size=CAN_DIAGNOSTICS_PAYLOAD_SIZE,
            session_epoch=9,
            motor_marker_mask=0x7F,
            window_flags=CAN_DIAGNOSTICS_WINDOW_VALID,
            window_reset_count=1,
            window_start_us=100,
            window_duration_us=1_000,
            target_tx_complete=(1,) * 7,
            position_request=(2,) * 7,
            position_response=(2,) * 7,
            position_timeout=(0,) * 7,
            temperature_request=(3,) * 7,
            temperature_response=(3,) * 7,
            temperature_timeout=(0,) * 7,
            motor_tx_drop=(0,) * 7,
            motor_rx_error=(0,) * 7,
            motor_busoff=(0,) * 7,
            main_can_busoff=(0, 0),
            main_can_rx_overflow=(0, 0),
            main_can_rx_high_water=(4, 2),
            unexpected_response_count=0,
            maintenance_response_count=0,
            query_target_overlap_count=6,
            target_retry_count=0,
            target_retry_exhausted_count=0,
            target_deadline_failure_count=0,
            main_can_tx_abort=(0, 0),
            main_can_tx_error=(7, 0),
            main_can_tx_recovery=(0, 0),
            main_can_completion_overflow=(0, 0),
            safety_preemption_count=9,
            max_safety_wait_us=10,
            max_fanout_us=11,
            max_rx_dispatch_latency_us=12,
            main_can_rx_frame=(13, 14),
            main_can_tx_busy=(15, 16),
            transition_failure_count=0,
        ),
        host_time_ns=2_000_000,
    )
    stages = (
        (ActionStage.SAFETY_ACCEPTED, 100_000, 0),
        (ActionStage.ACKNOWLEDGED, 120_000, 120),
        (ActionStage.CAN_TX_COMPLETE_EXACT, 150_000, 150),
        (ActionStage.POST_COMMAND_FEEDBACK, 170_000, 170),
    )
    for stage, host_ns, mcu_us in stages:
        recorder.record_action_lifecycle(
            ActionLifecycleUpdate(
                3,
                stage,
                host_ns,
                mcu_time_us=mcu_us,
                session_epoch=9,
                control_tick_id=12,
            )
        )
    recorder.close()

    with sqlite3.connect(recorder.db_path) as connection:
        lifecycle = connection.execute(
            """
            SELECT session_epoch, control_tick_id,
                   can_tx_complete_exact_host_ns,
                   can_tx_complete_exact_mcu_us,
                   accepted_to_ack_host_ns,
                   ack_to_can_tx_complete_us,
                   can_tx_complete_to_post_feedback_us
            FROM action_lifecycle WHERE action_sequence = 3
            """
        ).fetchone()
        time_model = connection.execute(
            "SELECT segment_id, slope_ns_per_us, residual_ns FROM time_sync_models"
        ).fetchone()
        diagnostics = connection.execute(
            "SELECT tx_error_count, max_fanout_us FROM can_diagnostics"
        ).fetchone()
    assert lifecycle == (9, 12, 150_000, 150, 20_000, 30, 20)
    assert time_model == (1, 1000.0, 25.0)
    assert diagnostics == (7, 11)


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
        timestamp_source="hardware_exposure",
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
        timestamp_source="arrival",
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
        timestamp_source="arrival",
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
