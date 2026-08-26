from __future__ import annotations

import hashlib
import json
import sqlite3
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from dummy_host.cameras import CameraFrame
from dummy_host.domain import ActionLifecycleUpdate, ActionStage
from dummy_host.dataset import DatasetFrame, ExportRecipe, export_raw_session
from dummy_host.dataset.raw_session import EpisodeWindow, RawSession, RawSessionError
from dummy_host.recording import ControlTickTiming, SessionRecorder
from dummy_host.schema import AppliedAction, ControlMode, RobotState
from dummy_host.teleop import KeyboardMapper, load_teleop_profile
from dummy_host.time_sync import TimeSyncExchange, TimeSyncModel


class MemorySink:
    def __init__(self) -> None:
        self.episodes: list[str] = []
        self.frames: list[DatasetFrame] = []
        self.metadata = None

    def begin_episode(self, *, episode_id: str, task_id: str, task: str) -> None:
        self.episodes.append(episode_id)

    def add_frame(self, frame: DatasetFrame) -> None:
        self.frames.append(frame)

    def end_episode(self, *, episode_id: str) -> None:
        assert episode_id == self.episodes[-1]

    def finalize(self, *, metadata):
        self.metadata = dict(metadata)
        return "memory://dataset"


def test_raw_session_v5_exports_only_exact_tx_completed_actions(config, tmp_path: Path) -> None:
    profile = load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")
    start_ns = 8_000_000_000
    position = np.concatenate((config.initial_pose_rad, np.asarray([0.5], dtype=np.float32)))
    state = RobotState(
        position=position.astype(np.float32),
        velocity=np.zeros(7, dtype=np.float32),
        monotonic_ns=start_ns + 10_000_000,
        mcu_time_us=(start_ns + 10_000_000) // 1000,
        mode=ControlMode.TELEOP,
        fault_bits=0,
        position_valid=True,
        velocity_valid=False,
        gripper_valid=True,
        last_received_sequence=3,
        target_age_ms=1,
        config_hash=config.config_hash,
        feedback_sweep_id=np.ones(7, dtype=np.uint32),
        coherent_sweep_id=1,
        coherent_reference_mcu_us=(start_ns + 10_000_000) // 1000,
    )
    command = KeyboardMapper(profile).map({"KEY_SPACE"}, state.monotonic_ns)
    session_epoch = 42
    action = AppliedAction(
        position.copy(), position.copy(), 3, state.monotonic_ns, False, (),
        session_epoch=session_epoch, control_tick_id=101,
    )

    def frame(role: str, number: int, depth: bool, capture_ns: int) -> CameraFrame:
        return CameraFrame(
            color=np.zeros((2, 3, 3), dtype=np.uint8),
            depth=np.zeros((2, 3), dtype=np.uint16) if depth else None,
            capture_time_ns=capture_ns,
            arrival_time_ns=capture_ns,
            device_timestamp_ms=1.0,
            frame_number=number,
            depth_device_timestamp_ms=1.0,
            depth_frame_number=number,
            color_depth_skew_ms=0.0,
            role=role,
            calibration_version=f"{role}-test-v1",
                depth_scale=0.001 if depth else None,
                timestamp_source="hardware_exposure" if role == "wrist" else "arrival",
        )

    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="export_source",
        extra_manifest={
            "data_classification": "temporary_uncalibrated_pipeline_test",
            "offline_training_only": True,
            "real_policy_execution_allowed": False,
        },
    )
    recorder.record_event(
        "episode_start",
        monotonic_ns=start_ns,
        payload={
            "episode_id": "episode-1",
            "status": "recording",
            "task_id": "pick",
            "task": "Pick the cube",
        },
    )
    recorder.update_runtime_metadata(
        firmware_version="fake-mcu-v2.2", session_epoch=session_epoch
    )
    recorder.record_time_sync(
        TimeSyncExchange(start_ns, start_ns // 1000, start_ns // 1000, start_ns),
        TimeSyncModel(1, 1, 1000.0, 0.0, 0, 0.0, 3, start_ns),
    )
    recorder.record_sample(
        command,
        state,
        action=action,
        camera_frames={
            "wrist": frame("wrist", 1, True, state.monotonic_ns),
            "global": frame("global", 2, False, state.monotonic_ns),
        },
        timing=ControlTickTiming(10, state.monotonic_ns, state.monotonic_ns, state.monotonic_ns),
    )
    state2 = replace(
        state,
        monotonic_ns=start_ns + 70_000_000,
        mcu_time_us=(start_ns + 70_000_000) // 1000,
        last_received_sequence=4,
        coherent_sweep_id=2,
        feedback_sweep_id=np.full(7, 2, dtype=np.uint32),
        coherent_reference_mcu_us=(start_ns + 70_000_000) // 1000,
    )
    command2 = KeyboardMapper(profile).map({"KEY_SPACE"}, state2.monotonic_ns)
    accepted_action2 = position.copy()
    accepted_action2[0] += np.float32(0.2)
    action2 = AppliedAction(
        accepted_action2.copy(), accepted_action2.copy(), 4, state2.monotonic_ns, False, (),
        session_epoch=session_epoch, control_tick_id=102,
    )
    recorder.record_sample(
        command2,
        state2,
        action=action2,
        camera_frames={
            "wrist": frame("wrist", 3, True, state2.monotonic_ns),
            "global": frame("global", 4, False, state2.monotonic_ns),
        },
        timing=ControlTickTiming(11, state2.monotonic_ns, state2.monotonic_ns, state2.monotonic_ns),
    )
    for sequence, base_ns in ((3, state.monotonic_ns), (4, state2.monotonic_ns)):
        for offset_ns, stage in enumerate(
            (
                ActionStage.RECEIVED,
                ActionStage.SAFETY_ACCEPTED,
                ActionStage.SEND_ENQUEUED,
                ActionStage.SERIAL_SEND_STARTED,
                ActionStage.SERIAL_SEND_FINISHED,
                ActionStage.ACKNOWLEDGED,
                ActionStage.CAN_QUEUED_EXACT,
                ActionStage.CAN_TX_COMPLETE_EXACT,
                ActionStage.POST_COMMAND_FEEDBACK,
            )
        ):
            recorder.record_action_lifecycle(
                ActionLifecycleUpdate(
                    sequence,
                    stage,
                    base_ns + offset_ns * 1_000_000,
                    mcu_time_us=base_ns // 1000 + offset_ns * 1000,
                    session_epoch=session_epoch,
                    control_tick_id=98 + sequence,
                )
            )
    state3 = replace(
        state2,
        monotonic_ns=start_ns + 120_000_000,
        mcu_time_us=(start_ns + 120_000_000) // 1000,
        last_received_sequence=5,
        coherent_sweep_id=3,
        feedback_sweep_id=np.full(7, 3, dtype=np.uint32),
        coherent_reference_mcu_us=(start_ns + 120_000_000) // 1000,
    )
    command3 = KeyboardMapper(profile).map({"KEY_SPACE"}, state3.monotonic_ns)
    action3 = AppliedAction(
        position.copy(), position.copy(), 5, state3.monotonic_ns, False, (),
        session_epoch=session_epoch, control_tick_id=103,
    )
    recorder.record_sample(
        command3,
        state3,
        action=action3,
        camera_frames={
            "wrist": frame("wrist", 5, True, state3.monotonic_ns),
            "global": frame("global", 6, False, state3.monotonic_ns),
        },
        timing=ControlTickTiming(12, state3.monotonic_ns, state3.monotonic_ns, state3.monotonic_ns),
    )
    for offset_ns, stage in zip(
        (0, 1, 2, 3, 4, 5, 6, 7, 309),
        (
            ActionStage.RECEIVED,
            ActionStage.SAFETY_ACCEPTED,
            ActionStage.SEND_ENQUEUED,
            ActionStage.SERIAL_SEND_STARTED,
            ActionStage.SERIAL_SEND_FINISHED,
            ActionStage.ACKNOWLEDGED,
            ActionStage.CAN_QUEUED_EXACT,
            ActionStage.CAN_TX_COMPLETE_EXACT,
            ActionStage.POST_COMMAND_FEEDBACK,
        ),
        strict=True,
    ):
        recorder.record_action_lifecycle(
            ActionLifecycleUpdate(
                5,
                stage,
                state3.monotonic_ns + offset_ns * 1_000_000,
                mcu_time_us=state3.mcu_time_us + offset_ns * 1000,
                session_epoch=session_epoch,
                control_tick_id=103,
            )
        )
    recorder.record_event(
        "episode_success",
        monotonic_ns=start_ns + 160_000_000,
        payload={"episode_id": "episode-1", "status": "accepted"},
    )
    recorder.close()

    sink = MemorySink()
    formal_recipe = ExportRecipe(
        recipe_id="dummy_dual_rgb_v1",
        version=1,
        required_camera_roles=("wrist", "global"),
        include_depth=True,
    )
    with pytest.raises(RawSessionError, match="formal export recipe rejects"):
        export_raw_session(recorder.session_dir, formal_recipe, sink)

    recipe = ExportRecipe(
        recipe_id="dummy_dual_rgb_temp_uncalibrated_v1",
        version=1,
        required_camera_roles=("wrist", "global"),
        include_depth=True,
        allow_uncalibrated_cameras=True,
        require_temporary_source=True,
    )
    report = export_raw_session(recorder.session_dir, recipe, sink)
    assert report.episodes_exported == 1
    assert report.frames_exported == 2
    assert report.sink_result == "memory://dataset"
    assert sink.frames[0].task == "Pick the cube"
    assert tuple(sink.frames[0].images) == ("wrist", "global")
    assert tuple(sink.frames[0].depths) == ("wrist",)
    assert sink.frames[0].depths["wrist"].dtype == np.float32
    assert sink.frames[1].source_raw_tick_index == 11
    assert sink.frames[1].timestamp_s == pytest.approx(0.05)
    assert sink.frames[1].interpolation_alpha == pytest.approx(5.0 / 6.0)
    np.testing.assert_array_equal(sink.frames[1].action, accepted_action2)
    assert max(frame.source_raw_tick_index for frame in sink.frames) == 11
    assert sink.metadata["dataset_format"] == "lerobot_v3"
    assert sink.metadata["robot_config_hash"] == config.config_hash
    assert sink.metadata["data_classification"] == "temporary_uncalibrated_pipeline_test"
    assert sink.metadata["offline_training_only"] is True
    assert sink.metadata["real_policy_execution_allowed"] is False
    assert sink.metadata["evidence_level"] == "v5_exact_tx_affine_clock"
    assert sink.metadata["legacy_export"] is False


def test_temporary_recipe_rejects_an_unclassified_source(config, tmp_path: Path) -> None:
    profile = load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="unclassified_source",
    )
    recorder.close()
    recipe = ExportRecipe(
        recipe_id="dummy_dual_rgb_temp_uncalibrated_v1",
        version=1,
        required_camera_roles=("wrist", "global"),
        allow_uncalibrated_cameras=True,
        require_temporary_source=True,
    )
    with pytest.raises(RawSessionError, match="--temporary-uncalibrated"):
        export_raw_session(recorder.session_dir, recipe, MemorySink())


def test_schema_v3_remains_inspectable_but_strict_export_is_rejected(
    config, tmp_path: Path
) -> None:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="legacy_v3",
    )
    recorder.close()
    with sqlite3.connect(recorder.db_path) as connection:
        connection.execute(
            "ALTER TABLE action_lifecycle RENAME COLUMN "
            "post_command_feedback_host_ns TO motor_observed_host_ns"
        )
        connection.execute(
            "ALTER TABLE action_lifecycle RENAME COLUMN "
            "post_command_feedback_mcu_us TO motor_observed_mcu_us"
        )
        connection.commit()
        connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    manifest = json.loads(recorder.manifest_path.read_text(encoding="utf-8"))
    manifest["schema_version"] = 3
    manifest["state_telemetry_version"] = 3
    recorder.manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    checksums = json.loads(recorder.checksums_path.read_text(encoding="utf-8"))
    for relative in ("manifest.json", "samples.sqlite"):
        checksums["files"][relative] = hashlib.sha256(
            (recorder.session_dir / relative).read_bytes()
        ).hexdigest()
    recorder.checksums_path.write_text(
        json.dumps(checksums, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    raw = RawSession(recorder.session_dir)
    frames = raw.iter_frames(
        EpisodeWindow("episode", "task", "Task", 0, 1, "accepted"),
        ExportRecipe("strict-v4", 1, ("front",)),
    )
    with pytest.raises(RawSessionError, match="not dataset-exportable"):
        next(frames)


def test_schema_v4_export_requires_explicit_legacy_mode(config, tmp_path: Path) -> None:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="legacy_v4",
    )
    recorder.close()
    manifest = json.loads(recorder.manifest_path.read_text(encoding="utf-8"))
    manifest["schema_version"] = 4
    manifest["binary_protocol_version"] = 4
    recorder.manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    checksums = json.loads(recorder.checksums_path.read_text(encoding="utf-8"))
    checksums["files"]["manifest.json"] = hashlib.sha256(
        recorder.manifest_path.read_bytes()
    ).hexdigest()
    recorder.checksums_path.write_text(
        json.dumps(checksums, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    raw = RawSession(recorder.session_dir)
    episode = EpisodeWindow("episode", "task", "Task", 0, 1, "accepted")
    with pytest.raises(RawSessionError, match="legacy-only"):
        list(raw.iter_frames(episode, ExportRecipe("strict-v5", 1, ("front",))))
    assert list(
        raw.iter_frames(
            episode,
            ExportRecipe("legacy-v4", 1, ("front",), legacy_mode=True),
        )
    ) == []
