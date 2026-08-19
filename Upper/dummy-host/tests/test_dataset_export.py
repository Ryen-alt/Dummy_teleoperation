from __future__ import annotations

from pathlib import Path

import numpy as np

from dummy_host.cameras import CameraFrame
from dummy_host.dataset import DatasetFrame, ExportRecipe, export_raw_session
from dummy_host.recording import SessionRecorder
from dummy_host.schema import AppliedAction, ControlMode, RobotState
from dummy_host.teleop import KeyboardMapper, load_teleop_profile


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


def test_raw_session_v2_exports_through_version_isolated_sink(config, tmp_path: Path) -> None:
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
        last_applied_sequence=3,
        target_age_ms=1,
        config_hash=config.config_hash,
    )
    command = KeyboardMapper(profile).map({"KEY_SPACE"}, state.monotonic_ns)
    action = AppliedAction(position.copy(), position.copy(), 3, state.monotonic_ns, False, ())

    def frame(role: str, number: int, depth: bool) -> CameraFrame:
        return CameraFrame(
            color=np.zeros((2, 3, 3), dtype=np.uint8),
            depth=np.zeros((2, 3), dtype=np.uint16) if depth else None,
            capture_time_ns=state.monotonic_ns,
            arrival_time_ns=state.monotonic_ns,
            device_timestamp_ms=1.0,
            frame_number=number,
            depth_device_timestamp_ms=1.0,
            depth_frame_number=number,
            color_depth_skew_ms=0.0,
            role=role,
            calibration_version=f"{role}-test-v1",
            depth_scale=0.001 if depth else None,
        )

    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="keyboard",
        session_name="export_source",
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
    recorder.record_sample(
        command,
        state,
        action=action,
        camera_frames={"wrist": frame("wrist", 1, True), "global": frame("global", 2, False)},
    )
    recorder.record_event(
        "episode_success",
        monotonic_ns=start_ns + 20_000_000,
        payload={"episode_id": "episode-1", "status": "accepted"},
    )
    recorder.close()

    sink = MemorySink()
    recipe = ExportRecipe(
        recipe_id="dummy_dual_rgb_v1",
        version=1,
        required_camera_roles=("wrist", "global"),
        include_depth=True,
    )
    report = export_raw_session(recorder.session_dir, recipe, sink)
    assert report.episodes_exported == 1
    assert report.frames_exported == 1
    assert report.sink_result == "memory://dataset"
    assert sink.frames[0].task == "Pick the cube"
    assert tuple(sink.frames[0].images) == ("wrist", "global")
    assert tuple(sink.frames[0].depths) == ("wrist",)
    assert sink.frames[0].depths["wrist"].dtype == np.float32
    assert sink.metadata["dataset_format"] == "lerobot_v3"
    assert sink.metadata["robot_config_hash"] == config.config_hash
