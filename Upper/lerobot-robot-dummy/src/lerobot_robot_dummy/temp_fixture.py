from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from dummy_host.cameras import CameraFrame
from dummy_host.domain import (
    ActionLifecycleUpdate,
    ActionProgressFlags,
    ActionProgressRecord,
    ActionStage,
)
from dummy_host.recording import ControlTickTiming, SessionRecorder
from dummy_host.protocol import CanDiagnostics
from dummy_host.schema import AppliedAction, ControlMode, RobotState, load_robot_config
from dummy_host.teleop import TeleopCommand, load_teleop_profile
from dummy_host.time_sync import TimeSyncExchange, TimeSyncModel

from .act_smoke import TEMP_CLASSIFICATION


def _rgb_frame(height: int, width: int, index: int, *, global_view: bool) -> np.ndarray:
    x = np.arange(width, dtype=np.uint16)[None, :]
    y = np.arange(height, dtype=np.uint16)[:, None]
    offset = 71 if global_view else 19
    return np.stack(
        (
            np.broadcast_to((x + index * 5 + offset) % 256, (height, width)),
            np.broadcast_to((y * 2 + index * 3 + offset) % 256, (height, width)),
            (x + y + index * 7 + offset) % 256,
        ),
        axis=2,
    ).astype(np.uint8)


def create_temp_raw_session(
    *,
    config_path: str | Path,
    input_config_path: str | Path,
    output_root: str | Path,
    session_name: str,
    episodes: int = 2,
    frames_per_episode: int = 12,
    image_height: int = 96,
    image_width: int = 96,
) -> Path:
    if episodes <= 0 or frames_per_episode < 8:
        raise ValueError("episodes must be positive and frames_per_episode must be at least 8")
    if image_height < 32 or image_width < 32:
        raise ValueError("fixture images must be at least 32x32")
    config = load_robot_config(config_path)
    profile = load_teleop_profile(input_config_path)
    recorder = SessionRecorder(
        output_root,
        config,
        profile,
        source="synthetic_temp_pipeline_fixture",
        firmware_version="dummy-ref-v2.2-fixture-not-hardware",
        session_name=session_name,
        queue_size=max(256, episodes * frames_per_episode * 10 + 16),
        extra_manifest={
            "data_classification": TEMP_CLASSIFICATION,
            "offline_training_only": True,
            "real_policy_execution_allowed": False,
            "synthetic_fixture": True,
            "camera_required": False,
            "camera_roles": ["wrist", "global"],
        },
    )
    base_ns = 20_000_000_000
    session_epoch = 0x54454D50
    recorder.update_runtime_metadata(
        firmware_version="dummy-ref-v2.2-fixture-not-hardware",
        session_epoch=session_epoch,
    )
    recorder.record_time_sync(
        TimeSyncExchange(base_ns, base_ns // 1000, base_ns // 1000, base_ns),
        TimeSyncModel(1, 1, 1000.0, 0.0, 0, 0.0, 3, base_ns),
    )
    recorder.record_can_diagnostics(
        CanDiagnostics(
            base_ns // 1000,
            1_000_000,
            (50,) * 7,
            (40,) * 7,
            (1,) * 7,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            2_000,
        ),
        host_time_ns=base_ns,
    )
    episode_stride_ns = frames_per_episode * 50_000_000 + 2_000_000
    sequence = 0
    for episode_index in range(episodes):
        episode_id = f"temp-pipeline-{episode_index:03d}"
        episode_start_ns = base_ns + episode_index * episode_stride_ns
        recorder.record_event(
            "episode_start",
            monotonic_ns=episode_start_ns,
            payload={
                "episode_id": episode_id,
                "status": "recording",
                "task_id": "temp_pipeline_smoke",
                "task": "TEMP uncalibrated ACT pipeline smoke",
            },
        )
        for frame_index in range(frames_per_episode):
            sequence += 1
            tick_ns = episode_start_ns + (frame_index + 1) * 50_000_000
            phase = np.float32((episode_index * frames_per_episode + frame_index) * 0.01)
            position = np.concatenate(
                (
                    config.initial_pose_rad.astype(np.float32).copy(),
                    np.asarray([0.45 + 0.05 * np.sin(float(phase))], dtype=np.float32),
                )
            )
            position[0] += phase
            requested = position.copy()
            requested[0] += np.float32(0.005)
            command = TeleopCommand(
                monotonic_ns=tick_ns,
                source="synthetic_temp_pipeline_fixture",
                joint_velocity_rad_s=np.zeros(6, dtype=np.float32),
                gripper_velocity_per_s=0.0,
                deadman=True,
                hold_requested=False,
                estop_requested=False,
                episode_event=None,
                connected=True,
                raw={"fixture": True, "frame_index": frame_index},
            )
            state = RobotState(
                position=position,
                velocity=np.zeros(7, dtype=np.float32),
                monotonic_ns=tick_ns,
                mcu_time_us=tick_ns // 1000,
                mode=ControlMode.TELEOP,
                fault_bits=0,
                position_valid=True,
                velocity_valid=True,
                gripper_valid=True,
                last_received_sequence=sequence,
                target_age_ms=1,
                config_hash=config.config_hash,
                feedback_age_ms=np.ones(7, dtype=np.uint32),
                feedback_sample_mcu_us=np.full(7, tick_ns // 1000, dtype=np.uint64),
                feedback_sweep_id=np.full(7, frame_index + 1, dtype=np.uint32),
                coherent_sweep_id=frame_index + 1,
                coherent_reference_mcu_us=tick_ns // 1000,
                action_progress=(
                    ActionProgressRecord(
                        sequence=sequence,
                        flags=int(ActionProgressFlags.CAN_QUEUED_EXACT)
                        | int(ActionProgressFlags.CAN_TX_COMPLETE_EXACT)
                        | int(ActionProgressFlags.POST_COMMAND_FEEDBACK),
                        can_queued_mcu_us=tick_ns // 1000 + 2_000,
                        can_tx_complete_mcu_us=tick_ns // 1000 + 3_000,
                        post_feedback_mcu_us=tick_ns // 1000 + 4_000,
                        feedback_sweep_id=frame_index + 1,
                    ),
                ),
            )
            action = AppliedAction(
                requested=requested,
                applied=requested.copy(),
                sequence=sequence,
                monotonic_ns=tick_ns,
                clipped=False,
                reasons=(),
                source="synthetic_temp_pipeline_fixture",
                session_epoch=session_epoch,
                control_tick_id=sequence,
            )
            frames = {}
            for role, is_global in (("wrist", False), ("global", True)):
                rgb = _rgb_frame(
                    image_height,
                    image_width,
                    sequence,
                    global_view=is_global,
                )
                frames[role] = CameraFrame(
                    color=rgb,
                    depth=None,
                    capture_time_ns=tick_ns + 1_000_000,
                    arrival_time_ns=tick_ns + 1_000_000,
                    device_timestamp_ms=sequence * 50.0,
                    frame_number=sequence,
                    depth_device_timestamp_ms=0.0,
                    depth_frame_number=0,
                    color_depth_skew_ms=0.0,
                    role=role,
                    calibration_version="uncalibrated-v0",
                    timestamp_source="arrival",
                )
            recorder.record_sample(
                command,
                state,
                action=action,
                camera_frames=frames,
                timing=ControlTickTiming(
                    frame_index,
                    tick_ns,
                    tick_ns,
                    tick_ns + 500_000,
                    target_generated_ns=tick_ns + 100_000,
                    send_enqueued_ns=tick_ns + 200_000,
                ),
            )
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
                        tick_ns + offset_ns * 400_000,
                        mcu_time_us=tick_ns // 1000 + offset_ns * 400,
                        session_epoch=session_epoch,
                        control_tick_id=sequence,
                    )
                )
        recorder.record_event(
            "episode_success",
            monotonic_ns=episode_start_ns + frames_per_episode * 50_000_000 + 1_000_000,
            payload={"episode_id": episode_id, "status": "accepted"},
        )
    recorder.close()
    return recorder.session_dir.resolve()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create a synthetic Raw Session v5 TEMP fixture for offline pipeline tests"
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--input-config", required=True)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--session-name", default="temp_v16_pipeline_fixture")
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--frames-per-episode", type=int, default=12)
    parser.add_argument("--image-height", type=int, default=96)
    parser.add_argument("--image-width", type=int, default=96)
    args = parser.parse_args()
    try:
        session = create_temp_raw_session(
            config_path=args.config,
            input_config_path=args.input_config,
            output_root=args.output_root,
            session_name=args.session_name,
            episodes=args.episodes,
            frames_per_episode=args.frames_per_episode,
            image_height=args.image_height,
            image_width=args.image_width,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        parser.error(str(exc))
    print(
        json.dumps(
            {
                "session": str(session),
                "data_classification": TEMP_CLASSIFICATION,
                "synthetic_fixture": True,
                "real_policy_execution_allowed": False,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
