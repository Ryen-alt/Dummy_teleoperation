from __future__ import annotations

from dataclasses import replace

import numpy as np

from dummy_host.cameras import (
    CameraFrame,
    CameraManager,
    CameraMetrics,
    DeviceClockMapper,
    SyntheticCamera,
)
from dummy_host.schema import ControlMode, RobotState
from dummy_host.sync import ObservationSynchronizer


def test_device_clock_mapper_uses_monotonic_anchor() -> None:
    mapper = DeviceClockMapper()
    assert mapper.map(1000.0, 10_000_000_000) == 10_000_000_000
    assert mapper.map(1010.0, 10_012_000_000) == 10_010_000_000
    assert mapper.map(1009.0, 10_020_000_000) == 10_020_000_000
    # A device clock reset starts a new host monotonic epoch.
    assert mapper.map(2.0, 11_000_000_000) == 11_000_000_000
    assert mapper.reset_count == 2


def test_camera_metrics_report_latency_skew_and_drops() -> None:
    metrics = CameraMetrics(started_ns=1_000_000_000)
    for index, (latency_ms, skew_ms) in enumerate(((5.0, 1.0), (15.0, 3.0)), start=1):
        capture_ns = 1_000_000_000 + index * 10_000_000
        metrics.observe(
            CameraFrame(
                color=np.empty((1, 1, 3), dtype=np.uint8),
                depth=np.empty((1, 1), dtype=np.uint16),
                capture_time_ns=capture_ns,
                arrival_time_ns=capture_ns + int(latency_ms * 1e6),
                device_timestamp_ms=float(index * 10),
                frame_number=index,
                depth_device_timestamp_ms=float(index * 10) + skew_ms,
                depth_frame_number=index,
                color_depth_skew_ms=skew_ms,
            )
        )
    metrics.record_drop(2)
    stats = metrics.snapshot(now_ns=2_000_000_000, device_clock_resets=1)
    assert stats.frames == 2
    assert stats.dropped_frames == 2
    assert stats.measured_fps == 2.0
    assert stats.mean_capture_latency_ms == 10.0
    assert stats.mean_color_depth_skew_ms == 2.0
    assert stats.device_clock_resets == 1


def test_camera_manager_builds_role_stable_observation(config) -> None:
    wrist_config = config.cameras["wrist"]
    global_config = replace(
        wrist_config,
        name="global",
        model="synthetic-rgb",
        driver="fake",
        depth_format="none",
        align_depth_to_color=False,
        calibration_version="global-test-v1",
    )
    wrist = SyntheticCamera(wrist_config)
    global_camera = SyntheticCamera(global_config)
    target_ns = 2_000_000_000
    for camera, offset_ns in ((wrist, -1_000_000), (global_camera, 2_000_000)):
        camera.publish(
            CameraFrame(
                color=np.zeros((camera.config.height, camera.config.width, 3), dtype=np.uint8),
                depth=(
                    np.zeros((camera.config.height, camera.config.width), dtype=np.uint16)
                    if camera.role == "wrist"
                    else None
                ),
                capture_time_ns=target_ns + offset_ns,
                arrival_time_ns=target_ns + offset_ns,
                device_timestamp_ms=0.0,
                frame_number=1,
                depth_device_timestamp_ms=0.0,
                depth_frame_number=1,
                color_depth_skew_ms=0.0,
                role=camera.role,
                calibration_version=camera.config.calibration_version,
            )
        )
    manager = CameraManager({"wrist": wrist, "global": global_camera})
    position = np.concatenate((config.initial_pose_rad, np.asarray([0.5], dtype=np.float32)))
    state = RobotState(
        position=position.astype(np.float32),
        velocity=np.zeros(7, dtype=np.float32),
        monotonic_ns=target_ns,
        mcu_time_us=target_ns // 1000,
        mode=ControlMode.HOLD,
        fault_bits=0,
        position_valid=True,
        velocity_valid=False,
        gripper_valid=True,
        last_received_sequence=0,
        last_applied_sequence=0,
        target_age_ms=0,
        config_hash=config.config_hash,
    )
    observation = ObservationSynchronizer(manager).build(state)
    assert tuple(observation.frames) == ("wrist", "global")
    policy = observation.as_policy_dict()
    assert "observation.images.wrist" in policy
    assert "observation.images.global" in policy
    assert "observation.depth.wrist" in policy
    assert "observation.depth.global" not in policy
    assert len(observation.schema_id) == 64


def test_optional_camera_start_failure_does_not_block_required_camera(config) -> None:
    class TrackingCamera(SyntheticCamera):
        def __init__(self, camera_config, *, fail: bool = False) -> None:
            super().__init__(camera_config)
            self.fail = fail
            self.starts = 0
            self.stops = 0

        def start(self) -> None:
            self.starts += 1
            if self.fail:
                raise RuntimeError("camera unavailable")
            super().start()

        def stop(self) -> None:
            self.stops += 1
            super().stop()

    wrist = TrackingCamera(config.cameras["wrist"])
    optional_config = replace(
        config.cameras["wrist"],
        name="global",
        required=False,
        model="synthetic-rgb",
        driver="fake",
    )
    optional = TrackingCamera(optional_config, fail=True)
    manager = CameraManager({"wrist": wrist, "global": optional})
    manager.start()
    assert wrist.starts == 1
    assert optional.starts == 1
    manager.stop()
    assert wrist.stops == 1
    assert optional.stops == 1
