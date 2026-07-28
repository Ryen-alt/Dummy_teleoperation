from __future__ import annotations

from dataclasses import dataclass

from .cameras import CameraFrame, D435Camera
from .schema import RobotState


class SynchronizationError(RuntimeError):
    pass


@dataclass(frozen=True)
class SynchronizedObservation:
    state: RobotState
    wrist: CameraFrame
    target_time_ns: int
    skew_ms: float

    def as_policy_dict(self) -> dict[str, object]:
        return {
            "observation.state": self.state.position.copy(),
            "observation.images.wrist": self.wrist.color.copy(),
            "observation.depth.wrist": self.wrist.depth.copy(),
            "timestamp_ns": self.target_time_ns,
            "camera_frame_number": self.wrist.frame_number,
            "camera_state_skew_ms": self.skew_ms,
            "gripper_state_valid": self.state.gripper_valid,
        }


class ObservationSynchronizer:
    def __init__(self, camera: D435Camera) -> None:
        self.camera = camera

    def build(self, state: RobotState, target_time_ns: int | None = None) -> SynchronizedObservation:
        if not state.position_valid:
            raise SynchronizationError("robot state is invalid")
        target = state.monotonic_ns if target_time_ns is None else target_time_ns
        try:
            frame = self.camera.nearest(target)
        except RuntimeError as exc:
            raise SynchronizationError(str(exc)) from exc
        skew_ms = abs(frame.capture_time_ns - state.monotonic_ns) / 1e6
        return SynchronizedObservation(state, frame, target, skew_ms)
