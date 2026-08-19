from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import Mapping

from .cameras import Camera, CameraFrame, CameraManager
from .domain import ObservationBundle
from .schema import RobotState


class SynchronizationError(RuntimeError):
    pass


@dataclass(frozen=True)
class SynchronizedObservation:
    state: RobotState
    frames: Mapping[str, CameraFrame]
    target_time_ns: int
    skew_ms: Mapping[str, float]
    observation_sequence: int
    schema_id: str

    @property
    def wrist(self) -> CameraFrame:
        try:
            return self.frames["wrist"]
        except KeyError as exc:
            raise SynchronizationError("wrist camera is not present") from exc

    def as_policy_dict(self) -> dict[str, object]:
        output: dict[str, object] = {
            "observation.state": self.state.position.copy(),
            "timestamp_ns": self.target_time_ns,
            "observation_sequence": self.observation_sequence,
            "schema_id": self.schema_id,
            "camera_state_skew_ms": dict(self.skew_ms),
            "gripper_state_valid": self.state.gripper_valid,
        }
        for role, frame in self.frames.items():
            output[f"observation.images.{role}"] = frame.color.copy()
            if frame.depth is not None:
                output[f"observation.depth.{role}"] = frame.depth.copy()
                output[f"observation.depth_scale.{role}"] = frame.depth_scale
            output[f"camera_frame_number.{role}"] = frame.frame_number
        return output

    def as_bundle(self, *, task: str | None = None) -> ObservationBundle:
        return ObservationBundle(
            state=self.state,
            images=self.frames,
            target_time_ns=self.target_time_ns,
            task=task,
            observation_sequence=self.observation_sequence,
            schema_id=self.schema_id,
        )


class ObservationSynchronizer:
    def __init__(self, cameras: CameraManager | Camera) -> None:
        self.cameras = (
            cameras if isinstance(cameras, CameraManager) else CameraManager({cameras.role: cameras})
        )
        self._sequence = 0

    def build(self, state: RobotState, target_time_ns: int | None = None) -> SynchronizedObservation:
        if not state.position_valid:
            raise SynchronizationError("robot state is invalid")
        target = state.monotonic_ns if target_time_ns is None else target_time_ns
        try:
            frames = self.cameras.nearest_all(target)
        except RuntimeError as exc:
            raise SynchronizationError(str(exc)) from exc
        if not frames:
            raise SynchronizationError("no camera frames are available")
        self._sequence += 1
        skew_ms = {
            role: abs(frame.capture_time_ns - target) / 1e6 for role, frame in frames.items()
        }
        schema_id = _schema_id(state, frames)
        return SynchronizedObservation(
            state,
            frames,
            target,
            skew_ms,
            self._sequence,
            schema_id,
        )


def _schema_id(state: RobotState, frames: Mapping[str, CameraFrame]) -> str:
    payload = {
        "state": {"dtype": str(state.position.dtype), "shape": list(state.position.shape)},
        "images": {
            role: {
                "color_dtype": str(frame.color.dtype),
                "color_shape": list(frame.color.shape),
                "depth_dtype": None if frame.depth is None else str(frame.depth.dtype),
                "depth_shape": None if frame.depth is None else list(frame.depth.shape),
            }
            for role, frame in sorted(frames.items())
        },
        "action": {
            "dtype": "float32",
            "shape": [7],
            "semantics": "absolute_joint_position",
            "unit": "rad_and_normalized_gripper",
        },
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
