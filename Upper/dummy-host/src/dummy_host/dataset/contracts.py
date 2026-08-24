from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping, Protocol, runtime_checkable

import numpy as np


@dataclass(frozen=True)
class ExportRecipe:
    recipe_id: str
    version: int
    required_camera_roles: tuple[str, ...]
    control_hz: int = 20
    dataset_format: str = "lerobot_v3"
    accepted_outcomes: tuple[str, ...] = ("accepted",)
    include_depth: bool = False
    allow_uncalibrated_cameras: bool = False
    require_temporary_source: bool = False
    metadata: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if (
            not isinstance(self.recipe_id, str)
            or not self.recipe_id
            or isinstance(self.version, bool)
            or self.version <= 0
            or isinstance(self.control_hz, bool)
            or self.control_hz <= 0
        ):
            raise ValueError("recipe_id, version and control_hz must be valid")
        if self.dataset_format != "lerobot_v3":
            raise ValueError("only the frozen lerobot_v3 export contract is supported")
        if (
            not self.required_camera_roles
            or any(not isinstance(role, str) or not role for role in self.required_camera_roles)
            or len(set(self.required_camera_roles)) != len(self.required_camera_roles)
        ):
            raise ValueError("required_camera_roles must be unique and non-empty")
        if (
            not self.accepted_outcomes
            or any(not isinstance(value, str) or not value for value in self.accepted_outcomes)
            or len(set(self.accepted_outcomes)) != len(self.accepted_outcomes)
        ):
            raise ValueError("accepted_outcomes must be unique and non-empty")
        if not all(
            isinstance(value, bool)
            for value in (
                self.include_depth,
                self.allow_uncalibrated_cameras,
                self.require_temporary_source,
            )
        ):
            raise ValueError("recipe camera and source gates must be boolean")
        if self.require_temporary_source and not self.allow_uncalibrated_cameras:
            raise ValueError(
                "require_temporary_source requires allow_uncalibrated_cameras"
            )
        if not isinstance(self.metadata, Mapping):
            raise ValueError("metadata must be a mapping")
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))

    @property
    def config_hash(self) -> str:
        payload = {
            "recipe_id": self.recipe_id,
            "version": self.version,
            "required_camera_roles": self.required_camera_roles,
            "control_hz": self.control_hz,
            "dataset_format": self.dataset_format,
            "accepted_outcomes": self.accepted_outcomes,
            "include_depth": self.include_depth,
            "allow_uncalibrated_cameras": self.allow_uncalibrated_cameras,
            "require_temporary_source": self.require_temporary_source,
            "metadata": dict(self.metadata),
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class DatasetFrame:
    observation_state: np.ndarray
    action: np.ndarray
    images: Mapping[str, np.ndarray]
    timestamp_s: float
    frame_index: int
    episode_id: str
    task_id: str
    task: str
    source_sample_index: int
    source_tick_ns: int
    depths: Mapping[str, np.ndarray] = field(default_factory=dict)

    def __post_init__(self) -> None:
        state = np.asarray(self.observation_state, dtype=np.float32)
        action = np.asarray(self.action, dtype=np.float32)
        if state.shape != (7,) or action.shape != (7,):
            raise ValueError("dataset state and action must each contain 7 values")
        if not np.isfinite(state).all() or not np.isfinite(action).all():
            raise ValueError("dataset state and action must be finite")
        state = state.copy()
        action = action.copy()
        state.setflags(write=False)
        action.setflags(write=False)
        object.__setattr__(self, "observation_state", state)
        object.__setattr__(self, "action", action)

        images: dict[str, np.ndarray] = {}
        for role, value in self.images.items():
            image = np.asarray(value)
            if not role or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(f"dataset image {role!r} must be RGB uint8 HWC")
            image = np.ascontiguousarray(image).copy()
            image.setflags(write=False)
            images[role] = image
        if not images:
            raise ValueError("dataset frame requires at least one image")
        object.__setattr__(self, "images", MappingProxyType(images))

        depths: dict[str, np.ndarray] = {}
        for role, value in self.depths.items():
            depth = np.asarray(value)
            if (
                role not in images
                or depth.dtype != np.float32
                or depth.shape != images[role].shape[:2]
                or not np.isfinite(depth).all()
                or np.any(depth < 0)
            ):
                raise ValueError(
                    f"dataset depth {role!r} must be non-negative float32 metres and match RGB"
                )
            depth = np.ascontiguousarray(depth).copy()
            depth.setflags(write=False)
            depths[role] = depth
        object.__setattr__(self, "depths", MappingProxyType(depths))
        if (
            not np.isfinite(self.timestamp_s)
            or self.timestamp_s < 0
            or self.frame_index < 0
            or self.source_sample_index < 0
            or self.source_tick_ns < 0
        ):
            raise ValueError("dataset frame indices and timestamps must be non-negative")
        if not self.episode_id or not self.task_id or not self.task:
            raise ValueError("dataset episode and task identities are required")


@runtime_checkable
class DatasetSink(Protocol):
    """Version-isolation seam implemented by a pinned LeRobot environment."""

    def begin_episode(self, *, episode_id: str, task_id: str, task: str) -> None: ...
    def add_frame(self, frame: DatasetFrame) -> None: ...
    def end_episode(self, *, episode_id: str) -> None: ...
    def finalize(self, *, metadata: Mapping[str, object]) -> object: ...


@dataclass(frozen=True)
class ExportReport:
    source_session: str
    recipe_hash: str
    episodes_exported: int
    frames_exported: int
    invalid_samples_excluded: int
    incomplete_episodes_excluded: int
    sink_result: object
