from __future__ import annotations

import json
from collections.abc import Callable, Mapping, Sequence
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Any

import numpy as np

from dummy_host.dataset import DatasetFrame

LEROBOT_VERSION = "0.4.0"


class LeRobotAdapterError(RuntimeError):
    pass


class LeRobotV3DatasetSink:
    """Offline adapter for the LeRobotDataset v3 API shipped by lerobot 0.4.0.

    Dataset creation is delayed until the first frame so camera roles and image
    shapes come from the validated Raw Session rather than duplicated config.
    """

    def __init__(
        self,
        *,
        repo_id: str,
        root: str | Path,
        fps: int,
        robot_type: str = "dummy",
        joint_names: Sequence[str] = (
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper",
        ),
        use_videos: bool = True,
        batch_encoding_size: int = 1,
        dataset_factory: Callable[..., Any] | None = None,
    ) -> None:
        if not repo_id or fps <= 0 or len(joint_names) != 7 or len(set(joint_names)) != 7:
            raise ValueError("repo_id, fps and seven unique joint names are required")
        if batch_encoding_size <= 0:
            raise ValueError("batch_encoding_size must be positive")
        self.repo_id = repo_id
        self.root = Path(root)
        self.fps = fps
        self.robot_type = robot_type
        self.joint_names = tuple(joint_names)
        self.use_videos = use_videos
        self.batch_encoding_size = batch_encoding_size
        self._factory = dataset_factory
        self._dataset: Any | None = None
        self._active_episode: str | None = None
        self._active_frames = 0
        self._episodes: list[dict[str, object]] = []
        self._image_shapes: dict[str, tuple[int, ...]] | None = None
        self._depth_shapes: dict[str, tuple[int, ...]] | None = None
        self._finalized = False

    def begin_episode(self, *, episode_id: str, task_id: str, task: str) -> None:
        self._ensure_open()
        if self._active_episode is not None:
            raise LeRobotAdapterError("an episode is already active")
        if not episode_id or not task_id or not task:
            raise ValueError("episode_id, task_id and task are required")
        self._active_episode = episode_id
        self._active_frames = 0
        self._episodes.append(
            {"episode_id": episode_id, "task_id": task_id, "task": task, "frames": 0}
        )

    def add_frame(self, frame: DatasetFrame) -> None:
        self._ensure_open()
        if self._active_episode is None or frame.episode_id != self._active_episode:
            raise LeRobotAdapterError("frame does not belong to the active episode")
        if frame.frame_index != self._active_frames:
            raise LeRobotAdapterError("episode frame indices must be contiguous and start at zero")
        if self._dataset is None:
            self._create_dataset(frame)
        self._validate_frame_schema(frame)
        payload: dict[str, object] = {
            "observation.state": frame.observation_state.copy(),
            "action": frame.action.copy(),
            "source.sample_index": np.asarray([frame.source_sample_index], dtype=np.int64),
            "source.tick_ns": np.asarray([frame.source_tick_ns], dtype=np.int64),
            "task": frame.task,
        }
        payload.update(
            {f"observation.images.{role}": image.copy() for role, image in frame.images.items()}
        )
        payload.update(
            {f"observation.depth.{role}": depth.copy() for role, depth in frame.depths.items()}
        )
        self._dataset.add_frame(payload)
        self._active_frames += 1
        self._episodes[-1]["frames"] = self._active_frames

    def end_episode(self, *, episode_id: str) -> None:
        self._ensure_open()
        if episode_id != self._active_episode:
            raise LeRobotAdapterError("episode end does not match the active episode")
        if self._dataset is None or self._active_frames == 0:
            raise LeRobotAdapterError("cannot save an empty episode")
        self._dataset.save_episode()
        self._active_episode = None
        self._active_frames = 0

    def finalize(self, *, metadata: Mapping[str, object]) -> object:
        self._ensure_open()
        if self._active_episode is not None:
            raise LeRobotAdapterError("cannot finalize while an episode is active")
        if self._dataset is None:
            raise LeRobotAdapterError("no accepted frames were exported")
        # Required by LeRobotDataset v3 to close Parquet writers and metadata footers.
        self._dataset.finalize()
        sidecar = {
            "adapter": "lerobot-robot-dummy",
            "adapter_version": 1,
            "lerobot_version": LEROBOT_VERSION,
            "repo_id": self.repo_id,
            "fps": self.fps,
            "robot_type": self.robot_type,
            "episodes": self._episodes,
            "source": dict(metadata),
        }
        self.root.mkdir(parents=True, exist_ok=True)
        sidecar_path = self.root / "dummy_export_metadata.json"
        sidecar_path.write_text(
            json.dumps(sidecar, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        self._finalized = True
        return {
            "repo_id": self.repo_id,
            "root": str(self.root.resolve()),
            "metadata": str(sidecar_path.resolve()),
        }

    def _create_dataset(self, frame: DatasetFrame) -> None:
        features: dict[str, dict[str, object]] = {
            "observation.state": {
                "dtype": "float32",
                "shape": (7,),
                "names": list(self.joint_names),
            },
            "action": {
                "dtype": "float32",
                "shape": (7,),
                "names": list(self.joint_names),
            },
            "source.sample_index": {
                "dtype": "int64",
                "shape": (1,),
                "names": ["sample_index"],
            },
            "source.tick_ns": {
                "dtype": "int64",
                "shape": (1,),
                "names": ["monotonic_ns"],
            },
        }
        image_dtype = "video" if self.use_videos else "image"
        for role, image in frame.images.items():
            features[f"observation.images.{role}"] = {
                "dtype": image_dtype,
                "shape": tuple(image.shape),
                "names": ["height", "width", "channels"],
            }
        for role, depth in frame.depths.items():
            features[f"observation.depth.{role}"] = {
                "dtype": "float32",
                "shape": tuple(depth.shape),
                "names": ["height", "width"],
            }
        factory = self._factory
        if factory is None:
            try:
                installed_version = version("lerobot")
                from lerobot.datasets import LeRobotDataset
            except (ImportError, PackageNotFoundError) as exc:
                raise LeRobotAdapterError(
                    "install the pinned lerobot-robot-dummy environment before export"
                ) from exc
            if installed_version != LEROBOT_VERSION:
                raise LeRobotAdapterError(
                    f"lerobot {LEROBOT_VERSION} is required, found {installed_version}"
                )
            factory = LeRobotDataset.create
        self._dataset = factory(
            repo_id=self.repo_id,
            root=self.root,
            fps=self.fps,
            robot_type=self.robot_type,
            features=features,
            use_videos=self.use_videos,
            batch_encoding_size=self.batch_encoding_size,
        )
        self._image_shapes = {role: tuple(value.shape) for role, value in frame.images.items()}
        self._depth_shapes = {role: tuple(value.shape) for role, value in frame.depths.items()}

    def _validate_frame_schema(self, frame: DatasetFrame) -> None:
        image_shapes = {role: tuple(value.shape) for role, value in frame.images.items()}
        depth_shapes = {role: tuple(value.shape) for role, value in frame.depths.items()}
        if image_shapes != self._image_shapes or depth_shapes != self._depth_shapes:
            raise LeRobotAdapterError("camera roles or shapes changed within the export")
        if abs(frame.timestamp_s * self.fps - frame.frame_index) > 0.25:
            raise LeRobotAdapterError("frame timestamp is not aligned with the configured dataset FPS")

    def _ensure_open(self) -> None:
        if self._finalized:
            raise LeRobotAdapterError("dataset sink is already finalized")
