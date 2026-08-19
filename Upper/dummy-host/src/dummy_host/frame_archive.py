from __future__ import annotations

from pathlib import Path
from typing import Protocol, runtime_checkable

import numpy as np

from .cameras import CameraFrame


@runtime_checkable
class FrameArchive(Protocol):
    @property
    def unique_frames(self) -> int: ...

    def write(self, frame: CameraFrame) -> str: ...
    def close(self) -> None: ...


class NpzFrameArchive:
    """Lossless, dependency-light frame archive grouped by logical camera role.

    The recorder depends only on FrameArchive, so an MP4/Zarr implementation can
    be selected by deployment configuration without touching the control loop.
    """

    def __init__(self, session_dir: str | Path, *, segment_size: int = 300) -> None:
        if segment_size <= 0:
            raise ValueError("segment_size must be positive")
        self.session_dir = Path(session_dir)
        self.frames_dir = self.session_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.segment_size = segment_size
        self._paths: dict[tuple[str, int, int], str] = {}
        self._per_role_count: dict[str, int] = {}

    @property
    def unique_frames(self) -> int:
        return len(self._paths)

    def write(self, frame: CameraFrame) -> str:
        key = (frame.role, frame.frame_number, frame.capture_time_ns)
        existing = self._paths.get(key)
        if existing is not None:
            return existing
        role_count = self._per_role_count.get(frame.role, 0)
        segment_index = role_count // self.segment_size
        segment = self.frames_dir / frame.role / f"segment_{segment_index:06d}"
        segment.mkdir(parents=True, exist_ok=True)
        frame_path = segment / f"frame_{frame.frame_number:010d}_{frame.capture_time_ns}.npz"
        payload: dict[str, object] = {
            "color_rgb": frame.color,
            "capture_time_ns": np.int64(frame.capture_time_ns),
            "arrival_time_ns": np.int64(frame.arrival_time_ns),
            "device_timestamp_ms": np.float64(frame.device_timestamp_ms),
            "frame_number": np.int64(frame.frame_number),
            "calibration_version": np.asarray(frame.calibration_version),
        }
        if frame.depth is not None:
            payload.update(
                {
                    "depth": frame.depth,
                    "depth_device_timestamp_ms": np.float64(frame.depth_device_timestamp_ms),
                    "depth_frame_number": np.int64(frame.depth_frame_number),
                    "color_depth_skew_ms": np.float64(frame.color_depth_skew_ms),
                    "depth_scale": np.float64(
                        np.nan if frame.depth_scale is None else frame.depth_scale
                    ),
                }
            )
        np.savez_compressed(frame_path, **payload)
        relative = frame_path.relative_to(self.session_dir).as_posix()
        self._paths[key] = relative
        self._per_role_count[frame.role] = role_count + 1
        return relative

    def close(self) -> None:
        return
