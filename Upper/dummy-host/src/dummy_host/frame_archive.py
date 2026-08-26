from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Protocol, runtime_checkable

import numpy as np

from .cameras import CameraFrame


DEFAULT_MINIMUM_FREE_BYTES = 2 * 1024**3


@runtime_checkable
class FrameArchive(Protocol):
    @property
    def unique_frames(self) -> int: ...

    def write(self, frame: CameraFrame) -> str: ...
    def close(self) -> None: ...


class NpzFrameArchive:
    """Atomic lossless frame archive grouped by logical camera role.

    Capture uses uncompressed NPZ by default. Compression is intentionally
    deferred to offline dataset export: zlib compression in the live recorder
    cannot sustain two 640x480 RGB streams plus D435 depth at 20 Hz. Each frame
    is written to a temporary file, flushed, and atomically renamed so a crash
    cannot make a partially written frame look committed.

    The recorder depends only on FrameArchive, so a segmented video/chunked
    depth implementation can still be selected without touching the control
    loop or Raw Session schema.
    """

    def __init__(
        self,
        session_dir: str | Path,
        *,
        segment_size: int = 300,
        compressed: bool = False,
        sync_files: bool = True,
        minimum_free_bytes: int = DEFAULT_MINIMUM_FREE_BYTES,
        disk_check_interval: int = 100,
    ) -> None:
        if segment_size <= 0:
            raise ValueError("segment_size must be positive")
        if minimum_free_bytes < 0:
            raise ValueError("minimum_free_bytes must be non-negative")
        if disk_check_interval <= 0:
            raise ValueError("disk_check_interval must be positive")
        self.session_dir = Path(session_dir)
        self.frames_dir = self.session_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.segment_size = segment_size
        self.compressed = compressed
        self.sync_files = sync_files
        self.minimum_free_bytes = minimum_free_bytes
        self.disk_check_interval = disk_check_interval
        self._writes_since_disk_check = disk_check_interval
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
            "timestamp_source": np.asarray(frame.timestamp_source),
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
        if self._writes_since_disk_check >= self.disk_check_interval:
            required_bytes = int(frame.color.nbytes)
            if frame.depth is not None:
                required_bytes += int(frame.depth.nbytes)
            free_bytes = shutil.disk_usage(self.frames_dir).free
            if free_bytes - required_bytes < self.minimum_free_bytes:
                raise OSError(
                    "camera archive free-space guard triggered: "
                    f"free={free_bytes} required_frame={required_bytes} "
                    f"reserve={self.minimum_free_bytes}"
                )
            self._writes_since_disk_check = 0
        temporary = frame_path.with_suffix(frame_path.suffix + ".partial")
        writer = np.savez_compressed if self.compressed else np.savez
        try:
            # Passing an open stream prevents NumPy from appending another
            # suffix and lets us fsync the exact temporary file before commit.
            with temporary.open("xb") as stream:
                writer(stream, **payload)
                stream.flush()
                if self.sync_files:
                    os.fsync(stream.fileno())
            os.replace(temporary, frame_path)
            if self.sync_files:
                directory_fd = os.open(
                    segment,
                    os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
                )
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise
        relative = frame_path.relative_to(self.session_dir).as_posix()
        self._paths[key] = relative
        self._per_role_count[frame.role] = role_count + 1
        self._writes_since_disk_check += 1
        return relative

    def close(self) -> None:
        return
