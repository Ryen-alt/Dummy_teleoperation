from __future__ import annotations

import collections
import logging
import threading
import time
from dataclasses import dataclass

import numpy as np

from .schema import CameraConfig

LOG = logging.getLogger(__name__)


class CameraError(RuntimeError):
    pass


@dataclass(frozen=True)
class CameraFrame:
    color: np.ndarray
    depth: np.ndarray
    capture_time_ns: int
    arrival_time_ns: int
    device_timestamp_ms: float
    frame_number: int


@dataclass(frozen=True)
class CameraStats:
    frames: int
    dropped_frames: int
    last_error: str | None


class DeviceClockMapper:
    """Maps RealSense device milliseconds onto host monotonic nanoseconds."""

    def __init__(self) -> None:
        self._device_origin_ms: float | None = None
        self._last_device_ms: float | None = None
        self._host_origin_ns: int | None = None

    def map(self, device_ms: float, arrival_ns: int) -> int:
        if self._device_origin_ms is None or (
            self._last_device_ms is not None and device_ms < self._last_device_ms
        ):
            self._device_origin_ms = device_ms
            self._host_origin_ns = arrival_ns
        self._last_device_ms = device_ms
        assert self._host_origin_ns is not None
        capture_ns = self._host_origin_ns + int((device_ms - self._device_origin_ms) * 1e6)
        # A bad device timestamp must not claim a capture in the future.
        return min(capture_ns, arrival_ns)


class D435Camera:
    def __init__(self, config: CameraConfig, *, buffer_size: int = 8) -> None:
        if config.model.upper() != "D435":
            raise CameraError(f"unsupported camera model {config.model}")
        if buffer_size <= 0:
            raise ValueError("buffer_size must be positive")
        self.config = config
        self._frames: collections.deque[CameraFrame] = collections.deque(maxlen=buffer_size)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._pipeline = None
        self._align = None
        self._mapper = DeviceClockMapper()
        self._frames_seen = 0
        self._dropped_frames = 0
        self._last_number: int | None = None
        self._last_error: str | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise CameraError("install dummy-host[d435] to use the D435") from exc

        pipeline = rs.pipeline()
        rs_config = rs.config()
        if self.config.device_serial:
            rs_config.enable_device(self.config.device_serial)
        rs_config.enable_stream(
            rs.stream.color,
            self.config.width,
            self.config.height,
            rs.format.bgr8,
            self.config.fps,
        )
        rs_config.enable_stream(
            rs.stream.depth,
            self.config.width,
            self.config.height,
            rs.format.z16,
            self.config.fps,
        )
        try:
            profile = pipeline.start(rs_config)
            self._configure_sensors(rs, profile)
        except BaseException as exc:
            try:
                pipeline.stop()
            except BaseException:
                pass
            raise CameraError(f"cannot start D435: {exc}") from exc
        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color) if self.config.align_depth_to_color else None
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, name="dummy-d435-wrist", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None

    def latest(self, *, max_age_ms: int | None = None, now_ns: int | None = None) -> CameraFrame:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        max_age_ms = self.config.max_frame_age_ms if max_age_ms is None else max_age_ms
        with self._lock:
            if not self._frames:
                raise CameraError("no D435 frame is available")
            frame = self._frames[-1]
        age_ms = (now_ns - frame.capture_time_ns) / 1e6
        if age_ms < 0 or age_ms > max_age_ms:
            raise CameraError(f"latest D435 frame is stale ({age_ms:.1f} ms)")
        return frame

    def nearest(self, target_ns: int, *, max_skew_ms: int | None = None) -> CameraFrame:
        max_skew_ms = self.config.max_sync_skew_ms if max_skew_ms is None else max_skew_ms
        with self._lock:
            if not self._frames:
                raise CameraError("no D435 frame is available")
            frame = min(self._frames, key=lambda item: abs(item.capture_time_ns - target_ns))
        skew_ms = abs(frame.capture_time_ns - target_ns) / 1e6
        if skew_ms > max_skew_ms:
            raise CameraError(f"D435/state skew is too large ({skew_ms:.1f} ms)")
        return frame

    def stats(self) -> CameraStats:
        return CameraStats(self._frames_seen, self._dropped_frames, self._last_error)

    def _configure_sensors(self, rs: object, profile: object) -> None:
        for sensor in profile.get_device().query_sensors():
            name = sensor.get_info(rs.camera_info.name).lower()
            if "rgb" in name:
                self._set_manual_option(rs, sensor, rs.option.exposure, self.config.color_exposure)
                self._set_manual_option(rs, sensor, rs.option.white_balance, self.config.color_white_balance)
            if "stereo" in name or "depth" in name:
                self._set_manual_option(rs, sensor, rs.option.exposure, self.config.depth_exposure)

    @staticmethod
    def _set_manual_option(rs: object, sensor: object, option: object, value: float | None) -> None:
        if value is None:
            return
        try:
            if option == rs.option.exposure and sensor.supports(rs.option.enable_auto_exposure):
                sensor.set_option(rs.option.enable_auto_exposure, 0)
            if option == rs.option.white_balance and sensor.supports(rs.option.enable_auto_white_balance):
                sensor.set_option(rs.option.enable_auto_white_balance, 0)
            sensor.set_option(option, float(value))
        except BaseException as exc:
            raise CameraError(f"cannot apply fixed D435 option {option}: {exc}") from exc

    def _capture_loop(self) -> None:
        assert self._pipeline is not None
        try:
            while not self._stop.is_set():
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                arrival_ns = time.monotonic_ns()
                if self._align is not None:
                    frames = self._align.process(frames)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    self._dropped_frames += 1
                    continue
                number = int(color_frame.get_frame_number())
                if self._last_number is not None and number > self._last_number + 1:
                    self._dropped_frames += number - self._last_number - 1
                self._last_number = number
                device_ms = float(color_frame.get_timestamp())
                frame = CameraFrame(
                    color=np.asanyarray(color_frame.get_data()).copy(),
                    depth=np.asanyarray(depth_frame.get_data()).copy(),
                    capture_time_ns=self._mapper.map(device_ms, arrival_ns),
                    arrival_time_ns=arrival_ns,
                    device_timestamp_ms=device_ms,
                    frame_number=number,
                )
                with self._lock:
                    self._frames.append(frame)
                self._frames_seen += 1
        except BaseException as exc:
            if not self._stop.is_set():
                self._last_error = str(exc)
                LOG.exception("D435 capture stopped")
