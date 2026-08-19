from __future__ import annotations

import collections
import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Mapping, Protocol, runtime_checkable

import numpy as np

from .schema import CameraConfig, CameraRigConfig

LOG = logging.getLogger(__name__)


class CameraError(RuntimeError):
    pass


@dataclass(frozen=True)
class CameraFrame:
    color: np.ndarray
    depth: np.ndarray | None
    capture_time_ns: int
    arrival_time_ns: int
    device_timestamp_ms: float
    frame_number: int
    depth_device_timestamp_ms: float
    depth_frame_number: int
    color_depth_skew_ms: float
    role: str = "wrist"
    calibration_version: str = "uncalibrated-v0"
    depth_scale: float | None = None

    def __post_init__(self) -> None:
        color = np.asarray(self.color)
        if color.dtype != np.uint8 or color.ndim != 3 or color.shape[2] != 3:
            raise ValueError("camera color must be RGB uint8 HWC")
        color = np.ascontiguousarray(color).copy()
        color.setflags(write=False)
        object.__setattr__(self, "color", color)

        if self.depth is not None:
            depth = np.asarray(self.depth)
            if depth.dtype != np.uint16 or depth.shape != color.shape[:2]:
                raise ValueError("camera depth must be uint16 and match the RGB image size")
            depth = np.ascontiguousarray(depth).copy()
            depth.setflags(write=False)
            object.__setattr__(self, "depth", depth)
        if self.capture_time_ns < 0 or self.arrival_time_ns < self.capture_time_ns:
            raise ValueError("camera timestamps must be monotonic and arrival must follow capture")
        if self.frame_number < 0 or self.depth_frame_number < 0:
            raise ValueError("camera frame numbers must be non-negative")
        if not self.role or not self.calibration_version:
            raise ValueError("camera role and calibration version are required")
        if self.depth_scale is not None and (
            not np.isfinite(self.depth_scale) or self.depth_scale <= 0
        ):
            raise ValueError("depth_scale must be null or a positive finite value")


@dataclass(frozen=True)
class CameraStats:
    frames: int
    dropped_frames: int
    last_error: str | None
    runtime_s: float
    measured_fps: float
    mean_capture_latency_ms: float
    p95_capture_latency_ms: float
    max_capture_latency_ms: float
    mean_color_depth_skew_ms: float
    p95_color_depth_skew_ms: float
    max_color_depth_skew_ms: float
    device_clock_resets: int


@runtime_checkable
class Camera(Protocol):
    @property
    def role(self) -> str: ...

    @property
    def required(self) -> bool: ...

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def latest(self, *, max_age_ms: int | None = None, now_ns: int | None = None) -> CameraFrame: ...
    def nearest(self, target_ns: int, *, max_skew_ms: int | None = None) -> CameraFrame: ...
    def stats(self) -> CameraStats: ...


class CameraMetrics:
    """Bounded rolling metrics suitable for long D435 soak tests."""

    def __init__(self, *, sample_window: int = 10_000, started_ns: int | None = None) -> None:
        if sample_window <= 0:
            raise ValueError("sample_window must be positive")
        self.started_ns = time.monotonic_ns() if started_ns is None else started_ns
        self.frames = 0
        self.dropped_frames = 0
        self.capture_latency_ms: collections.deque[float] = collections.deque(maxlen=sample_window)
        self.color_depth_skew_ms: collections.deque[float] = collections.deque(maxlen=sample_window)

    def observe(self, frame: CameraFrame) -> None:
        self.frames += 1
        self.capture_latency_ms.append((frame.arrival_time_ns - frame.capture_time_ns) / 1e6)
        self.color_depth_skew_ms.append(frame.color_depth_skew_ms)

    def record_drop(self, count: int = 1) -> None:
        if count > 0:
            self.dropped_frames += count

    def snapshot(
        self,
        *,
        now_ns: int | None = None,
        last_error: str | None = None,
        device_clock_resets: int = 0,
    ) -> CameraStats:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        runtime_s = max(0.0, (now_ns - self.started_ns) / 1e9)

        def summary(values: collections.deque[float]) -> tuple[float, float, float]:
            if not values:
                return 0.0, 0.0, 0.0
            array = np.asarray(values, dtype=np.float64)
            return float(array.mean()), float(np.percentile(array, 95)), float(array.max())

        latency_mean, latency_p95, latency_max = summary(self.capture_latency_ms)
        skew_mean, skew_p95, skew_max = summary(self.color_depth_skew_ms)
        return CameraStats(
            frames=self.frames,
            dropped_frames=self.dropped_frames,
            last_error=last_error,
            runtime_s=runtime_s,
            measured_fps=0.0 if runtime_s == 0 else self.frames / runtime_s,
            mean_capture_latency_ms=latency_mean,
            p95_capture_latency_ms=latency_p95,
            max_capture_latency_ms=latency_max,
            mean_color_depth_skew_ms=skew_mean,
            p95_color_depth_skew_ms=skew_p95,
            max_color_depth_skew_ms=skew_max,
            device_clock_resets=device_clock_resets,
        )


class DeviceClockMapper:
    """Maps RealSense device milliseconds onto host monotonic nanoseconds."""

    def __init__(self) -> None:
        self._device_origin_ms: float | None = None
        self._last_device_ms: float | None = None
        self._host_origin_ns: int | None = None
        self.reset_count = 0

    def map(self, device_ms: float, arrival_ns: int) -> int:
        if self._device_origin_ms is None or (
            self._last_device_ms is not None and device_ms < self._last_device_ms
        ):
            if self._device_origin_ms is not None:
                self.reset_count += 1
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
        self._metrics = CameraMetrics()
        self._depth_scale: float | None = None
        self._last_number: int | None = None
        self._last_error: str | None = None

    @property
    def role(self) -> str:
        return self.config.name

    @property
    def required(self) -> bool:
        return self.config.required

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
            self._depth_scale = float(
                profile.get_device().first_depth_sensor().get_depth_scale()
            )
        except BaseException as exc:
            try:
                pipeline.stop()
            except BaseException:
                pass
            raise CameraError(f"cannot start D435: {exc}") from exc
        # A camera instance may be stopped and started again.  Do not let frames,
        # timing origins, or errors from a previous run skew the new soak test.
        with self._lock:
            self._frames.clear()
            self._mapper = DeviceClockMapper()
            self._metrics = CameraMetrics()
            self._last_number = None
            self._last_error = None
        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color) if self.config.align_depth_to_color else None
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._capture_loop,
            name=f"dummy-d435-{self.role}",
            daemon=True,
        )
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
        with self._lock:
            return self._metrics.snapshot(
                last_error=self._last_error,
                device_clock_resets=self._mapper.reset_count,
            )

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
                    with self._lock:
                        self._metrics.record_drop()
                    continue
                number = int(color_frame.get_frame_number())
                if self._last_number is not None and number > self._last_number + 1:
                    with self._lock:
                        self._metrics.record_drop(number - self._last_number - 1)
                self._last_number = number
                device_ms = float(color_frame.get_timestamp())
                depth_device_ms = float(depth_frame.get_timestamp())
                frame = CameraFrame(
                    # RealSense is configured for BGR for broad SDK support;
                    # normalize once at the device boundary to RGB uint8 HWC.
                    color=np.asanyarray(color_frame.get_data())[:, :, ::-1].copy(),
                    depth=np.asanyarray(depth_frame.get_data()).copy(),
                    capture_time_ns=self._mapper.map(device_ms, arrival_ns),
                    arrival_time_ns=arrival_ns,
                    device_timestamp_ms=device_ms,
                    frame_number=number,
                    depth_device_timestamp_ms=depth_device_ms,
                    depth_frame_number=int(depth_frame.get_frame_number()),
                    color_depth_skew_ms=abs(device_ms - depth_device_ms),
                    role=self.role,
                    calibration_version=self.config.calibration_version,
                    depth_scale=self._depth_scale,
                )
                with self._lock:
                    self._frames.append(frame)
                    self._metrics.observe(frame)
        except BaseException as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._last_error = str(exc)
                LOG.exception("D435 capture stopped")


class SyntheticCamera:
    """Configurable bounded camera for tests, simulation and offline adapters."""

    def __init__(self, config: CameraConfig, *, buffer_size: int = 8) -> None:
        if buffer_size <= 0:
            raise ValueError("buffer_size must be positive")
        self.config = config
        self._frames: collections.deque[CameraFrame] = collections.deque(maxlen=buffer_size)
        self._metrics = CameraMetrics()
        self._lock = threading.Lock()
        self._started = False

    @property
    def role(self) -> str:
        return self.config.name

    @property
    def required(self) -> bool:
        return self.config.required

    def start(self) -> None:
        self._started = True

    def stop(self) -> None:
        self._started = False

    def publish(self, frame: CameraFrame) -> None:
        if frame.role != self.role:
            raise CameraError(f"frame role {frame.role!r} does not match camera {self.role!r}")
        if frame.color.dtype != np.uint8 or frame.color.shape != (
            self.config.height,
            self.config.width,
            3,
        ):
            raise CameraError("synthetic frame color does not match configured RGB shape")
        with self._lock:
            self._frames.append(frame)
            self._metrics.observe(frame)

    def latest(self, *, max_age_ms: int | None = None, now_ns: int | None = None) -> CameraFrame:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        max_age_ms = self.config.max_frame_age_ms if max_age_ms is None else max_age_ms
        with self._lock:
            if not self._frames:
                raise CameraError(f"no {self.role} frame is available")
            frame = self._frames[-1]
        age_ms = (now_ns - frame.capture_time_ns) / 1e6
        if age_ms < 0 or age_ms > max_age_ms:
            raise CameraError(f"latest {self.role} frame is stale ({age_ms:.1f} ms)")
        return frame

    def nearest(self, target_ns: int, *, max_skew_ms: int | None = None) -> CameraFrame:
        max_skew_ms = self.config.max_sync_skew_ms if max_skew_ms is None else max_skew_ms
        with self._lock:
            if not self._frames:
                raise CameraError(f"no {self.role} frame is available")
            frame = min(self._frames, key=lambda item: abs(item.capture_time_ns - target_ns))
        skew_ms = abs(frame.capture_time_ns - target_ns) / 1e6
        if skew_ms > max_skew_ms:
            raise CameraError(f"{self.role}/state skew is too large ({skew_ms:.1f} ms)")
        return frame

    def stats(self) -> CameraStats:
        with self._lock:
            return self._metrics.snapshot()


class OpenCVCamera(SyntheticCamera):
    """UVC/OpenCV RGB camera with host-arrival timestamps."""

    def __init__(self, config: CameraConfig, *, buffer_size: int = 8) -> None:
        super().__init__(config, buffer_size=buffer_size)
        self._capture = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frame_number = 0
        self._last_error: str | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        try:
            import cv2
        except ImportError as exc:
            raise CameraError("install dummy-host[opencv] to use an OpenCV camera") from exc
        device: int | str = self.config.device_serial
        if isinstance(device, str) and device.isdigit():
            device = int(device)
        capture = cv2.VideoCapture(device)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        capture.set(cv2.CAP_PROP_FPS, self.config.fps)
        if not capture.isOpened():
            capture.release()
            raise CameraError(f"cannot open OpenCV camera {self.config.device_serial!r}")
        with self._lock:
            self._frames.clear()
            self._metrics = CameraMetrics()
            self._last_error = None
        self._capture = capture
        self._stop.clear()
        self._started = True
        self._thread = threading.Thread(
            target=self._capture_loop,
            name=f"dummy-camera-{self.role}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        self._started = False

    def stats(self) -> CameraStats:
        with self._lock:
            return self._metrics.snapshot(last_error=self._last_error)

    def _capture_loop(self) -> None:
        try:
            import cv2

            assert self._capture is not None
            while not self._stop.is_set():
                ok, bgr = self._capture.read()
                arrival_ns = time.monotonic_ns()
                if not ok:
                    with self._lock:
                        self._metrics.record_drop()
                    continue
                self._frame_number += 1
                frame = CameraFrame(
                    color=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
                    depth=None,
                    capture_time_ns=arrival_ns,
                    arrival_time_ns=arrival_ns,
                    device_timestamp_ms=float("nan"),
                    frame_number=self._frame_number,
                    depth_device_timestamp_ms=float("nan"),
                    depth_frame_number=0,
                    color_depth_skew_ms=0.0,
                    role=self.role,
                    calibration_version=self.config.calibration_version,
                )
                with self._lock:
                    self._frames.append(frame)
                    self._metrics.observe(frame)
        except BaseException as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._last_error = str(exc)
                LOG.exception("OpenCV camera %s stopped", self.role)


class CameraManager:
    """Own cameras by stable logical role and provide synchronized snapshots."""

    def __init__(self, cameras: Mapping[str, Camera]) -> None:
        self._cameras = dict(cameras)
        if not self._cameras:
            raise ValueError("CameraManager requires at least one camera")
        for role, camera in self._cameras.items():
            if role != camera.role:
                raise ValueError(f"camera mapping role {role!r} does not match {camera.role!r}")
        self._started: list[Camera] = []

    @classmethod
    def from_config(
        cls,
        rig: CameraRigConfig,
        *,
        factories: Mapping[str, Callable[[CameraConfig], Camera]] | None = None,
    ) -> "CameraManager":
        builders: dict[str, Callable[[CameraConfig], Camera]] = {
            "realsense": D435Camera,
            "opencv": OpenCVCamera,
            "fake": SyntheticCamera,
        }
        if factories:
            builders.update(factories)
        cameras: dict[str, Camera] = {}
        for role, config in rig.cameras.items():
            if not config.enabled:
                continue
            builder = builders.get(config.driver)
            if builder is None:
                raise CameraError(f"no camera factory registered for driver {config.driver!r}")
            cameras[role] = builder(config)
        return cls(cameras)

    @property
    def roles(self) -> tuple[str, ...]:
        return tuple(self._cameras)

    def get(self, role: str) -> Camera:
        try:
            return self._cameras[role]
        except KeyError as exc:
            raise CameraError(f"camera role {role!r} is not configured") from exc

    def start(self) -> None:
        if self._started:
            return
        for camera in self._cameras.values():
            try:
                camera.start()
                self._started.append(camera)
            except BaseException:
                if camera.required:
                    self.stop()
                    raise
                LOG.exception("optional camera %s failed to start", camera.role)
                try:
                    camera.stop()
                except BaseException:
                    LOG.exception("failed to clean up optional camera %s", camera.role)

    def stop(self) -> None:
        for camera in reversed(self._started):
            try:
                camera.stop()
            except BaseException:
                LOG.exception("failed to stop camera %s", camera.role)
        self._started.clear()

    def nearest_all(self, target_ns: int, *, required_only: bool = False) -> dict[str, CameraFrame]:
        frames: dict[str, CameraFrame] = {}
        errors: list[str] = []
        for role, camera in self._cameras.items():
            if required_only and not camera.required:
                continue
            try:
                frames[role] = camera.nearest(target_ns)
            except CameraError as exc:
                if camera.required:
                    errors.append(f"{role}: {exc}")
        if errors:
            raise CameraError("; ".join(errors))
        return frames

    def latest_all(self, *, now_ns: int | None = None) -> dict[str, CameraFrame]:
        frames: dict[str, CameraFrame] = {}
        errors: list[str] = []
        for role, camera in self._cameras.items():
            try:
                frames[role] = camera.latest(now_ns=now_ns)
            except CameraError as exc:
                if camera.required:
                    errors.append(f"{role}: {exc}")
        if errors:
            raise CameraError("; ".join(errors))
        return frames

    def stats(self) -> dict[str, CameraStats]:
        return {role: camera.stats() for role, camera in self._cameras.items()}
