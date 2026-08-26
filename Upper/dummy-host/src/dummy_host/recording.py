from __future__ import annotations

import hashlib
import json
import math
import os
import queue
import sqlite3
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Mapping

import numpy as np

from .cameras import CameraFrame
from .frame_archive import DEFAULT_MINIMUM_FREE_BYTES, FrameArchive, NpzFrameArchive
from .domain import ActionLifecycleUpdate, ActionStage
from .kinematics.calibration import CartesianCalibration
from .protocol import PROTOCOL_VERSION, CanDiagnostics
from .schema import AppliedAction, RobotConfig, RobotState
from .teleop import TeleopCommand, TeleopProfile
from .time_sync import TimeSyncExchange, TimeSyncModel


class RecorderError(RuntimeError):
    pass


class RecorderBackpressure(RecorderError):
    pass


def estimate_camera_archive_bytes(robot_config: RobotConfig, duration_s: float) -> int:
    """Conservative upper bound for atomic, uncompressed camera capture."""

    if not np.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("duration_s must be positive and finite")
    samples = math.ceil(duration_s * robot_config.control_rate_hz)
    bytes_per_sample = 0
    for camera in robot_config.camera_rig.cameras.values():
        if not camera.enabled:
            continue
        pixels = camera.width * camera.height
        bytes_per_sample += pixels * 3
        if camera.depth_format.strip().lower() not in {"", "none"}:
            bytes_per_sample += pixels * 2
        # ZIP container headers and scalar metadata arrays. The 5% multiplier
        # below covers filesystem allocation and future metadata additions.
        bytes_per_sample += 4096
    return math.ceil(samples * bytes_per_sample * 1.05)


@dataclass(frozen=True)
class RecorderStats:
    samples: int
    events: int
    camera_frames: int
    queue_high_watermark: int


@dataclass(frozen=True)
class ControlTickTiming:
    raw_tick_index: int
    planned_ns: int
    actual_start_ns: int
    actual_end_ns: int
    target_generated_ns: int | None = None
    send_enqueued_ns: int | None = None
    missed_periods: int = 0
    next_rebase_deadline_ns: int = 0
    transport_diagnostics: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        values = (
            self.raw_tick_index,
            self.planned_ns,
            self.actual_start_ns,
            self.actual_end_ns,
            self.missed_periods,
            self.next_rebase_deadline_ns,
        )
        if any(value < 0 for value in values) or self.actual_end_ns < self.actual_start_ns:
            raise ValueError("control tick timing is invalid")


@dataclass(frozen=True)
class _SampleRecord:
    command: TeleopCommand
    state: RobotState
    requested_action: np.ndarray | None
    applied_action: np.ndarray | None
    action_sequence: int | None
    action_clipped: bool
    action_reasons: tuple[str, ...]
    camera_frames: Mapping[str, CameraFrame]
    valid: bool
    invalid_reason: str | None
    timing: ControlTickTiming
    session_epoch: int
    control_tick_id: int
    time_sync_model_id: int | None


@dataclass(frozen=True)
class _EventRecord:
    monotonic_ns: int
    event: str
    payload: Mapping[str, object]


@dataclass(frozen=True)
class _TimeSyncRecord:
    exchange: TimeSyncExchange
    model: TimeSyncModel | None


@dataclass(frozen=True)
class _CanDiagnosticsRecord:
    host_time_ns: int
    diagnostics: CanDiagnostics


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _float32_blob(value: np.ndarray | None) -> bytes | None:
    if value is None:
        return None
    return np.asarray(value, dtype="<f4").tobytes(order="C")


class SessionRecorder:
    """Bounded asynchronous raw-session recorder.

    The control thread only copies one sample and performs put_nowait(). SQLite,
    JSONL, multi-camera frame compression and checksums are handled off-thread. A
    full queue is a safety error, not permission to silently drop control data.
    """

    def __init__(
        self,
        root: str | Path,
        robot_config: RobotConfig,
        teleop_profile: TeleopProfile,
        *,
        source: str,
        firmware_version: str | None = None,
        queue_size: int = 64,
        frame_segment_size: int = 300,
        frame_archive_factory: Callable[[Path], FrameArchive] | None = None,
        minimum_camera_free_bytes: int = DEFAULT_MINIMUM_FREE_BYTES,
        session_name: str | None = None,
        extra_manifest: Mapping[str, object] | None = None,
    ) -> None:
        if queue_size <= 0:
            raise ValueError("queue_size must be positive")
        if frame_segment_size <= 0:
            raise ValueError("frame_segment_size must be positive")
        if minimum_camera_free_bytes < 0:
            raise ValueError("minimum_camera_free_bytes must be non-negative")
        if not source:
            raise ValueError("source must be non-empty")
        if session_name is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_name = f"session_{timestamp}_{os.getpid()}_{time.time_ns() % 1_000_000_000:09d}"
        if Path(session_name).name != session_name:
            raise ValueError("session_name must be one directory name")

        self.session_dir = Path(root) / session_name
        self.session_dir.mkdir(parents=True, exist_ok=False)
        self.frames_dir = self.session_dir / "frames"
        self.frames_dir.mkdir(parents=True)
        self.db_path = self.session_dir / "samples.sqlite"
        self.events_path = self.session_dir / "events.jsonl"
        self.manifest_path = self.session_dir / "manifest.json"
        self.checksums_path = self.session_dir / "checksums.json"
        archived_calibrations: dict[str, dict[str, object]] = {}
        if robot_config.camera_rig.calibrations:
            calibrations_dir = self.session_dir / "calibrations"
            calibrations_dir.mkdir()
            for role, calibration in robot_config.camera_rig.calibrations.items():
                archive_path = calibrations_dir / f"{role}.yaml"
                try:
                    archive_path.write_bytes(Path(calibration.source_path).read_bytes())
                except OSError as exc:
                    raise RecorderError(
                        f"cannot archive camera calibration for {role}: {exc}"
                    ) from exc
                archived_hash = hashlib.sha256(archive_path.read_bytes()).hexdigest()
                if archived_hash != calibration.file_hash:
                    raise RecorderError(
                        f"camera calibration for {role} changed while the session was starting"
                    )
                archived_calibrations[role] = {
                    "calibration_id": calibration.calibration_id,
                    "sha256": calibration.file_hash,
                    "archive_path": archive_path.relative_to(self.session_dir).as_posix(),
                    "source_path": calibration.source_path,
                    "calibrated_utc": calibration.calibrated_utc,
                    "parent_frame": calibration.parent_frame,
                }
        self._frame_segment_size = frame_segment_size
        self._frame_archive_factory = frame_archive_factory or (
            lambda session_dir: NpzFrameArchive(
                session_dir,
                segment_size=self._frame_segment_size,
                compressed=False,
                sync_files=True,
                minimum_free_bytes=minimum_camera_free_bytes,
            )
        )
        self._queue: queue.Queue[tuple[str, object] | None] = queue.Queue(maxsize=queue_size)
        # Sample producers stop before consuming the whole queue so shutdown,
        # HOLD and Episode audit events still have bounded space available.
        self._critical_event_reserve = min(4, max(0, queue_size - 1))
        self._sample_queue_limit = queue_size - self._critical_event_reserve
        self._thread = threading.Thread(target=self._writer, name="dummy-session-writer", daemon=True)
        self._writer_error: BaseException | None = None
        self._closed = False
        self._samples = 0
        self._events = 0
        self._camera_frames = 0
        self._queue_high_watermark = 0
        self._stats_lock = threading.Lock()
        self._time_sync_lock = threading.Lock()
        self._latest_time_sync_model_id: int | None = None
        self._session_epoch = 0
        self._manifest: dict[str, object] = {
            "schema_version": 5,
            "state_telemetry_version": 4,
            "binary_protocol_version": PROTOCOL_VERSION,
            "created_utc": _utc_now(),
            "closed_utc": None,
            "clean_shutdown": False,
            "robot_id": robot_config.robot_id,
            "robot_config_version": robot_config.config_version,
            "robot_config_hash": robot_config.config_hash,
            "robot_calibration_id": robot_config.robot_calibration_id,
            "hardware_parameters_verified": robot_config.hardware_parameters_verified,
            "external_target_execution_ready": robot_config.external_target_execution_ready,
            "camera_rig_id": robot_config.camera_rig.rig_id,
            "camera_rig_version": robot_config.camera_rig.version,
            "camera_rig_hash": robot_config.camera_rig.config_hash,
            "camera_calibrations": archived_calibrations,
            "cartesian_calibration": None,
            "teleop_config_version": teleop_profile.version,
            "teleop_config_hash": teleop_profile.config_hash,
            "action_source": source,
            "firmware_version": firmware_version,
            "session_epoch": None,
            "control_rate_hz": robot_config.control_rate_hz,
            "joint_order": list(robot_config.joint_order),
            "joint_unit": robot_config.joint_unit,
            "array_encoding": "little-endian float32 blobs",
            "timestamp_chain_version": 2,
            "timestamp_chain": [
                "input_event_ns",
                "input_snapshot_ns",
                "control_planned_ns",
                "control_actual_start_ns",
                "control_actual_end_ns",
                "target_generated_ns",
                "send_enqueued_ns",
                "serial_send_started_host_ns",
                "serial_send_finished_host_ns",
                "acknowledged_host_ns",
                "acknowledged_mcu_us",
                "can_queued_exact_host_ns",
                "can_queued_exact_mcu_us",
                "can_tx_complete_exact_host_ns",
                "can_tx_complete_exact_mcu_us",
                "post_command_feedback_host_ns",
                "post_command_feedback_mcu_us",
                "state_host_ns",
                "state_mcu_us",
                "camera_capture_ns",
                "camera_arrival_ns",
            ],
            "action_lifecycle": [
                "received",
                "safety_accepted",
                "send_enqueued",
                "serial_send_started",
                "serial_send_finished",
                "acknowledged",
                "can_queued_exact",
                "can_tx_complete_exact",
                "post_command_feedback",
                "superseded",
                "preempted_by_safety",
                "rejected",
                "failed",
            ],
            "camera_archive": (
                "atomic lossless uncompressed RGB/depth NPZ by role; "
                "compression deferred to offline dataset export"
            ),
            "camera_archive_compression": "none",
            "camera_archive_atomic_commit": True,
            "camera_archive_minimum_free_bytes": minimum_camera_free_bytes,
            "recorder_queue_capacity": queue_size,
            "recorder_critical_event_reserve": self._critical_event_reserve,
            "time_sync_rate_hz": 2,
            "time_sync_model": "filtered four-timestamp affine MCU-us to host-monotonic-ns",
            "can_diagnostics_rate_hz": 1,
        }
        if extra_manifest:
            self._manifest["extra"] = dict(extra_manifest)
        self._write_json_atomic(self.manifest_path, self._manifest)
        self._thread.start()

    @property
    def stats(self) -> RecorderStats:
        with self._stats_lock:
            return RecorderStats(
                self._samples,
                self._events,
                self._camera_frames,
                self._queue_high_watermark,
            )

    def update_runtime_metadata(
        self, *, firmware_version: str, session_epoch: int | None = None
    ) -> None:
        self._require_open()
        if not firmware_version:
            raise ValueError("firmware_version must be non-empty")
        self._manifest["firmware_version"] = firmware_version
        if session_epoch is not None:
            if not 0 < session_epoch <= 0xFFFFFFFF:
                raise ValueError("session_epoch must be a non-zero uint32")
            self._session_epoch = session_epoch
            self._manifest["session_epoch"] = session_epoch
        self._write_json_atomic(self.manifest_path, self._manifest)

    def archive_cartesian_calibration(
        self, calibration: CartesianCalibration
    ) -> None:
        """Archive the exact validated identity used by Cartesian control."""

        self._require_open()
        if self._manifest.get("cartesian_calibration") is not None:
            raise RecorderError("Cartesian calibration is already archived")
        calibrations_dir = self.session_dir / "calibrations"
        calibrations_dir.mkdir(exist_ok=True)
        archive_path = calibrations_dir / "cartesian.yaml"
        try:
            archive_path.write_bytes(Path(calibration.source_path).read_bytes())
        except OSError as exc:
            raise RecorderError(f"cannot archive Cartesian calibration: {exc}") from exc
        archived_hash = hashlib.sha256(archive_path.read_bytes()).hexdigest()
        if archived_hash != calibration.file_hash:
            raise RecorderError(
                "Cartesian calibration changed while the session was starting"
            )
        record = calibration.as_dict()
        record["archive_path"] = archive_path.relative_to(self.session_dir).as_posix()
        self._manifest["cartesian_calibration"] = record
        self._write_json_atomic(self.manifest_path, self._manifest)

    def record_sample(
        self,
        command: TeleopCommand,
        state: RobotState,
        *,
        action: AppliedAction | None = None,
        camera_frame: CameraFrame | None = None,
        camera_frames: Mapping[str, CameraFrame] | None = None,
        valid: bool = True,
        invalid_reason: str | None = None,
        timing: ControlTickTiming | None = None,
    ) -> None:
        self._require_open()
        if camera_frame is not None and camera_frames is not None:
            raise ValueError("provide camera_frame or camera_frames, not both")
        frames = ({camera_frame.role: camera_frame} if camera_frame is not None else dict(camera_frames or {}))
        for role, frame in frames.items():
            if role != frame.role:
                raise ValueError(f"camera frame role {frame.role!r} does not match key {role!r}")
        # Camera adapters own immutable, already-copied frame arrays. Keep bounded
        # reference here so the 20 Hz control thread does not copy megabytes of
        # image data; the writer releases the reference after persisting it.
        frame_copy = frames
        timing = timing or ControlTickTiming(
            raw_tick_index=0,
            planned_ns=command.monotonic_ns,
            actual_start_ns=command.monotonic_ns,
            actual_end_ns=command.monotonic_ns,
        )
        record = _SampleRecord(
            command=command,
            state=RobotState(
                position=state.position.astype(np.float32, copy=True),
                velocity=state.velocity.astype(np.float32, copy=True),
                monotonic_ns=state.monotonic_ns,
                mcu_time_us=state.mcu_time_us,
                mode=state.mode,
                fault_bits=state.fault_bits,
                position_valid=state.position_valid,
                velocity_valid=state.velocity_valid,
                gripper_valid=state.gripper_valid,
                last_received_sequence=state.last_received_sequence,
                target_age_ms=state.target_age_ms,
                config_hash=state.config_hash,
                following_error=state.following_error.astype(np.float32, copy=True),
                following_error_duration_ms=state.following_error_duration_ms.copy(),
                feedback_age_ms=state.feedback_age_ms.copy(),
                feedback_loss_count=state.feedback_loss_count.copy(),
                consecutive_feedback_loss=state.consecutive_feedback_loss.copy(),
                node_fault_bits=state.node_fault_bits.copy(),
                node_validity=state.node_validity.copy(),
                hold_reason_bits=state.hold_reason_bits,
                telemetry_validity=state.telemetry_validity,
                can_transport_status=state.can_transport_status,
                feedback_sample_mcu_us=state.feedback_sample_mcu_us.copy(),
                feedback_sweep_id=state.feedback_sweep_id.copy(),
                coherent_sweep_id=state.coherent_sweep_id,
                feedback_max_skew_us=state.feedback_max_skew_us,
                coherent_reference_mcu_us=state.coherent_reference_mcu_us,
                state_repeated=state.state_repeated,
                action_progress=state.action_progress,
            ),
            requested_action=None if action is None else action.requested.astype(np.float32, copy=True),
            applied_action=None if action is None else action.applied.astype(np.float32, copy=True),
            action_sequence=None if action is None else action.sequence,
            action_clipped=False if action is None else action.clipped,
            action_reasons=() if action is None else action.reasons,
            camera_frames=frame_copy,
            valid=valid,
            invalid_reason=invalid_reason,
            timing=timing,
            session_epoch=(
                action.session_epoch
                if action is not None and action.session_epoch > 0
                else self._session_epoch
            ),
            control_tick_id=0 if action is None else action.control_tick_id,
            time_sync_model_id=self._current_time_sync_model_id(),
        )
        self._enqueue(("sample", record), sample=True)

    def record_event(
        self, event: str, *, monotonic_ns: int | None = None, payload: Mapping[str, object] | None = None
    ) -> None:
        self._require_open()
        if not event:
            raise ValueError("event must be non-empty")
        self._enqueue(
            (
                "event",
                _EventRecord(
                    time.monotonic_ns() if monotonic_ns is None else monotonic_ns,
                    event,
                    {} if payload is None else dict(payload),
                ),
            )
        )

    def record_action_lifecycle(self, update: ActionLifecycleUpdate) -> None:
        self._require_open()
        self._enqueue(("action_lifecycle", update))

    def record_time_sync(
        self, exchange: TimeSyncExchange, model: TimeSyncModel | None
    ) -> None:
        self._require_open()
        # Publish the model to sample producers only after its writer item is
        # ahead of every sample that may reference it.
        self._enqueue(("time_sync", _TimeSyncRecord(exchange, model)))
        if model is not None:
            with self._time_sync_lock:
                self._latest_time_sync_model_id = model.model_id

    def record_can_diagnostics(
        self, diagnostics: CanDiagnostics, *, host_time_ns: int | None = None
    ) -> None:
        self._require_open()
        timestamp = time.monotonic_ns() if host_time_ns is None else host_time_ns
        if timestamp < 0:
            raise ValueError("CAN diagnostics host timestamp must be non-negative")
        self._enqueue(
            ("can_diagnostics", _CanDiagnosticsRecord(timestamp, diagnostics))
        )

    def _current_time_sync_model_id(self) -> int | None:
        with self._time_sync_lock:
            return self._latest_time_sync_model_id

    def close(self, *, clean_shutdown: bool = True) -> RecorderStats:
        if self._closed:
            return self.stats
        self._closed = True
        while self._thread.is_alive():
            try:
                self._queue.put(None, timeout=0.1)
                break
            except queue.Full:
                self._raise_writer_error()
        self._thread.join(timeout=30.0)
        if self._thread.is_alive():
            raise RecorderError("session writer did not stop within 30 seconds")
        self._raise_writer_error()
        stats = self.stats
        self._manifest.update(
            {
                "closed_utc": _utc_now(),
                "clean_shutdown": bool(clean_shutdown),
                "stats": asdict(stats),
            }
        )
        self._write_json_atomic(self.manifest_path, self._manifest)
        checksums = self._build_checksums()
        self._write_json_atomic(self.checksums_path, checksums)
        return stats

    def _enqueue(self, item: tuple[str, object], *, sample: bool = False) -> None:
        self._raise_writer_error()
        if sample and self._queue.qsize() >= self._sample_queue_limit:
            raise RecorderBackpressure(
                "recorder sample queue reached its safety limit; HOLD without "
                f"dropping data ({self._sample_queue_limit}/{self._queue.maxsize}, "
                f"{self._critical_event_reserve} slots reserved for critical events)"
            )
        try:
            self._queue.put_nowait(item)
        except queue.Full as exc:
            raise RecorderBackpressure(
                "recorder queue is full; HOLD instead of dropping data"
            ) from exc
        with self._stats_lock:
            self._queue_high_watermark = max(self._queue_high_watermark, self._queue.qsize())

    def _writer(self) -> None:
        connection = None
        events = None
        try:
            connection = sqlite3.connect(self.db_path)
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute("PRAGMA synchronous=FULL")
            self._create_schema(connection)
            events = self.events_path.open("a", encoding="utf-8", newline="\n", buffering=1)
            frame_archive = self._frame_archive_factory(self.session_dir)
            uncommitted = 0
            while True:
                item = self._queue.get()
                if item is None:
                    break
                kind, payload = item
                if kind == "sample":
                    assert isinstance(payload, _SampleRecord)
                    self._write_sample(connection, payload, frame_archive)
                    with self._stats_lock:
                        self._samples += 1
                    uncommitted += 1
                elif kind == "event":
                    assert isinstance(payload, _EventRecord)
                    event_line = {
                        "monotonic_ns": payload.monotonic_ns,
                        "event": payload.event,
                        "payload": payload.payload,
                    }
                    events.write(json.dumps(event_line, separators=(",", ":"), ensure_ascii=False) + "\n")
                    with self._stats_lock:
                        self._events += 1
                    uncommitted += 1
                elif kind == "action_lifecycle":
                    assert isinstance(payload, ActionLifecycleUpdate)
                    self._write_action_lifecycle(connection, payload)
                    uncommitted += 1
                elif kind == "time_sync":
                    assert isinstance(payload, _TimeSyncRecord)
                    self._write_time_sync(connection, payload)
                    uncommitted += 1
                elif kind == "can_diagnostics":
                    assert isinstance(payload, _CanDiagnosticsRecord)
                    self._write_can_diagnostics(connection, payload)
                    uncommitted += 1
                else:
                    raise RecorderError(f"unknown recorder item {kind}")
                if uncommitted >= 20:
                    connection.commit()
                    uncommitted = 0
            connection.commit()
            connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
            frame_archive.close()
        except BaseException as exc:
            self._writer_error = exc
        finally:
            if events is not None:
                events.close()
            if connection is not None:
                connection.close()

    @staticmethod
    def _create_schema(connection: sqlite3.Connection) -> None:
        connection.execute(
            """
            CREATE TABLE IF NOT EXISTS samples (
                sample_index INTEGER PRIMARY KEY AUTOINCREMENT,
                tick_ns INTEGER NOT NULL,
                raw_tick_index INTEGER NOT NULL,
                session_epoch INTEGER NOT NULL,
                control_tick_id INTEGER NOT NULL,
                time_sync_model_id INTEGER,
                input_event_ns INTEGER NOT NULL,
                input_snapshot_ns INTEGER NOT NULL,
                control_planned_ns INTEGER NOT NULL,
                control_actual_start_ns INTEGER NOT NULL,
                control_actual_end_ns INTEGER NOT NULL,
                control_missed_periods INTEGER NOT NULL,
                next_rebase_deadline_ns INTEGER NOT NULL,
                transport_diagnostics_json TEXT NOT NULL,
                target_generated_ns INTEGER,
                send_enqueued_ns INTEGER,
                source TEXT NOT NULL,
                teleop_mode TEXT NOT NULL,
                connected INTEGER NOT NULL,
                deadman INTEGER NOT NULL,
                hold_requested INTEGER NOT NULL,
                estop_requested INTEGER NOT NULL,
                episode_event TEXT,
                joint_velocity_rad_s BLOB NOT NULL,
                cartesian_twist BLOB NOT NULL,
                gripper_velocity_per_s REAL NOT NULL,
                raw_input_json TEXT NOT NULL,
                requested_action BLOB,
                applied_action BLOB,
                action_sequence INTEGER,
                action_clipped INTEGER NOT NULL,
                action_reasons_json TEXT NOT NULL,
                state_position BLOB NOT NULL,
                state_velocity BLOB NOT NULL,
                state_host_ns INTEGER NOT NULL,
                state_mcu_us INTEGER NOT NULL,
                state_mode INTEGER NOT NULL,
                state_fault_bits INTEGER NOT NULL,
                position_valid INTEGER NOT NULL,
                velocity_valid INTEGER NOT NULL,
                gripper_valid INTEGER NOT NULL,
                last_received_sequence INTEGER NOT NULL,
                target_age_ms INTEGER NOT NULL,
                state_following_error BLOB NOT NULL,
                following_error_duration_ms BLOB NOT NULL,
                feedback_age_ms BLOB NOT NULL,
                feedback_loss_count BLOB NOT NULL,
                consecutive_feedback_loss BLOB NOT NULL,
                node_fault_bits BLOB NOT NULL,
                node_validity BLOB NOT NULL,
                state_can_transport_status INTEGER NOT NULL,
                state_hold_reason_bits INTEGER NOT NULL,
                state_telemetry_validity INTEGER NOT NULL,
                feedback_sample_mcu_us BLOB NOT NULL,
                feedback_sweep_id BLOB NOT NULL,
                coherent_sweep_id INTEGER NOT NULL,
                feedback_max_skew_us INTEGER NOT NULL,
                coherent_reference_mcu_us INTEGER NOT NULL,
                state_repeated INTEGER NOT NULL,
                action_progress_json TEXT NOT NULL,
                camera_frame_number INTEGER,
                camera_capture_ns INTEGER,
                camera_arrival_ns INTEGER,
                camera_color_depth_skew_ms REAL,
                sample_valid INTEGER NOT NULL,
                invalid_reason TEXT
            )
            """
        )
        connection.execute(
            """
            CREATE TABLE IF NOT EXISTS camera_samples (
                sample_index INTEGER NOT NULL,
                role TEXT NOT NULL,
                frame_number INTEGER NOT NULL,
                capture_ns INTEGER NOT NULL,
                arrival_ns INTEGER NOT NULL,
                timestamp_source TEXT NOT NULL,
                color_depth_skew_ms REAL NOT NULL,
                calibration_version TEXT NOT NULL,
                frame_path TEXT NOT NULL,
                PRIMARY KEY (sample_index, role),
                FOREIGN KEY (sample_index) REFERENCES samples(sample_index)
            )
            """
        )
        connection.execute(
            """
            CREATE TABLE IF NOT EXISTS action_lifecycle (
                action_sequence INTEGER PRIMARY KEY,
                session_epoch INTEGER NOT NULL DEFAULT 0,
                control_tick_id INTEGER NOT NULL DEFAULT 0,
                received_host_ns INTEGER,
                safety_accepted_host_ns INTEGER,
                send_enqueued_host_ns INTEGER,
                serial_send_started_host_ns INTEGER,
                serial_send_finished_host_ns INTEGER,
                acknowledged_host_ns INTEGER,
                acknowledged_mcu_us INTEGER,
                can_queued_exact_host_ns INTEGER,
                can_queued_exact_mcu_us INTEGER,
                can_tx_complete_exact_host_ns INTEGER,
                can_tx_complete_exact_mcu_us INTEGER,
                post_command_feedback_host_ns INTEGER,
                post_command_feedback_mcu_us INTEGER,
                accepted_to_ack_host_ns INTEGER,
                ack_to_can_tx_complete_us INTEGER,
                can_tx_complete_to_post_feedback_us INTEGER,
                terminal_stage TEXT,
                detail TEXT
            )
            """
        )
        connection.execute(
            """
            CREATE TABLE IF NOT EXISTS time_sync_exchanges (
                exchange_id INTEGER PRIMARY KEY AUTOINCREMENT,
                host_t0_ns INTEGER NOT NULL,
                mcu_rx_us INTEGER NOT NULL,
                mcu_tx_us INTEGER NOT NULL,
                host_t3_ns INTEGER NOT NULL,
                rtt_ns INTEGER NOT NULL,
                model_id INTEGER
            )
            """
        )
        connection.execute(
            """
            CREATE TABLE IF NOT EXISTS time_sync_models (
                model_id INTEGER PRIMARY KEY,
                segment_id INTEGER NOT NULL,
                slope_ns_per_us REAL NOT NULL,
                intercept_ns REAL NOT NULL,
                rtt_ns INTEGER NOT NULL,
                residual_ns REAL NOT NULL,
                sample_count INTEGER NOT NULL,
                created_host_ns INTEGER NOT NULL
            )
            """
        )
        connection.execute(
            """
            CREATE TABLE IF NOT EXISTS can_diagnostics (
                diagnostic_index INTEGER PRIMARY KEY AUTOINCREMENT,
                host_time_ns INTEGER NOT NULL,
                window_start_us INTEGER NOT NULL,
                window_duration_us INTEGER NOT NULL,
                target_tx_complete_json TEXT NOT NULL,
                position_response_json TEXT NOT NULL,
                temperature_response_json TEXT NOT NULL,
                position_timeout_count INTEGER NOT NULL,
                temperature_timeout_count INTEGER NOT NULL,
                tx_abort_count INTEGER NOT NULL,
                tx_error_count INTEGER NOT NULL,
                tx_recovery_count INTEGER NOT NULL,
                safety_preemption_count INTEGER NOT NULL,
                max_safety_wait_us INTEGER NOT NULL,
                max_fanout_us INTEGER NOT NULL
            )
            """
        )

    def _write_sample(
        self,
        connection: sqlite3.Connection,
        record: _SampleRecord,
        frame_archive: FrameArchive,
    ) -> None:
        command = record.command
        state = record.state
        frame = record.camera_frames.get("wrist")
        cursor = connection.execute(
            """
            INSERT INTO samples (
                tick_ns, raw_tick_index, session_epoch, control_tick_id,
                time_sync_model_id, input_event_ns, input_snapshot_ns,
                control_planned_ns, control_actual_start_ns, control_actual_end_ns,
                control_missed_periods, next_rebase_deadline_ns,
                transport_diagnostics_json, target_generated_ns,
                send_enqueued_ns, source, teleop_mode, connected, deadman,
                hold_requested, estop_requested,
                episode_event, joint_velocity_rad_s, cartesian_twist,
                gripper_velocity_per_s, raw_input_json,
                requested_action, applied_action, action_sequence, action_clipped,
                action_reasons_json, state_position, state_velocity, state_host_ns, state_mcu_us,
                state_mode, state_fault_bits, position_valid, velocity_valid, gripper_valid,
                last_received_sequence, target_age_ms,
                state_following_error, following_error_duration_ms, feedback_age_ms,
                feedback_loss_count, consecutive_feedback_loss, node_fault_bits,
                node_validity, state_can_transport_status,
                state_hold_reason_bits, state_telemetry_validity,
                feedback_sample_mcu_us, feedback_sweep_id, coherent_sweep_id,
                feedback_max_skew_us, coherent_reference_mcu_us,
                state_repeated, action_progress_json,
                camera_frame_number, camera_capture_ns, camera_arrival_ns,
                camera_color_depth_skew_ms, sample_valid, invalid_reason
            ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
            """,
            (
                command.monotonic_ns,
                record.timing.raw_tick_index,
                record.session_epoch,
                record.control_tick_id,
                record.time_sync_model_id,
                command.event_ns if command.event_ns is not None else command.monotonic_ns,
                command.monotonic_ns,
                record.timing.planned_ns,
                record.timing.actual_start_ns,
                record.timing.actual_end_ns,
                record.timing.missed_periods,
                record.timing.next_rebase_deadline_ns,
                json.dumps(
                    dict(record.timing.transport_diagnostics),
                    separators=(",", ":"),
                    sort_keys=True,
                ),
                record.timing.target_generated_ns,
                record.timing.send_enqueued_ns,
                command.source,
                command.teleop_mode,
                int(command.connected),
                int(command.deadman),
                int(command.hold_requested),
                int(command.estop_requested),
                command.episode_event,
                _float32_blob(command.joint_velocity_rad_s),
                _float32_blob(command.cartesian_twist),
                command.gripper_velocity_per_s,
                json.dumps(command.raw, separators=(",", ":"), ensure_ascii=False),
                _float32_blob(record.requested_action),
                _float32_blob(record.applied_action),
                record.action_sequence,
                int(record.action_clipped),
                json.dumps(record.action_reasons, separators=(",", ":")),
                _float32_blob(state.position),
                _float32_blob(state.velocity),
                state.monotonic_ns,
                state.mcu_time_us,
                int(state.mode),
                state.fault_bits,
                int(state.position_valid),
                int(state.velocity_valid),
                int(state.gripper_valid),
                state.last_received_sequence,
                state.target_age_ms,
                _float32_blob(state.following_error),
                np.asarray(state.following_error_duration_ms, dtype="<u4").tobytes(),
                np.asarray(state.feedback_age_ms, dtype="<u4").tobytes(),
                np.asarray(state.feedback_loss_count, dtype="<u4").tobytes(),
                np.asarray(state.consecutive_feedback_loss, dtype="<u2").tobytes(),
                np.asarray(state.node_fault_bits, dtype="<u2").tobytes(),
                np.asarray(state.node_validity, dtype="u1").tobytes(),
                state.can_transport_status,
                state.hold_reason_bits,
                state.telemetry_validity,
                np.asarray(state.feedback_sample_mcu_us, dtype="<u8").tobytes(),
                np.asarray(state.feedback_sweep_id, dtype="<u4").tobytes(),
                state.coherent_sweep_id,
                state.feedback_max_skew_us,
                state.coherent_reference_mcu_us,
                int(state.state_repeated),
                json.dumps(
                    [asdict(progress) for progress in state.action_progress],
                    separators=(",", ":"),
                ),
                None if frame is None else frame.frame_number,
                None if frame is None else frame.capture_time_ns,
                None if frame is None else frame.arrival_time_ns,
                None if frame is None else frame.color_depth_skew_ms,
                int(record.valid),
                record.invalid_reason,
            ),
        )
        sample_index = int(cursor.lastrowid)
        for role, camera_frame in record.camera_frames.items():
            before = frame_archive.unique_frames
            frame_path = frame_archive.write(camera_frame)
            connection.execute(
                """
                INSERT INTO camera_samples (
                    sample_index, role, frame_number, capture_ns, arrival_ns,
                    color_depth_skew_ms, calibration_version, frame_path,
                    timestamp_source
                ) VALUES (?,?,?,?,?,?,?,?,?)
                """,
                (
                    sample_index,
                    role,
                    camera_frame.frame_number,
                    camera_frame.capture_time_ns,
                    camera_frame.arrival_time_ns,
                    camera_frame.color_depth_skew_ms,
                    camera_frame.calibration_version,
                    frame_path,
                    camera_frame.timestamp_source,
                ),
            )
            if frame_archive.unique_frames > before:
                with self._stats_lock:
                    self._camera_frames += 1

    @staticmethod
    def _write_action_lifecycle(
        connection: sqlite3.Connection, update: ActionLifecycleUpdate
    ) -> None:
        connection.execute(
            """
            INSERT INTO action_lifecycle (
                action_sequence, session_epoch, control_tick_id
            ) VALUES (?,?,?)
            ON CONFLICT(action_sequence) DO UPDATE SET
                session_epoch = CASE
                    WHEN excluded.session_epoch != 0 THEN excluded.session_epoch
                    ELSE action_lifecycle.session_epoch END,
                control_tick_id = CASE
                    WHEN excluded.control_tick_id != 0 THEN excluded.control_tick_id
                    ELSE action_lifecycle.control_tick_id END
            """,
            (update.sequence, update.session_epoch, update.control_tick_id),
        )
        columns = {
            ActionStage.RECEIVED: ("received_host_ns",),
            ActionStage.SAFETY_ACCEPTED: ("safety_accepted_host_ns",),
            ActionStage.SEND_ENQUEUED: ("send_enqueued_host_ns",),
            ActionStage.SERIAL_SEND_STARTED: ("serial_send_started_host_ns",),
            ActionStage.SERIAL_SEND_FINISHED: ("serial_send_finished_host_ns",),
            ActionStage.ACKNOWLEDGED: ("acknowledged_host_ns", "acknowledged_mcu_us"),
            ActionStage.CAN_QUEUED_EXACT: (
                "can_queued_exact_host_ns",
                "can_queued_exact_mcu_us",
            ),
            ActionStage.CAN_TX_COMPLETE_EXACT: (
                "can_tx_complete_exact_host_ns",
                "can_tx_complete_exact_mcu_us",
            ),
            ActionStage.POST_COMMAND_FEEDBACK: (
                "post_command_feedback_host_ns",
                "post_command_feedback_mcu_us",
            ),
        }
        selected = columns.get(update.stage)
        if selected is not None:
            values: tuple[object, ...] = (update.host_time_ns,)
            if len(selected) == 2:
                values += (update.mcu_time_us,)
            assignments = ", ".join(f"{column} = ?" for column in selected)
            connection.execute(
                f"UPDATE action_lifecycle SET {assignments} WHERE action_sequence = ?",
                (*values, update.sequence),
            )
        connection.execute(
            """
            UPDATE action_lifecycle SET
                accepted_to_ack_host_ns = CASE
                    WHEN acknowledged_host_ns >= safety_accepted_host_ns
                    THEN acknowledged_host_ns - safety_accepted_host_ns END,
                ack_to_can_tx_complete_us = CASE
                    WHEN can_tx_complete_exact_mcu_us >= acknowledged_mcu_us
                    THEN can_tx_complete_exact_mcu_us - acknowledged_mcu_us END,
                can_tx_complete_to_post_feedback_us = CASE
                    WHEN post_command_feedback_mcu_us >= can_tx_complete_exact_mcu_us
                    THEN post_command_feedback_mcu_us - can_tx_complete_exact_mcu_us END
            WHERE action_sequence = ?
            """,
            (update.sequence,),
        )
        if update.stage in (
            ActionStage.SUPERSEDED,
            ActionStage.PREEMPTED_BY_SAFETY,
            ActionStage.REJECTED,
            ActionStage.FAILED,
        ):
            connection.execute(
                "UPDATE action_lifecycle SET terminal_stage = ?, detail = ? "
                "WHERE action_sequence = ?",
                (update.stage.value, update.detail, update.sequence),
            )

    @staticmethod
    def _write_time_sync(
        connection: sqlite3.Connection, record: _TimeSyncRecord
    ) -> None:
        exchange = record.exchange
        model = record.model
        if model is not None:
            connection.execute(
                """
                INSERT INTO time_sync_models (
                    model_id, segment_id, slope_ns_per_us, intercept_ns,
                    rtt_ns, residual_ns, sample_count, created_host_ns
                ) VALUES (?,?,?,?,?,?,?,?)
                """,
                (
                    model.model_id,
                    model.segment_id,
                    model.slope_ns_per_us,
                    model.intercept_ns,
                    model.rtt_ns,
                    model.residual_ns,
                    model.sample_count,
                    model.created_host_ns,
                ),
            )
        connection.execute(
            """
            INSERT INTO time_sync_exchanges (
                host_t0_ns, mcu_rx_us, mcu_tx_us, host_t3_ns, rtt_ns, model_id
            ) VALUES (?,?,?,?,?,?)
            """,
            (
                exchange.host_t0_ns,
                exchange.mcu_rx_us,
                exchange.mcu_tx_us,
                exchange.host_t3_ns,
                exchange.rtt_ns,
                None if model is None else model.model_id,
            ),
        )

    @staticmethod
    def _write_can_diagnostics(
        connection: sqlite3.Connection, record: _CanDiagnosticsRecord
    ) -> None:
        value = record.diagnostics
        connection.execute(
            """
            INSERT INTO can_diagnostics (
                host_time_ns, window_start_us, window_duration_us,
                target_tx_complete_json, position_response_json,
                temperature_response_json, position_timeout_count,
                temperature_timeout_count, tx_abort_count, tx_error_count,
                tx_recovery_count, safety_preemption_count, max_safety_wait_us,
                max_fanout_us
            ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)
            """,
            (
                record.host_time_ns,
                value.window_start_us,
                value.window_duration_us,
                json.dumps(value.target_tx_complete, separators=(",", ":")),
                json.dumps(value.position_response, separators=(",", ":")),
                json.dumps(value.temperature_response, separators=(",", ":")),
                value.position_timeout_count,
                value.temperature_timeout_count,
                value.tx_abort_count,
                value.tx_error_count,
                value.tx_recovery_count,
                value.safety_preemption_count,
                value.max_safety_wait_us,
                value.max_fanout_us,
            ),
        )

    def _build_checksums(self) -> dict[str, object]:
        files: dict[str, str] = {}
        for path in sorted(self.session_dir.rglob("*")):
            if not path.is_file() or path == self.checksums_path:
                continue
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(block)
            files[path.relative_to(self.session_dir).as_posix()] = digest.hexdigest()
        return {"algorithm": "sha256", "created_utc": _utc_now(), "files": files}

    @staticmethod
    def _write_json_atomic(path: Path, value: Mapping[str, object]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)

    def _raise_writer_error(self) -> None:
        if self._writer_error is not None:
            raise RecorderError(f"session writer failed: {self._writer_error}") from self._writer_error

    def _require_open(self) -> None:
        if self._closed:
            raise RecorderError("session recorder is closed")
        self._raise_writer_error()
