from __future__ import annotations

import hashlib
import json
import os
import queue
import sqlite3
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Mapping

import numpy as np

from .cameras import CameraFrame
from .frame_archive import FrameArchive, NpzFrameArchive
from .schema import AppliedAction, RobotConfig, RobotState
from .teleop import TeleopCommand, TeleopProfile


class RecorderError(RuntimeError):
    pass


class RecorderBackpressure(RecorderError):
    pass


@dataclass(frozen=True)
class RecorderStats:
    samples: int
    events: int
    camera_frames: int
    queue_high_watermark: int


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


@dataclass(frozen=True)
class _EventRecord:
    monotonic_ns: int
    event: str
    payload: Mapping[str, object]


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
        session_name: str | None = None,
        extra_manifest: Mapping[str, object] | None = None,
    ) -> None:
        if queue_size <= 0:
            raise ValueError("queue_size must be positive")
        if frame_segment_size <= 0:
            raise ValueError("frame_segment_size must be positive")
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
            )
        )
        self._queue: queue.Queue[tuple[str, object] | None] = queue.Queue(maxsize=queue_size)
        self._thread = threading.Thread(target=self._writer, name="dummy-session-writer", daemon=True)
        self._writer_error: BaseException | None = None
        self._closed = False
        self._samples = 0
        self._events = 0
        self._camera_frames = 0
        self._queue_high_watermark = 0
        self._stats_lock = threading.Lock()
        self._manifest: dict[str, object] = {
            "schema_version": 2,
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
            "teleop_config_version": teleop_profile.version,
            "teleop_config_hash": teleop_profile.config_hash,
            "action_source": source,
            "firmware_version": firmware_version,
            "control_rate_hz": robot_config.control_rate_hz,
            "joint_order": list(robot_config.joint_order),
            "joint_unit": robot_config.joint_unit,
            "array_encoding": "little-endian float32 blobs",
            "camera_archive": "pluggable FrameArchive; default lossless RGB/depth NPZ by role",
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

    def update_runtime_metadata(self, *, firmware_version: str) -> None:
        self._require_open()
        if not firmware_version:
            raise ValueError("firmware_version must be non-empty")
        self._manifest["firmware_version"] = firmware_version
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
                last_applied_sequence=state.last_applied_sequence,
                target_age_ms=state.target_age_ms,
                config_hash=state.config_hash,
            ),
            requested_action=None if action is None else action.requested.astype(np.float32, copy=True),
            applied_action=None if action is None else action.applied.astype(np.float32, copy=True),
            action_sequence=None if action is None else action.sequence,
            action_clipped=False if action is None else action.clipped,
            action_reasons=() if action is None else action.reasons,
            camera_frames=frame_copy,
            valid=valid,
            invalid_reason=invalid_reason,
        )
        self._enqueue(("sample", record))

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

    def _enqueue(self, item: tuple[str, object]) -> None:
        self._raise_writer_error()
        try:
            self._queue.put_nowait(item)
        except queue.Full as exc:
            raise RecorderBackpressure(
                "recorder queue is full; HOLD instead of dropping a control sample"
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
                source TEXT NOT NULL,
                connected INTEGER NOT NULL,
                deadman INTEGER NOT NULL,
                hold_requested INTEGER NOT NULL,
                estop_requested INTEGER NOT NULL,
                episode_event TEXT,
                joint_velocity_rad_s BLOB NOT NULL,
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
                last_applied_sequence INTEGER NOT NULL,
                target_age_ms INTEGER NOT NULL,
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
                color_depth_skew_ms REAL NOT NULL,
                calibration_version TEXT NOT NULL,
                frame_path TEXT NOT NULL,
                PRIMARY KEY (sample_index, role),
                FOREIGN KEY (sample_index) REFERENCES samples(sample_index)
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
                tick_ns, source, connected, deadman, hold_requested, estop_requested,
                episode_event, joint_velocity_rad_s, gripper_velocity_per_s, raw_input_json,
                requested_action, applied_action, action_sequence, action_clipped,
                action_reasons_json, state_position, state_velocity, state_host_ns, state_mcu_us,
                state_mode, state_fault_bits, position_valid, velocity_valid, gripper_valid,
                last_received_sequence, last_applied_sequence, target_age_ms,
                camera_frame_number, camera_capture_ns, camera_arrival_ns,
                camera_color_depth_skew_ms, sample_valid, invalid_reason
            ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
            """,
            (
                command.monotonic_ns,
                command.source,
                int(command.connected),
                int(command.deadman),
                int(command.hold_requested),
                int(command.estop_requested),
                command.episode_event,
                _float32_blob(command.joint_velocity_rad_s),
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
                state.last_applied_sequence,
                state.target_age_ms,
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
                    color_depth_skew_ms, calibration_version, frame_path
                ) VALUES (?,?,?,?,?,?,?,?)
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
                ),
            )
            if frame_archive.unique_frames > before:
                with self._stats_lock:
                    self._camera_frames += 1

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
