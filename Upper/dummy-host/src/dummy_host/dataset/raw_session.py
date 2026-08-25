from __future__ import annotations

import json
import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Mapping

import numpy as np

from ..apps.session_check import SessionCheckError, check_session
from .contracts import DatasetFrame, ExportRecipe


class RawSessionError(RuntimeError):
    pass


@dataclass(frozen=True)
class EpisodeWindow:
    episode_id: str
    task_id: str
    task: str
    start_ns: int
    end_ns: int
    outcome: str


class RawSession:
    def __init__(self, session_dir: str | Path) -> None:
        self.session_dir = Path(session_dir)
        try:
            report = check_session(self.session_dir)
        except SessionCheckError as exc:
            raise RawSessionError(str(exc)) from exc
        if not report.ok:
            raise RawSessionError(f"raw session integrity failed: {report.errors}")
        self.manifest = json.loads(
            (self.session_dir / "manifest.json").read_text(encoding="utf-8")
        )
        if self.manifest.get("schema_version") not in (2, 3, 4):
            raise RawSessionError(
                "session inspection supports Raw Session schema versions 2, 3 and 4"
            )
        if self.manifest.get("clean_shutdown") is not True:
            raise RawSessionError("dataset export requires a cleanly finalized raw session")

    @property
    def manifest_extra(self) -> Mapping[str, object]:
        value = self.manifest.get("extra")
        return value if isinstance(value, dict) else {}

    @property
    def data_classification(self) -> str:
        return str(self.manifest_extra.get("data_classification", "legacy_unspecified"))

    def camera_calibration_versions(self) -> dict[str, tuple[str, ...]]:
        db_path = self.session_dir / "samples.sqlite"
        try:
            with sqlite3.connect(
                f"file:{db_path.as_posix()}?mode=ro&immutable=1", uri=True
            ) as connection:
                rows = connection.execute(
                    """
                    SELECT role, calibration_version
                    FROM camera_samples
                    GROUP BY role, calibration_version
                    ORDER BY role, calibration_version
                    """
                ).fetchall()
        except sqlite3.Error as exc:
            raise RawSessionError(f"cannot read camera calibration identities: {exc}") from exc
        versions: dict[str, list[str]] = {}
        for role, version in rows:
            versions.setdefault(str(role), []).append(str(version))
        return {role: tuple(values) for role, values in versions.items()}

    def validate_export_recipe(self, recipe: ExportRecipe) -> None:
        if recipe.require_temporary_source:
            if self.data_classification != "temporary_uncalibrated_pipeline_test":
                raise RawSessionError(
                    "temporary export recipe requires a source collected with "
                    "--temporary-uncalibrated"
                )
            if self.manifest_extra.get("offline_training_only") is not True:
                raise RawSessionError("temporary source must be marked offline_training_only")
            if self.manifest_extra.get("real_policy_execution_allowed") is not False:
                raise RawSessionError(
                    "temporary source must explicitly forbid real policy execution"
                )
        versions = self.camera_calibration_versions()
        missing_roles = [role for role in recipe.required_camera_roles if role not in versions]
        if missing_roles:
            raise RawSessionError(
                "required camera roles have no recorded frames: " + ", ".join(missing_roles)
            )
        if recipe.allow_uncalibrated_cameras:
            return
        archived = self.manifest.get("camera_calibrations")
        archived = archived if isinstance(archived, dict) else {}
        invalid_roles: list[str] = []
        for role in recipe.required_camera_roles:
            role_versions = versions[role]
            calibration = archived.get(role)
            calibration_id = (
                calibration.get("calibration_id") if isinstance(calibration, dict) else None
            )
            if (
                not isinstance(calibration_id, str)
                or not calibration_id
                or any(version != calibration_id for version in role_versions)
                or any(version.lower().startswith("uncalibrated") for version in role_versions)
            ):
                invalid_roles.append(f"{role}={list(role_versions)}")
        if invalid_roles:
            raise RawSessionError(
                "formal export recipe rejects missing/mismatched/uncalibrated camera identity: "
                + "; ".join(invalid_roles)
            )

    def episodes(self) -> tuple[EpisodeWindow, ...]:
        active: dict[str, dict[str, object]] = {}
        completed: list[EpisodeWindow] = []
        events_path = self.session_dir / "events.jsonl"
        for line_number, line in enumerate(events_path.read_text(encoding="utf-8").splitlines(), 1):
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError as exc:
                raise RawSessionError(f"invalid events.jsonl line {line_number}: {exc}") from exc
            name = event.get("event")
            payload = event.get("payload")
            if not isinstance(payload, dict):
                continue
            episode_id = payload.get("episode_id")
            if not isinstance(episode_id, str) or not episode_id:
                continue
            timestamp = int(event.get("monotonic_ns", -1))
            if name == "episode_start":
                active[episode_id] = {
                    "task_id": str(payload.get("task_id", "")),
                    "task": str(payload.get("task", "")),
                    "start_ns": timestamp,
                }
            elif name in {"episode_success", "episode_failure", "episode_cancel"}:
                start = active.pop(episode_id, None)
                if start is None:
                    continue
                outcome = {
                    "episode_success": "accepted",
                    "episode_failure": "failed",
                    "episode_cancel": "cancelled",
                }[str(name)]
                completed.append(
                    EpisodeWindow(
                        episode_id,
                        str(start["task_id"]),
                        str(start["task"]),
                        int(start["start_ns"]),
                        timestamp,
                        outcome,
                    )
                )
        return tuple(completed)

    def iter_frames(
        self,
        episode: EpisodeWindow,
        recipe: ExportRecipe,
    ) -> Iterator[DatasetFrame]:
        db_path = self.session_dir / "samples.sqlite"
        if self.manifest.get("schema_version") != 4:
            raise RawSessionError(
                "strict exact-sequence export requires Raw Session schema version 4; "
                "schema v3 lacks CAN_QUEUED_EXACT/POST_COMMAND_FEEDBACK evidence"
            )
        with sqlite3.connect(
            f"file:{db_path.as_posix()}?mode=ro&immutable=1", uri=True
        ) as connection:
            rows = connection.execute(
                """
                SELECT s.sample_index, s.tick_ns, s.raw_tick_index,
                       s.control_actual_start_ns, s.state_host_ns,
                       s.state_position, s.applied_action, s.action_sequence
                FROM samples AS s
                JOIN action_lifecycle AS a
                  ON a.action_sequence = s.action_sequence
                WHERE s.control_actual_start_ns >= ?
                  AND s.control_actual_start_ns <= ?
                  AND s.sample_valid = 1
                  AND s.state_fault_bits = 0
                  AND s.position_valid = 1
                  AND a.received_host_ns IS NOT NULL
                  AND a.safety_accepted_host_ns IS NOT NULL
                  AND a.send_enqueued_host_ns IS NOT NULL
                  AND a.acknowledged_host_ns IS NOT NULL
                  AND a.can_queued_exact_host_ns IS NOT NULL
                  AND a.post_command_feedback_host_ns IS NOT NULL
                  AND a.terminal_stage IS NULL
                  AND a.post_command_feedback_mcu_us >= a.can_queued_exact_mcu_us
                  AND (a.post_command_feedback_mcu_us - a.can_queued_exact_mcu_us) <= ?
                ORDER BY s.control_actual_start_ns, s.sample_index
                """,
                (
                    episode.start_ns,
                    episode.end_ns,
                    int(recipe.max_action_observation_latency_ms * 1_000),
                ),
            ).fetchall()
            if len(rows) < 2:
                return
            samples: list[dict[str, object]] = []
            for row in rows:
                sample_index, tick_ns, raw_tick, control_ns, state_ns, state_blob, action_blob, _ = row
                state = np.frombuffer(state_blob, dtype="<f4").copy()
                action = np.frombuffer(action_blob, dtype="<f4").copy()
                if state.shape != (7,) or action.shape != (7,):
                    raise RawSessionError(f"sample {sample_index} has invalid state/action shape")
                if not np.isfinite(state).all() or not np.isfinite(action).all():
                    raise RawSessionError(f"sample {sample_index} contains NaN or Inf")
                samples.append(
                    {
                        "sample_index": int(sample_index),
                        "tick_ns": int(tick_ns),
                        "raw_tick": int(raw_tick),
                        "control_ns": int(control_ns),
                        "state_ns": int(state_ns),
                        "state": state,
                        "action": action,
                    }
                )

            camera_by_role: dict[str, list[tuple[int, int, str]]] = {}
            camera_rows = connection.execute(
                """
                SELECT c.role, c.capture_ns, c.sample_index, c.frame_path
                FROM camera_samples AS c
                JOIN samples AS s ON s.sample_index = c.sample_index
                WHERE s.control_actual_start_ns >= ? AND s.control_actual_start_ns <= ?
                ORDER BY c.role, c.capture_ns
                """,
                (episode.start_ns, episode.end_ns),
            ).fetchall()
            for role, capture_ns, sample_index, frame_path in camera_rows:
                camera_by_role.setdefault(str(role), []).append(
                    (int(capture_ns), int(sample_index), str(frame_path))
                )
            if any(role not in camera_by_role for role in recipe.required_camera_roles):
                return

            control_times = np.asarray(
                [int(sample["control_ns"]) for sample in samples], dtype=np.int64
            )
            period_ns = max(1, round(1e9 / recipe.control_hz))
            grid_start_ns = max(episode.start_ns, int(control_times[0]))
            grid_end_ns = min(episode.end_ns, int(control_times[-1]))
            frame_index = 0
            target_ns = grid_start_ns
            while target_ns <= grid_end_ns:
                right = int(np.searchsorted(control_times, target_ns, side="left"))
                if right == 0:
                    left = right = 0
                elif right >= len(samples):
                    left = right = len(samples) - 1
                else:
                    left = right - 1
                left_time = int(samples[left]["control_ns"])
                right_time = int(samples[right]["control_ns"])
                if right != left and right_time - left_time > int(period_ns * 1.5):
                    target_ns += period_ns
                    continue
                alpha = 0.0 if right_time == left_time else (
                    (target_ns - left_time) / (right_time - left_time)
                )
                alpha = float(np.clip(alpha, 0.0, 1.0))
                state = (
                    (1.0 - alpha) * samples[left]["state"]
                    + alpha * samples[right]["state"]
                ).astype(np.float32)
                action = (
                    (1.0 - alpha) * samples[left]["action"]
                    + alpha * samples[right]["action"]
                ).astype(np.float32)
                source_index = left if alpha < 0.5 else right
                source = samples[source_index]
                images: dict[str, np.ndarray] = {}
                depths: dict[str, np.ndarray] = {}
                for role in recipe.required_camera_roles:
                    _, _, relative_path = min(
                        camera_by_role[role], key=lambda item: abs(item[0] - target_ns)
                    )
                    path = self.session_dir / relative_path
                    with np.load(path, allow_pickle=False) as archive:
                        color = archive["color_rgb"].copy()
                        if recipe.include_depth:
                            depth_key = (
                                "depth_z16"
                                if "depth_z16" in archive
                                else ("depth" if "depth" in archive else None)
                            )
                            if depth_key is not None:
                                if "depth_scale" not in archive:
                                    raise RawSessionError(
                                        f"resampled frame {frame_index} camera {role} has no depth scale"
                                    )
                                depth_scale = float(archive["depth_scale"])
                                if not np.isfinite(depth_scale) or depth_scale <= 0:
                                    raise RawSessionError(
                                        f"resampled frame {frame_index} camera {role} has invalid depth scale"
                                    )
                                depths[role] = (
                                    archive[depth_key].astype(np.float32) * depth_scale
                                )
                    if color.dtype != np.uint8 or color.ndim != 3 or color.shape[2] != 3:
                        raise RawSessionError(
                            f"resampled frame {frame_index} camera {role} is not RGB uint8 HWC"
                        )
                    images[role] = color
                yield DatasetFrame(
                    observation_state=state,
                    action=action,
                    images=images,
                    timestamp_s=(target_ns - grid_start_ns) / 1e9,
                    frame_index=frame_index,
                    episode_id=episode.episode_id,
                    task_id=episode.task_id,
                    task=episode.task,
                    source_sample_index=int(source["sample_index"]),
                    source_tick_ns=int(source["tick_ns"]),
                    source_raw_tick_index=int(source["raw_tick"]),
                    source_control_time_ns=int(source["control_ns"]),
                    interpolation_alpha=alpha,
                    depths=depths,
                )
                frame_index += 1
                target_ns += period_ns

    def sample_counts(self, episode: EpisodeWindow) -> tuple[int, int]:
        db_path = self.session_dir / "samples.sqlite"
        with sqlite3.connect(
            f"file:{db_path.as_posix()}?mode=ro&immutable=1", uri=True
        ) as connection:
            timing_column = (
                "control_actual_start_ns"
                if self.manifest.get("schema_version") == 4
                else "tick_ns"
            )
            row = connection.execute(
                f"""
                SELECT COUNT(*), COALESCE(SUM(CASE WHEN sample_valid = 0 THEN 1 ELSE 0 END), 0)
                FROM samples WHERE {timing_column} >= ? AND {timing_column} <= ?
                """,
                (episode.start_ns, episode.end_ns),
            ).fetchone()
        assert row is not None
        return int(row[0]), int(row[1])
