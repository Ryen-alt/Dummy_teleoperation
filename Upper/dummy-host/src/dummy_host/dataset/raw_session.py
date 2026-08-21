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
        if self.manifest.get("schema_version") != 2:
            raise RawSessionError("dataset export requires Raw Session schema version 2")
        if self.manifest.get("clean_shutdown") is not True:
            raise RawSessionError("dataset export requires a cleanly finalized raw session")

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
        with sqlite3.connect(
            f"file:{db_path.as_posix()}?mode=ro&immutable=1", uri=True
        ) as connection:
            rows = connection.execute(
                """
                SELECT sample_index, tick_ns, state_position, applied_action,
                       sample_valid, state_fault_bits
                FROM samples
                WHERE tick_ns >= ? AND tick_ns <= ?
                ORDER BY tick_ns, sample_index
                """,
                (episode.start_ns, episode.end_ns),
            ).fetchall()
            frame_index = 0
            for sample_index, tick_ns, state_blob, action_blob, valid, fault_bits in rows:
                if not valid or fault_bits or state_blob is None or action_blob is None:
                    continue
                state = np.frombuffer(state_blob, dtype="<f4").copy()
                action = np.frombuffer(action_blob, dtype="<f4").copy()
                if state.shape != (7,) or action.shape != (7,):
                    raise RawSessionError(f"sample {sample_index} has invalid state/action shape")
                if not np.isfinite(state).all() or not np.isfinite(action).all():
                    raise RawSessionError(f"sample {sample_index} contains NaN or Inf")
                camera_rows = connection.execute(
                    "SELECT role, frame_path FROM camera_samples WHERE sample_index = ?",
                    (sample_index,),
                ).fetchall()
                paths = {str(role): str(path) for role, path in camera_rows}
                if any(role not in paths for role in recipe.required_camera_roles):
                    continue
                images: dict[str, np.ndarray] = {}
                depths: dict[str, np.ndarray] = {}
                for role in recipe.required_camera_roles:
                    path = self.session_dir / paths[role]
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
                                        f"sample {sample_index} camera {role} has no depth scale"
                                    )
                                depth_scale = float(archive["depth_scale"])
                                if not np.isfinite(depth_scale) or depth_scale <= 0:
                                    raise RawSessionError(
                                        f"sample {sample_index} camera {role} has invalid depth scale"
                                    )
                                depths[role] = (
                                    archive[depth_key].astype(np.float32) * depth_scale
                                )
                    if color.dtype != np.uint8 or color.ndim != 3 or color.shape[2] != 3:
                        raise RawSessionError(f"sample {sample_index} camera {role} is not RGB uint8 HWC")
                    images[role] = color
                yield DatasetFrame(
                    observation_state=state,
                    action=action,
                    images=images,
                    # LeRobot datasets use a fixed-FPS episode timeline. Preserve
                    # the exact source monotonic timestamp separately for audit.
                    timestamp_s=frame_index / recipe.control_hz,
                    frame_index=frame_index,
                    episode_id=episode.episode_id,
                    task_id=episode.task_id,
                    task=episode.task,
                    source_sample_index=int(sample_index),
                    source_tick_ns=int(tick_ns),
                    depths=depths,
                )
                frame_index += 1

    def sample_counts(self, episode: EpisodeWindow) -> tuple[int, int]:
        db_path = self.session_dir / "samples.sqlite"
        with sqlite3.connect(
            f"file:{db_path.as_posix()}?mode=ro&immutable=1", uri=True
        ) as connection:
            row = connection.execute(
                """
                SELECT COUNT(*), COALESCE(SUM(CASE WHEN sample_valid = 0 THEN 1 ELSE 0 END), 0)
                FROM samples WHERE tick_ns >= ? AND tick_ns <= ?
                """,
                (episode.start_ns, episode.end_ns),
            ).fetchone()
        assert row is not None
        return int(row[0]), int(row[1])
