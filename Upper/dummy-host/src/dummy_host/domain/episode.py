from __future__ import annotations

import threading
import uuid
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping

from .._compat import StrEnum


class EpisodeError(RuntimeError):
    pass


class EpisodeStatus(StrEnum):
    IDLE = "idle"
    ARMED = "armed"
    RECORDING = "recording"
    FINALIZING = "finalizing"
    ACCEPTED = "accepted"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class EpisodeSnapshot:
    episode_id: str | None
    status: EpisodeStatus
    task_id: str | None
    task: str | None
    started_ns: int | None
    ended_ns: int | None
    failure_reason: str | None
    metadata: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))


class EpisodeManager:
    """Thread-safe, explicit lifecycle for multiple Episodes in one raw Session."""

    def __init__(self, *, id_factory=None) -> None:
        self._id_factory = id_factory or (lambda: uuid.uuid4().hex)
        self._lock = threading.Lock()
        self._snapshot = EpisodeSnapshot(None, EpisodeStatus.IDLE, None, None, None, None, None)

    @property
    def snapshot(self) -> EpisodeSnapshot:
        with self._lock:
            return self._snapshot

    def arm(
        self,
        *,
        task_id: str,
        task: str,
        now_ns: int,
        metadata: Mapping[str, object] | None = None,
    ) -> EpisodeSnapshot:
        if not task_id or not task or now_ns < 0:
            raise EpisodeError("task_id, task and a non-negative timestamp are required")
        with self._lock:
            if self._snapshot.status in {
                EpisodeStatus.ARMED,
                EpisodeStatus.RECORDING,
                EpisodeStatus.FINALIZING,
            }:
                raise EpisodeError(f"cannot arm while Episode is {self._snapshot.status.value}")
            self._snapshot = EpisodeSnapshot(
                self._id_factory(),
                EpisodeStatus.ARMED,
                task_id,
                task,
                None,
                None,
                None,
                {} if metadata is None else metadata,
            )
            return self._snapshot

    def start(self, *, now_ns: int) -> EpisodeSnapshot:
        with self._lock:
            if self._snapshot.status is not EpisodeStatus.ARMED:
                raise EpisodeError("Episode must be armed before recording")
            self._snapshot = EpisodeSnapshot(
                self._snapshot.episode_id,
                EpisodeStatus.RECORDING,
                self._snapshot.task_id,
                self._snapshot.task,
                now_ns,
                None,
                None,
                self._snapshot.metadata,
            )
            return self._snapshot

    def begin(
        self,
        *,
        task_id: str,
        task: str,
        now_ns: int,
        metadata: Mapping[str, object] | None = None,
    ) -> EpisodeSnapshot:
        self.arm(task_id=task_id, task=task, now_ns=now_ns, metadata=metadata)
        return self.start(now_ns=now_ns)

    def finish(
        self,
        outcome: EpisodeStatus | str,
        *,
        now_ns: int,
        failure_reason: str | None = None,
    ) -> EpisodeSnapshot:
        outcome = EpisodeStatus(outcome)
        if outcome not in {
            EpisodeStatus.ACCEPTED,
            EpisodeStatus.FAILED,
            EpisodeStatus.CANCELLED,
        }:
            raise EpisodeError(f"invalid terminal Episode outcome {outcome.value}")
        with self._lock:
            if self._snapshot.status is not EpisodeStatus.RECORDING:
                raise EpisodeError("only a recording Episode can be finalized")
            if outcome is EpisodeStatus.FAILED and not failure_reason:
                failure_reason = "operator_marked_failure"
            self._snapshot = EpisodeSnapshot(
                self._snapshot.episode_id,
                outcome,
                self._snapshot.task_id,
                self._snapshot.task,
                self._snapshot.started_ns,
                now_ns,
                failure_reason,
                self._snapshot.metadata,
            )
            return self._snapshot
