"""Explicit, single-run acceptance bounds; the original session stays intact.

Samples use [start, stop). Actions belong to the window by their sample, and
may finish during a bounded drain. Diagnostic snapshots bracket the window.
A9 histograms remain cumulative from the same firmware window (they cannot
be subtracted); checkers must retain that more conservative evidence.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


MAX_BOUNDARY_DELAY_NS = 2_000_000_000
MAX_ACTION_DRAIN_NS = 250_000_000


@dataclass(frozen=True)
class AcceptanceWindow:
    start_ns: int
    stop_ns: int
    diagnostic_start_ns: int
    diagnostic_stop_ns: int
    evidence_end_ns: int
    session_epoch: int


def read_events(path: Path) -> list[dict]:
    records = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line:
            continue
        record = json.loads(line)
        if not isinstance(record, dict):
            raise ValueError(f"events line {number} is not an object")
        records.append(record)
    return records


def load_acceptance_window(session: Path, manifest: dict) -> AcceptanceWindow | None:
    records = read_events(session / "events.jsonl")
    starts = [r for r in records if r.get("event") == "acceptance_window_started"]
    stops = [r for r in records if r.get("event") == "acceptance_window_stopped"]
    contract = manifest.get("acceptance_window_contract")
    if contract is None and not starts and not stops:
        return None  # Legacy whole-session checks retain their original gates.
    for record in records:
        when = record.get("monotonic_ns")
        if isinstance(when, bool) or not isinstance(when, int) or when < 0:
            raise ValueError("acceptance audit event has an invalid timestamp")
    if contract != 1 or len(starts) != 1 or len(stops) != 1:
        raise ValueError("acceptance requires contract v1 and exactly one started/stopped window")
    start, stop = starts[0], stops[0]
    a, b = start.get("payload"), stop.get("payload")
    if not isinstance(a, dict) or not isinstance(b, dict):
        raise ValueError("acceptance window payload must be an object")
    epoch = manifest.get("session_epoch")
    if isinstance(epoch, bool) or not isinstance(epoch, int) or not 0 < epoch <= 0xFFFFFFFF:
        raise ValueError("acceptance window requires a nonzero uint32 epoch")
    for payload in (a, b):
        if (payload.get("session_epoch") != epoch
                or payload.get("robot_config_hash") != manifest.get("robot_config_hash")):
            raise ValueError("acceptance window epoch/config identity changed")
    values = (start.get("monotonic_ns"), stop.get("monotonic_ns"),
              a.get("diagnostic_start_ns"), b.get("diagnostic_stop_ns"),
              b.get("evidence_end_ns"))
    if any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in values):
        raise ValueError("acceptance window timestamps must be nonnegative integers")
    window = AcceptanceWindow(*values, epoch)
    if not (window.diagnostic_start_ns <= window.start_ns < window.stop_ns
            <= window.diagnostic_stop_ns <= window.evidence_end_ns):
        raise ValueError("acceptance window boundaries are reversed")
    if (window.start_ns - window.diagnostic_start_ns > MAX_BOUNDARY_DELAY_NS
            or window.evidence_end_ns - window.stop_ns > MAX_BOUNDARY_DELAY_NS):
        raise ValueError("acceptance diagnostic boundary is too far from the control window")
    collection = {}
    for name in ("collection_started", "collection_stopped"):
        matches = [r.get("monotonic_ns") for r in records if r.get("event") == name]
        if len(matches) != 1 or isinstance(matches[0], bool) or not isinstance(matches[0], int):
            raise ValueError("acceptance requires one complete collection audit window")
        collection[name] = matches[0]
    if not (collection["collection_started"] <= window.diagnostic_start_ns
            and window.evidence_end_ns <= collection["collection_stopped"]):
        raise ValueError("acceptance evidence lies outside the collection audit")
    return window
