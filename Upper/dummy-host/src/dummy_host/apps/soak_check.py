from __future__ import annotations

import argparse
import json
import sqlite3
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

from ..domain import HoldReasonBits
from .session_check import SessionCheckError, check_session


class SoakCheckError(RuntimeError):
    pass


@dataclass(frozen=True)
class SoakThresholds:
    minimum_duration_s: float = 3600.0
    minimum_coherent_ratio: float = 0.995
    maximum_feedback_skew_ms: float = 30.0
    maximum_fanout_ms: float = 10.0
    maximum_post_feedback_p99_ms: float = 100.0
    maximum_post_feedback_ms: float = 250.0
    maximum_serial_safety_wait_ms: float = 10.0
    maximum_can_safety_wait_ms: float = 5.0
    control_rate_hz: float = 20.0
    target_rate_hz: float = 50.0
    position_rate_hz: float = 40.0
    temperature_rate_hz: float = 1.0
    motion_rate_tolerance: float = 0.10
    temperature_rate_tolerance: float = 0.25

    def __post_init__(self) -> None:
        values = tuple(asdict(self).values())
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not np.isfinite(value)
            or value <= 0
            for value in values
        ):
            raise ValueError("soak thresholds must be positive finite numbers")
        if self.minimum_coherent_ratio > 1.0:
            raise ValueError("minimum_coherent_ratio cannot exceed one")
        if (
            self.motion_rate_tolerance >= 1.0
            or self.temperature_rate_tolerance >= 1.0
        ):
            raise ValueError("rate tolerances must be less than one")


@dataclass(frozen=True)
class SoakMetrics:
    duration_s: float
    samples: int
    invalid_samples: int
    fault_samples: int
    control_rate_hz: float
    coherent_ratio: float
    maximum_feedback_skew_ms: float
    action_sequences: int
    incomplete_action_sequences: int
    superseded_actions: int
    rejected_actions: int
    failed_actions: int
    action_credit_miss_events: int
    bad_mode_rejections: int
    target_ttl_hold_samples: int
    reliable_rx_overflow: int
    can_abort_error_count: int
    can_safety_preemption_count: int
    time_sync_models: int
    maximum_fanout_ms: float
    post_feedback_p99_ms: float
    maximum_post_feedback_ms: float
    maximum_serial_safety_wait_ms: float
    maximum_can_safety_wait_ms: float
    target_rate_hz_per_node: tuple[float, ...]
    position_rate_hz_per_node: tuple[float, ...]
    temperature_rate_hz_per_node: tuple[float, ...]


@dataclass(frozen=True)
class SoakCheckReport:
    ok: bool
    session: str
    thresholds: SoakThresholds
    metrics: SoakMetrics
    failures: tuple[str, ...]
    warnings: tuple[str, ...]


def _rate_failures(
    label: str,
    values: tuple[float, ...],
    expected: float,
    tolerance: float,
) -> list[str]:
    if len(values) != 7:
        return [f"{label} rate evidence does not contain seven nodes"]
    low = expected * (1.0 - tolerance)
    high = expected * (1.0 + tolerance)
    return [
        (
            f"{label} node {index} rate {value:.3f} Hz is outside "
            f"{low:.3f}..{high:.3f} Hz"
        )
        for index, value in enumerate(values, 1)
        if not low <= value <= high
    ]


def evaluate_soak_metrics(
    metrics: SoakMetrics, thresholds: SoakThresholds = SoakThresholds()
) -> tuple[str, ...]:
    failures: list[str] = []
    if metrics.duration_s < thresholds.minimum_duration_s:
        failures.append(
            f"duration {metrics.duration_s:.3f} s is below {thresholds.minimum_duration_s:.3f} s"
        )
    if metrics.samples < 2:
        failures.append("fewer than two control samples were recorded")
    control_low = thresholds.control_rate_hz * (
        1.0 - thresholds.motion_rate_tolerance
    )
    control_high = thresholds.control_rate_hz * (
        1.0 + thresholds.motion_rate_tolerance
    )
    if not control_low <= metrics.control_rate_hz <= control_high:
        failures.append(
            f"control sample rate {metrics.control_rate_hz:.3f} Hz is outside "
            f"{control_low:.3f}..{control_high:.3f} Hz"
        )
    if metrics.coherent_ratio < thresholds.minimum_coherent_ratio:
        failures.append(
            f"coherent ratio {metrics.coherent_ratio:.6f} is below "
            f"{thresholds.minimum_coherent_ratio:.6f}"
        )
    if metrics.maximum_feedback_skew_ms > thresholds.maximum_feedback_skew_ms:
        failures.append(
            f"feedback skew {metrics.maximum_feedback_skew_ms:.3f} ms exceeds "
            f"{thresholds.maximum_feedback_skew_ms:.3f} ms"
        )
    zero_gates = {
        "invalid control sample": metrics.invalid_samples,
        "fault sample": metrics.fault_samples,
        "incomplete action lifecycle": metrics.incomplete_action_sequences,
        "SUPERSEDED action": metrics.superseded_actions,
        "rejected action": metrics.rejected_actions,
        "failed action": metrics.failed_actions,
        "action_credit_miss": metrics.action_credit_miss_events,
        "BAD_MODE rejection": metrics.bad_mode_rejections,
        "target TTL HOLD sample": metrics.target_ttl_hold_samples,
        "reliable RX overflow": metrics.reliable_rx_overflow,
        "CAN abort/error": metrics.can_abort_error_count,
        "CAN safety preemption": metrics.can_safety_preemption_count,
    }
    if metrics.action_sequences <= 0:
        failures.append("no action lifecycle was recorded")
    for label, count in zero_gates.items():
        if count:
            failures.append(f"{label} count is {count}, expected zero")
    if metrics.time_sync_models <= 0:
        failures.append("no affine time-sync model was recorded")
    if metrics.maximum_fanout_ms >= thresholds.maximum_fanout_ms:
        failures.append(
            f"CAN fan-out {metrics.maximum_fanout_ms:.3f} ms is not below "
            f"{thresholds.maximum_fanout_ms:.3f} ms"
        )
    if metrics.post_feedback_p99_ms >= thresholds.maximum_post_feedback_p99_ms:
        failures.append(
            f"post-feedback p99 {metrics.post_feedback_p99_ms:.3f} ms is not below "
            f"{thresholds.maximum_post_feedback_p99_ms:.3f} ms"
        )
    if metrics.maximum_post_feedback_ms >= thresholds.maximum_post_feedback_ms:
        failures.append(
            f"post-feedback max {metrics.maximum_post_feedback_ms:.3f} ms is not below "
            f"{thresholds.maximum_post_feedback_ms:.3f} ms"
        )
    if metrics.maximum_serial_safety_wait_ms >= thresholds.maximum_serial_safety_wait_ms:
        failures.append(
            f"serial safety wait {metrics.maximum_serial_safety_wait_ms:.3f} ms is not below "
            f"{thresholds.maximum_serial_safety_wait_ms:.3f} ms"
        )
    if metrics.maximum_can_safety_wait_ms >= thresholds.maximum_can_safety_wait_ms:
        failures.append(
            f"CAN safety wait {metrics.maximum_can_safety_wait_ms:.3f} ms is not below "
            f"{thresholds.maximum_can_safety_wait_ms:.3f} ms"
        )
    failures.extend(
        _rate_failures(
            "target",
            metrics.target_rate_hz_per_node,
            thresholds.target_rate_hz,
            thresholds.motion_rate_tolerance,
        )
    )
    failures.extend(
        _rate_failures(
            "position",
            metrics.position_rate_hz_per_node,
            thresholds.position_rate_hz,
            thresholds.motion_rate_tolerance,
        )
    )
    failures.extend(
        _rate_failures(
            "temperature",
            metrics.temperature_rate_hz_per_node,
            thresholds.temperature_rate_hz,
            thresholds.temperature_rate_tolerance,
        )
    )
    return tuple(failures)


def _event_evidence(
    path: Path,
) -> tuple[dict[str, int], tuple[int, ...], tuple[int, ...]]:
    counts: dict[str, int] = {}
    collection_starts: list[int] = []
    collection_stops: list[int] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line:
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise SoakCheckError(f"invalid events.jsonl: {exc}") from exc
        event = value.get("event")
        if isinstance(event, str):
            counts[event] = counts.get(event, 0) + 1
            monotonic_ns = value.get("monotonic_ns")
            if event in {"collection_started", "collection_stopped"}:
                if (
                    isinstance(monotonic_ns, bool)
                    or not isinstance(monotonic_ns, int)
                    or monotonic_ns < 0
                ):
                    raise SoakCheckError(
                        f"{event} must contain a non-negative monotonic_ns"
                    )
                if event == "collection_started":
                    collection_starts.append(monotonic_ns)
                else:
                    collection_stops.append(monotonic_ns)
    return counts, tuple(collection_starts), tuple(collection_stops)


def _decode_counter_array(value: object, *, field: str) -> tuple[int, ...]:
    try:
        decoded = json.loads(str(value))
    except json.JSONDecodeError as exc:
        raise SoakCheckError(f"invalid {field}: {exc}") from exc
    if (
        not isinstance(decoded, list)
        or len(decoded) != 7
        or any(
            isinstance(item, bool) or not isinstance(item, int) or item < 0
            for item in decoded
        )
    ):
        raise SoakCheckError(f"{field} must contain seven non-negative counters")
    return tuple(decoded)


def check_soak_session(
    session_dir: str | Path,
    thresholds: SoakThresholds = SoakThresholds(),
) -> SoakCheckReport:
    session_dir = Path(session_dir)
    try:
        integrity = check_session(session_dir)
    except SessionCheckError as exc:
        raise SoakCheckError(str(exc)) from exc
    manifest = json.loads((session_dir / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("schema_version") != 5:
        raise SoakCheckError("soak acceptance requires Raw Session schema v5")
    failures = list(integrity.errors)
    if manifest.get("control_rate_hz") != thresholds.control_rate_hz:
        failures.append(
            "soak acceptance requires a fixed "
            f"{thresholds.control_rate_hz:.3f} Hz control configuration"
        )
    if not integrity.clean_shutdown:
        failures.append("session did not close cleanly")

    db_path = session_dir / "samples.sqlite"
    try:
        with sqlite3.connect(
            f"file:{db_path.as_posix()}?mode=ro&immutable=1", uri=True
        ) as connection:
            sample_row = connection.execute(
                """
                SELECT COUNT(*),
                       COALESCE(MIN(control_actual_start_ns), 0),
                       COALESCE(MAX(control_actual_start_ns), 0),
                       COALESCE(SUM(CASE WHEN coherent_sweep_id > 0
                                             AND position_valid = 1 THEN 1 ELSE 0 END), 0),
                       COALESCE(MAX(feedback_max_skew_us), 0),
                       COALESCE(SUM(CASE WHEN (state_hold_reason_bits & ?) != 0
                                         THEN 1 ELSE 0 END), 0),
                       COALESCE(SUM(CASE WHEN state_fault_bits != 0
                                         THEN 1 ELSE 0 END), 0)
                FROM samples
                """,
                (int(HoldReasonBits.TARGET_TIMEOUT),),
            ).fetchone()
            assert sample_row is not None
            (
                samples,
                first_ns,
                last_ns,
                coherent,
                max_skew_us,
                ttl_holds,
                fault_samples,
            ) = (int(value) for value in sample_row)
            lifecycle_row = connection.execute(
                """
                SELECT COUNT(*),
                       COALESCE(SUM(CASE WHEN acknowledged_host_ns IS NULL
                                             OR can_tx_complete_exact_host_ns IS NULL
                                             OR post_command_feedback_host_ns IS NULL
                                         THEN 1 ELSE 0 END), 0),
                       COALESCE(SUM(CASE WHEN terminal_stage = 'superseded' THEN 1 ELSE 0 END), 0),
                       COALESCE(SUM(CASE WHEN terminal_stage = 'rejected' THEN 1 ELSE 0 END), 0),
                       COALESCE(SUM(CASE WHEN terminal_stage = 'failed' THEN 1 ELSE 0 END), 0),
                       COALESCE(SUM(CASE WHEN detail LIKE '%BAD_MODE%' THEN 1 ELSE 0 END), 0)
                FROM action_lifecycle
                """
            ).fetchone()
            assert lifecycle_row is not None
            (
                action_sequences,
                incomplete_actions,
                superseded_actions,
                rejected_actions,
                failed_actions,
                bad_mode_rejections,
            ) = (int(value) for value in lifecycle_row)
            latencies = [
                int(row[0]) / 1000.0
                for row in connection.execute(
                    """
                    SELECT can_tx_complete_to_post_feedback_us
                    FROM action_lifecycle
                    WHERE can_tx_complete_to_post_feedback_us IS NOT NULL
                    """
                )
            ]
            model_row = connection.execute(
                "SELECT COUNT(*) FROM time_sync_models"
            ).fetchone()
            assert model_row is not None
            diagnostics_row = connection.execute(
                """
                SELECT window_duration_us, target_tx_complete_json,
                       position_response_json, temperature_response_json,
                       tx_abort_count + tx_error_count,
                       safety_preemption_count, max_safety_wait_us, max_fanout_us
                FROM can_diagnostics
                ORDER BY host_time_ns DESC, diagnostic_index DESC
                LIMIT 1
                """
            ).fetchone()
            transport_rows = connection.execute(
                "SELECT transport_diagnostics_json FROM samples"
            ).fetchall()
    except sqlite3.Error as exc:
        raise SoakCheckError(f"cannot read soak evidence: {exc}") from exc

    if diagnostics_row is None:
        diagnostic_duration_s = 0.0
        target_counts = position_counts = temperature_counts = (0,) * 7
        can_abort_error = can_preemption = can_safety_wait_us = max_fanout_us = 0
        failures.append("no CAN diagnostic snapshot was recorded")
    else:
        diagnostic_duration_s = int(diagnostics_row[0]) / 1e6
        target_counts = _decode_counter_array(
            diagnostics_row[1], field="target_tx_complete_json"
        )
        position_counts = _decode_counter_array(
            diagnostics_row[2], field="position_response_json"
        )
        temperature_counts = _decode_counter_array(
            diagnostics_row[3], field="temperature_response_json"
        )
        can_abort_error = int(diagnostics_row[4])
        can_preemption = int(diagnostics_row[5])
        can_safety_wait_us = int(diagnostics_row[6])
        max_fanout_us = int(diagnostics_row[7])

    def rates(counts: tuple[int, ...]) -> tuple[float, ...]:
        return tuple(
            0.0 if diagnostic_duration_s <= 0 else count / diagnostic_duration_s
            for count in counts
        )

    reliable_rx_overflow = 0
    max_serial_safety_wait_ns = 0
    for row in transport_rows:
        try:
            values = json.loads(str(row[0]))
        except json.JSONDecodeError as exc:
            raise SoakCheckError(f"invalid transport diagnostics: {exc}") from exc
        if not isinstance(values, dict):
            raise SoakCheckError("transport diagnostics must be JSON objects")
        reliable_rx_overflow = max(
            reliable_rx_overflow, int(values.get("reliable_rx_overflow", 0))
        )
        max_serial_safety_wait_ns = max(
            max_serial_safety_wait_ns, int(values.get("max_safety_wait_ns", 0))
        )

    event_counts, collection_starts, collection_stops = _event_evidence(
        session_dir / "events.jsonl"
    )
    if len(collection_starts) != 1 or len(collection_stops) != 1:
        failures.append(
            "soak evidence requires exactly one collection_started and "
            "one collection_stopped event"
        )
        duration_s = max(0.0, (last_ns - first_ns) / 1e9)
    elif collection_stops[0] < collection_starts[0]:
        failures.append("collection_stopped precedes collection_started")
        duration_s = 0.0
    else:
        duration_s = (collection_stops[0] - collection_starts[0]) / 1e9
    metrics = SoakMetrics(
        duration_s=duration_s,
        samples=samples,
        invalid_samples=integrity.invalid_samples,
        fault_samples=fault_samples,
        control_rate_hz=0.0 if duration_s <= 0 else samples / duration_s,
        coherent_ratio=0.0 if samples == 0 else coherent / samples,
        maximum_feedback_skew_ms=max_skew_us / 1000.0,
        action_sequences=action_sequences,
        incomplete_action_sequences=incomplete_actions,
        superseded_actions=superseded_actions,
        rejected_actions=rejected_actions,
        failed_actions=failed_actions,
        action_credit_miss_events=event_counts.get("action_credit_miss", 0),
        bad_mode_rejections=bad_mode_rejections,
        target_ttl_hold_samples=ttl_holds,
        reliable_rx_overflow=reliable_rx_overflow,
        can_abort_error_count=can_abort_error,
        can_safety_preemption_count=can_preemption,
        time_sync_models=int(model_row[0]),
        maximum_fanout_ms=max_fanout_us / 1000.0,
        post_feedback_p99_ms=(
            0.0 if not latencies else float(np.percentile(latencies, 99))
        ),
        maximum_post_feedback_ms=0.0 if not latencies else max(latencies),
        maximum_serial_safety_wait_ms=max_serial_safety_wait_ns / 1e6,
        maximum_can_safety_wait_ms=can_safety_wait_us / 1000.0,
        target_rate_hz_per_node=rates(target_counts),
        position_rate_hz_per_node=rates(position_counts),
        temperature_rate_hz_per_node=rates(temperature_counts),
    )
    failures.extend(evaluate_soak_metrics(metrics, thresholds))
    return SoakCheckReport(
        ok=not failures,
        session=str(session_dir.resolve()),
        thresholds=thresholds,
        metrics=metrics,
        failures=tuple(dict.fromkeys(failures)),
        warnings=integrity.warnings,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Apply dummy-ref-v2.2 strict integration/soak acceptance gates"
    )
    parser.add_argument("--session", required=True)
    parser.add_argument("--minimum-duration-s", type=float, default=3600.0)
    parser.add_argument("--json-output")
    args = parser.parse_args()
    thresholds = replace(
        SoakThresholds(), minimum_duration_s=args.minimum_duration_s
    )
    try:
        report = check_soak_session(args.session, thresholds)
    except (OSError, ValueError, SoakCheckError) as exc:
        parser.error(str(exc))
    rendered = json.dumps(asdict(report), indent=2, ensure_ascii=False)
    print(rendered)
    if args.json_output:
        Path(args.json_output).write_text(rendered + "\n", encoding="utf-8")
    if not report.ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
