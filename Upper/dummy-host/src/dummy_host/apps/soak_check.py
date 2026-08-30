from __future__ import annotations

import argparse
import json
import sqlite3
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

from ..domain import ControlMode, HoldReasonBits
from ..protocol import (
    CAN_DIAGNOSTICS_FORMAT_VERSION,
    CAN_DIAGNOSTICS_PAYLOAD_SIZE,
    CAN_DIAGNOSTICS_WINDOW_VALID,
)
from .session_check import SessionCheckError, check_session


class SoakCheckError(RuntimeError):
    pass


@dataclass(frozen=True)
class SoakThresholds:
    minimum_duration_s: float = 3600.0
    minimum_coherent_ratio: float = 0.995
    maximum_feedback_skew_ms: float = 30.0
    maximum_fanout_ms: float = 15.0
    maximum_rx_dispatch_latency_ms: float = 0.2
    maximum_rx_high_water: float = 16.0
    maximum_target_retry_10_min: float = 1.0
    maximum_target_retry_60_min: float = 9.0
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
    hold_samples: int
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
    can_recovery_count: int
    can_busoff_count: int
    can_rx_overflow_count: int
    can_completion_overflow_count: int
    motor_tx_drop_count: int
    motor_rx_error_count: int
    motor_busoff_count: int
    can_unexpected_response_count: int
    target_retry_count: int
    target_retry_exhausted_count: int
    target_deadline_failure_count: int
    transition_failure_count: int
    position_timeout_rate: float
    diagnostic_window_valid: bool
    can_safety_preemption_count: int
    time_sync_models: int
    maximum_fanout_ms: float
    maximum_rx_dispatch_latency_ms: float
    maximum_rx_high_water: int
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
        "HOLD sample": metrics.hold_samples,
        "incomplete action lifecycle": metrics.incomplete_action_sequences,
        "SUPERSEDED action": metrics.superseded_actions,
        "rejected action": metrics.rejected_actions,
        "failed action": metrics.failed_actions,
        "action_credit_miss": metrics.action_credit_miss_events,
        "BAD_MODE rejection": metrics.bad_mode_rejections,
        "target TTL HOLD sample": metrics.target_ttl_hold_samples,
        "reliable RX overflow": metrics.reliable_rx_overflow,
        "CAN abort/error": metrics.can_abort_error_count,
        "CAN recovery": metrics.can_recovery_count,
        "CAN bus-off": metrics.can_busoff_count,
        "CAN RX overflow": metrics.can_rx_overflow_count,
        "CAN completion overflow": metrics.can_completion_overflow_count,
        "motor TX drop": metrics.motor_tx_drop_count,
        "motor RX error": metrics.motor_rx_error_count,
        "motor bus-off": metrics.motor_busoff_count,
        "unexpected CAN response": metrics.can_unexpected_response_count,
        "target retry exhausted": metrics.target_retry_exhausted_count,
        "target deadline failure": metrics.target_deadline_failure_count,
        "stream transition failure": metrics.transition_failure_count,
        "CAN safety preemption": metrics.can_safety_preemption_count,
    }
    if metrics.action_sequences <= 0:
        failures.append("no action lifecycle was recorded")
    for label, count in zero_gates.items():
        if count:
            failures.append(f"{label} count is {count}, expected zero")
    if metrics.time_sync_models <= 0:
        failures.append("no affine time-sync model was recorded")
    if not metrics.diagnostic_window_valid:
        failures.append("CAN diagnostics window identity or validity changed")
    if metrics.position_timeout_rate >= 0.001:
        failures.append(
            f"position timeout rate {metrics.position_timeout_rate:.6%} is not below 0.1%"
        )
    retry_limit = (
        thresholds.maximum_target_retry_60_min
        if metrics.duration_s >= 3600.0
        else thresholds.maximum_target_retry_10_min
    )
    if metrics.target_retry_count > retry_limit:
        failures.append(
            f"target retry count {metrics.target_retry_count} exceeds {retry_limit:.0f}"
        )
    if metrics.maximum_fanout_ms >= thresholds.maximum_fanout_ms:
        failures.append(
            f"CAN fan-out {metrics.maximum_fanout_ms:.3f} ms is not below "
            f"{thresholds.maximum_fanout_ms:.3f} ms"
        )
    if (
        metrics.maximum_rx_dispatch_latency_ms
        >= thresholds.maximum_rx_dispatch_latency_ms
    ):
        failures.append(
            "CAN RX dispatch latency "
            f"{metrics.maximum_rx_dispatch_latency_ms:.3f} ms is not below "
            f"{thresholds.maximum_rx_dispatch_latency_ms:.3f} ms"
        )
    if metrics.maximum_rx_high_water > thresholds.maximum_rx_high_water:
        failures.append(
            f"CAN RX queue high-water {metrics.maximum_rx_high_water} exceeds "
            f"{thresholds.maximum_rx_high_water:.0f}"
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


def _decode_counter_array(
    value: object, *, field: str, length: int = 7
) -> tuple[int, ...]:
    try:
        decoded = json.loads(str(value))
    except json.JSONDecodeError as exc:
        raise SoakCheckError(f"invalid {field}: {exc}") from exc
    if (
        not isinstance(decoded, list)
        or len(decoded) != length
        or any(
            isinstance(item, bool) or not isinstance(item, int) or item < 0
            for item in decoded
        )
    ):
        raise SoakCheckError(
            f"{field} must contain {length} non-negative counters"
        )
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
    if manifest.get("schema_version") != 6:
        raise SoakCheckError("v2.2.2 soak acceptance requires Raw Session schema v6")
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
                                         THEN 1 ELSE 0 END), 0),
                       COALESCE(SUM(CASE WHEN state_mode IN (?, ?)
                                         THEN 1 ELSE 0 END), 0)
                FROM samples
                """,
                (
                    int(HoldReasonBits.TARGET_TIMEOUT),
                    int(ControlMode.HOLD),
                    int(ControlMode.FAULT),
                ),
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
                hold_samples,
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
            diagnostics_rows = connection.execute(
                """
                SELECT host_time_ns, format_version, payload_size,
                       session_epoch, motor_marker_mask, window_flags,
                       window_reset_count, window_start_us, window_duration_us,
                       target_tx_complete_json, position_request_json,
                       position_response_json, position_timeout_json,
                       temperature_request_json, temperature_response_json,
                       temperature_timeout_json, motor_tx_drop_json,
                       motor_rx_error_json, motor_busoff_json,
                       main_can_busoff_json, main_can_rx_overflow_json,
                       main_can_rx_high_water_json, unexpected_response_count,
                       maintenance_response_count, query_target_overlap_count,
                       target_retry_count, target_retry_exhausted_count,
                       target_deadline_failure_count, main_can_tx_abort_json,
                       main_can_tx_error_json, main_can_tx_recovery_json,
                       main_can_completion_overflow_json,
                       safety_preemption_count, max_safety_wait_us,
                       max_fanout_us, max_rx_dispatch_latency_us,
                       transition_failure_count
                FROM can_diagnostics
                ORDER BY host_time_ns, diagnostic_index
                """
            ).fetchall()
            transport_rows = connection.execute(
                "SELECT transport_diagnostics_json FROM samples"
            ).fetchall()
    except sqlite3.Error as exc:
        raise SoakCheckError(f"cannot read soak evidence: {exc}") from exc

    diagnostic_duration_s = 0.0
    target_counts = position_counts = temperature_counts = (0,) * 7
    position_request_counts = position_timeout_counts = (0,) * 7
    can_abort_error = can_preemption = can_safety_wait_us = max_fanout_us = 0
    can_recovery = can_busoff = can_rx_overflow = can_completion_overflow = 0
    motor_tx_drop = motor_rx_error = motor_busoff = 0
    can_unexpected = target_retry = target_retry_exhausted = 0
    target_deadline_failure = transition_failure = 0
    max_rx_dispatch_latency_us = max_rx_high_water = 0
    diagnostic_window_valid = False
    if len(diagnostics_rows) < 2:
        failures.append(
            "CAN soak evidence requires first and last diagnostic snapshots"
        )
    else:
        first = diagnostics_rows[0]
        last = diagnostics_rows[-1]
        identity_indices = (3, 6, 7)
        diagnostic_window_valid = all(
            first[index] == last[index] for index in identity_indices
        )
        diagnostic_window_valid = diagnostic_window_valid and all(
            int(row[1]) == CAN_DIAGNOSTICS_FORMAT_VERSION
            and int(row[2]) == CAN_DIAGNOSTICS_PAYLOAD_SIZE
            and int(row[4]) == 0x7F
            and int(row[5]) & CAN_DIAGNOSTICS_WINDOW_VALID
            == CAN_DIAGNOSTICS_WINDOW_VALID
            for row in diagnostics_rows
        )
        manifest_epoch = manifest.get("session_epoch")
        diagnostic_window_valid = diagnostic_window_valid and (
            isinstance(manifest_epoch, int)
            and int(first[3]) == manifest_epoch
        )
        duration_delta_us = int(last[8]) - int(first[8])
        if duration_delta_us <= 0:
            diagnostic_window_valid = False
            failures.append("CAN diagnostic window duration did not advance")
        else:
            diagnostic_duration_s = duration_delta_us / 1e6

        counter_rollback = False

        def counter_deltas(
            first_values: tuple[int, ...],
            last_values: tuple[int, ...],
            *,
            field: str,
        ) -> tuple[int, ...]:
            nonlocal counter_rollback
            if any(end < start for start, end in zip(first_values, last_values)):
                counter_rollback = True
                failures.append(f"CAN diagnostic counter rolled back: {field}")
                return (0,) * len(first_values)
            return tuple(
                end - start for start, end in zip(first_values, last_values)
            )

        def array_delta(index: int, field: str, length: int = 7) -> tuple[int, ...]:
            return counter_deltas(
                _decode_counter_array(first[index], field=field, length=length),
                _decode_counter_array(last[index], field=field, length=length),
                field=field,
            )

        def scalar_delta(index: int, field: str) -> int:
            return counter_deltas(
                (int(first[index]),), (int(last[index]),), field=field
            )[0]

        target_counts = array_delta(9, "target_tx_complete_json")
        position_request_counts = array_delta(10, "position_request_json")
        position_counts = array_delta(11, "position_response_json")
        position_timeout_counts = array_delta(12, "position_timeout_json")
        temperature_counts = array_delta(14, "temperature_response_json")
        motor_tx_drop = sum(array_delta(16, "motor_tx_drop_json"))
        motor_rx_error = sum(array_delta(17, "motor_rx_error_json"))
        motor_busoff = sum(array_delta(18, "motor_busoff_json"))
        can_busoff = sum(array_delta(19, "main_can_busoff_json", 2))
        can_rx_overflow = sum(
            array_delta(20, "main_can_rx_overflow_json", 2)
        )
        max_rx_high_water = max(
            max(
                _decode_counter_array(
                    row[21], field="main_can_rx_high_water_json", length=2
                )
            )
            for row in diagnostics_rows
        )
        can_unexpected = scalar_delta(22, "unexpected_response_count")
        target_retry = scalar_delta(25, "target_retry_count")
        target_retry_exhausted = scalar_delta(
            26, "target_retry_exhausted_count"
        )
        target_deadline_failure = scalar_delta(
            27, "target_deadline_failure_count"
        )
        can_abort_error = sum(array_delta(28, "main_can_tx_abort_json", 2))
        can_abort_error += sum(array_delta(29, "main_can_tx_error_json", 2))
        can_recovery = sum(array_delta(30, "main_can_tx_recovery_json", 2))
        can_completion_overflow = sum(
            array_delta(31, "main_can_completion_overflow_json", 2)
        )
        can_preemption = scalar_delta(32, "safety_preemption_count")
        can_safety_wait_us = max(int(row[33]) for row in diagnostics_rows)
        max_fanout_us = max(int(row[34]) for row in diagnostics_rows)
        max_rx_dispatch_latency_us = max(
            int(row[35]) for row in diagnostics_rows
        )
        transition_failure = scalar_delta(36, "transition_failure_count")
        diagnostic_window_valid = diagnostic_window_valid and not counter_rollback

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
        hold_samples=hold_samples,
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
        can_recovery_count=can_recovery,
        can_busoff_count=can_busoff,
        can_rx_overflow_count=can_rx_overflow,
        can_completion_overflow_count=can_completion_overflow,
        motor_tx_drop_count=motor_tx_drop,
        motor_rx_error_count=motor_rx_error,
        motor_busoff_count=motor_busoff,
        can_unexpected_response_count=can_unexpected,
        target_retry_count=target_retry,
        target_retry_exhausted_count=target_retry_exhausted,
        target_deadline_failure_count=target_deadline_failure,
        transition_failure_count=transition_failure,
        position_timeout_rate=(
            0.0
            if sum(position_request_counts) == 0
            else sum(position_timeout_counts) / sum(position_request_counts)
        ),
        diagnostic_window_valid=diagnostic_window_valid,
        can_safety_preemption_count=can_preemption,
        time_sync_models=int(model_row[0]),
        maximum_fanout_ms=max_fanout_us / 1000.0,
        maximum_rx_dispatch_latency_ms=max_rx_dispatch_latency_us / 1000.0,
        maximum_rx_high_water=max_rx_high_water,
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
        description="Apply dummy-ref-v2.2.2 strict integration/soak acceptance gates"
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
