from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from dummy_host.apps.soak_check import SoakThresholds, check_soak_session
from dummy_host.can_a9 import load_can_timing_profile_events
from dummy_host.can_r5 import (
    CanR5Decision,
    CanR5RuntimeEvidence,
    CanR5Thresholds,
    decision_as_dict,
    evaluate_can_r5,
)
from dummy_host.schema import RobotConfig, load_robot_config


def _runtime_event_evidence(
    path: Path, session_epoch: int
) -> tuple[tuple[int, ...], int, int]:
    fanouts: list[tuple[int, int, int]] = []
    starts: list[int] = []
    stops: list[int] = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not line:
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"invalid events JSON on line {line_number}: {exc}"
            ) from exc
        event = record.get("event")
        monotonic_ns = record.get("monotonic_ns")
        if event in {"collection_started", "collection_stopped"}:
            if (
                isinstance(monotonic_ns, bool)
                or not isinstance(monotonic_ns, int)
                or monotonic_ns < 0
            ):
                raise ValueError(f"{event} has invalid monotonic_ns")
            (starts if event == "collection_started" else stops).append(
                monotonic_ns
            )
            continue
        if event != "can_target_fanout":
            continue
        payload = record.get("payload")
        if not isinstance(payload, dict):
            raise ValueError("can_target_fanout payload is not an object")
        duration = payload.get("duration_us")
        epoch = payload.get("session_epoch")
        action_sequence = payload.get("action_sequence")
        if (
            isinstance(duration, bool)
            or not isinstance(duration, int)
            or duration <= 0
            or epoch != session_epoch
            or isinstance(action_sequence, bool)
            or not isinstance(action_sequence, int)
            or action_sequence <= 0
            or isinstance(monotonic_ns, bool)
            or not isinstance(monotonic_ns, int)
            or monotonic_ns < 0
        ):
            raise ValueError(
                "can_target_fanout contains invalid sequence, duration, "
                "timestamp, or session epoch"
            )
        fanouts.append((monotonic_ns, action_sequence, duration))
    if len(starts) != 1 or len(stops) != 1 or stops[0] < starts[0]:
        raise ValueError(
            "R5 requires exactly one ordered collection_started/stopped window"
        )
    selected = [
        (sequence, duration)
        for monotonic_ns, sequence, duration in fanouts
        if starts[0] <= monotonic_ns <= stops[0]
    ]
    if len({sequence for sequence, _ in selected}) != len(selected):
        raise ValueError("R5 fanout evidence repeats an action sequence")
    return tuple(duration for _, duration in selected), starts[0], stops[0]


def check_can_r5_session(
    session_dir: str | Path,
    config: RobotConfig,
    *,
    minimum_required_position_hz: int = 20,
    thresholds: CanR5Thresholds = CanR5Thresholds(),
) -> CanR5Decision:
    session_dir = Path(session_dir)
    manifest = json.loads(
        (session_dir / "manifest.json").read_text(encoding="utf-8")
    )
    if not isinstance(manifest, dict):
        raise ValueError("session manifest must be a JSON object")
    evidence_failures: list[str] = []
    if manifest.get("robot_config_hash") != config.config_hash:
        evidence_failures.append(
            "session robot_config_hash does not match the supplied configuration"
        )
    if manifest.get("firmware_version") != "dummy-ref-v2.2.2":
        evidence_failures.append(
            "R5 requires firmware_version dummy-ref-v2.2.2"
        )
    session_epoch = manifest.get("session_epoch")
    if (
        isinstance(session_epoch, bool)
        or not isinstance(session_epoch, int)
        or not 0 < session_epoch <= 0xFFFFFFFF
    ):
        raise ValueError("session manifest does not contain a non-zero uint32 epoch")

    soak_thresholds = replace(
        SoakThresholds(),
        minimum_duration_s=thresholds.minimum_duration_s,
        control_rate_hz=float(config.control_rate_hz),
        target_rate_hz=float(config.can_target_hz_per_node),
        position_rate_hz=float(config.can_position_hz_per_node),
        temperature_rate_hz=float(config.can_temperature_hz_per_node),
    )
    soak = check_soak_session(session_dir, soak_thresholds)
    events_path = session_dir / "events.jsonl"
    fanouts, collection_start_ns, collection_stop_ns = _runtime_event_evidence(
        events_path, session_epoch
    )
    profile = load_can_timing_profile_events(
        events_path,
        minimum_monotonic_ns=collection_start_ns,
        maximum_monotonic_ns=collection_stop_ns,
    )
    if profile.session_epoch != session_epoch:
        evidence_failures.append(
            "A9 profile session_epoch does not match the session manifest"
        )
    if profile.window_duration_us < int(thresholds.minimum_duration_s * 1e6):
        evidence_failures.append(
            "A9 profile window is shorter than the required R5 duration"
        )
    fanout_p99_ms = (
        0.0 if not fanouts else float(np.percentile(fanouts, 99)) / 1000.0
    )
    metrics = soak.metrics
    runtime = CanR5RuntimeEvidence(
        duration_s=metrics.duration_s,
        soak_passed=soak.ok and not evidence_failures,
        soak_failures=tuple((*soak.failures, *evidence_failures)),
        coherent_sweep_p99_ms=metrics.coherent_sweep_p99_ms,
        exact_fanout_p99_ms=fanout_p99_ms,
        exact_fanout_samples=len(fanouts),
        expected_exact_fanout_samples=metrics.action_sequences,
        position_timeout_count=metrics.position_timeout_count,
        temperature_timeout_count=metrics.temperature_timeout_count,
        can_abort_error_count=metrics.can_abort_error_count,
        can_recovery_count=metrics.can_recovery_count,
        can_busoff_count=metrics.can_busoff_count,
        can_rx_overflow_count=metrics.can_rx_overflow_count,
        can_completion_overflow_count=metrics.can_completion_overflow_count,
        motor_tx_drop_count=metrics.motor_tx_drop_count,
        motor_rx_error_count=metrics.motor_rx_error_count,
        motor_busoff_count=metrics.motor_busoff_count,
        can_unexpected_response_count=metrics.can_unexpected_response_count,
        target_retry_count=metrics.target_retry_count,
        target_deadline_failure_count=metrics.target_deadline_failure_count,
        transition_failure_count=metrics.transition_failure_count,
        action_credit_miss_events=metrics.action_credit_miss_events,
    )
    return evaluate_can_r5(
        profile,
        runtime,
        current_node_quiet_us=config.can_node_quiet_us,
        current_response_timeout_us=config.can_response_timeout_us,
        current_position_hz_per_node=config.can_position_hz_per_node,
        control_rate_hz=config.control_rate_hz,
        minimum_required_position_hz=minimum_required_position_hz,
        thresholds=thresholds,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Apply the R5 10-minute internal A9/CAN acceptance gate and "
            "select scheduling branch A, B, or C"
        )
    )
    parser.add_argument("--session", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--minimum-required-position-hz", type=int, default=20)
    parser.add_argument("--json-output")
    args = parser.parse_args()
    try:
        decision = check_can_r5_session(
            args.session,
            load_robot_config(args.config),
            minimum_required_position_hz=args.minimum_required_position_hz,
        )
    except (OSError, TypeError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))
    rendered = json.dumps(
        decision_as_dict(decision), indent=2, ensure_ascii=False, sort_keys=True
    )
    print(rendered)
    if args.json_output:
        Path(args.json_output).write_text(rendered + "\n", encoding="utf-8")
    if not decision.passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
