from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import pytest

from dummy_host.apps.can_r5_check import _runtime_event_evidence
from dummy_host.can_a9 import load_can_timing_profile_events
from dummy_host.can_r5 import CanR5RuntimeEvidence, evaluate_can_r5


def _profile():
    return load_can_timing_profile_events(
        Path(__file__).parent / "fixtures" / "can_a9_valid_events.jsonl"
    )


def _runtime(**changes: object) -> CanR5RuntimeEvidence:
    value = CanR5RuntimeEvidence(
        duration_s=600.0,
        soak_passed=True,
        soak_failures=(),
        coherent_sweep_p99_ms=20.0,
        exact_fanout_p99_ms=9.0,
        exact_fanout_samples=12_000,
        expected_exact_fanout_samples=12_000,
        position_timeout_count=0,
        temperature_timeout_count=0,
        can_abort_error_count=0,
        can_recovery_count=0,
        can_busoff_count=0,
        can_rx_overflow_count=0,
        can_completion_overflow_count=0,
        motor_tx_drop_count=0,
        motor_rx_error_count=0,
        motor_busoff_count=0,
        can_unexpected_response_count=0,
        target_retry_count=0,
        target_deadline_failure_count=0,
        transition_failure_count=0,
        action_credit_miss_events=0,
    )
    return replace(value, **changes)


def test_r5_branch_a_passes_only_with_measured_configuration() -> None:
    decision = evaluate_can_r5(
        _profile(),
        _runtime(),
        current_node_quiet_us=500,
        current_response_timeout_us=500,
        current_position_hz_per_node=40,
    )
    assert decision.result == "PASS"
    assert decision.passed
    assert decision.branch == "A_KEEP_40_HZ"
    assert decision.configuration_ready
    assert decision.recommended_position_hz_per_node == 40


def test_r5_valid_evidence_rejects_placeholder_configuration() -> None:
    decision = evaluate_can_r5(
        _profile(),
        _runtime(),
        current_node_quiet_us=5000,
        current_response_timeout_us=4000,
        current_position_hz_per_node=40,
    )
    assert decision.result == "RECONFIGURE"
    assert not decision.passed
    assert decision.recommended_node_quiet_us == 500
    assert decision.recommended_response_timeout_us == 500


def test_r5_does_not_trade_control_rate_for_can_margin() -> None:
    decision = evaluate_can_r5(
        _profile(),
        _runtime(),
        current_node_quiet_us=500,
        current_response_timeout_us=500,
        current_position_hz_per_node=40,
        control_rate_hz=10,
    )
    assert decision.result == "FAIL"
    assert any("remain at 20 Hz" in item for item in decision.failures)


def test_r5_selects_branch_b_without_increasing_retries() -> None:
    slow = replace(
        _profile(),
        position_p99_us=(3100,) * 7,
        position_p999_us=(3300,) * 7,
        position_max_us=(3500,) * 7,
    )
    decision = evaluate_can_r5(
        slow,
        _runtime(),
        current_node_quiet_us=5000,
        current_response_timeout_us=4000,
        current_position_hz_per_node=40,
    )
    assert decision.result == "RETEST_B"
    assert decision.branch == "B_REDUCE_TO_30_HZ"
    assert decision.recommended_position_hz_per_node == 30
    assert "do not increase" in decision.retry_policy


def test_r5_a9_margin_miss_has_reachable_30_hz_fallback() -> None:
    delayed_tail = replace(
        _profile(),
        position_p99_us=(1408,) * 7,
        position_p999_us=(1408,) * 7,
        position_max_us=(1412,) * 7,
    )
    first_pass = evaluate_can_r5(
        delayed_tail,
        _runtime(),
        current_node_quiet_us=5000,
        current_response_timeout_us=4000,
        current_position_hz_per_node=40,
    )

    assert first_pass.result == "RETEST_B"
    assert first_pass.branch == "B_REDUCE_TO_30_HZ"
    assert first_pass.recommended_position_hz_per_node == 30
    assert first_pass.recommended_node_quiet_us == 1608
    assert first_pass.recommended_response_timeout_us == 1608

    retest = evaluate_can_r5(
        delayed_tail,
        _runtime(),
        current_node_quiet_us=1608,
        current_response_timeout_us=1608,
        current_position_hz_per_node=30,
    )

    assert retest.result == "PASS"
    assert retest.branch == "B_VERIFY_30_HZ"
    assert retest.configuration_ready


def test_r5_branch_b_passes_after_measured_rate_and_timeout_are_retested() -> None:
    slow = replace(
        _profile(),
        position_p99_us=(3100,) * 7,
        position_p999_us=(3300,) * 7,
        position_max_us=(3500,) * 7,
    )
    decision = evaluate_can_r5(
        slow,
        _runtime(),
        current_node_quiet_us=3500,
        current_response_timeout_us=3500,
        current_position_hz_per_node=30,
    )
    assert decision.result == "PASS"
    assert decision.branch == "B_VERIFY_30_HZ"
    assert decision.single_transaction_budget_us == 4761


def test_r5_zero_gates_and_strict_percentile_boundaries_fail() -> None:
    decision = evaluate_can_r5(
        _profile(),
        _runtime(
            coherent_sweep_p99_ms=50.0,
            exact_fanout_p99_ms=15.0,
            exact_fanout_samples=11_999,
            position_timeout_count=1,
            action_credit_miss_events=1,
        ),
        current_node_quiet_us=500,
        current_response_timeout_us=500,
        current_position_hz_per_node=40,
    )
    assert decision.result == "FAIL"
    assert not decision.passed
    assert any("coherent sweep p99" in item for item in decision.failures)
    assert any("exact target fanout p99" in item for item in decision.failures)
    assert any("action lifecycles" in item for item in decision.failures)
    assert any("position timeout" in item for item in decision.failures)
    assert any("action-credit" in item for item in decision.failures)


def test_r5_branch_c_requires_explicit_bounded_concurrency_review() -> None:
    slow = replace(
        _profile(),
        position_p99_us=(3600,) * 7,
        position_p999_us=(3800,) * 7,
        position_max_us=(4000,) * 7,
    )
    decision = evaluate_can_r5(
        slow,
        _runtime(),
        current_node_quiet_us=5000,
        current_response_timeout_us=4000,
        current_position_hz_per_node=40,
        minimum_required_position_hz=40,
    )
    assert decision.result == "REVIEW_C"
    assert decision.branch == "C_BOUNDED_CONCURRENCY_REVIEW"
    assert any("unlimited outstanding is forbidden" in item for item in decision.failures)


def test_r5_fanout_percentile_uses_only_the_collection_window(tmp_path: Path) -> None:
    records = [
        {
            "event": "can_target_fanout",
            "monotonic_ns": 9,
            "payload": {
                "action_sequence": 1,
                "session_epoch": 7,
                "duration_us": 99_000,
            },
        },
        {"event": "collection_started", "monotonic_ns": 10},
        {
            "event": "can_target_fanout",
            "monotonic_ns": 11,
            "payload": {
                "action_sequence": 2,
                "session_epoch": 7,
                "duration_us": 900,
            },
        },
        {"event": "collection_stopped", "monotonic_ns": 12},
    ]
    path = tmp_path / "events.jsonl"
    path.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )

    durations, start_ns, stop_ns = _runtime_event_evidence(path, 7)

    assert durations == (900,)
    assert (start_ns, stop_ns) == (10, 12)


def test_r5_rejects_duplicate_exact_fanout_evidence(tmp_path: Path) -> None:
    records = [
        {"event": "collection_started", "monotonic_ns": 10},
        *(
            {
                "event": "can_target_fanout",
                "monotonic_ns": timestamp,
                "payload": {
                    "action_sequence": 2,
                    "session_epoch": 7,
                    "duration_us": 900,
                },
            }
            for timestamp in (11, 12)
        ),
        {"event": "collection_stopped", "monotonic_ns": 13},
    ]
    path = tmp_path / "events.jsonl"
    path.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="repeats an action sequence"):
        _runtime_event_evidence(path, 7)
