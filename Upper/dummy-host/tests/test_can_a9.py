from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
import sys

from dummy_host.can_a9 import evaluate_can_a9
from dummy_host.apps.can_a9_profile import main as can_a9_profile_main
from dummy_host.protocol import (
    CAN_TIMING_PROFILE_FORMAT_VERSION,
    CAN_TIMING_PROFILE_PAYLOAD_SIZE,
    CAN_TIMING_PROFILE_WINDOW_VALID,
    CanTimingProfile,
    pack_can_timing_profile,
    unpack_can_timing_profile,
)


def _profile() -> CanTimingProfile:
    return CanTimingProfile(
        format_version=CAN_TIMING_PROFILE_FORMAT_VERSION,
        payload_size=CAN_TIMING_PROFILE_PAYLOAD_SIZE,
        session_epoch=7,
        window_reset_count=1,
        window_start_us=10,
        window_duration_us=600_000_000,
        motor_page_valid_mask=(0x7F,) * 4,
        window_flags=CAN_TIMING_PROFILE_WINDOW_VALID,
        position_samples=(24_000,) * 7,
        position_p50_us=(100,) * 7,
        position_p99_us=(200,) * 7,
        position_p999_us=(300,) * 7,
        position_max_us=(450,) * 7,
        temperature_samples=(600,) * 7,
        temperature_p50_us=(120,) * 7,
        temperature_p99_us=(220,) * 7,
        temperature_p999_us=(300,) * 7,
        temperature_max_us=(450,) * 7,
        motor_flags=(0x0F,) * 7,
        motor_can_samples=(30_000,) * 7,
        motor_can_p999_x10_us=(1000,) * 7,
        motor_can_max_x10_us=(1500,) * 7,
        motor_jitter_p999_x10_us=(100,) * 7,
        motor_jitter_max_x10_us=(200,) * 7,
        motor_control_p999_x10_us=(200,) * 7,
        motor_control_max_x10_us=(300,) * 7,
        motor_missed_ticks=(0,) * 7,
        timing_request=(150,) * 7,
        timing_response=(150,) * 7,
        timing_timeout=(0,) * 7,
    )


def test_can_timing_profile_fixed_payload_round_trip() -> None:
    profile = _profile()
    payload = pack_can_timing_profile(profile)
    assert len(payload) == 520
    assert unpack_can_timing_profile(payload) == profile


def test_a9_evaluator_applies_margin_and_schema_invariant() -> None:
    evaluation = evaluate_can_a9(_profile())
    assert evaluation.passed
    assert evaluation.result == "PASS"
    assert evaluation.measured_node_quiet_us == 200
    assert evaluation.recommended_response_timeout_us == 500
    # response_timeout <= node_quiet is a RobotConfig invariant.
    assert evaluation.recommended_node_quiet_us == 500
    assert evaluation.response_latency_us == {
        "position": {"p50": 100, "p99": 200, "p99_9": 300, "max": 450},
        "temperature": {"p50": 120, "p99": 220, "p99_9": 300, "max": 450},
    }


def test_a9_evaluator_rejects_missed_tick_and_relaxed_threshold() -> None:
    profile = replace(
        _profile(),
        motor_missed_ticks=(1, 0, 0, 0, 0, 0, 0),
        position_p999_us=(900,) * 7,
    )
    evaluation = evaluate_can_a9(profile)
    assert not evaluation.passed
    assert any("missed" in failure for failure in evaluation.failures)
    assert evaluation.recommended_response_timeout_us == 1100
    assert any("1000 us" in failure for failure in evaluation.failures)


def test_can_a9_cli_reports_fixed_fixture_with_explicit_pass(
    monkeypatch, capsys
) -> None:
    fixture = Path(__file__).parent / "fixtures" / "can_a9_valid_events.jsonl"
    monkeypatch.setattr(
        sys, "argv", ["dummy-host-can-a9-profile", "--events", str(fixture)]
    )

    can_a9_profile_main()

    output = json.loads(capsys.readouterr().out)
    assert output["result"] == "PASS"
    assert output["passed"] is True
    assert output["evidence_complete"] is True
    assert output["response_latency_us"]["position"] == {
        "p50": 106,
        "p99": 206,
        "p99_9": 300,
        "max": 450,
    }
    assert output["recommended_node_quiet_us"] == 500
    assert output["recommended_response_timeout_us"] == 500
