from __future__ import annotations

from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest
import json
import sqlite3
import hashlib

from dummy_host.apps.can_r5_check import check_can_r5_session
from dummy_host.apps.soak_check import (
    SoakMetrics,
    SoakThresholds,
    check_soak_session,
    evaluate_soak_metrics,
)
from dummy_host.can_a9 import load_can_timing_profile_events
from dummy_host.can_r5 import CanR5Thresholds
from dummy_host.domain import ActionLifecycleUpdate, ActionStage
from dummy_host.protocol import (
    CAN_DIAGNOSTICS_FORMAT_VERSION,
    CAN_DIAGNOSTICS_PAYLOAD_SIZE,
    CAN_DIAGNOSTICS_WINDOW_VALID,
    CanDiagnostics,
)
from dummy_host.recording import ControlTickTiming, SessionRecorder
from dummy_host.schema import AppliedAction, ControlMode, RobotState
from dummy_host.teleop import KeyboardMapper, load_teleop_profile
from dummy_host.time_sync import TimeSyncExchange, TimeSyncModel


def _passing_metrics() -> SoakMetrics:
    return SoakMetrics(
        duration_s=3600.0,
        samples=72_000,
        invalid_samples=0,
        fault_samples=0,
        hold_samples=0,
        control_rate_hz=20.0,
        coherent_ratio=0.999,
        maximum_feedback_skew_ms=29.0,
        coherent_sweep_p99_ms=20.0,
        action_sequences=70_000,
        incomplete_action_sequences=0,
        superseded_actions=0,
        rejected_actions=0,
        failed_actions=0,
        action_credit_miss_events=0,
        bad_mode_rejections=0,
        target_ttl_hold_samples=0,
        reliable_rx_overflow=0,
        can_abort_error_count=0,
        can_recovery_count=0,
        can_busoff_count=0,
        can_rx_overflow_count=0,
        can_completion_overflow_count=0,
        motor_tx_drop_count=0,
        motor_rx_error_count=0,
        motor_busoff_count=0,
        can_unexpected_response_count=0,
        target_retry_count=9,
        target_retry_exhausted_count=0,
        target_deadline_failure_count=0,
        transition_failure_count=0,
        position_timeout_count=0,
        temperature_timeout_count=0,
        position_timeout_rate=0.0,
        diagnostic_window_valid=True,
        can_safety_preemption_count=0,
        time_sync_models=7_000,
        maximum_fanout_ms=9.9,
        maximum_rx_dispatch_latency_ms=0.199,
        maximum_rx_high_water=16,
        post_feedback_p99_ms=99.9,
        maximum_post_feedback_ms=249.9,
        maximum_serial_safety_wait_ms=9.9,
        maximum_can_safety_wait_ms=4.9,
        target_rate_hz_per_node=(50.0,) * 7,
        position_rate_hz_per_node=(40.0,) * 7,
        temperature_rate_hz_per_node=(1.0,) * 7,
    )


def test_v22_soak_metrics_pass_every_strict_gate() -> None:
    assert evaluate_soak_metrics(_passing_metrics()) == ()


def test_v22_soak_metrics_reject_boundary_and_fault_evidence() -> None:
    metrics = replace(
        _passing_metrics(),
        coherent_ratio=0.994,
        invalid_samples=1,
        incomplete_action_sequences=1,
        can_abort_error_count=1,
        maximum_fanout_ms=15.0,
        post_feedback_p99_ms=100.0,
        maximum_serial_safety_wait_ms=10.0,
        target_rate_hz_per_node=(40.0,) * 7,
    )
    failures = evaluate_soak_metrics(
        metrics, SoakThresholds(minimum_duration_s=600.0)
    )
    assert any("coherent ratio" in failure for failure in failures)
    assert any("invalid control sample" in failure for failure in failures)
    assert any("incomplete action lifecycle" in failure for failure in failures)
    assert any("CAN abort/error" in failure for failure in failures)
    assert any("CAN fan-out" in failure for failure in failures)
    assert any("post-feedback p99" in failure for failure in failures)
    assert any("serial safety wait" in failure for failure in failures)
    assert sum("target node" in failure for failure in failures) == 7


def _can_diagnostics(
    *,
    epoch: int,
    start_us: int,
    duration_us: int,
    target: int,
    position: int,
    temperature: int,
) -> CanDiagnostics:
    return CanDiagnostics(
        format_version=CAN_DIAGNOSTICS_FORMAT_VERSION,
        payload_size=CAN_DIAGNOSTICS_PAYLOAD_SIZE,
        session_epoch=epoch,
        motor_marker_mask=0x7F,
        window_flags=CAN_DIAGNOSTICS_WINDOW_VALID,
        window_reset_count=1,
        window_start_us=start_us,
        window_duration_us=duration_us,
        target_tx_complete=(target,) * 7,
        position_request=(position,) * 7,
        position_response=(position,) * 7,
        position_timeout=(0,) * 7,
        temperature_request=(temperature,) * 7,
        temperature_response=(temperature,) * 7,
        temperature_timeout=(0,) * 7,
        motor_tx_drop=(0,) * 7,
        motor_rx_error=(0,) * 7,
        motor_busoff=(0,) * 7,
        main_can_busoff=(0, 0),
        main_can_rx_overflow=(0, 0),
        main_can_rx_high_water=(8, 0),
        unexpected_response_count=0,
        maintenance_response_count=0,
        query_target_overlap_count=0,
        target_retry_count=0,
        target_retry_exhausted_count=0,
        target_deadline_failure_count=0,
        main_can_tx_abort=(0, 0),
        main_can_tx_error=(0, 0),
        main_can_tx_recovery=(0, 0),
        main_can_completion_overflow=(0, 0),
        safety_preemption_count=0,
        max_safety_wait_us=4_000,
        max_fanout_us=9_000,
        max_rx_dispatch_latency_us=100,
        main_can_rx_frame=(position + temperature, 0),
        main_can_tx_busy=(0, 0),
        transition_failure_count=0,
    )


def _record_soak_fixture(config, tmp_path: Path, *, acceptance=False) -> Path:
    profile = load_teleop_profile(
        Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    )
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="test",
        session_name="soak_fixture",
        queue_size=256,
    )
    epoch = 77
    start_ns = 10_000_000_000
    recorder.update_runtime_metadata(
        firmware_version="dummy-ref-v2.2.2", session_epoch=epoch
    )
    recorder.record_time_sync(
        TimeSyncExchange(start_ns, start_ns // 1000, start_ns // 1000, start_ns),
        TimeSyncModel(1, 1, 1000.0, 0.0, 0, 0.0, 3, start_ns),
    )
    recorder.record_can_diagnostics(
        _can_diagnostics(
            epoch=epoch,
            start_us=start_ns // 1_000,
            duration_us=0,
            target=0,
            position=0,
            temperature=0,
        ),
        host_time_ns=start_ns,
    )
    mapper = KeyboardMapper(profile)
    position = np.concatenate(
        (config.initial_pose_rad, np.asarray([0.5], dtype=np.float32))
    )
    recorder.record_event("collection_started", monotonic_ns=start_ns - 100_000_000 if acceptance else start_ns)
    if acceptance:
        recorder.enable_acceptance_window()
        recorder.record_event("acceptance_window_started", monotonic_ns=start_ns,
            payload={"session_epoch": epoch, "robot_config_hash": config.config_hash,
                     "diagnostic_start_ns": start_ns})
    for index in range(1, 21):
        tick_ns = start_ns + (index - 1) * 50_000_000
        state = RobotState(
            position=position.copy(),
            velocity=np.zeros(7, dtype=np.float32),
            monotonic_ns=tick_ns,
            mcu_time_us=tick_ns // 1000,
            mode=ControlMode.TELEOP,
            fault_bits=0,
            position_valid=True,
            velocity_valid=True,
            gripper_valid=True,
            last_received_sequence=index,
            target_age_ms=1,
            config_hash=config.config_hash,
            feedback_sweep_id=np.full(7, index, dtype=np.uint32),
            coherent_sweep_id=index,
            feedback_max_skew_us=29_000,
            coherent_reference_mcu_us=tick_ns // 1000,
        )
        if acceptance and index == 1:
            startup_ns = start_ns - 50_000_000
            recorder.record_sample(mapper.map(set(), startup_ns),
                replace(state, mode=ControlMode.HOLD),
                timing=ControlTickTiming(0, startup_ns, startup_ns, startup_ns),
                valid=False, invalid_reason="startup waiting for control")
        action = AppliedAction(
            position.copy(),
            position.copy(),
            index,
            tick_ns,
            False,
            (),
            session_epoch=epoch,
            control_tick_id=index,
        )
        recorder.record_sample(
            mapper.map({"KEY_SPACE"}, tick_ns),
            state,
            action=action,
            timing=ControlTickTiming(index, tick_ns, tick_ns, tick_ns),
        )
        for offset, stage in enumerate(
            (
                ActionStage.SAFETY_ACCEPTED,
                ActionStage.SEND_ENQUEUED,
                ActionStage.ACKNOWLEDGED,
                ActionStage.CAN_QUEUED_EXACT,
                ActionStage.CAN_TX_COMPLETE_EXACT,
                ActionStage.POST_COMMAND_FEEDBACK,
            )
        ):
            recorder.record_action_lifecycle(
                ActionLifecycleUpdate(
                    index,
                    stage,
                    tick_ns + offset * 1_000_000,
                    mcu_time_us=tick_ns // 1000 + offset * 1_000,
                    session_epoch=epoch,
                    control_tick_id=index,
                )
            )
        recorder.record_event(
            "can_target_fanout",
            monotonic_ns=tick_ns + 2_000_000,
            payload={
                "action_sequence": index,
                "session_epoch": epoch,
                "duration_us": 900,
            },
        )
    recorder.record_can_diagnostics(
        _can_diagnostics(
            epoch=epoch,
            start_us=start_ns // 1_000,
            duration_us=1_000_000,
            target=50,
            position=40,
            temperature=1,
        ),
        host_time_ns=start_ns + 1_000_000_000,
    )
    a9_fixture = load_can_timing_profile_events(
        Path(__file__).parent / "fixtures" / "can_a9_valid_events.jsonl"
    )
    recorder.record_event(
        "can_timing_profile",
        monotonic_ns=start_ns + 1_001_000_000 if acceptance else start_ns + 999_000_000,
        payload=asdict(
            replace(
                a9_fixture,
                session_epoch=epoch,
                window_start_us=start_ns // 1_000,
                window_duration_us=1_000_000,
            )
        ),
    )
    if acceptance:
        recorder.record_event("acceptance_window_stopped", monotonic_ns=start_ns + 1_000_000_000,
            payload={"session_epoch": epoch, "robot_config_hash": config.config_hash,
                     "diagnostic_stop_ns": start_ns + 1_000_000_000,
                     "evidence_end_ns": start_ns + 1_002_000_000})
    recorder.record_event(
        "collection_stopped", monotonic_ns=start_ns + (1_003_000_000 if acceptance else 1_000_000_000)
    )
    recorder.close()
    return recorder.session_dir


@pytest.mark.parametrize("acceptance", [False, True])
def test_soak_checker_reads_a_complete_v6_evidence_session(config, tmp_path: Path, acceptance) -> None:
    session = _record_soak_fixture(config, tmp_path, acceptance=acceptance)
    report = check_soak_session(
        session, SoakThresholds(minimum_duration_s=1.0)
    )
    assert report.ok, report.failures
    assert report.metrics.coherent_ratio == 1.0
    assert report.metrics.control_rate_hz == 20.0
    assert report.metrics.target_rate_hz_per_node == (50.0,) * 7

    r5 = check_can_r5_session(
        session,
        config,
        thresholds=CanR5Thresholds(
            minimum_duration_s=1.0,
            minimum_fanout_samples=10,
        ),
    )
    assert r5.result == "RECONFIGURE"
    assert r5.runtime.exact_fanout_samples == 20
    assert r5.runtime.exact_fanout_p99_ms == 0.9


def _seal_fixture(session: Path) -> None:
    """Seal synthetic mutations so semantic checks, not checksum failures, run."""
    db = sqlite3.connect(session / "samples.sqlite")
    db.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    db.close()
    checksums = json.loads((session / "checksums.json").read_text())
    checksums["files"] = {name: hashlib.sha256((session / name).read_bytes()).hexdigest()
                          for name in checksums["files"]}
    (session / "checksums.json").write_text(json.dumps(checksums))


@pytest.mark.parametrize("sql, expected", [
    ("UPDATE samples SET state_mode=2 WHERE sample_index=10", "HOLD sample"),
    ("UPDATE samples SET sample_valid=0 WHERE sample_index=10", "invalid control sample"),
    ("UPDATE samples SET session_epoch=78 WHERE sample_index=10", "changed session epoch"),
    ("UPDATE samples SET control_missed_periods=1 WHERE sample_index=10", "invalid control sample"),
    ("DELETE FROM action_lifecycle WHERE action_sequence=10", "no lifecycle record"),
    ("UPDATE action_lifecycle SET terminal_stage='preempted_by_safety' WHERE action_sequence=10", "invalid ownership"),
    ("UPDATE action_lifecycle SET post_command_feedback_host_ns=NULL WHERE action_sequence=20", "incomplete action"),
    ("UPDATE action_lifecycle SET post_command_feedback_host_ns=12000000000 WHERE action_sequence=20", "completion boundary"),
    ("UPDATE action_lifecycle SET send_enqueued_host_ns=9999999999 WHERE action_sequence=1", "outside the single"),
    ("DELETE FROM can_diagnostics WHERE host_time_ns=11000000000", "first and last diagnostic"),
])
def test_acceptance_preserves_strict_failure_gates(config, tmp_path, sql, expected):
    session = _record_soak_fixture(config, tmp_path, acceptance=True)
    with sqlite3.connect(session / "samples.sqlite") as db:
        db.execute(sql)
    _seal_fixture(session)
    report = check_soak_session(session, SoakThresholds(minimum_duration_s=1))
    assert not report.ok
    assert any(expected in failure for failure in report.failures), report.failures


@pytest.mark.parametrize("mutation", ["missing_stop", "duplicate_start", "epoch", "reversed", "far_boundary"])
def test_acceptance_rejects_missing_or_ambiguous_markers(config, tmp_path, mutation):
    session = _record_soak_fixture(config, tmp_path, acceptance=True)
    path = session / "events.jsonl"
    events = [json.loads(line) for line in path.read_text().splitlines()]
    start = next(e for e in events if e["event"] == "acceptance_window_started")
    if mutation == "missing_stop":
        events = [e for e in events if e["event"] != "acceptance_window_stopped"]
    elif mutation == "duplicate_start":
        events.append(start)
    elif mutation == "epoch":
        start["payload"]["session_epoch"] += 1
    elif mutation == "reversed":
        start["monotonic_ns"] += 2_000_000_000
    else:
        start["payload"]["diagnostic_start_ns"] -= 3_000_000_000
    path.write_text("".join(json.dumps(e) + "\n" for e in events))
    _seal_fixture(session)
    with pytest.raises(ValueError, match="acceptance"):
        check_soak_session(session, SoakThresholds(minimum_duration_s=1))


@pytest.mark.parametrize("change", ["epoch", "rollback"])
def test_acceptance_checks_intermediate_diagnostics(config, tmp_path, change):
    session = _record_soak_fixture(config, tmp_path, acceptance=True)
    with sqlite3.connect(session / "samples.sqlite") as db:
        columns = [r[1] for r in db.execute("PRAGMA table_info(can_diagnostics)") if r[1] != "diagnostic_index"]
        row = dict(zip(columns, db.execute(f"SELECT {','.join(columns)} FROM can_diagnostics ORDER BY host_time_ns DESC LIMIT 1").fetchone()))
        row["host_time_ns"] = 10_500_000_000
        row["window_duration_us"] = 500000
        if change == "epoch":
            row["session_epoch"] += 1
        else:
            row["unexpected_response_count"] = 1 # final snapshot goes back to zero
        db.execute(f"INSERT INTO can_diagnostics ({','.join(columns)}) VALUES ({','.join('?' for _ in columns)})", tuple(row.values()))
    _seal_fixture(session)
    report = check_soak_session(session, SoakThresholds(minimum_duration_s=1))
    assert not report.ok
    assert not report.metrics.diagnostic_window_valid


@pytest.mark.parametrize("change", ["inactive_final", "wrong_epoch", "wrong_origin", "different_sequence"])
def test_r5_rejects_cross_window_profile_and_fanout_evidence(config, tmp_path, change):
    session = _record_soak_fixture(config, tmp_path, acceptance=True)
    path = session / "events.jsonl"
    events = [json.loads(line) for line in path.read_text().splitlines()]
    profile = next(e for e in events if e["event"] == "can_timing_profile")
    if change == "inactive_final":
        earlier = json.loads(json.dumps(profile))
        earlier["monotonic_ns"] -= 10_000_000
        events.insert(events.index(profile), earlier)
        profile["payload"]["window_flags"] &= ~1
    elif change == "wrong_epoch":
        profile["payload"]["session_epoch"] += 1
    elif change == "wrong_origin":
        profile["payload"]["window_start_us"] += 1
    else:
        next(e for e in events if e["event"] == "can_target_fanout")["payload"]["action_sequence"] = 999
    path.write_text("".join(json.dumps(e) + "\n" for e in events))
    _seal_fixture(session)
    with pytest.raises(ValueError, match="A9 acceptance|fanout sequences"):
        check_can_r5_session(session, config,
            thresholds=CanR5Thresholds(minimum_duration_s=1, minimum_fanout_samples=10))


def test_last_action_can_complete_in_bounded_drain(config, tmp_path):
    session = _record_soak_fixture(config, tmp_path, acceptance=True)
    with sqlite3.connect(session / "samples.sqlite") as db:
        db.execute("""UPDATE action_lifecycle SET can_tx_complete_exact_host_ns=11000500000,
            post_command_feedback_host_ns=11001000000 WHERE action_sequence=20""")
    path = session / "events.jsonl"
    events = [json.loads(line) for line in path.read_text().splitlines()]
    next(e for e in events if e["event"] == "can_target_fanout"
         and e["payload"]["action_sequence"] == 20)["monotonic_ns"] = 11_000_500_000
    path.write_text("".join(json.dumps(e) + "\n" for e in events))
    _seal_fixture(session)
    before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in session.iterdir() if p.is_file()}
    report = check_soak_session(session, SoakThresholds(minimum_duration_s=1))
    assert report.ok, report.failures
    r5 = check_can_r5_session(session, config,
        thresholds=CanR5Thresholds(minimum_duration_s=1, minimum_fanout_samples=10))
    assert r5.runtime.exact_fanout_samples == 20
    assert r5.result == "RECONFIGURE"
    assert before == {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in session.iterdir() if p.is_file()}
