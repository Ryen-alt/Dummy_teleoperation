from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np

from dummy_host.apps.soak_check import (
    SoakMetrics,
    SoakThresholds,
    check_soak_session,
    evaluate_soak_metrics,
)
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


def test_soak_checker_reads_a_complete_v6_evidence_session(config, tmp_path: Path) -> None:
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
    recorder.record_event("collection_started", monotonic_ns=start_ns)
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
                ActionStage.ACKNOWLEDGED,
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
    recorder.record_event(
        "collection_stopped", monotonic_ns=start_ns + 1_000_000_000
    )
    recorder.close()

    report = check_soak_session(
        recorder.session_dir, SoakThresholds(minimum_duration_s=1.0)
    )
    assert report.ok, report.failures
    assert report.metrics.coherent_ratio == 1.0
    assert report.metrics.control_rate_hz == 20.0
    assert report.metrics.target_rate_hz_per_node == (50.0,) * 7
