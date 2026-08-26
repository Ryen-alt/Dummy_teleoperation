from __future__ import annotations

import argparse
import html
import json
import sqlite3
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

from .session_check import SessionCheckReport, check_session


@dataclass(frozen=True)
class CameraQa:
    role: str
    sample_references: int
    unique_frames: int
    missing_frame_numbers: int
    mean_capture_latency_ms: float
    p95_capture_latency_ms: float
    max_capture_latency_ms: float
    mean_color_depth_skew_ms: float
    p95_color_depth_skew_ms: float
    max_color_depth_skew_ms: float
    calibration_versions: tuple[str, ...]
    timestamp_sources: tuple[str, ...]


@dataclass(frozen=True)
class SessionQaReport:
    ok: bool
    export_ready: bool
    data_classification: str
    offline_training_only: bool
    real_policy_execution_allowed: bool
    session: str
    integrity: SessionCheckReport
    duration_s: float
    measured_sample_hz: float
    tick_interval_p95_ms: float
    tick_interval_max_ms: float
    action_samples: int
    qualified_action_samples: int
    clipped_action_samples: int
    action_reason_counts: dict[str, int]
    fault_samples: int
    episode_outcomes: dict[str, int]
    state_min: tuple[float, ...]
    state_max: tuple[float, ...]
    action_min: tuple[float, ...]
    action_max: tuple[float, ...]
    cameras: tuple[CameraQa, ...]
    time_sync_models: int
    time_sync_rtt_p95_ms: float
    time_sync_residual_p95_ms: float
    can_diagnostic_windows: int
    can_timeout_count: int
    can_tx_error_count: int
    max_can_fanout_us: int
    errors: tuple[str, ...]
    warnings: tuple[str, ...]


def _percentile(values: Iterable[float], percentile: float) -> float:
    array = np.asarray(tuple(values), dtype=np.float64)
    return 0.0 if array.size == 0 else float(np.percentile(array, percentile))


def _range(values: list[np.ndarray]) -> tuple[tuple[float, ...], tuple[float, ...]]:
    if not values:
        empty = tuple(0.0 for _ in range(7))
        return empty, empty
    matrix = np.stack(values)
    return tuple(float(value) for value in matrix.min(axis=0)), tuple(
        float(value) for value in matrix.max(axis=0)
    )


def _decode_vector(blob: bytes | None, *, sample_index: int, field: str) -> np.ndarray | None:
    if blob is None:
        return None
    value = np.frombuffer(blob, dtype="<f4").copy()
    if value.shape != (7,) or not np.isfinite(value).all():
        raise ValueError(f"sample {sample_index} has invalid {field}")
    return value


def _episode_outcomes(events_path: Path) -> dict[str, int]:
    outcomes = {"accepted": 0, "failed": 0, "cancelled": 0, "incomplete": 0}
    active: set[str] = set()
    if not events_path.is_file():
        return outcomes
    for line in events_path.read_text(encoding="utf-8").splitlines():
        if not line:
            continue
        event = json.loads(line)
        payload = event.get("payload")
        if not isinstance(payload, dict):
            continue
        episode_id = payload.get("episode_id")
        if not isinstance(episode_id, str) or not episode_id:
            continue
        name = event.get("event")
        if name == "episode_start":
            active.add(episode_id)
        elif name in {"episode_success", "episode_failure", "episode_cancel"}:
            active.discard(episode_id)
            outcome = {
                "episode_success": "accepted",
                "episode_failure": "failed",
                "episode_cancel": "cancelled",
            }[str(name)]
            outcomes[outcome] += 1
    outcomes["incomplete"] = len(active)
    return outcomes


def analyze_session(session_dir: str | Path) -> tuple[SessionQaReport, tuple[np.ndarray, ...]]:
    session_dir = Path(session_dir)
    integrity = check_session(session_dir)
    manifest = json.loads((session_dir / "manifest.json").read_text(encoding="utf-8"))
    errors = list(integrity.errors)
    warnings = list(integrity.warnings)
    states: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    ticks: list[int] = []
    action_samples = 0
    clipped_samples = 0
    action_reason_counts: dict[str, int] = {}
    fault_samples = 0
    qualified_action_samples = 0
    time_sync_rtts_ms: list[float] = []
    time_sync_residuals_ms: list[float] = []
    can_diagnostic_windows = 0
    can_timeout_count = 0
    can_tx_error_count = 0
    max_can_fanout_us = 0
    try:
        schema_version = int(manifest.get("schema_version", 0))
        timing_column = "control_actual_start_ns" if schema_version >= 3 else "tick_ns"
        with sqlite3.connect(
            f"file:{(session_dir / 'samples.sqlite').as_posix()}?mode=ro&immutable=1",
            uri=True,
        ) as connection:
            rows = connection.execute(
                f"""
                SELECT sample_index, {timing_column}, state_position, applied_action,
                       action_clipped, action_reasons_json, state_fault_bits
                FROM samples ORDER BY {timing_column}, sample_index
                """
            ).fetchall()
            for (
                sample_index,
                tick_ns,
                state_blob,
                action_blob,
                clipped,
                reasons_json,
                fault_bits,
            ) in rows:
                ticks.append(int(tick_ns))
                try:
                    state = _decode_vector(
                        state_blob, sample_index=int(sample_index), field="state_position"
                    )
                    action = _decode_vector(
                        action_blob, sample_index=int(sample_index), field="applied_action"
                    )
                except ValueError as exc:
                    errors.append(str(exc))
                    continue
                if state is not None:
                    states.append(state)
                if action is not None:
                    actions.append(action)
                    action_samples += 1
                clipped_samples += int(bool(clipped))
                try:
                    reasons = json.loads(str(reasons_json))
                except json.JSONDecodeError:
                    errors.append(f"sample {sample_index} has invalid action_reasons_json")
                    reasons = []
                if isinstance(reasons, list):
                    for reason in reasons:
                        label = str(reason)
                        action_reason_counts[label] = action_reason_counts.get(label, 0) + 1
                fault_samples += int(int(fault_bits) != 0)

            source_column = (
                "timestamp_source"
                if schema_version >= 5
                else "'legacy_unknown' AS timestamp_source"
            )
            camera_rows = connection.execute(
                f"""
                SELECT role, frame_number, capture_ns, arrival_ns,
                       color_depth_skew_ms, calibration_version, frame_path,
                       {source_column}
                FROM camera_samples
                ORDER BY role, capture_ns, frame_number
                """
            ).fetchall()
            if schema_version >= 5:
                time_sync_rows = connection.execute(
                    "SELECT rtt_ns, residual_ns FROM time_sync_models"
                ).fetchall()
                time_sync_rtts_ms = [int(row[0]) / 1e6 for row in time_sync_rows]
                time_sync_residuals_ms = [float(row[1]) / 1e6 for row in time_sync_rows]
                can_row = connection.execute(
                    """
                    SELECT COUNT(*),
                           COALESCE(MAX(position_timeout_count + temperature_timeout_count), 0),
                           COALESCE(MAX(tx_abort_count + tx_error_count), 0),
                           COALESCE(MAX(max_fanout_us), 0)
                    FROM can_diagnostics
                    """
                ).fetchone()
                assert can_row is not None
                (
                    can_diagnostic_windows,
                    can_timeout_count,
                    can_tx_error_count,
                    max_can_fanout_us,
                ) = (int(value) for value in can_row)
                qualified_row = connection.execute(
                    """
                    SELECT COUNT(*)
                    FROM samples AS s
                    JOIN action_lifecycle AS a
                      ON a.action_sequence = s.action_sequence
                    JOIN time_sync_models AS m
                      ON m.model_id = s.time_sync_model_id
                    WHERE s.sample_valid = 1
                      AND s.state_fault_bits = 0
                      AND s.position_valid = 1
                      AND s.state_mode IN (3, 4)
                      AND s.coherent_sweep_id > 0
                      AND s.session_epoch > 0
                      AND s.control_tick_id > 0
                      AND s.session_epoch = a.session_epoch
                      AND s.control_tick_id = a.control_tick_id
                      AND a.acknowledged_host_ns IS NOT NULL
                      AND a.can_tx_complete_exact_host_ns IS NOT NULL
                      AND a.post_command_feedback_host_ns IS NOT NULL
                      AND a.terminal_stage IS NULL
                      AND a.post_command_feedback_mcu_us >= a.can_tx_complete_exact_mcu_us
                      AND a.post_command_feedback_mcu_us - a.can_tx_complete_exact_mcu_us <= 250000
                    """
                ).fetchone()
                assert qualified_row is not None
                qualified_action_samples = int(qualified_row[0])
    except sqlite3.Error as exc:
        errors.append(f"cannot analyze samples.sqlite: {exc}")
        camera_rows = []

    cameras: list[CameraQa] = []
    roles = sorted({str(row[0]) for row in camera_rows})
    for role in roles:
        role_rows = [row for row in camera_rows if str(row[0]) == role]
        unique = {}
        for row in role_rows:
            unique.setdefault(str(row[6]), row)
        unique_rows = sorted(unique.values(), key=lambda row: (int(row[2]), int(row[1])))
        missing = 0
        previous: int | None = None
        for row in unique_rows:
            number = int(row[1])
            if previous is not None and number > previous + 1:
                missing += number - previous - 1
            previous = number
        latencies = [(int(row[3]) - int(row[2])) / 1e6 for row in unique_rows]
        skews = [float(row[4]) for row in unique_rows]
        cameras.append(
            CameraQa(
                role=role,
                sample_references=len(role_rows),
                unique_frames=len(unique_rows),
                missing_frame_numbers=missing,
                mean_capture_latency_ms=0.0 if not latencies else float(np.mean(latencies)),
                p95_capture_latency_ms=_percentile(latencies, 95),
                max_capture_latency_ms=0.0 if not latencies else max(latencies),
                mean_color_depth_skew_ms=0.0 if not skews else float(np.mean(skews)),
                p95_color_depth_skew_ms=_percentile(skews, 95),
                max_color_depth_skew_ms=0.0 if not skews else max(skews),
                calibration_versions=tuple(sorted({str(row[5]) for row in unique_rows})),
                timestamp_sources=tuple(sorted({str(row[7]) for row in unique_rows})),
            )
        )

    tick_intervals_ms = np.diff(np.asarray(ticks, dtype=np.int64)) / 1e6
    duration_s = 0.0 if len(ticks) < 2 else (ticks[-1] - ticks[0]) / 1e9
    measured_hz = 0.0 if duration_s <= 0 else (len(ticks) - 1) / duration_s
    interval_p95_ms = _percentile(tick_intervals_ms, 95)
    interval_max_ms = 0.0 if tick_intervals_ms.size == 0 else float(tick_intervals_ms.max())
    outcomes = _episode_outcomes(session_dir / "events.jsonl")
    extra = manifest.get("extra")
    extra = extra if isinstance(extra, dict) else {}
    data_classification = str(extra.get("data_classification", "legacy_unspecified"))
    offline_training_only = extra.get("offline_training_only") is True
    real_policy_execution_allowed = extra.get("real_policy_execution_allowed") is True
    required_roles: set[str] = set()
    if extra.get("camera_required") is True:
        camera_roles = extra.get("camera_roles")
        if isinstance(camera_roles, list):
            required_roles = {str(role) for role in camera_roles}
    missing_roles = sorted(required_roles - set(roles))
    if missing_roles:
        errors.append(f"required camera roles have no recorded frames: {', '.join(missing_roles)}")
    if outcomes["incomplete"]:
        warnings.append(f"incomplete episodes: {outcomes['incomplete']}")
    if outcomes["accepted"] == 0:
        warnings.append("no accepted Episode is available for default dataset export")
    if fault_samples:
        warnings.append(f"fault bits were non-zero in {fault_samples} sample(s)")
    if clipped_samples:
        warnings.append(f"SafetyFilter clipped {clipped_samples} action sample(s)")
    if integrity.invalid_samples:
        warnings.append(f"invalid samples: {integrity.invalid_samples}")
    expected_hz = manifest.get("control_rate_hz")
    if isinstance(expected_hz, (int, float)) and expected_hz > 0 and measured_hz > 0:
        expected_period_ms = 1000.0 / float(expected_hz)
        if abs(measured_hz - float(expected_hz)) / float(expected_hz) > 0.05:
            warnings.append(
                f"measured sample rate {measured_hz:.3f} Hz differs from configured "
                f"{float(expected_hz):.3f} Hz by more than 5%"
            )
        if interval_p95_ms > expected_period_ms * 1.2:
            warnings.append(
                f"tick interval p95 {interval_p95_ms:.3f} ms exceeds 120% of period"
            )
        if interval_max_ms > expected_period_ms * 2.0:
            warnings.append(
                f"tick interval max {interval_max_ms:.3f} ms exceeds two control periods"
            )
    if not cameras:
        warnings.append("no camera frames; only state/action export recipes can use this session")
    if data_classification == "temporary_uncalibrated_pipeline_test":
        warnings.append(
            "TEMP/UNCALIBRATED: offline pipeline training only; real policy execution is forbidden"
        )
        if not offline_training_only or real_policy_execution_allowed:
            errors.append("temporary session safety classification fields are inconsistent")
    if schema_version < 4:
        warnings.append(
            "schema v2/v3 is inspectable but not strict-export-ready because exact "
            "CAN_QUEUED_EXACT/POST_COMMAND_FEEDBACK evidence is unavailable"
        )
    elif schema_version == 4:
        warnings.append(
            "schema v4 export is legacy-only and does not provide exact CAN TX "
            "completion or affine clock evidence"
        )
    elif schema_version in (5, 6):
        if not time_sync_rtts_ms:
            warnings.append("no fitted time-sync model is available")
        if can_diagnostic_windows == 0:
            warnings.append("no CAN diagnostic window is available")
        if qualified_action_samples < 2:
            warnings.append(
                "fewer than two actions satisfy the strict ACK/TX-complete/post-feedback gate"
            )
        if can_timeout_count or can_tx_error_count:
            warnings.append("CAN diagnostics contain timeout or TX error evidence")

    state_min, state_max = _range(states)
    action_min, action_max = _range(actions)
    ok = integrity.ok and integrity.clean_shutdown and not errors
    export_ready = (
        schema_version in (5, 6)
        and ok
        and outcomes["accepted"] > 0
        and action_samples > 0
        and qualified_action_samples >= 2
        and bool(time_sync_rtts_ms)
        and can_diagnostic_windows > 0
        and can_timeout_count == 0
        and can_tx_error_count == 0
        and not missing_roles
    )
    report = SessionQaReport(
        ok=ok,
        export_ready=export_ready,
        data_classification=data_classification,
        offline_training_only=offline_training_only,
        real_policy_execution_allowed=real_policy_execution_allowed,
        session=str(session_dir.resolve()),
        integrity=integrity,
        duration_s=duration_s,
        measured_sample_hz=measured_hz,
        tick_interval_p95_ms=interval_p95_ms,
        tick_interval_max_ms=interval_max_ms,
        action_samples=action_samples,
        qualified_action_samples=qualified_action_samples,
        clipped_action_samples=clipped_samples,
        action_reason_counts=dict(sorted(action_reason_counts.items())),
        fault_samples=fault_samples,
        episode_outcomes=outcomes,
        state_min=state_min,
        state_max=state_max,
        action_min=action_min,
        action_max=action_max,
        cameras=tuple(cameras),
        time_sync_models=len(time_sync_rtts_ms),
        time_sync_rtt_p95_ms=_percentile(time_sync_rtts_ms, 95),
        time_sync_residual_p95_ms=_percentile(time_sync_residuals_ms, 95),
        can_diagnostic_windows=can_diagnostic_windows,
        can_timeout_count=can_timeout_count,
        can_tx_error_count=can_tx_error_count,
        max_can_fanout_us=max_can_fanout_us,
        errors=tuple(errors),
        warnings=tuple(dict.fromkeys(warnings)),
    )
    return report, tuple(states)


def _trajectory_svg(states: tuple[np.ndarray, ...]) -> str:
    if not states:
        return "<p>No valid state trajectory is available.</p>"
    values = np.stack(states)
    width = 1000.0
    lane_height = 54.0
    left = 70.0
    plot_width = width - left - 10.0
    height = lane_height * 7
    colors = ("#2563eb", "#16a34a", "#dc2626", "#9333ea", "#ea580c", "#0891b2", "#4b5563")
    fragments = [f'<svg viewBox="0 0 {width:.0f} {height:.0f}" role="img" aria-label="state trajectories">']
    for joint in range(7):
        lane_top = joint * lane_height
        series = values[:, joint].astype(np.float64)
        low = float(series.min())
        high = float(series.max())
        span = high - low
        if span <= 1e-12:
            normalized = np.full(series.shape, 0.5)
        else:
            normalized = (series - low) / span
        if len(series) == 1:
            x_values = np.asarray([left])
        else:
            x_values = left + np.arange(len(series)) * plot_width / (len(series) - 1)
        y_values = lane_top + 45.0 - normalized * 36.0
        points = " ".join(f"{x:.1f},{y:.1f}" for x, y in zip(x_values, y_values, strict=True))
        label = f"J{joint + 1}" if joint < 6 else "Grip"
        fragments.append(
            f'<line x1="{left}" y1="{lane_top + 49}" x2="{width - 10}" y2="{lane_top + 49}" stroke="#e5e7eb"/>'
            f'<text x="4" y="{lane_top + 22}" font-size="13">{label}</text>'
            f'<text x="4" y="{lane_top + 40}" font-size="10">{low:.4f}..{high:.4f}</text>'
            f'<polyline fill="none" stroke="{colors[joint]}" stroke-width="1.5" points="{points}"/>'
        )
    fragments.append("</svg>")
    return "".join(fragments)


def render_html(report: SessionQaReport, states: tuple[np.ndarray, ...]) -> str:
    cameras = "".join(
        "<tr>"
        f"<td>{html.escape(camera.role)}</td><td>{camera.unique_frames}</td>"
        f"<td>{camera.missing_frame_numbers}</td>"
        f"<td>{camera.p95_capture_latency_ms:.2f}</td>"
        f"<td>{camera.p95_color_depth_skew_ms:.2f}</td>"
        f"<td>{html.escape(', '.join(camera.calibration_versions))}</td>"
        f"<td>{html.escape(', '.join(camera.timestamp_sources))}</td>"
        "</tr>"
        for camera in report.cameras
    )
    notices = "".join(
        f"<li>{html.escape(value)}</li>" for value in (*report.errors, *report.warnings)
    ) or "<li>None</li>"
    reasons = "".join(
        f"<li>{html.escape(reason)}: {count}</li>"
        for reason, count in report.action_reason_counts.items()
    ) or "<li>None</li>"
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>Dummy Raw Session QA</title>
<style>body{{font:14px system-ui;margin:2rem;max-width:1100px}}table{{border-collapse:collapse}}
td,th{{border:1px solid #d1d5db;padding:.4rem .6rem}}code{{word-break:break-all}}svg{{width:100%;height:auto;border:1px solid #e5e7eb}}</style></head>
<body><h1>Dummy Raw Session QA</h1><p><code>{html.escape(report.session)}</code></p>
<p><strong>Classification:</strong> {html.escape(report.data_classification)} &nbsp;
<strong>Offline training only:</strong> {report.offline_training_only} &nbsp;
<strong>Real policy execution:</strong> {report.real_policy_execution_allowed}</p>
<p><strong>Integrity:</strong> {report.ok} &nbsp; <strong>Export ready:</strong> {report.export_ready}</p>
<table><tr><th>Samples</th><th>Duration (s)</th><th>Measured Hz</th><th>Tick p95 (ms)</th><th>Actions</th><th>Qualified actions</th><th>Fault rows</th></tr>
<tr><td>{report.integrity.samples}</td><td>{report.duration_s:.3f}</td><td>{report.measured_sample_hz:.3f}</td>
<td>{report.tick_interval_p95_ms:.3f}</td><td>{report.action_samples}</td><td>{report.qualified_action_samples}</td><td>{report.fault_samples}</td></tr></table>
<h2>Execution evidence</h2>
<table><tr><th>Clock models</th><th>RTT p95 ms</th><th>Residual p95 ms</th><th>CAN windows</th><th>CAN timeouts</th><th>CAN TX errors</th><th>Max fan-out us</th></tr>
<tr><td>{report.time_sync_models}</td><td>{report.time_sync_rtt_p95_ms:.3f}</td><td>{report.time_sync_residual_p95_ms:.3f}</td><td>{report.can_diagnostic_windows}</td><td>{report.can_timeout_count}</td><td>{report.can_tx_error_count}</td><td>{report.max_can_fanout_us}</td></tr></table>
<h2>Safety action reasons</h2><ul>{reasons}</ul>
<h2>State trajectory</h2>{_trajectory_svg(states)}
<h2>Cameras</h2><table><tr><th>Role</th><th>Unique frames</th><th>Frame gaps</th><th>Latency p95 ms</th><th>RGB/depth skew p95 ms</th><th>Calibration</th><th>Timestamp source</th></tr>{cameras}</table>
<h2>Errors and warnings</h2><ul>{notices}</ul></body></html>"""


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze and visualize a Dummy Raw Session v2-v5"
    )
    parser.add_argument("--session", required=True)
    parser.add_argument("--json-output")
    parser.add_argument("--html-output")
    args = parser.parse_args()
    report, states = analyze_session(args.session)
    rendered = json.dumps(asdict(report), indent=2, ensure_ascii=False, allow_nan=False)
    print(rendered)
    if args.json_output:
        Path(args.json_output).write_text(rendered + "\n", encoding="utf-8")
    if args.html_output:
        Path(args.html_output).write_text(render_html(report, states), encoding="utf-8")
    if not report.ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
