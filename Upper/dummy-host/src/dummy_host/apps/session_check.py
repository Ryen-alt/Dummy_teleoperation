from __future__ import annotations

import argparse
import hashlib
import json
import sqlite3
from dataclasses import asdict, dataclass
from pathlib import Path

from ..protocol import PROTOCOL_VERSION


class SessionCheckError(RuntimeError):
    pass


@dataclass(frozen=True)
class SessionCheckReport:
    ok: bool
    clean_shutdown: bool
    integrity: str
    samples: int
    invalid_samples: int
    camera_frames_referenced: int
    camera_files: int
    max_sent_sequence: int
    max_received_sequence: int
    max_completed_sequence: int
    errors: tuple[str, ...]
    warnings: tuple[str, ...]


def _load_json(path: Path) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SessionCheckError(f"cannot read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise SessionCheckError(f"{path} must contain a JSON object")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def check_session(session_dir: str | Path) -> SessionCheckReport:
    session_dir = Path(session_dir)
    manifest = _load_json(session_dir / "manifest.json")
    checksums = _load_json(session_dir / "checksums.json")
    expected_files = checksums.get("files")
    if not isinstance(expected_files, dict):
        raise SessionCheckError("checksums.json files must be a mapping")
    errors: list[str] = []
    warnings: list[str] = []
    try:
        manifest_schema_version = int(manifest.get("schema_version", 1))
    except (TypeError, ValueError):
        manifest_schema_version = 0
        errors.append("manifest schema_version is invalid")
    if manifest_schema_version not in (2, 3, 4, 5, 6):
        errors.append(
            f"unsupported Raw Session schema v{manifest_schema_version}; expected v2 through v6"
        )
    expected_protocol = {4: 4, 5: PROTOCOL_VERSION, 6: PROTOCOL_VERSION}.get(
        manifest_schema_version
    )
    if expected_protocol is not None and manifest.get("binary_protocol_version") != expected_protocol:
        errors.append(
            f"Raw Session schema v{manifest_schema_version} requires binary protocol "
            f"v{expected_protocol} evidence; "
            f"found {manifest.get('binary_protocol_version')!r}"
        )
    for relative, expected in expected_files.items():
        if not isinstance(relative, str) or not isinstance(expected, str):
            errors.append("checksums.json contains a non-string path or digest")
            continue
        path = session_dir / relative
        if not path.is_file():
            errors.append(f"missing checksummed file: {relative}")
            continue
        actual = _sha256(path)
        if actual != expected:
            errors.append(f"checksum mismatch: {relative}")

    actual_files = {
        path.relative_to(session_dir).as_posix()
        for path in session_dir.rglob("*")
        if path.is_file() and path.name != "checksums.json"
    }
    unlisted = sorted(actual_files - set(str(key) for key in expected_files))
    if unlisted:
        warnings.append(f"unlisted files: {', '.join(unlisted)}")

    calibration_records = manifest.get("camera_calibrations", {})
    if not isinstance(calibration_records, dict):
        errors.append("manifest camera_calibrations must be a mapping")
    else:
        for role, value in calibration_records.items():
            if not isinstance(role, str) or not isinstance(value, dict):
                errors.append("manifest contains an invalid camera calibration record")
                continue
            archive_path = value.get("archive_path")
            expected_hash = value.get("sha256")
            if not isinstance(archive_path, str) or not isinstance(expected_hash, str):
                errors.append(f"camera calibration {role} has no archive_path or sha256")
                continue
            path = session_dir / archive_path
            if not path.is_file():
                errors.append(f"missing archived camera calibration: {archive_path}")
            elif _sha256(path) != expected_hash:
                errors.append(f"camera calibration hash mismatch: {role}")

    cartesian_calibration = manifest.get("cartesian_calibration")
    if cartesian_calibration is not None:
        if not isinstance(cartesian_calibration, dict):
            errors.append("manifest Cartesian calibration must be a mapping or null")
        else:
            archive_path = cartesian_calibration.get("archive_path")
            expected_hash = cartesian_calibration.get("sha256")
            if not isinstance(archive_path, str) or not isinstance(expected_hash, str):
                errors.append("Cartesian calibration has no archive_path or sha256")
            else:
                path = session_dir / archive_path
                if not path.is_file():
                    errors.append(f"missing archived Cartesian calibration: {archive_path}")
                elif _sha256(path) != expected_hash:
                    errors.append("Cartesian calibration hash mismatch")

    db_path = session_dir / "samples.sqlite"
    try:
        connection = sqlite3.connect(
            f"file:{db_path.as_posix()}?mode=ro&immutable=1", uri=True
        )
        integrity_row = connection.execute("PRAGMA integrity_check").fetchone()
        integrity = "missing" if integrity_row is None else str(integrity_row[0])
        schema_version = manifest_schema_version
        if schema_version >= 5:
            required_columns = {
                "samples": {
                    "session_epoch",
                    "control_tick_id",
                    "time_sync_model_id",
                    "coherent_reference_mcu_us",
                },
                "camera_samples": {"timestamp_source"},
                "action_lifecycle": {
                    "session_epoch",
                    "control_tick_id",
                    "can_tx_complete_exact_host_ns",
                    "can_tx_complete_exact_mcu_us",
                    "accepted_to_ack_host_ns",
                    "ack_to_can_tx_complete_us",
                    "can_tx_complete_to_post_feedback_us",
                },
                "time_sync_models": {
                    "model_id",
                    "segment_id",
                    "slope_ns_per_us",
                    "intercept_ns",
                    "rtt_ns",
                    "residual_ns",
                },
                "time_sync_exchanges": {"rtt_ns", "model_id"},
                "can_diagnostics": {"max_fanout_us", "tx_error_count"},
            }
            if schema_version >= 6:
                required_columns["can_diagnostics"].update(
                    {
                        "format_version",
                        "payload_size",
                        "session_epoch",
                        "motor_marker_mask",
                        "window_flags",
                        "window_reset_count",
                        "position_request_json",
                        "position_timeout_json",
                        "temperature_request_json",
                        "temperature_timeout_json",
                        "motor_tx_drop_json",
                        "motor_rx_error_json",
                        "motor_busoff_json",
                        "main_can_busoff_json",
                        "main_can_rx_overflow_json",
                        "main_can_rx_high_water_json",
                        "unexpected_response_count",
                        "maintenance_response_count",
                        "query_target_overlap_count",
                        "target_retry_count",
                        "target_retry_exhausted_count",
                        "target_deadline_failure_count",
                        "main_can_tx_abort_json",
                        "main_can_tx_error_json",
                        "main_can_tx_recovery_json",
                        "main_can_completion_overflow_json",
                        "max_rx_dispatch_latency_us",
                        "main_can_rx_frame_json",
                        "main_can_tx_busy_json",
                        "transition_failure_count",
                    }
                )
            table_names = {
                str(row[0])
                for row in connection.execute(
                    "SELECT name FROM sqlite_master WHERE type = 'table'"
                )
            }
            for table, expected_columns in required_columns.items():
                if table not in table_names:
                    errors.append(f"Raw Session v5 is missing table {table}")
                    continue
                actual_columns = {
                    str(row[1])
                    for row in connection.execute(f"PRAGMA table_info({table})")
                }
                missing = sorted(expected_columns - actual_columns)
                if missing:
                    errors.append(
                        f"Raw Session v5 table {table} is missing columns: "
                        + ", ".join(missing)
                    )
            if schema_version >= 6:
                if manifest.get("can_diagnostics_format_version") != 2:
                    errors.append(
                        "Raw Session v6 requires can_diagnostics_format_version=2"
                    )
                invalid_diagnostics = connection.execute(
                    """
                    SELECT COUNT(*) FROM can_diagnostics
                    WHERE format_version != 2 OR payload_size != 380
                    """
                ).fetchone()
                if invalid_diagnostics and int(invalid_diagnostics[0]) > 0:
                    errors.append(
                        "Raw Session v6 contains non-v2 CAN diagnostic payloads"
                    )
        if schema_version >= 3:
            sample_row = connection.execute(
                """
                SELECT COUNT(*),
                       COALESCE(SUM(CASE WHEN sample_valid = 0 THEN 1 ELSE 0 END), 0),
                       COALESCE(MAX(action_sequence), 0),
                       COALESCE(MAX(last_received_sequence), 0)
                FROM samples
                """
            ).fetchone()
            lifecycle_column = (
                "post_command_feedback_host_ns"
                if schema_version >= 4
                else "motor_observed_host_ns"
            )
            applied_row = connection.execute(
                f"""
                SELECT COALESCE(MAX(action_sequence), 0)
                FROM action_lifecycle
                WHERE {lifecycle_column} IS NOT NULL
                """
            ).fetchone()
            assert sample_row is not None and applied_row is not None
            row = (*sample_row, applied_row[0])
        else:
            row = connection.execute(
                """
                SELECT COUNT(*),
                       COALESCE(SUM(CASE WHEN sample_valid = 0 THEN 1 ELSE 0 END), 0),
                       COALESCE(MAX(action_sequence), 0),
                       COALESCE(MAX(last_received_sequence), 0),
                       COALESCE(MAX(last_applied_sequence), 0)
                FROM samples
                """
            ).fetchone()
        if schema_version >= 2:
            frame_rows = connection.execute(
                "SELECT DISTINCT frame_path FROM camera_samples"
            ).fetchall()
            referenced_paths = {str(value[0]) for value in frame_rows}
        else:
            legacy = connection.execute(
                "SELECT COUNT(DISTINCT camera_frame_number) FROM samples"
            ).fetchone()
            referenced_paths = set()
            referenced = 0 if legacy is None else int(legacy[0])
        if schema_version >= 5:
            invalid_camera_sources = connection.execute(
                """
                SELECT COUNT(*) FROM camera_samples
                WHERE timestamp_source NOT IN ('hardware_exposure', 'arrival')
                """
            ).fetchone()
            missing_epoch = connection.execute(
                """
                SELECT COUNT(*) FROM samples
                WHERE action_sequence IS NOT NULL
                  AND (session_epoch = 0 OR control_tick_id = 0)
                """
            ).fetchone()
            model_count = connection.execute(
                "SELECT COUNT(*) FROM time_sync_models"
            ).fetchone()
            if invalid_camera_sources and int(invalid_camera_sources[0]) > 0:
                errors.append(
                    "Raw Session v5 contains camera frames without an explicit timestamp source"
                )
            if missing_epoch and int(missing_epoch[0]) > 0:
                errors.append(
                    "Raw Session v5 action samples are missing session epoch/control tick identity"
                )
            if model_count and int(model_count[0]) == 0:
                warnings.append("Raw Session v5 contains no fitted time-sync model")
        connection.close()
    except sqlite3.Error as exc:
        raise SessionCheckError(f"cannot validate samples.sqlite: {exc}") from exc
    if integrity != "ok":
        errors.append(f"SQLite integrity_check returned {integrity}")
    assert row is not None
    samples, invalid, sent, received, applied = (int(value) for value in row)
    frame_files = {
        path.relative_to(session_dir).as_posix()
        for path in (session_dir / "frames").rglob("*.npz")
    }
    camera_files = len(frame_files)
    if schema_version >= 2:
        referenced = len(referenced_paths)
        missing_references = sorted(referenced_paths - frame_files)
        orphan_files = sorted(frame_files - referenced_paths)
        if missing_references:
            errors.append(f"missing camera files: {', '.join(missing_references)}")
        if orphan_files:
            warnings.append(f"unreferenced camera files: {', '.join(orphan_files)}")
    if referenced != camera_files:
        errors.append(
            f"camera reference/file mismatch: referenced={referenced}, files={camera_files}"
        )
    if sent > received or sent > applied:
        warnings.append(
            f"target sequence did not close with recorded completion evidence: "
            f"sent={sent}, received={received}, completed={applied}"
        )
    clean_shutdown = manifest.get("clean_shutdown") is True
    if not clean_shutdown:
        warnings.append("manifest reports an unclean shutdown")
    return SessionCheckReport(
        ok=not errors,
        clean_shutdown=clean_shutdown,
        integrity=integrity,
        samples=samples,
        invalid_samples=invalid,
        camera_frames_referenced=referenced,
        camera_files=camera_files,
        max_sent_sequence=sent,
        max_received_sequence=received,
        max_completed_sequence=applied,
        errors=tuple(errors),
        warnings=tuple(warnings),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify a recorded Dummy raw session")
    parser.add_argument("--session", required=True)
    args = parser.parse_args()
    report = check_session(args.session)
    print(json.dumps(asdict(report), indent=2, ensure_ascii=False))
    if not report.ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
