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
    if (
        manifest_schema_version >= 4
        and manifest.get("binary_protocol_version") != PROTOCOL_VERSION
    ):
        errors.append(
            "Raw Session schema v4 requires binary protocol v4 evidence; "
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
