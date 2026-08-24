from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal

import numpy as np
import yaml

from .geometry import (
    average_transforms,
    invert_transform,
    matrix_to_quaternion_xyzw,
    project_rotation,
    rotation_angle_rad,
    transform_error,
    validate_transform,
)
from .intrinsics import CameraIntrinsics
from .report import write_axis_visualization, write_hand_eye_html


class HandEyeError(ValueError):
    pass


@dataclass(frozen=True)
class PoseRecord:
    pose_id: str
    camera_role: str
    split: Literal["train", "holdout"]
    joint_position_rad: np.ndarray
    base_T_tool0: np.ndarray
    camera_T_board: np.ndarray
    board_reprojection_rms_px: float
    source_path: str


@dataclass(frozen=True)
class HandEyeSolution:
    mode: Literal["eye-in-hand", "eye-to-hand"]
    parent_frame: str
    parent_T_camera: np.ndarray
    board_transform_name: str
    board_transform: np.ndarray
    pair_count: int
    translation_rank: int
    rotation_condition: float


def _load_record(path: Path, *, default_split: str) -> PoseRecord:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise HandEyeError(f"cannot load pose record {path}: {exc}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise HandEyeError(f"{path}: pose schema_version must be 1")
    split = raw.get("split", default_split)
    if split not in {"train", "holdout"}:
        raise HandEyeError(f"{path}: split must be train or holdout")
    try:
        joints = np.asarray(raw["joint_position_rad"], dtype=np.float64)
        base_T_tool0 = validate_transform(np.asarray(raw["base_T_tool0"], dtype=np.float64))
        camera_T_board = validate_transform(
            np.asarray(raw["camera_T_board"], dtype=np.float64)
        )
        reprojection = float(raw["detection"]["reprojection_rms_px"])
        record = PoseRecord(
            pose_id=str(raw["pose_id"]),
            camera_role=str(raw["camera_role"]),
            split=split,
            joint_position_rad=joints,
            base_T_tool0=base_T_tool0,
            camera_T_board=camera_T_board,
            board_reprojection_rms_px=reprojection,
            source_path=str(path.resolve()),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise HandEyeError(f"{path}: invalid pose record: {exc}") from exc
    if joints.shape != (7,) or not np.isfinite(joints).all():
        raise HandEyeError(f"{path}: joint_position_rad must contain seven finite values")
    if not record.pose_id or not record.camera_role or not np.isfinite(reprojection):
        raise HandEyeError(f"{path}: pose identity and finite reprojection error are required")
    return record


def load_pose_records(directory: str | Path, *, holdout_every: int = 5) -> list[PoseRecord]:
    root = Path(directory)
    paths = sorted(root.glob("pose_*.json"))
    if len(paths) < 4:
        raise HandEyeError(f"need at least four pose_*.json records in {root}")
    records = [
        _load_record(
            path,
            default_split="holdout" if (index + 1) % holdout_every == 0 else "train",
        )
        for index, path in enumerate(paths)
    ]
    roles = {record.camera_role for record in records}
    if len(roles) != 1:
        raise HandEyeError(f"pose directory mixes camera roles: {sorted(roles)}")
    if sum(record.split == "train" for record in records) < 3:
        raise HandEyeError("need at least three training poses")
    if not any(record.split == "holdout" for record in records):
        raise HandEyeError("need at least one held-out pose")
    return records


def _motion_pairs(
    records: list[PoseRecord],
    mode: Literal["eye-in-hand", "eye-to-hand"],
) -> list[tuple[np.ndarray, np.ndarray]]:
    pairs: list[tuple[np.ndarray, np.ndarray]] = []
    for first_index in range(len(records)):
        for second_index in range(first_index + 1, len(records)):
            first = records[first_index]
            second = records[second_index]
            a = invert_transform(second.base_T_tool0) @ first.base_T_tool0
            if mode == "eye-in-hand":
                b = second.camera_T_board @ invert_transform(first.camera_T_board)
            else:
                b = invert_transform(second.camera_T_board) @ first.camera_T_board
            # Almost-identical poses add rows without adding information.
            if (
                rotation_angle_rad(a[:3, :3]) < np.deg2rad(0.5)
                and np.linalg.norm(a[:3, 3]) < 0.002
            ):
                continue
            pairs.append((a, b))
    if len(pairs) < 3:
        raise HandEyeError("pose set does not contain at least three distinct relative motions")
    return pairs


def solve_ax_xb(pairs: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, int, float]:
    rotation_rows: list[np.ndarray] = []
    for a, b in pairs:
        validate_transform(a, name="A")
        validate_transform(b, name="B")
        rotation_rows.append(
            np.kron(np.eye(3), a[:3, :3]) - np.kron(b[:3, :3].T, np.eye(3))
        )
    rotation_system = np.vstack(rotation_rows)
    _, singular_values, right = np.linalg.svd(rotation_system)
    raw = right[-1].reshape((3, 3), order="F")
    candidates = [project_rotation(raw), project_rotation(-raw)]

    def residual(rotation: np.ndarray) -> float:
        return float(
            sum(
                np.linalg.norm(a[:3, :3] @ rotation - rotation @ b[:3, :3]) ** 2
                for a, b in pairs
            )
        )

    rotation = min(candidates, key=residual)
    translation_left: list[np.ndarray] = []
    translation_right: list[np.ndarray] = []
    for a, b in pairs:
        translation_left.append(a[:3, :3] - np.eye(3))
        translation_right.append(rotation @ b[:3, 3] - a[:3, 3])
    left = np.vstack(translation_left)
    right_value = np.concatenate(translation_right)
    translation, _, rank, _ = np.linalg.lstsq(left, right_value, rcond=None)
    if rank < 3:
        raise HandEyeError(
            "pose rotations do not constrain all three translation axes; add diverse tilts"
        )
    # Small is poor: the null solution is not separated from the next singular direction.
    denominator = max(float(singular_values[-1]), 1e-15)
    condition = float(singular_values[-2] / denominator)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result, int(rank), condition


def solve_hand_eye(
    records: list[PoseRecord],
    *,
    mode: Literal["eye-in-hand", "eye-to-hand"],
) -> HandEyeSolution:
    training = [record for record in records if record.split == "train"]
    pairs = _motion_pairs(training, mode)
    unknown, rank, condition = solve_ax_xb(pairs)
    if mode == "eye-in-hand":
        tool0_T_camera = unknown
        base_T_board = average_transforms(
            record.base_T_tool0 @ tool0_T_camera @ record.camera_T_board
            for record in training
        )
        return HandEyeSolution(
            mode=mode,
            parent_frame="tool0",
            parent_T_camera=tool0_T_camera,
            board_transform_name="base_T_board",
            board_transform=base_T_board,
            pair_count=len(pairs),
            translation_rank=rank,
            rotation_condition=condition,
        )
    tool0_T_board = unknown
    base_T_camera = average_transforms(
        record.base_T_tool0 @ tool0_T_board @ invert_transform(record.camera_T_board)
        for record in training
    )
    return HandEyeSolution(
        mode=mode,
        parent_frame="base_link",
        parent_T_camera=base_T_camera,
        board_transform_name="tool0_T_board",
        board_transform=tool0_T_board,
        pair_count=len(pairs),
        translation_rank=rank,
        rotation_condition=condition,
    )


def _predict_camera_T_board(record: PoseRecord, solution: HandEyeSolution) -> np.ndarray:
    if solution.mode == "eye-in-hand":
        return (
            invert_transform(record.base_T_tool0 @ solution.parent_T_camera)
            @ solution.board_transform
        )
    return (
        invert_transform(solution.parent_T_camera)
        @ record.base_T_tool0
        @ solution.board_transform
    )


def _metric_summary(values: list[dict[str, object]], split: str) -> dict[str, float | int]:
    selected = [value for value in values if value["split"] == split]
    translation = np.asarray([value["translation_error_mm"] for value in selected])
    rotation = np.asarray([value["rotation_error_deg"] for value in selected])
    if not selected:
        return {"count": 0}
    return {
        "count": len(selected),
        "translation_mean_mm": float(translation.mean()),
        "translation_p95_mm": float(np.percentile(translation, 95)),
        "translation_max_mm": float(translation.max()),
        "rotation_mean_deg": float(rotation.mean()),
        "rotation_p95_deg": float(np.percentile(rotation, 95)),
        "rotation_max_deg": float(rotation.max()),
    }


def write_hand_eye_result(
    records: list[PoseRecord],
    solution: HandEyeSolution,
    intrinsics: CameraIntrinsics,
    *,
    calibration_id: str,
    board_id: str,
    board_definition_sha256: str,
    output_path: str | Path,
) -> dict[str, object]:
    roles = {record.camera_role for record in records}
    if len(roles) != 1:
        raise HandEyeError("records must contain exactly one camera role")
    role = next(iter(roles))
    expected_mode = "eye-in-hand" if role == "wrist" else "eye-to-hand"
    if solution.mode != expected_mode:
        raise HandEyeError(
            f"camera role {role!r} requires {expected_mode}, not {solution.mode}"
        )
    per_pose: list[dict[str, object]] = []
    for record in records:
        predicted = _predict_camera_T_board(record, solution)
        translation_mm, rotation_deg = transform_error(record.camera_T_board, predicted)
        per_pose.append(
            {
                "pose_id": record.pose_id,
                "split": record.split,
                "source": record.source_path,
                "translation_error_mm": translation_mm,
                "rotation_error_deg": rotation_deg,
                "board_reprojection_rms_px": record.board_reprojection_rms_px,
            }
        )
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    calibrated_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    rotation_xyzw = matrix_to_quaternion_xyzw(solution.parent_T_camera[:3, :3])
    raw = {
        "schema_version": 1,
        "calibration_id": calibration_id,
        "calibrated_utc": calibrated_utc,
        "camera": {
            "model": intrinsics.camera_model,
            "device_serial": intrinsics.device_serial,
            "width": intrinsics.width,
            "height": intrinsics.height,
        },
        "intrinsics": {
            "matrix": intrinsics.intrinsic_matrix.reshape(-1).tolist(),
            "distortion_model": intrinsics.distortion_model,
            "coefficients": intrinsics.distortion_coefficients.tolist(),
        },
        "extrinsics": {
            "parent_frame": solution.parent_frame,
            "translation_m": solution.parent_T_camera[:3, 3].tolist(),
            "rotation_xyzw": rotation_xyzw.tolist(),
        },
        "hand_eye": {
            "mode": solution.mode,
            "method": "numpy-ax-xb-svd",
            "camera_role": role,
            "board_id": board_id,
            "board_definition_sha256": board_definition_sha256,
            "intrinsics_source_sha256": intrinsics.file_hash,
            "pair_count": solution.pair_count,
            "translation_rank": solution.translation_rank,
            "rotation_nullspace_separation": solution.rotation_condition,
            solution.board_transform_name: solution.board_transform.tolist(),
        },
    }
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    report_path = output.with_suffix(".report.json")
    axis_path = output.with_suffix(".axes.svg")
    html_path = output.with_suffix(".report.html")
    frames: list[tuple[str, np.ndarray]] = [("base_link", np.eye(4))]
    if solution.mode == "eye-to-hand":
        frames.append(("camera", solution.parent_T_camera))
    for record in records:
        frames.append((f"tool0:{record.pose_id}", record.base_T_tool0))
        if solution.mode == "eye-in-hand":
            frames.append(
                (f"camera:{record.pose_id}", record.base_T_tool0 @ solution.parent_T_camera)
            )
    axis_file = write_axis_visualization(frames, axis_path)
    report: dict[str, object] = {
        "schema_version": 1,
        "calibration_id": calibration_id,
        "calibration_file": str(output.resolve()),
        "mode": solution.mode,
        "camera_role": role,
        "parent_frame": solution.parent_frame,
        "parent_T_camera": solution.parent_T_camera.tolist(),
        solution.board_transform_name: solution.board_transform.tolist(),
        "solver": raw["hand_eye"],
        "metrics": {
            "train": _metric_summary(per_pose, "train"),
            "holdout": _metric_summary(per_pose, "holdout"),
        },
        "poses": per_pose,
        "axis_visualization": axis_file,
    }
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    report["json_report"] = str(report_path.resolve())
    report["html_report"] = write_hand_eye_html(report, html_path)
    # Rewrite JSON once so it also points to the HTML report.
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return report
