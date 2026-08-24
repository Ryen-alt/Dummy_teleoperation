from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from dummy_host.cameras import CameraFrame
from dummy_host.domain.models import RobotState

from .board import BoardDefinition, detect_board, estimate_camera_T_board
from .intrinsics import CameraIntrinsics
from .urdf import UrdfKinematics


class PoseCaptureError(ValueError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def next_pose_ordinal(directory: str | Path) -> int:
    root = Path(directory)
    ordinals: list[int] = []
    for path in root.glob("pose_*.json"):
        try:
            ordinals.append(int(path.stem.split("_", 1)[1]))
        except (IndexError, ValueError):
            continue
    return max(ordinals, default=0) + 1


def save_pose_record(
    output_directory: str | Path,
    *,
    ordinal: int,
    camera_role: str,
    split: str,
    frame: CameraFrame,
    state: RobotState,
    kinematics: UrdfKinematics,
    board: BoardDefinition,
    intrinsics: CameraIntrinsics,
    min_corners: int = 8,
) -> dict[str, object]:
    try:
        import cv2
    except ImportError as exc:
        raise PoseCaptureError("install dummy-host[opencv] to capture calibration poses") from exc
    if ordinal <= 0:
        raise PoseCaptureError("pose ordinal must be positive")
    if split not in {"train", "holdout"}:
        raise PoseCaptureError("pose split must be train or holdout")
    if not state.position_valid:
        raise PoseCaptureError("robot position feedback is not valid")
    if state.fault_bits:
        raise PoseCaptureError(f"robot is in fault state 0x{state.fault_bits:04x}")
    if frame.role != camera_role:
        raise PoseCaptureError(f"frame role {frame.role!r} != requested role {camera_role!r}")
    height, width = frame.color.shape[:2]
    if (width, height) != (intrinsics.width, intrinsics.height):
        raise PoseCaptureError(
            f"frame resolution {(width, height)} != intrinsics "
            f"{(intrinsics.width, intrinsics.height)}"
        )
    detection = detect_board(frame.color, board, min_corners=min_corners)
    camera_T_board, reprojection_px = estimate_camera_T_board(
        detection,
        intrinsics.intrinsic_matrix,
        intrinsics.distortion_coefficients,
    )
    joint_position = np.asarray(state.position, dtype=np.float64)
    base_T_tool0 = kinematics.base_T_tool0(joint_position[:6])
    root = Path(output_directory)
    root.mkdir(parents=True, exist_ok=True)
    pose_id = f"{ordinal:04d}"
    image_path = root / f"pose_{pose_id}.png"
    json_path = root / f"pose_{pose_id}.json"
    if image_path.exists() or json_path.exists():
        raise PoseCaptureError(f"pose {pose_id} already exists in {root}")
    bgr = cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(image_path), bgr):
        raise PoseCaptureError(f"could not write pose image {image_path}")
    sync_skew_ms = (frame.capture_time_ns - state.monotonic_ns) / 1e6
    record: dict[str, object] = {
        "schema_version": 1,
        "pose_id": pose_id,
        "camera_role": camera_role,
        "split": split,
        "image": image_path.name,
        "image_sha256": _sha256(image_path),
        "joint_position_rad": joint_position.tolist(),
        "base_T_tool0": base_T_tool0.tolist(),
        "camera_T_board": camera_T_board.tolist(),
        "capture": {
            "robot_monotonic_ns": state.monotonic_ns,
            "camera_capture_time_ns": frame.capture_time_ns,
            "sync_skew_ms": sync_skew_ms,
            "frame_number": frame.frame_number,
            "robot_mode": state.mode.name,
            "robot_config_hash": state.config_hash,
        },
        "urdf": {
            "path": str(kinematics.path.resolve()),
            "base_frame": kinematics.base_link,
            "tip_frame": kinematics.tip_link,
        },
        "board": {
            "board_id": board.board_id,
            "definition_sha256": board.file_hash,
        },
        "intrinsics": {
            "calibration_id": intrinsics.calibration_id,
            "file_sha256": intrinsics.file_hash,
        },
        "detection": {
            "corner_count": detection.corner_count,
            "marker_count": detection.marker_count,
            "corner_ids": detection.corner_ids.tolist(),
            "reprojection_rms_px": reprojection_px,
        },
    }
    try:
        json_path.write_text(
            json.dumps(record, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    except BaseException:
        image_path.unlink(missing_ok=True)
        raise
    return {**record, "record_path": str(json_path.resolve())}
