from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .board import (
    BoardDefinition,
    BoardDetection,
    BoardError,
    detect_board,
    estimate_camera_T_board,
)


class IntrinsicError(ValueError):
    pass


@dataclass(frozen=True)
class CameraIntrinsics:
    schema_version: int
    calibration_id: str
    calibrated_utc: str
    camera_model: str
    device_serial: str
    width: int
    height: int
    intrinsic_matrix: np.ndarray
    distortion_model: str
    distortion_coefficients: np.ndarray
    source_path: str
    file_hash: str


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise IntrinsicError(f"cannot read {path}: {exc}") from exc
    return digest.hexdigest()


def _load_mapping(path: Path) -> dict[str, Any]:
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise IntrinsicError(f"cannot load intrinsics {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise IntrinsicError("intrinsic calibration root must be a mapping")
    return raw


def load_intrinsics(path: str | Path) -> CameraIntrinsics:
    """Load either an intrinsics-only result or a final camera calibration."""
    source = Path(path)
    raw = _load_mapping(source)
    if raw.get("schema_version") != 1:
        raise IntrinsicError("intrinsic calibration schema_version must be 1")
    camera = raw.get("camera")
    intrinsics = raw.get("intrinsics")
    if not isinstance(camera, dict) or not isinstance(intrinsics, dict):
        raise IntrinsicError("camera and intrinsics mappings are required")
    try:
        calibration_id = str(raw["calibration_id"])
        calibrated_utc = str(raw["calibrated_utc"])
        model = str(camera["model"])
        device_serial = str(camera["device_serial"])
        width = int(camera["width"])
        height = int(camera["height"])
        matrix = np.asarray(intrinsics["matrix"], dtype=np.float64).reshape(3, 3)
        distortion_model = str(intrinsics["distortion_model"])
        distortion = np.asarray(intrinsics["coefficients"], dtype=np.float64).reshape(-1)
    except (KeyError, TypeError, ValueError) as exc:
        raise IntrinsicError(f"invalid intrinsic calibration: {exc}") from exc
    if not all((calibration_id, calibrated_utc, model, device_serial)):
        raise IntrinsicError("calibration and camera identity fields must be non-empty")
    if width <= 0 or height <= 0:
        raise IntrinsicError("camera resolution must be positive")
    if (
        not np.isfinite(matrix).all()
        or matrix[0, 0] <= 0
        or matrix[1, 1] <= 0
        or not np.allclose(matrix[2], [0.0, 0.0, 1.0], atol=1e-6)
        or distortion.size < 4
        or not np.isfinite(distortion).all()
    ):
        raise IntrinsicError("invalid intrinsic matrix or distortion vector")
    return CameraIntrinsics(
        schema_version=1,
        calibration_id=calibration_id,
        calibrated_utc=calibrated_utc,
        camera_model=model,
        device_serial=device_serial,
        width=width,
        height=height,
        intrinsic_matrix=matrix,
        distortion_model=distortion_model,
        distortion_coefficients=distortion,
        source_path=str(source.resolve()),
        file_hash=_sha256(source),
    )


def discover_images(directory: str | Path) -> list[Path]:
    root = Path(directory)
    extensions = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
    try:
        images = sorted(
            path for path in root.iterdir() if path.is_file() and path.suffix.lower() in extensions
        )
    except OSError as exc:
        raise IntrinsicError(f"cannot scan image directory {root}: {exc}") from exc
    if not images:
        raise IntrinsicError(f"no calibration images found in {root}")
    return images


def _read_rgb(path: Path) -> np.ndarray:
    try:
        import cv2
    except ImportError as exc:
        raise IntrinsicError("install dummy-host[opencv] to solve intrinsics") from exc
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise IntrinsicError(f"cannot decode image {path}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def _pose_reprojection(
    detection: BoardDetection,
    matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[float, np.ndarray]:
    transform, rms_px = estimate_camera_T_board(detection, matrix, distortion)
    return rms_px, transform


def solve_intrinsics(
    image_paths: list[Path],
    definition: BoardDefinition,
    *,
    camera_model: str,
    device_serial: str,
    calibration_id: str,
    output_path: str | Path,
    holdout_every: int = 5,
    min_corners: int = 8,
    min_train_images: int = 8,
) -> dict[str, object]:
    try:
        import cv2
    except ImportError as exc:
        raise IntrinsicError("install dummy-host[opencv] to solve intrinsics") from exc
    if holdout_every < 2:
        raise IntrinsicError("holdout_every must be at least 2")
    if min_train_images < 3:
        raise IntrinsicError("at least three training images are required")
    observations: list[tuple[Path, BoardDetection, tuple[int, int]]] = []
    rejected: list[dict[str, str]] = []
    expected_size: tuple[int, int] | None = None
    for path in image_paths:
        image = _read_rgb(path)
        size = (int(image.shape[1]), int(image.shape[0]))
        if expected_size is None:
            expected_size = size
        if size != expected_size:
            rejected.append({"image": str(path), "reason": f"resolution {size} != {expected_size}"})
            continue
        try:
            detection = detect_board(image, definition, min_corners=min_corners)
        except BoardError as exc:
            rejected.append({"image": str(path), "reason": str(exc)})
            continue
        observations.append((path, detection, size))
    train = [item for index, item in enumerate(observations) if (index + 1) % holdout_every != 0]
    holdout = [item for index, item in enumerate(observations) if (index + 1) % holdout_every == 0]
    if len(train) < min_train_images:
        raise IntrinsicError(
            f"only {len(train)} usable training images; need at least {min_train_images}"
        )
    if not holdout:
        raise IntrinsicError("no held-out images; capture more views or change holdout_every")
    assert expected_size is not None
    object_points = [item[1].object_points.astype(np.float32) for item in train]
    image_points = [item[1].image_points.astype(np.float32) for item in train]
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points,
        image_points,
        expected_size,
        None,
        None,
    )
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
    per_image: list[dict[str, object]] = []
    for split, samples in (("train", train), ("holdout", holdout)):
        for path, detection, _ in samples:
            reprojection_px, camera_T_board = _pose_reprojection(detection, matrix, distortion)
            per_image.append(
                {
                    "image": str(path.resolve()),
                    "split": split,
                    "corner_count": detection.corner_count,
                    "marker_count": detection.marker_count,
                    "reprojection_rms_px": reprojection_px,
                    "camera_T_board": camera_T_board.tolist(),
                }
            )

    def _summary(split: str) -> dict[str, float | int]:
        values = np.asarray(
            [item["reprojection_rms_px"] for item in per_image if item["split"] == split],
            dtype=np.float64,
        )
        return {
            "count": int(values.size),
            "mean_px": float(values.mean()),
            "p95_px": float(np.percentile(values, 95)),
            "max_px": float(values.max()),
        }

    calibrated_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    raw = {
        "schema_version": 1,
        "calibration_type": "camera_intrinsics",
        "calibration_id": calibration_id,
        "calibrated_utc": calibrated_utc,
        "camera": {
            "model": camera_model,
            "device_serial": str(device_serial),
            "width": expected_size[0],
            "height": expected_size[1],
        },
        "board": {
            "board_id": definition.board_id,
            "definition_sha256": definition.file_hash,
        },
        "intrinsics": {
            "matrix": np.asarray(matrix).reshape(-1).tolist(),
            "distortion_model": "brown_conrady",
            "coefficients": distortion.tolist(),
        },
        "fit": {
            "opencv_rms_px": float(rms),
            "train": _summary("train"),
            "holdout": _summary("holdout"),
            "holdout_every": holdout_every,
        },
    }
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    report = {
        "schema_version": 1,
        "calibration_id": calibration_id,
        "intrinsics_file": str(output.resolve()),
        "intrinsics_sha256": _sha256(output),
        "board_definition": definition.source_path,
        "board_definition_sha256": definition.file_hash,
        "fit": raw["fit"],
        "images": per_image,
        "rejected_images": rejected,
    }
    report_path = output.with_suffix(".report.json")
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return {**report, "report": str(report_path.resolve())}
