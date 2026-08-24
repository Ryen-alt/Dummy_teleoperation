from __future__ import annotations

import base64
import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .geometry import make_transform


class BoardError(ValueError):
    pass


@dataclass(frozen=True)
class BoardDefinition:
    schema_version: int
    board_id: str
    dictionary: str
    squares_x: int
    squares_y: int
    square_length_m: float
    marker_length_m: float
    paper_size: str
    paper_orientation: str
    dpi: int
    source_path: str
    file_hash: str

    @property
    def width_m(self) -> float:
        return self.squares_x * self.square_length_m

    @property
    def height_m(self) -> float:
        return self.squares_y * self.square_length_m


@dataclass(frozen=True)
class BoardDetection:
    image_points: np.ndarray
    object_points: np.ndarray
    corner_ids: np.ndarray
    marker_count: int

    @property
    def corner_count(self) -> int:
        return int(self.corner_ids.size)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise BoardError(f"cannot read board definition {path}: {exc}") from exc
    return digest.hexdigest()


def load_board_definition(path: str | Path) -> BoardDefinition:
    source = Path(path)
    try:
        raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise BoardError(f"cannot load board definition {source}: {exc}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise BoardError("board definition schema_version must be 1")
    if raw.get("type") != "charuco":
        raise BoardError("only type=charuco is supported")
    paper = raw.get("paper", {})
    if not isinstance(paper, dict):
        raise BoardError("paper must be a mapping")
    try:
        definition = BoardDefinition(
            schema_version=1,
            board_id=str(raw["board_id"]),
            dictionary=str(raw["dictionary"]),
            squares_x=int(raw["squares_x"]),
            squares_y=int(raw["squares_y"]),
            square_length_m=float(raw["square_length_m"]),
            marker_length_m=float(raw["marker_length_m"]),
            paper_size=str(paper.get("size", "A4")),
            paper_orientation=str(paper.get("orientation", "landscape")),
            dpi=int(paper.get("dpi", 600)),
            source_path=str(source.resolve()),
            file_hash=_sha256(source),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise BoardError(f"invalid board definition: {exc}") from exc
    if not definition.board_id.strip() or not definition.dictionary.startswith("DICT_"):
        raise BoardError("board_id and an OpenCV DICT_* dictionary are required")
    if definition.squares_x < 3 or definition.squares_y < 3:
        raise BoardError("ChArUco board must contain at least 3x3 squares")
    if (
        not np.isfinite(definition.square_length_m)
        or not np.isfinite(definition.marker_length_m)
        or definition.marker_length_m <= 0
        or definition.square_length_m <= definition.marker_length_m
    ):
        raise BoardError("square length must be finite and larger than marker length")
    if definition.paper_size != "A4" or definition.paper_orientation != "landscape":
        raise BoardError("the generator currently supports A4 landscape only")
    if not 72 <= definition.dpi <= 1200:
        raise BoardError("paper dpi must be between 72 and 1200")
    return definition


def _opencv_board(definition: BoardDefinition) -> Any:
    try:
        import cv2
    except ImportError as exc:
        raise BoardError("install dummy-host[opencv] to use calibration boards") from exc
    dictionary_id = getattr(cv2.aruco, definition.dictionary, None)
    if dictionary_id is None:
        raise BoardError(f"OpenCV does not provide {definition.dictionary!r}")
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.CharucoBoard(
        (definition.squares_x, definition.squares_y),
        definition.square_length_m,
        definition.marker_length_m,
        dictionary,
    )


def generate_printable_board(
    definition: BoardDefinition,
    output_path: str | Path,
) -> dict[str, object]:
    try:
        import cv2
    except ImportError as exc:
        raise BoardError("install dummy-host[opencv] to generate the board") from exc
    output = Path(output_path)
    if output.suffix.lower() != ".png":
        raise BoardError("printable board raster output must use a .png suffix")
    pixels_per_m = definition.dpi / 0.0254
    paper_width_px = round(0.297 * pixels_per_m)
    paper_height_px = round(0.210 * pixels_per_m)
    board_width_px = round(definition.width_m * pixels_per_m)
    board_height_px = round(definition.height_m * pixels_per_m)
    if board_width_px > paper_width_px or board_height_px > paper_height_px:
        raise BoardError("configured board does not fit on A4 landscape paper")
    rendered = _opencv_board(definition).generateImage(
        (board_width_px, board_height_px), marginSize=0, borderBits=1
    )
    page = np.full((paper_height_px, paper_width_px), 255, dtype=np.uint8)
    left = (paper_width_px - board_width_px) // 2
    top = (paper_height_px - board_height_px) // 2
    page[top : top + board_height_px, left : left + board_width_px] = rendered
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), page):
        raise BoardError(f"OpenCV could not write {output}")
    image_hash = _sha256(output)
    encoded_ok, encoded_png = cv2.imencode(".png", page)
    if not encoded_ok:
        raise BoardError("OpenCV could not encode the board for its print SVG")
    embedded_png = base64.b64encode(encoded_png.tobytes()).decode("ascii")
    print_svg = output.with_suffix(".print.svg")
    print_svg.write_text(
        "\n".join(
            (
                '<?xml version="1.0" encoding="UTF-8"?>',
                '<svg xmlns="http://www.w3.org/2000/svg" '
                'xmlns:xlink="http://www.w3.org/1999/xlink" '
                'width="297mm" height="210mm" viewBox="0 0 297 210">',
                f'<image x="0" y="0" width="297" height="210" '
                f'preserveAspectRatio="none" href="data:image/png;base64,{embedded_png}"/>',
                "</svg>",
                "",
            )
        ),
        encoding="utf-8",
    )
    manifest = {
        "schema_version": 1,
        "board_id": definition.board_id,
        "board_definition_sha256": definition.file_hash,
        "image_sha256": image_hash,
        "print": {
            "paper": "A4 landscape",
            "dpi": definition.dpi,
            "scale": "100% / actual size",
            "fit_to_page": False,
            "recommended_file": print_svg.name,
            "page_width_mm": 297.0,
            "page_height_mm": 210.0,
            "expected_board_width_mm": definition.width_m * 1000.0,
            "expected_board_height_mm": definition.height_m * 1000.0,
            "verify_square_mm": definition.square_length_m * 1000.0,
        },
    }
    manifest_path = output.with_suffix(output.suffix + ".manifest.yaml")
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return {
        **manifest,
        "image": str(output.resolve()),
        "print_svg": str(print_svg.resolve()),
        "manifest": str(manifest_path.resolve()),
    }


def detect_board(
    image_rgb_or_gray: np.ndarray,
    definition: BoardDefinition,
    *,
    min_corners: int = 8,
) -> BoardDetection:
    try:
        import cv2
    except ImportError as exc:
        raise BoardError("install dummy-host[opencv] to detect the board") from exc
    image = np.asarray(image_rgb_or_gray)
    if image.ndim == 3 and image.shape[2] == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    elif image.ndim == 2:
        gray = image
    else:
        raise BoardError("board image must be gray or RGB HWC")
    board = _opencv_board(definition)
    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    if charuco_corners is None or charuco_ids is None:
        raise BoardError("ChArUco board was not detected")
    ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    image_points = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
    if ids.size < min_corners:
        raise BoardError(f"detected {ids.size} ChArUco corners; need at least {min_corners}")
    all_object_points = np.asarray(board.getChessboardCorners(), dtype=np.float32).reshape(-1, 3)
    if ids.min(initial=0) < 0 or ids.max(initial=-1) >= len(all_object_points):
        raise BoardError("detector returned a corner ID outside the board definition")
    marker_count = 0 if marker_ids is None else int(np.asarray(marker_ids).size)
    return BoardDetection(
        image_points=image_points,
        object_points=all_object_points[ids],
        corner_ids=ids,
        marker_count=marker_count,
    )


def estimate_camera_T_board(
    detection: BoardDetection,
    intrinsic_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
) -> tuple[np.ndarray, float]:
    try:
        import cv2
    except ImportError as exc:
        raise BoardError("install dummy-host[opencv] to estimate board pose") from exc
    matrix = np.asarray(intrinsic_matrix, dtype=np.float64)
    distortion = np.asarray(distortion_coefficients, dtype=np.float64).reshape(-1)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise BoardError("invalid intrinsic matrix or distortion coefficients")
    success, rvec, tvec = cv2.solvePnP(
        detection.object_points,
        detection.image_points,
        matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        raise BoardError("solvePnP failed for the detected ChArUco board")
    rotation, _ = cv2.Rodrigues(rvec)
    transform = make_transform(rotation, np.asarray(tvec).reshape(3))
    projected, _ = cv2.projectPoints(
        detection.object_points, rvec, tvec, matrix, distortion
    )
    residual = projected.reshape(-1, 2) - detection.image_points
    rms_px = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    return transform, rms_px
