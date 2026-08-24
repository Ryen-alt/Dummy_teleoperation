from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from dummy_host.calibration.board import (
    detect_board,
    generate_printable_board,
    load_board_definition,
)
from dummy_host.calibration.geometry import (
    axis_angle_rotation,
    invert_transform,
    make_transform,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
    transform_error,
)
from dummy_host.calibration.hand_eye import PoseRecord, solve_hand_eye, write_hand_eye_result
from dummy_host.calibration.intrinsics import CameraIntrinsics, load_intrinsics, solve_intrinsics
from dummy_host.calibration.urdf import UrdfError, UrdfKinematics
from dummy_host.cameras import CameraError, CameraManager
from dummy_host.schema import load_camera_calibration


REPOSITORY = Path(__file__).parents[3]
URDF = REPOSITORY / "Dummy_URDF" / "dummy.urdf"
BOARD = REPOSITORY / "Upper" / "dummy-host" / "configs" / "calibration_board_charuco.yaml"


def _transform(axis: list[float], angle: float, xyz: list[float]) -> np.ndarray:
    return make_transform(
        axis_angle_rotation(np.asarray(axis, dtype=np.float64), angle),
        np.asarray(xyz, dtype=np.float64),
    )


def _robot_poses() -> list[np.ndarray]:
    return [
        _transform([1.0, 0.2, 0.1], 0.15 + index * 0.17, [0.03 * index, -0.02 * index, 0.2 + 0.01 * index])
        @ _transform([0.0, 1.0, 0.0], index * 0.13, [0.0, 0.0, 0.0])
        for index in range(8)
    ]


def test_geometry_quaternion_and_inverse_round_trip() -> None:
    transform = _transform([0.2, 1.0, -0.3], 0.72, [0.1, -0.2, 0.3])
    quaternion = matrix_to_quaternion_xyzw(transform[:3, :3])
    assert np.allclose(quaternion_xyzw_to_matrix(quaternion), transform[:3, :3])
    assert np.allclose(transform @ invert_transform(transform), np.eye(4), atol=1e-12)


def test_urdf_defines_identity_tool0_and_produces_fk() -> None:
    model = UrdfKinematics(URDF)
    assert model.joint_names == tuple(f"joint_{index}" for index in range(1, 7))
    assert model.chain[-1].name == "tool0_fixed"
    pose = np.asarray([0.1, 0.2, -0.3, 0.4, -0.2, 0.5])
    base_T_tool0 = model.base_T_tool0(pose)
    assert np.allclose(base_T_tool0[3], [0.0, 0.0, 0.0, 1.0])
    assert np.allclose(base_T_tool0[:3, :3].T @ base_T_tool0[:3, :3], np.eye(3))
    with pytest.raises(UrdfError, match="joint_2"):
        model.base_T_tool0(np.zeros(6))


def test_calibration_can_select_one_camera_role(config) -> None:
    manager = CameraManager.from_config(config.camera_rig, roles={"wrist"})
    assert manager.roles == ("wrist",)
    with pytest.raises(CameraError, match="global"):
        CameraManager.from_config(config.camera_rig, roles={"global"})


def test_printable_board_manifest_and_detection(tmp_path: Path) -> None:
    cv2 = pytest.importorskip("cv2")

    definition = load_board_definition(BOARD)
    output = tmp_path / "charuco.png"
    manifest = generate_printable_board(definition, output)
    assert manifest["print"]["fit_to_page"] is False
    assert manifest["print"]["verify_square_mm"] == 30.0
    assert Path(manifest["print_svg"]).is_file()
    assert 'width="297mm" height="210mm"' in Path(manifest["print_svg"]).read_text(
        encoding="utf-8"
    )
    assert Path(manifest["manifest"]).is_file()
    image = cv2.imread(str(output), cv2.IMREAD_GRAYSCALE)
    detection = detect_board(image, definition, min_corners=20)
    assert detection.corner_count == 24


def test_intrinsic_solver_keeps_deterministic_holdout(tmp_path: Path) -> None:
    cv2 = pytest.importorskip("cv2")

    definition = load_board_definition(BOARD)
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, definition.dictionary)
    )
    board = cv2.aruco.CharucoBoard(
        (definition.squares_x, definition.squares_y),
        definition.square_length_m,
        definition.marker_length_m,
        dictionary,
    )
    source = board.generateImage((700, 500), marginSize=0, borderBits=1)
    source_corners = np.float32([[0, 0], [699, 0], [699, 499], [0, 499]])
    quads = [
        [[120, 80], [520, 100], [550, 390], [90, 370]],
        [[30, 30], [400, 60], [430, 320], [50, 400]],
        [[180, 20], [610, 70], [580, 410], [160, 360]],
        [[90, 110], [480, 50], [550, 350], [120, 420]],
        [[200, 100], [580, 120], [600, 390], [190, 370]],
        [[40, 100], [450, 80], [480, 400], [60, 360]],
        [[150, 50], [550, 20], [590, 350], [120, 390]],
        [[80, 40], [500, 100], [520, 420], [100, 350]],
        [[210, 60], [620, 100], [570, 400], [180, 370]],
        [[70, 130], [480, 90], [550, 360], [100, 440]],
    ]
    paths: list[Path] = []
    for index, quad in enumerate(quads):
        homography = cv2.getPerspectiveTransform(source_corners, np.float32(quad))
        image = cv2.warpPerspective(source, homography, (640, 480), borderValue=255)
        path = tmp_path / f"frame_{index:04d}.png"
        assert cv2.imwrite(str(path), image)
        paths.append(path)
    output = tmp_path / "intrinsics.yaml"
    report = solve_intrinsics(
        paths,
        definition,
        camera_model="synthetic-camera",
        device_serial="synthetic-1",
        calibration_id="synthetic-intrinsics-v1",
        output_path=output,
        holdout_every=5,
        min_corners=8,
        min_train_images=8,
    )
    loaded = load_intrinsics(output)
    assert (loaded.width, loaded.height) == (640, 480)
    assert report["fit"]["train"]["count"] == 8
    assert report["fit"]["holdout"]["count"] == 2


def _records(mode: str) -> tuple[list[PoseRecord], np.ndarray, np.ndarray]:
    robot_poses = _robot_poses()
    if mode == "eye-in-hand":
        parent_T_camera = _transform([0.2, 1.0, 0.3], 0.35, [0.03, -0.02, 0.05])
        board_transform = _transform([0.0, 0.0, 1.0], -0.4, [0.15, -0.08, 0.25])
        camera_poses = [
            invert_transform(robot @ parent_T_camera) @ board_transform
            for robot in robot_poses
        ]
        role = "wrist"
    else:
        board_transform = _transform([1.0, 0.3, 0.2], -0.25, [0.01, 0.02, 0.08])
        parent_T_camera = _transform([0.1, 1.0, 0.4], 0.5, [0.3, -0.2, 0.4])
        camera_poses = [
            invert_transform(parent_T_camera) @ robot @ board_transform
            for robot in robot_poses
        ]
        role = "global"
    records = [
        PoseRecord(
            pose_id=f"{index + 1:04d}",
            camera_role=role,
            split="holdout" if index in {4, 7} else "train",
            joint_position_rad=np.zeros(7),
            base_T_tool0=robot,
            camera_T_board=camera,
            board_reprojection_rms_px=0.1,
            source_path=f"pose_{index + 1:04d}.json",
        )
        for index, (robot, camera) in enumerate(zip(robot_poses, camera_poses, strict=True))
    ]
    return records, parent_T_camera, board_transform


@pytest.mark.parametrize("mode", ["eye-in-hand", "eye-to-hand"])
def test_numpy_hand_eye_solver_recovers_both_mounting_modes(mode: str) -> None:
    records, expected_camera, expected_board = _records(mode)
    solution = solve_hand_eye(records, mode=mode)
    assert transform_error(expected_camera, solution.parent_T_camera)[0] < 1e-6
    assert transform_error(expected_camera, solution.parent_T_camera)[1] < 1e-8
    assert transform_error(expected_board, solution.board_transform)[0] < 1e-6
    assert solution.translation_rank == 3


def test_hand_eye_output_matches_formal_calibration_schema(tmp_path: Path) -> None:
    records, _, _ = _records("eye-in-hand")
    solution = solve_hand_eye(records, mode="eye-in-hand")
    intrinsics = CameraIntrinsics(
        schema_version=1,
        calibration_id="wrist-intrinsics-v1",
        calibrated_utc="2026-08-23T00:00:00Z",
        camera_model="D435",
        device_serial="1234",
        width=640,
        height=480,
        intrinsic_matrix=np.asarray([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]),
        distortion_model="brown_conrady",
        distortion_coefficients=np.zeros(5),
        source_path="intrinsics.yaml",
        file_hash="0" * 64,
    )
    output = tmp_path / "wrist.yaml"
    report = write_hand_eye_result(
        records,
        solution,
        intrinsics,
        calibration_id="wrist-full-v1",
        board_id="board-v1",
        board_definition_sha256="1" * 64,
        output_path=output,
    )
    loaded = load_camera_calibration(output)
    assert loaded.parent_frame == "tool0"
    assert report["metrics"]["holdout"]["count"] == 2
    assert Path(report["axis_visualization"]).is_file()
    assert Path(report["html_report"]).is_file()
    machine_report = json.loads(Path(report["json_report"]).read_text(encoding="utf-8"))
    assert machine_report["mode"] == "eye-in-hand"
