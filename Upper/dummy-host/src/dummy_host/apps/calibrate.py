from __future__ import annotations

import argparse
import hashlib
import json
import logging
import time
from pathlib import Path

import numpy as np

from dummy_host.apps.camera_check import wait_for_first_frames
from dummy_host.calibration.board import (
    BoardError,
    detect_board,
    generate_printable_board,
    load_board_definition,
)
from dummy_host.calibration.hand_eye import (
    HandEyeError,
    load_pose_records,
    solve_hand_eye,
    write_hand_eye_result,
)
from dummy_host.calibration.intrinsics import (
    IntrinsicError,
    discover_images,
    load_intrinsics,
    solve_intrinsics,
)
from dummy_host.calibration.pose import PoseCaptureError, next_pose_ordinal, save_pose_record
from dummy_host.calibration.urdf import UrdfError, UrdfKinematics
from dummy_host.cameras import CameraError, CameraManager
from dummy_host.robot_driver import DummyRobot, RobotError
from dummy_host.schema import ConfigError, load_robot_config
from dummy_host.transport_serial import SerialTransport, TransportError

LOG = logging.getLogger(__name__)
DEFAULT_URDF = "Dummy_URDF/dummy.urdf"
DEFAULT_BOARD = "Upper/dummy-host/configs/calibration_board_charuco.yaml"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json_print(value: object) -> None:
    print(json.dumps(value, indent=2, ensure_ascii=False))


def _validate_urdf(args: argparse.Namespace) -> None:
    kinematics = UrdfKinematics(args.urdf)
    result = kinematics.describe()
    # The historical CAD zero pose is useful for structural validation even
    # though joint_2's configured operating limit begins slightly above zero.
    result["urdf_zero_base_T_tool0"] = kinematics.base_T_tool0(
        np.zeros(6), check_limits=False
    ).tolist()
    _json_print(result)


def _fk(args: argparse.Namespace) -> None:
    kinematics = UrdfKinematics(args.urdf)
    values = np.asarray(args.joint_deg if args.joint_deg is not None else args.joint_rad)
    unit = "deg" if args.joint_deg is not None else "rad"
    positions = np.deg2rad(values) if unit == "deg" else values
    transform = kinematics.base_T_tool0(positions, check_limits=not args.ignore_limits)
    _json_print(
        {
            "base_frame": kinematics.base_link,
            "tip_frame": kinematics.tip_link,
            "joint_names": list(kinematics.joint_names),
            "joint_position_rad": positions.tolist(),
            "base_T_tool0": transform.tolist(),
        }
    )


def _board_generate(args: argparse.Namespace) -> None:
    definition = load_board_definition(args.board)
    _json_print(generate_printable_board(definition, args.output))


def _camera_and_config(args: argparse.Namespace) -> tuple[object, CameraManager]:
    config = load_robot_config(args.config, camera_rig_path=args.camera_rig)
    if args.role not in config.camera_rig.cameras:
        raise ConfigError(f"camera role {args.role!r} is not in the selected rig")
    manager = CameraManager.from_config(config.camera_rig, roles={args.role})
    return config, manager


def _intrinsics_capture(args: argparse.Namespace) -> None:
    try:
        import cv2
    except ImportError as exc:
        raise IntrinsicError("install dummy-host[opencv] to capture intrinsic images") from exc
    config, manager = _camera_and_config(args)
    camera_config = config.camera_rig.cameras[args.role]
    definition = load_board_definition(args.board)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    existing = list(output.glob("frame_*.png"))
    if existing:
        raise IntrinsicError(
            f"{output} already contains frame_*.png; use a new directory to keep runs immutable"
        )
    accepted: list[dict[str, object]] = []
    rejected = 0
    manager.start()
    try:
        frames = wait_for_first_frames(manager, timeout_s=args.startup_timeout)
        last_frame_number = -1
        next_capture = time.monotonic()
        deadline = time.monotonic() + args.timeout
        while len(accepted) < args.frames:
            if time.monotonic() >= deadline:
                raise IntrinsicError(
                    f"capture timed out with {len(accepted)}/{args.frames} accepted views"
                )
            frame = frames[args.role]
            if frame.frame_number == last_frame_number or time.monotonic() < next_capture:
                time.sleep(0.02)
                frames = manager.latest_all()
                continue
            last_frame_number = frame.frame_number
            next_capture = time.monotonic() + args.interval
            try:
                detection = detect_board(frame.color, definition, min_corners=args.min_corners)
            except BoardError as exc:
                rejected += 1
                print(f"[intrinsics] rejected frame={frame.frame_number}: {exc}")
                frames = manager.latest_all()
                continue
            index = len(accepted) + 1
            path = output / f"frame_{index:04d}.png"
            if not cv2.imwrite(str(path), cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR)):
                raise IntrinsicError(f"could not write {path}")
            accepted.append(
                {
                    "image": path.name,
                    "image_sha256": _sha256(path),
                    "frame_number": frame.frame_number,
                    "capture_time_ns": frame.capture_time_ns,
                    "corner_count": detection.corner_count,
                    "marker_count": detection.marker_count,
                }
            )
            print(
                f"[intrinsics] accepted {index}/{args.frames} frame={frame.frame_number} "
                f"corners={detection.corner_count}"
            )
            frames = manager.latest_all()
    finally:
        manager.stop()
    manifest = {
        "schema_version": 1,
        "capture_type": "camera_intrinsics",
        "camera_role": args.role,
        "camera": {
            "model": camera_config.model,
            "device_serial": camera_config.device_serial,
            "width": camera_config.width,
            "height": camera_config.height,
            "fps": camera_config.fps,
        },
        "board": {
            "board_id": definition.board_id,
            "definition_sha256": definition.file_hash,
        },
        "accepted": accepted,
        "rejected_count": rejected,
    }
    manifest_path = output / "capture_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    _json_print({"output_dir": str(output.resolve()), "manifest": str(manifest_path.resolve())})


def _intrinsics_solve(args: argparse.Namespace) -> None:
    definition = load_board_definition(args.board)
    report = solve_intrinsics(
        discover_images(args.images),
        definition,
        camera_model=args.camera_model,
        device_serial=args.device_serial,
        calibration_id=args.calibration_id,
        output_path=args.output,
        holdout_every=args.holdout_every,
        min_corners=args.min_corners,
        min_train_images=args.min_train_images,
    )
    _json_print(
        {
            "intrinsics": report["intrinsics_file"],
            "report": report["report"],
            "fit": report["fit"],
            "rejected_images": len(report["rejected_images"]),
        }
    )


def _pose_capture(args: argparse.Namespace) -> None:
    config, manager = _camera_and_config(args)
    camera_config = config.camera_rig.cameras[args.role]
    intrinsics = load_intrinsics(args.intrinsics)
    if (
        camera_config.model != intrinsics.camera_model
        or camera_config.device_serial != intrinsics.device_serial
        or (camera_config.width, camera_config.height) != (intrinsics.width, intrinsics.height)
    ):
        raise PoseCaptureError(
            "selected camera identity/resolution does not match the intrinsic calibration"
        )
    definition = load_board_definition(args.board)
    kinematics = UrdfKinematics(args.urdf)
    output = Path(args.output_dir)
    robot = DummyRobot(
        config,
        SerialTransport(args.port, args.baudrate),
        camera_manager=manager,
    )
    saved: list[dict[str, object]] = []
    with robot:
        wait_for_first_frames(manager, timeout_s=args.startup_timeout)
        for capture_index in range(args.count):
            if args.prompt:
                input(
                    f"Pose {capture_index + 1}/{args.count}: keep the robot and board still, "
                    "then press Enter to record..."
                )
            if args.settle > 0:
                time.sleep(args.settle)
            state = robot.read_state()
            frame = manager.get(args.role).nearest(
                state.monotonic_ns,
                max_skew_ms=camera_config.max_sync_skew_ms,
            )
            ordinal = next_pose_ordinal(output)
            split = (
                "holdout" if ordinal % args.holdout_every == 0 else "train"
            ) if args.split == "auto" else args.split
            record = save_pose_record(
                output,
                ordinal=ordinal,
                camera_role=args.role,
                split=split,
                frame=frame,
                state=state,
                kinematics=kinematics,
                board=definition,
                intrinsics=intrinsics,
                min_corners=args.min_corners,
            )
            saved.append(record)
            print(
                f"[pose] saved {record['record_path']} split={split} "
                f"reprojection={record['detection']['reprojection_rms_px']:.3f}px"
            )
    _json_print({"output_dir": str(output.resolve()), "saved": len(saved)})


def _hand_eye_solve(args: argparse.Namespace) -> None:
    records = load_pose_records(args.poses, holdout_every=args.holdout_every)
    definition = load_board_definition(args.board)
    intrinsics = load_intrinsics(args.intrinsics)
    solution = solve_hand_eye(records, mode=args.mode)
    report = write_hand_eye_result(
        records,
        solution,
        intrinsics,
        calibration_id=args.calibration_id,
        board_id=definition.board_id,
        board_definition_sha256=definition.file_hash,
        output_path=args.output,
    )
    _json_print(
        {
            "calibration": report["calibration_file"],
            "json_report": report["json_report"],
            "html_report": report["html_report"],
            "axis_visualization": report["axis_visualization"],
            "metrics": report["metrics"],
        }
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Dummy robot URDF, camera intrinsic, and hand-eye calibration tools"
    )
    parser.add_argument("--log-level", default="INFO")
    commands = parser.add_subparsers(dest="command", required=True)

    validate = commands.add_parser("validate-urdf", help="verify base_link -> tool0 contract")
    validate.add_argument("--urdf", default=DEFAULT_URDF)
    validate.set_defaults(handler=_validate_urdf)

    fk = commands.add_parser("fk", help="print base_T_tool0 for one six-joint pose")
    fk.add_argument("--urdf", default=DEFAULT_URDF)
    group = fk.add_mutually_exclusive_group(required=True)
    group.add_argument("--joint-rad", nargs=6, type=float)
    group.add_argument("--joint-deg", nargs=6, type=float)
    fk.add_argument("--ignore-limits", action="store_true")
    fk.set_defaults(handler=_fk)

    board = commands.add_parser("board-generate", help="generate a printable ChArUco board")
    board.add_argument("--board", default=DEFAULT_BOARD)
    board.add_argument("--output", required=True)
    board.set_defaults(handler=_board_generate)

    capture = commands.add_parser(
        "intrinsics-capture", help="capture accepted ChArUco views from one camera role"
    )
    capture.add_argument("--config", required=True)
    capture.add_argument("--camera-rig", required=True)
    capture.add_argument("--role", choices=("wrist", "global"), required=True)
    capture.add_argument("--board", default=DEFAULT_BOARD)
    capture.add_argument("--output-dir", required=True)
    capture.add_argument("--frames", type=int, default=30)
    capture.add_argument("--interval", type=float, default=0.5)
    capture.add_argument("--timeout", type=float, default=300.0)
    capture.add_argument("--startup-timeout", type=float, default=5.0)
    capture.add_argument("--min-corners", type=int, default=12)
    capture.set_defaults(handler=_intrinsics_capture)

    solve = commands.add_parser(
        "intrinsics-solve", help="solve intrinsics with deterministic held-out views"
    )
    solve.add_argument("--images", required=True)
    solve.add_argument("--board", default=DEFAULT_BOARD)
    solve.add_argument("--camera-model", required=True)
    solve.add_argument("--device-serial", required=True)
    solve.add_argument("--calibration-id", required=True)
    solve.add_argument("--output", required=True)
    solve.add_argument("--holdout-every", type=int, default=5)
    solve.add_argument("--min-corners", type=int, default=12)
    solve.add_argument("--min-train-images", type=int, default=16)
    solve.set_defaults(handler=_intrinsics_solve)

    pose = commands.add_parser(
        "pose-capture", help="record image, camera_T_board and base_T_tool0 for static poses"
    )
    pose.add_argument("--config", required=True)
    pose.add_argument("--camera-rig", required=True)
    pose.add_argument("--role", choices=("wrist", "global"), required=True)
    pose.add_argument("--port", required=True)
    pose.add_argument("--baudrate", type=int, default=115_200)
    pose.add_argument("--urdf", default=DEFAULT_URDF)
    pose.add_argument("--board", default=DEFAULT_BOARD)
    pose.add_argument("--intrinsics", required=True)
    pose.add_argument("--output-dir", required=True)
    pose.add_argument("--count", type=int, default=1)
    pose.add_argument("--prompt", action="store_true")
    pose.add_argument("--settle", type=float, default=0.5)
    pose.add_argument("--startup-timeout", type=float, default=5.0)
    pose.add_argument("--min-corners", type=int, default=12)
    pose.add_argument("--split", choices=("auto", "train", "holdout"), default="auto")
    pose.add_argument("--holdout-every", type=int, default=5)
    pose.set_defaults(handler=_pose_capture)

    hand_eye = commands.add_parser(
        "hand-eye-solve", help="solve D435 eye-in-hand or phone eye-to-hand extrinsics"
    )
    hand_eye.add_argument("--mode", choices=("eye-in-hand", "eye-to-hand"), required=True)
    hand_eye.add_argument("--poses", required=True)
    hand_eye.add_argument("--intrinsics", required=True)
    hand_eye.add_argument("--board", default=DEFAULT_BOARD)
    hand_eye.add_argument("--calibration-id", required=True)
    hand_eye.add_argument("--output", required=True)
    hand_eye.add_argument("--holdout-every", type=int, default=5)
    hand_eye.set_defaults(handler=_hand_eye_solve)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    logging.basicConfig(level=args.log_level.upper())
    numeric_positive = (
        "frames",
        "interval",
        "timeout",
        "startup_timeout",
        "min_corners",
        "min_train_images",
        "count",
        "holdout_every",
    )
    for name in numeric_positive:
        value = getattr(args, name, None)
        if value is not None and value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if getattr(args, "holdout_every", 2) < 2:
        parser.error("--holdout-every must be at least 2")
    try:
        args.handler(args)
    except (
        BoardError,
        CameraError,
        ConfigError,
        HandEyeError,
        IntrinsicError,
        PoseCaptureError,
        RobotError,
        TransportError,
        UrdfError,
    ) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
