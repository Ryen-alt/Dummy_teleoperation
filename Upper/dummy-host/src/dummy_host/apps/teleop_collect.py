from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from dataclasses import asdict
from pathlib import Path
from threading import Event, Thread

from dummy_host.cameras import CameraManager
from dummy_host.calibration.urdf import UrdfError
from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.input_evdev import (
    EvdevKeyboardSource,
    InputDeviceError,
    create_gamepad_source,
    resolve_gamepad_endpoint,
)
from dummy_host.frame_archive import DEFAULT_MINIMUM_FREE_BYTES
from dummy_host.kinematics import (
    CartesianCalibration,
    DummyUrdfKinematics,
    load_cartesian_calibration,
)
from dummy_host.kinematics.contracts import KinematicsError
from dummy_host.recording import SessionRecorder, estimate_camera_archive_bytes
from dummy_host.robot_driver import DummyRobot
from dummy_host.schema import ConfigError, load_robot_config, validate_camera_rig_for_formal_collection
from dummy_host.teleop import load_teleop_profile, validate_profile_for_robot
from dummy_host.teleop_runtime import run_teleop_collection
from dummy_host.transport_serial import SerialTransport


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Collect keyboard/gamepad teleoperation through the common DummyRobot safety path"
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--input-config", required=True)
    parser.add_argument("--source", choices=("keyboard", "gamepad"), required=True)
    parser.add_argument(
        "--mode",
        choices=("joint", "cartesian"),
        default="joint",
        help="parallel teleoperation frontend; both modes emit the same absolute joint action",
    )
    parser.add_argument(
        "--urdf",
        help="canonical Dummy URDF; required for --mode cartesian",
    )
    parser.add_argument(
        "--cartesian-calibration",
        help=(
            "independent Cartesian-ready/TCP calibration YAML; mandatory and "
            "validated for real Cartesian execution"
        ),
    )
    parser.add_argument(
        "--device",
        required=True,
        help="Linux evdev path; gamepad source also accepts 'auto'",
    )
    transport = parser.add_mutually_exclusive_group(required=True)
    transport.add_argument("--simulate", action="store_true", help="use FakeMcuTransport")
    transport.add_argument("--execute", action="store_true", help="use the real USB CDC serial link")
    parser.add_argument("--port", help="required with --execute, for example /dev/ttyACM0")
    parser.add_argument("--session-root", required=True)
    parser.add_argument("--duration", type=float)
    parser.add_argument(
        "--progress-interval",
        type=float,
        default=5.0,
        help="seconds between progress messages written to stderr",
    )
    parser.add_argument("--task-id", default="teleop_unspecified")
    parser.add_argument("--task", default="Unspecified teleoperation task")
    parser.add_argument(
        "--with-cameras",
        "--with-d435",
        dest="with_cameras",
        action="store_true",
        help="start every enabled camera in the selected rig (--with-d435 is a compatibility alias)",
    )
    parser.add_argument("--camera-rig", help="optional independently versioned camera-rig YAML")
    parser.add_argument("--require-camera", action="store_true")
    parser.add_argument(
        "--temporary-uncalibrated",
        action="store_true",
        help=(
            "mark this camera session as TEMP/UNCALIBRATED for offline pipeline tests only; "
            "never authorizes real policy execution"
        ),
    )
    parser.add_argument(
        "--allow-joint",
        type=int,
        choices=range(1, 7),
        action="append",
        help="joint allowed to move; repeat for multiple joints (required for --execute)",
    )
    parser.add_argument("--allow-gripper", action="store_true")
    args = parser.parse_args()
    if args.execute and not args.port:
        parser.error("--execute requires --port")
    if args.require_camera and not args.with_cameras:
        parser.error("--require-camera also requires --with-cameras")
    if args.temporary_uncalibrated and not args.with_cameras:
        parser.error("--temporary-uncalibrated requires --with-cameras")
    if args.temporary_uncalibrated and args.require_camera:
        parser.error("--temporary-uncalibrated cannot be combined with --require-camera")
    if args.progress_interval <= 0:
        parser.error("--progress-interval must be positive")
    if args.execute and not args.allow_joint and not args.allow_gripper:
        parser.error("--execute requires at least one --allow-joint or --allow-gripper")
    if args.mode == "cartesian" and args.source != "gamepad":
        parser.error("--mode cartesian currently requires --source gamepad")
    if args.mode == "cartesian" and not args.urdf:
        parser.error("--mode cartesian requires --urdf Dummy_URDF/dummy.urdf")
    if args.mode != "cartesian" and args.cartesian_calibration:
        parser.error("--cartesian-calibration is only valid with --mode cartesian")
    if args.mode == "cartesian" and args.execute and not args.cartesian_calibration:
        parser.error("real Cartesian execution requires --cartesian-calibration")
    if (
        args.mode == "cartesian"
        and args.allow_joint
        and set(args.allow_joint) != set(range(1, 7))
    ):
        parser.error("Cartesian teleoperation cannot use a partial joint allow-list")
    if args.mode == "cartesian" and args.execute and set(args.allow_joint or ()) != set(
        range(1, 7)
    ):
        parser.error("real Cartesian teleoperation requires --allow-joint 1 through 6")

    config = load_robot_config(args.config, camera_rig_path=args.camera_rig)
    if args.require_camera:
        try:
            validate_camera_rig_for_formal_collection(config.camera_rig)
        except ConfigError as exc:
            parser.error(str(exc))
    profile = load_teleop_profile(args.input_config)
    validate_profile_for_robot(profile, config)
    if args.mode == "cartesian" and profile.cartesian is None:
        parser.error("input configuration does not define a cartesian section")
    kinematics = None
    cartesian_calibration: CartesianCalibration | None = None
    if args.mode == "cartesian":
        assert profile.cartesian is not None
        try:
            if args.cartesian_calibration:
                cartesian_calibration = load_cartesian_calibration(
                    args.cartesian_calibration
                )
                cartesian_calibration.validate_for(
                    config,
                    args.urdf,
                    require_validated=args.execute,
                )
            tool0_T_tip = (
                None
                if cartesian_calibration is None
                else cartesian_calibration.tool0_T_tcp
            )
            tip_frame = (
                "tool0"
                if tool0_T_tip is None
                else cartesian_calibration.tip_frame
            )
            kinematics = DummyUrdfKinematics(
                args.urdf,
                joint_min_rad=config.joint_limit_min_rad,
                joint_max_rad=config.joint_limit_max_rad,
                joint_limit_margin_rad=profile.cartesian.joint_limit_margin_rad,
                position_tolerance_m=profile.cartesian.position_tolerance_m,
                orientation_tolerance_rad=profile.cartesian.orientation_tolerance_rad,
                max_iterations=profile.cartesian.max_iterations,
                damping=profile.cartesian.damping,
                finite_difference_rad=profile.cartesian.finite_difference_rad,
                max_solver_step_rad=profile.cartesian.max_solver_step_rad,
                max_solution_step_rad=profile.cartesian.max_solution_step_rad,
                translation_scale_m=profile.cartesian.translation_scale_m,
                tool0_T_tip=tool0_T_tip,
                tip_frame=tip_frame,
                calibration_hash=(
                    None
                    if cartesian_calibration is None
                    else cartesian_calibration.file_hash
                ),
            )
        except (KinematicsError, UrdfError) as exc:
            parser.error(str(exc))
    try:
        resolved_device = (
            args.device
            if args.source == "keyboard"
            else resolve_gamepad_endpoint(args.device)
        )
        input_source = (
            EvdevKeyboardSource(resolved_device, profile)
            if args.source == "keyboard"
            else create_gamepad_source(
                resolved_device,
                profile,
                teleop_mode=args.mode,
            )
        )
    except InputDeviceError as exc:
        parser.exit(
            2,
            f"error: {exc}\n"
            "hint: for a gamepad use --device auto, or pass one existing "
            "*-event-joystick link.\n",
        )
    camera_manager = CameraManager.from_config(config.camera_rig) if args.with_cameras else None
    packet_transport = FakeMcuTransport(config) if args.simulate else SerialTransport(args.port)
    robot = DummyRobot(config, packet_transport, camera_manager=camera_manager)
    estimated_camera_bytes = (
        estimate_camera_archive_bytes(config, args.duration)
        if args.with_cameras and args.duration is not None
        else 0
    )
    session_root = Path(args.session_root)
    session_root.mkdir(parents=True, exist_ok=True)
    available_bytes = shutil.disk_usage(session_root).free
    required_bytes = estimated_camera_bytes + DEFAULT_MINIMUM_FREE_BYTES
    if estimated_camera_bytes and available_bytes < required_bytes:
        parser.error(
            "insufficient free space for lossless camera capture: "
            f"estimated={estimated_camera_bytes} free={available_bytes} "
            f"required_with_reserve={required_bytes}"
        )
    recorder = SessionRecorder(
        args.session_root,
        config,
        profile,
        source=args.source,
        extra_manifest={
            "transport": "fake_mcu" if args.simulate else "usb_cdc",
            "teleop_mode": args.mode,
            "teleop_semantics_version": 1,
            "kinematics": None if kinematics is None else kinematics.describe(),
            "cartesian_calibration": (
                None
                if cartesian_calibration is None
                else cartesian_calibration.as_dict()
            ),
            "cartesian_control_frame": (
                None if kinematics is None else kinematics.base_link
            ),
            "cartesian_tip_frame": (
                None if kinematics is None else kinematics.tip_link
            ),
            "real_cartesian_execution_allowed": bool(
                args.execute
                and cartesian_calibration is not None
                and cartesian_calibration.validated
            ),
            "input_device": resolved_device,
            "cameras_enabled": args.with_cameras,
            "camera_roles": list(camera_manager.roles) if camera_manager is not None else [],
            "camera_required": args.require_camera,
            "data_classification": (
                "temporary_uncalibrated_pipeline_test"
                if args.temporary_uncalibrated
                else ("formal_candidate" if args.require_camera else "engineering")
            ),
            "offline_training_only": args.temporary_uncalibrated,
            "real_policy_execution_allowed": False,
            "allowed_joints": list(range(1, 7))
            if args.simulate and args.allow_joint is None
            else sorted(set(args.allow_joint or ())),
            "gripper_allowed": args.allow_gripper,
            "estimated_camera_archive_bytes": estimated_camera_bytes,
            "free_bytes_at_start": available_bytes,
        },
    )
    if cartesian_calibration is not None:
        recorder.archive_cartesian_calibration(cartesian_calibration)
    clean_shutdown = False
    progress_stop = Event()
    progress_started = time.monotonic()

    def report_progress() -> None:
        while not progress_stop.wait(args.progress_interval):
            elapsed_s = time.monotonic() - progress_started
            remaining = (
                "unbounded"
                if args.duration is None
                else f"{max(0.0, args.duration - elapsed_s):.1f}s"
            )
            current = recorder.stats
            print(
                f"[collect] running elapsed={elapsed_s:.1f}s remaining={remaining} "
                f"samples={current.samples} events={current.events} "
                f"camera_frames={current.camera_frames} "
                f"queue_high_watermark={current.queue_high_watermark}",
                file=sys.stderr,
                flush=True,
            )

    progress_thread = Thread(
        target=report_progress,
        name="dummy-collect-progress",
        daemon=True,
    )
    print(
        f"[collect] started session_dir={recorder.session_dir} "
        f"duration={'unbounded' if args.duration is None else f'{args.duration:.1f}s'} "
        f"progress_interval={args.progress_interval:.1f}s "
        f"estimated_camera_gib={estimated_camera_bytes / 1024**3:.2f} "
        f"free_gib={available_bytes / 1024**3:.2f}",
        file=sys.stderr,
        flush=True,
    )
    progress_thread.start()
    try:
        stats = run_teleop_collection(
            robot,
            input_source,
            recorder,
            profile,
            duration_s=args.duration,
            require_camera=args.require_camera,
            allowed_joints=(
                None
                if args.simulate and args.allow_joint is None
                else set(args.allow_joint or ())
            ),
            allow_gripper=args.allow_gripper,
            task_id=args.task_id,
            task=args.task,
            teleop_mode=args.mode,
            kinematics=kinematics,
            cartesian_calibration=cartesian_calibration,
        )
        clean_shutdown = True
    except BaseException as exc:
        try:
            recorder.record_event("session_error", payload={"error": str(exc)})
        except BaseException:
            pass
        raise
    finally:
        progress_stop.set()
        progress_thread.join()
        recorder_stats = recorder.close(clean_shutdown=clean_shutdown)
    print(
        json.dumps(
            {
                "session_dir": str(recorder.session_dir),
                "run": asdict(stats),
                "recorder": asdict(recorder_stats),
            },
            indent=2,
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
