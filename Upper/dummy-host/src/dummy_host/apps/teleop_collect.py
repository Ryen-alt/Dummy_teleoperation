from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from threading import Event, Thread

from dummy_host.cameras import CameraManager
from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.input_evdev import (
    EvdevKeyboardSource,
    InputDeviceError,
    create_gamepad_source,
    resolve_gamepad_endpoint,
)
from dummy_host.recording import SessionRecorder
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

    config = load_robot_config(args.config, camera_rig_path=args.camera_rig)
    if args.require_camera:
        try:
            validate_camera_rig_for_formal_collection(config.camera_rig)
        except ConfigError as exc:
            parser.error(str(exc))
    profile = load_teleop_profile(args.input_config)
    validate_profile_for_robot(profile, config)
    try:
        resolved_device = (
            args.device
            if args.source == "keyboard"
            else resolve_gamepad_endpoint(args.device)
        )
        input_source = (
            EvdevKeyboardSource(resolved_device, profile)
            if args.source == "keyboard"
            else create_gamepad_source(resolved_device, profile)
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
    recorder = SessionRecorder(
        args.session_root,
        config,
        profile,
        source=args.source,
        extra_manifest={
            "transport": "fake_mcu" if args.simulate else "usb_cdc",
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
        },
    )
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
        f"progress_interval={args.progress_interval:.1f}s",
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
