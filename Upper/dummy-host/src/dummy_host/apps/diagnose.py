from __future__ import annotations

import argparse
import json
import logging
import time
from dataclasses import asdict

from dummy_host.cameras import CameraManager
from dummy_host.protocol import PROTOCOL_VERSION
from dummy_host.robot_driver import DummyRobot
from dummy_host.schema import load_robot_config
from dummy_host.transport_serial import SerialTransport


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only Dummy robot and D435 diagnostics")
    parser.add_argument("--config", required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115_200)
    parser.add_argument("--without-camera", action="store_true")
    parser.add_argument("--camera-rig", help="optional independently versioned camera-rig YAML")
    parser.add_argument("--log-level", default="INFO")
    parser.add_argument("--duration", type=float, help="stop after this many seconds")
    parser.add_argument("--interval", type=float, default=0.5, help="status print interval in seconds")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    if args.interval <= 0:
        parser.error("--interval must be positive")
    logging.basicConfig(level=args.log_level.upper())

    config = load_robot_config(args.config, camera_rig_path=args.camera_rig)
    camera_manager = None if args.without_camera else CameraManager.from_config(config.camera_rig)
    robot = DummyRobot(
        config,
        SerialTransport(args.port, args.baudrate),
        camera_manager=camera_manager,
    )
    print(f"host_config_hash={config.config_hash}")
    try:
        with robot:
            print(f"firmware_version={robot.firmware_version}")
            print(f"binary_protocol_version={PROTOCOL_VERSION}")
            print(f"session_epoch={robot.session_id}")
            print(f"firmware_capabilities=0x{robot.firmware_capabilities:08x}")
            print(
                "can_diagnostics_start="
                + json.dumps(
                    asdict(robot.read_can_diagnostics()),
                    sort_keys=True,
                    separators=(",", ":"),
                )
            )
            started = time.monotonic()
            while args.duration is None or time.monotonic() - started < args.duration:
                state = robot.read_state()
                camera_text = "disabled"
                if camera_manager is not None:
                    camera_text = ", ".join(
                        f"{role}:frames={stats.frames} dropped={stats.dropped_frames} "
                        f"error={stats.last_error}"
                        for role, stats in camera_manager.stats().items()
                    )
                print(
                    f"mode={state.mode.name} q={state.position.tolist()} "
                    f"dq={state.velocity.tolist()} "
                    f"valid=p{int(state.position_valid)}/v{int(state.velocity_valid)}"
                    f"/g{int(state.gripper_valid)} "
                    f"follow={state.following_error.tolist()} "
                    f"can_age_ms={state.feedback_age_ms.tolist()} "
                    f"can_loss={state.feedback_loss_count.tolist()} "
                    f"can_status=0x{state.can_transport_status:02x} "
                    f"sweep={state.coherent_sweep_id} "
                    f"sweep_skew_us={state.feedback_max_skew_us} "
                    f"post_feedback={state.last_post_command_feedback_sequence} "
                    f"hold=0x{state.hold_reason_bits:04x} "
                    f"fault=0x{state.fault_bits:04x} "
                    f"node_fault={[hex(int(v)) for v in state.node_fault_bits]} "
                    f"can_exact={state.last_can_queued_exact_sequence} "
                    f"state_age_ms={(time.monotonic_ns() - state.monotonic_ns) / 1e6:.1f} "
                    f"camera=({camera_text})"
                )
                transport = robot.health().details.get("transport", {})
                if transport:
                    print(
                        "serial_transport="
                        + json.dumps(transport, sort_keys=True, separators=(",", ":"))
                    )
                time.sleep(args.interval)
            print(
                "can_diagnostics_end="
                + json.dumps(
                    asdict(robot.read_can_diagnostics()),
                    sort_keys=True,
                    separators=(",", ":"),
                )
            )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
