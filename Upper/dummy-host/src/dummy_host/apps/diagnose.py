from __future__ import annotations

import argparse
import logging
import time

from dummy_host.cameras import D435Camera
from dummy_host.robot_driver import DummyRobot
from dummy_host.schema import load_robot_config
from dummy_host.transport_serial import SerialTransport


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only Dummy robot and D435 diagnostics")
    parser.add_argument("--config", required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115_200)
    parser.add_argument("--without-camera", action="store_true")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(level=args.log_level.upper())

    config = load_robot_config(args.config)
    camera = None if args.without_camera else D435Camera(config.cameras["wrist"])
    robot = DummyRobot(config, SerialTransport(args.port, args.baudrate), camera=camera)
    print(f"host_config_hash={config.config_hash}")
    try:
        with robot:
            print(f"firmware_version={robot.firmware_version}")
            while True:
                state = robot.read_state()
                camera_text = "disabled"
                if camera is not None:
                    stats = camera.stats()
                    camera_text = f"frames={stats.frames} dropped={stats.dropped_frames} error={stats.last_error}"
                print(
                    f"mode={state.mode.name} q={state.position.tolist()} "
                    f"fault=0x{state.fault_bits:04x} applied={state.last_applied_sequence} "
                    f"state_age_ms={(time.monotonic_ns() - state.monotonic_ns) / 1e6:.1f} "
                    f"camera=({camera_text})"
                )
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

