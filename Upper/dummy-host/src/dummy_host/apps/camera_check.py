from __future__ import annotations

import argparse
import logging
import time

from dummy_host.cameras import D435Camera
from dummy_host.schema import load_robot_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Check the single wrist D435 stream")
    parser.add_argument("--config", required=True)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(level=args.log_level.upper())

    config = load_robot_config(args.config)
    camera = D435Camera(config.cameras["wrist"])
    camera.start()
    deadline = time.monotonic() + args.seconds
    last_number = None
    try:
        while time.monotonic() < deadline:
            frame = camera.latest()
            if frame.frame_number != last_number:
                print(
                    f"frame={frame.frame_number} color={frame.color.shape}/{frame.color.dtype} "
                    f"depth={frame.depth.shape}/{frame.depth.dtype} "
                    f"age_ms={(time.monotonic_ns() - frame.capture_time_ns) / 1e6:.1f}"
                )
                last_number = frame.frame_number
            time.sleep(0.1)
    finally:
        camera.stop()
    print(camera.stats())


if __name__ == "__main__":
    main()

