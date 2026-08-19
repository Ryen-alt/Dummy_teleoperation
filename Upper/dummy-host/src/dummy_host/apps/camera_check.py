from __future__ import annotations

import argparse
import json
import logging
import time
from dataclasses import asdict

from dummy_host.cameras import CameraManager
from dummy_host.schema import load_robot_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Check every enabled camera in a configured rig")
    parser.add_argument("--config", required=True)
    parser.add_argument("--camera-rig", help="optional independently versioned camera-rig YAML")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--log-level", default="INFO")
    parser.add_argument("--json-output")
    args = parser.parse_args()
    logging.basicConfig(level=args.log_level.upper())

    config = load_robot_config(args.config, camera_rig_path=args.camera_rig)
    cameras = CameraManager.from_config(config.camera_rig)
    cameras.start()
    deadline = time.monotonic() + args.seconds
    last_numbers: dict[str, int] = {}
    try:
        while time.monotonic() < deadline:
            for role, frame in cameras.latest_all().items():
                if frame.frame_number == last_numbers.get(role):
                    continue
                print(
                    f"role={role} frame={frame.frame_number} "
                    f"color={frame.color.shape}/{frame.color.dtype} "
                    f"depth={None if frame.depth is None else frame.depth.shape} "
                    f"age_ms={(time.monotonic_ns() - frame.capture_time_ns) / 1e6:.1f} "
                    f"color_depth_skew_ms={frame.color_depth_skew_ms:.3f}"
                )
                last_numbers[role] = frame.frame_number
            time.sleep(0.1)
    finally:
        cameras.stop()
    stats = {role: asdict(value) for role, value in cameras.stats().items()}
    rendered = json.dumps(stats, indent=2, ensure_ascii=False)
    print(rendered)
    if args.json_output:
        with open(args.json_output, "w", encoding="utf-8") as output:
            output.write(rendered + "\n")


if __name__ == "__main__":
    main()

