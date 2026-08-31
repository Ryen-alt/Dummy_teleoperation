from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from dummy_host.can_a9 import evaluate_can_a9
from dummy_host.protocol import CAN_TIMING_PROFILE_WINDOW_ACTIVE, CanTimingProfile
from dummy_host.robot_driver import DummyRobot
from dummy_host.schema import load_robot_config
from dummy_host.transport_serial import SerialTransport


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read and evaluate the firmware-internal CAN A9 timing profile"
    )
    parser.add_argument("--config", help="required with --port")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port")
    source.add_argument(
        "--events",
        type=Path,
        help="teleop session events.jsonl containing active timing snapshots",
    )
    parser.add_argument("--baudrate", type=int, default=115_200)
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="print an incomplete profile without returning status 2",
    )
    args = parser.parse_args()

    if args.events is not None:
        latest: dict[str, object] | None = None
        latest_active: dict[str, object] | None = None
        for line in args.events.read_text(encoding="utf-8").splitlines():
            record = json.loads(line)
            if record.get("event") == "can_timing_profile":
                payload = record.get("payload")
                if not isinstance(payload, dict):
                    parser.error("can_timing_profile event payload is not an object")
                latest = payload
                if int(payload.get("window_flags", 0)) & CAN_TIMING_PROFILE_WINDOW_ACTIVE:
                    latest_active = payload
        if latest is None:
            parser.error("events file contains no can_timing_profile record")
        latest = latest_active if latest_active is not None else latest
        converted = {
            key: tuple(value) if isinstance(value, list) else value
            for key, value in latest.items()
        }
        profile = CanTimingProfile(**converted)
    else:
        if args.config is None:
            parser.error("--config is required with --port")
        config = load_robot_config(args.config)
        robot = DummyRobot(config, SerialTransport(args.port, args.baudrate))
        with robot:
            profile = robot.read_can_timing_profile()
    evaluation = evaluate_can_a9(profile)
    print(json.dumps(asdict(evaluation), sort_keys=True, indent=2))
    if not evaluation.passed and not args.allow_incomplete:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
