from __future__ import annotations

import argparse
import json
import sqlite3
from dataclasses import asdict
from pathlib import Path

from dummy_host.can_a9 import (
    evaluate_can_a9, load_can_timing_profile_events, load_acceptance_timing_profile,
)
from dummy_host.acceptance_window import load_acceptance_window
from dummy_host.apps.session_check import check_session
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

    window = None
    if args.events is not None:
        try:
            manifest_path = args.events.parent / "manifest.json"
            if manifest_path.exists():
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                window = load_acceptance_window(args.events.parent, manifest)
            if window is None:
                profile = load_can_timing_profile_events(args.events)
            else:
                integrity = check_session(args.events.parent)
                if integrity.errors or not integrity.clean_shutdown:
                    raise ValueError("acceptance A9 requires a complete, intact session")
                with sqlite3.connect(f"file:{(args.events.parent / 'samples.sqlite').as_posix()}?mode=ro", uri=True) as db:
                    row = db.execute("SELECT window_start_us FROM can_diagnostics WHERE host_time_ns = ?",
                                     (window.diagnostic_start_ns,)).fetchone()
                if row is None:
                    raise ValueError("acceptance A9 is missing its diagnostic baseline")
                profile = load_acceptance_timing_profile(args.events, window,
                                                        diagnostic_start_us=int(row[0]))
        except (OSError, TypeError, ValueError) as exc:
            parser.error(str(exc))
    else:
        if args.config is None:
            parser.error("--config is required with --port")
        config = load_robot_config(args.config)
        robot = DummyRobot(config, SerialTransport(args.port, args.baudrate))
        with robot:
            profile = robot.read_can_timing_profile()
    evaluation = evaluate_can_a9(profile)
    rendered = asdict(evaluation)
    if window is not None:
        rendered["acceptance_window"] = asdict(window)
        rendered["histogram_scope"] = "cumulative from the same firmware window, including startup"
    print(json.dumps(rendered, sort_keys=True, indent=2))
    if not evaluation.passed and not args.allow_incomplete:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
