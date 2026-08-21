from __future__ import annotations

import argparse
import json
import logging
import time
from dataclasses import asdict

from dummy_host.cameras import CameraError, CameraFrame, CameraManager
from dummy_host.schema import load_robot_config


def wait_for_first_frames(
    cameras: CameraManager,
    *,
    timeout_s: float,
    poll_interval_s: float = 0.05,
) -> dict[str, CameraFrame]:
    """Wait for required cameras to publish instead of racing their capture threads."""
    if timeout_s <= 0:
        raise ValueError("camera startup timeout must be positive")
    if poll_interval_s <= 0:
        raise ValueError("camera startup poll interval must be positive")

    deadline = time.monotonic() + timeout_s
    last_error: CameraError | None = None
    while True:
        try:
            return cameras.latest_all()
        except CameraError as exc:
            last_error = exc

        capture_errors = {
            role: stats.last_error
            for role, stats in cameras.stats().items()
            if stats.last_error is not None
        }
        if capture_errors:
            details = "; ".join(
                f"{role}: {error}" for role, error in capture_errors.items()
            )
            raise CameraError(
                f"camera capture stopped while waiting for the first frame: {details}"
            ) from last_error

        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0:
            raise CameraError(
                f"timed out after {timeout_s:.1f}s waiting for the first camera frame; "
                f"last status: {last_error}"
            ) from last_error
        time.sleep(min(poll_interval_s, remaining_s))


def main() -> None:
    parser = argparse.ArgumentParser(description="Check every enabled camera in a configured rig")
    parser.add_argument("--config", required=True)
    parser.add_argument("--camera-rig", help="optional independently versioned camera-rig YAML")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=5.0,
        help="seconds to wait for required cameras to publish their first frame (default: 5)",
    )
    parser.add_argument("--log-level", default="INFO")
    parser.add_argument("--json-output")
    args = parser.parse_args()
    logging.basicConfig(level=args.log_level.upper())

    config = load_robot_config(args.config, camera_rig_path=args.camera_rig)
    cameras = CameraManager.from_config(config.camera_rig)
    cameras.start()
    last_numbers: dict[str, int] = {}
    try:
        frames = wait_for_first_frames(cameras, timeout_s=args.startup_timeout)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            for role, frame in frames.items():
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
            frames = cameras.latest_all()
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
