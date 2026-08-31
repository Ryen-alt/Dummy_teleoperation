#!/usr/bin/env python3
"""Fail when CAN-critical sources diverge between the 42/35 motor trees."""

from __future__ import annotations

import argparse
from pathlib import Path


CAN_CRITICAL_FILES = (
    "Core/Inc/can.h",
    "Core/Src/can.c",
    "Ctrl/Sensor/Encoder/mt6816_base.h",
    "Port/Platform/Utils/st_hardware.h",
    "Port/Platform/retarget.c",
    "UserApp/common_inc.h",
    "UserApp/protocols/interface_can.cpp",
    "UserApp/protocols/timing_profiler.cpp",
    "UserApp/protocols/timing_profiler.h",
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Check that CAN-critical 42/35 motor firmware files match"
    )
    parser.add_argument(
        "--firmware-root",
        type=Path,
        default=Path(__file__).resolve().parent,
    )
    args = parser.parse_args()
    root = args.firmware_root.resolve()
    motor_42 = root / "dummy-42motor-fw"
    motor_35 = root / "dummy-35motor-fw"

    mismatches: list[str] = []
    for relative in CAN_CRITICAL_FILES:
        source_42 = motor_42 / relative
        source_35 = motor_35 / relative
        if not source_42.is_file() or not source_35.is_file():
            mismatches.append(f"missing: {relative}")
        elif source_42.read_bytes() != source_35.read_bytes():
            mismatches.append(f"different: {relative}")

    if mismatches:
        parser.error("motor firmware parity failure: " + "; ".join(mismatches))
    print(f"motor firmware parity OK ({len(CAN_CRITICAL_FILES)} files)")


if __name__ == "__main__":
    main()
