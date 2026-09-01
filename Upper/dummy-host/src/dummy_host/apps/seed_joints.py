from __future__ import annotations

import argparse
import json

import numpy as np

from dummy_host.robot_driver import DummyRobot
from dummy_host.schema import load_robot_config
from dummy_host.transport_serial import SerialTransport


STATIONARY_CONFIRMATION = "I_CONFIRM_ROBOT_DID_NOT_MOVE_SINCE_REFERENCE"


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Select post-reboot arm encoder branches from a known stationary "
            "URDF pose; this operation does not enable or move any motor"
        )
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115_200)
    parser.add_argument(
        "--position-rad",
        type=float,
        nargs=6,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        required=True,
        help="known physical arm pose in URDF radians",
    )
    parser.add_argument(
        "--confirm-stationary-reference",
        required=True,
        help=f"must equal {STATIONARY_CONFIRMATION}",
    )
    args = parser.parse_args()
    if args.confirm_stationary_reference != STATIONARY_CONFIRMATION:
        parser.error(
            "--confirm-stationary-reference does not contain the required "
            "stationary-pose acknowledgement"
        )

    config = load_robot_config(args.config)
    reference = np.asarray(args.position_rad, dtype=np.float32)
    robot = DummyRobot(
        config,
        SerialTransport(args.port, args.baudrate),
    )
    with robot:
        before = robot.read_state()
        print(
            "before="
            + json.dumps(
                {
                    "mode": before.mode.name,
                    "position_valid": before.position_valid,
                    "position_rad": before.position.tolist(),
                    "fault_bits": before.fault_bits,
                },
                separators=(",", ":"),
            )
        )
        after = robot.seed_absolute_joint_position(
            reference,
            stationary_reference_confirmed=True,
        )
        print(
            "after="
            + json.dumps(
                {
                    "mode": after.mode.name,
                    "position_valid": after.position_valid,
                    "position_rad": after.position.tolist(),
                    "fault_bits": after.fault_bits,
                    "coherent_sweep_id": after.coherent_sweep_id,
                },
                separators=(",", ":"),
            )
        )


if __name__ == "__main__":
    main()
