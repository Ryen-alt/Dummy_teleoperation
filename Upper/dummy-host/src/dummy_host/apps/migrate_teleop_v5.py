from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from dummy_host.teleop import (
    IK_DAMPING_MAX,
    IK_DAMPING_MIN,
    IK_SIGMA_HARD,
    IK_SIGMA_WARN,
    TeleopConfigError,
    load_teleop_profile,
)


def migrate_v4_document(raw: object) -> dict[str, object]:
    if not isinstance(raw, dict) or raw.get("version") != 4:
        raise TeleopConfigError("migration input must be a teleop schema v4 mapping")
    migrated = dict(raw)
    cartesian_raw = migrated.get("cartesian")
    if cartesian_raw is not None:
        if not isinstance(cartesian_raw, dict):
            raise TeleopConfigError("cartesian must be a mapping")
        cartesian = dict(cartesian_raw)
        solver_raw = cartesian.get("solver")
        if not isinstance(solver_raw, dict):
            raise TeleopConfigError("cartesian.solver must be a mapping")
        solver = dict(solver_raw)
        solver.pop("damping", None)
        solver.pop("finite_difference_rad", None)
        solver.update(
            {
                "sigma_warn": IK_SIGMA_WARN,
                "sigma_hard": IK_SIGMA_HARD,
                "damping_min": IK_DAMPING_MIN,
                "damping_max": IK_DAMPING_MAX,
                "task_trust_region": 0.25,
                "soft_limit_zone_rad": 0.08,
            }
        )
        cartesian["solver"] = solver
        migrated["cartesian"] = cartesian
    migrated["version"] = 5
    return migrated


def migrate_file(input_path: str | Path, output_path: str | Path) -> Path:
    source = Path(input_path)
    destination = Path(output_path)
    if source.resolve() == destination.resolve():
        raise TeleopConfigError("migration output must differ from the v4 source")
    if destination.exists():
        raise TeleopConfigError(f"refusing to overwrite existing migration output {destination}")
    try:
        raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise TeleopConfigError(f"cannot load {source}: {exc}") from exc
    migrated = migrate_v4_document(raw)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        yaml.safe_dump(migrated, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    # Parse the artifact through the normal strict loader before reporting it.
    load_teleop_profile(destination)
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Explicitly migrate a teleop input profile from schema v4 to v5"
    )
    parser.add_argument("input")
    parser.add_argument("output")
    args = parser.parse_args()
    try:
        output = migrate_file(args.input, args.output)
    except TeleopConfigError as exc:
        parser.error(str(exc))
    print(output)


if __name__ == "__main__":
    main()
