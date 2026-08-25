from __future__ import annotations

import argparse
import json
from collections.abc import Mapping
from dataclasses import asdict
from pathlib import Path

import yaml

from dummy_host.dataset import ExportRecipe, export_raw_session

from .sink import LeRobotV3DatasetSink


def _load_recipe(path: str | Path) -> ExportRecipe:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("export recipe root must be a mapping")
    roles = raw.get("required_camera_roles")
    outcomes = raw.get("accepted_outcomes", ["accepted"])
    metadata = raw.get("metadata", {})
    if not isinstance(roles, list) or not isinstance(outcomes, list):
        raise ValueError("required_camera_roles and accepted_outcomes must be YAML lists")
    if not isinstance(metadata, Mapping):
        raise ValueError("recipe metadata must be a mapping")
    include_depth = raw.get("include_depth", False)
    allow_uncalibrated = raw.get("allow_uncalibrated_cameras", False)
    require_temporary = raw.get("require_temporary_source", False)
    if not all(
        isinstance(value, bool)
        for value in (include_depth, allow_uncalibrated, require_temporary)
    ):
        raise ValueError("recipe camera and source gates must be boolean")
    return ExportRecipe(
        recipe_id=str(raw.get("recipe_id", "")),
        version=int(raw.get("version", 0)),
        required_camera_roles=tuple(roles),
        control_hz=int(raw.get("control_hz", 0)),
        dataset_format=str(raw.get("dataset_format", "")),
        accepted_outcomes=tuple(outcomes),
        include_depth=include_depth,
        allow_uncalibrated_cameras=allow_uncalibrated,
        require_temporary_source=require_temporary,
        max_action_observation_latency_ms=float(
            raw.get("max_action_observation_latency_ms", 250.0)
        ),
        metadata=metadata,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a verified Raw Session v3 to LeRobotDataset v3")
    parser.add_argument("--session", required=True)
    parser.add_argument("--recipe", required=True)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--robot-type", default="dummy")
    parser.add_argument("--images", action="store_true", help="store PNG images instead of MP4 videos")
    parser.add_argument("--batch-encoding-size", type=int, default=1)
    args = parser.parse_args()

    recipe = _load_recipe(args.recipe)
    sink = LeRobotV3DatasetSink(
        repo_id=args.repo_id,
        root=args.output,
        fps=recipe.control_hz,
        robot_type=args.robot_type,
        use_videos=not args.images,
        batch_encoding_size=args.batch_encoding_size,
    )
    report = export_raw_session(args.session, recipe, sink)
    print(json.dumps(asdict(report), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
