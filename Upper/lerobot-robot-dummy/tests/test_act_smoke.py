from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from lerobot_robot_dummy.act_smoke import (
    ActSmokeError,
    TEMP_CLASSIFICATION,
    audit_local_dataset,
    build_train_command,
)
from lerobot_robot_dummy.cli import _load_recipe
from lerobot_robot_dummy.temp_fixture import create_temp_raw_session


class FakeDataset:
    features = {
        "observation.state": {"dtype": "float32", "shape": (7,)},
        "action": {"dtype": "float32", "shape": (7,)},
        "observation.images.wrist": {"dtype": "image", "shape": (48, 64, 3)},
        "observation.images.global": {"dtype": "image", "shape": (48, 64, 3)},
    }
    meta = SimpleNamespace(total_episodes=1, fps=20)

    def __init__(self, repo_id: str, *, root: Path) -> None:
        self.repo_id = repo_id
        self.root = root

    def __len__(self) -> int:
        return 1

    def __getitem__(self, index: int) -> dict[str, np.ndarray]:
        assert index == 0
        return {
            "observation.state": np.zeros(7, dtype=np.float32),
            "action": np.ones(7, dtype=np.float32),
        }


def _write_sidecar(root: Path, *, classification: str = TEMP_CLASSIFICATION) -> None:
    root.mkdir()
    (root / "dummy_export_metadata.json").write_text(
        json.dumps(
            {
                "source": {
                    "data_classification": classification,
                    "offline_training_only": True,
                    "real_policy_execution_allowed": False,
                }
            }
        ),
        encoding="utf-8",
    )


def test_audit_accepts_explicit_temp_dataset_and_builds_compact_act_command(
    tmp_path: Path,
) -> None:
    dataset_root = tmp_path / "dataset"
    _write_sidecar(dataset_root)
    audit = audit_local_dataset(
        "local/dummy-temp",
        dataset_root,
        dataset_factory=FakeDataset,
    )
    assert audit.frames == 1
    assert audit.camera_keys == (
        "observation.images.global",
        "observation.images.wrist",
    )
    assert audit.offline_training_only
    assert not audit.real_policy_execution_allowed

    command = build_train_command(
        repo_id="local/dummy-temp",
        dataset_root=dataset_root,
        output_dir=tmp_path / "act",
        device="cpu",
        steps=2,
        batch_size=1,
        num_workers=0,
        compact_model=True,
        executable="/usr/bin/true",
    )
    assert command[0] == "/usr/bin/true"
    assert "--policy.type=act" in command
    assert "--policy.push_to_hub=false" in command
    assert "--policy.pretrained_backbone_weights=null" in command
    assert "--policy.dim_model=128" in command
    assert "--steps=2" in command


def test_audit_rejects_dataset_without_temp_provenance(tmp_path: Path) -> None:
    dataset_root = tmp_path / "dataset"
    _write_sidecar(dataset_root, classification="engineering")
    with pytest.raises(ActSmokeError, match="TEMP smoke"):
        audit_local_dataset(
            "local/dummy-temp",
            dataset_root,
            dataset_factory=FakeDataset,
        )


def test_temp_recipe_enables_both_uncalibrated_source_gates() -> None:
    recipe = _load_recipe(
        Path(__file__).parents[1] / "configs" / "export_recipe.temp_uncalibrated.yaml"
    )
    assert recipe.allow_uncalibrated_cameras
    assert recipe.require_temporary_source
    assert recipe.required_camera_roles == ("wrist", "global")


def test_temp_fixture_writes_a_complete_dual_camera_raw_session(tmp_path: Path) -> None:
    upper = Path(__file__).parents[2]
    session = create_temp_raw_session(
        config_path=upper / "dummy-host" / "configs" / "robot_config.yaml",
        input_config_path=upper / "dummy-host" / "configs" / "teleop_inputs.yaml",
        output_root=tmp_path,
        session_name="fixture",
        episodes=1,
        frames_per_episode=8,
        image_height=32,
        image_width=32,
    )
    manifest = json.loads((session / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["clean_shutdown"] is True
    assert manifest["firmware_version"] == "dummy-ref-v1.6-fixture-not-hardware"
    assert manifest["stats"]["samples"] == 8
    assert manifest["stats"]["camera_frames"] == 16
    assert manifest["extra"]["data_classification"] == TEMP_CLASSIFICATION
    assert manifest["extra"]["real_policy_execution_allowed"] is False
