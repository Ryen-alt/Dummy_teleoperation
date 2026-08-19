from __future__ import annotations

from pathlib import Path

import yaml

from dummy_host.schema import ConfigError, load_camera_rig_config, load_robot_config


def test_config_is_deterministic_and_calibrated(config) -> None:
    again = load_robot_config(Path(__file__).parents[1] / "configs" / "robot_config.yaml")
    assert config.config_hash == again.config_hash
    assert len(config.config_hash) == 64
    assert tuple(config.cameras) == ("wrist",)
    assert config.cameras["wrist"].model == "D435"
    assert config.config_version == 3
    assert config.robot_calibration_id == "dummy_v2_001-arm-gripper-20260811-v1"
    assert config.hardware_parameters_verified
    assert config.external_target_execution_ready


def test_wrong_joint_order_is_rejected(tmp_path, config) -> None:
    source = Path(__file__).parents[1] / "configs" / "robot_config.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["joint_order"][0], raw["joint_order"][1] = raw["joint_order"][1], raw["joint_order"][0]
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(raw), encoding="utf-8")
    try:
        load_robot_config(bad)
    except ConfigError as exc:
        assert "joint_order" in str(exc)
    else:
        raise AssertionError("bad joint order was accepted")


def test_execution_ready_requires_verified_hardware(tmp_path) -> None:
    source = Path(__file__).parents[1] / "configs" / "robot_config.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["hardware_parameters_verified"] = False
    raw["external_target_execution_ready"] = True
    bad = tmp_path / "bad-gate.yaml"
    bad.write_text(yaml.safe_dump(raw), encoding="utf-8")
    try:
        load_robot_config(bad)
    except ConfigError as exc:
        assert "requires hardware_parameters_verified" in str(exc)
    else:
        raise AssertionError("unsafe external execution gate was accepted")


def test_camera_rig_has_independent_hash(tmp_path) -> None:
    source = Path(__file__).parents[1] / "configs" / "robot_config.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    changed = yaml.safe_load(source.read_text(encoding="utf-8"))
    changed["cameras"]["wrist"]["color_exposure"] = 123.0
    path = tmp_path / "camera-change.yaml"
    path.write_text(yaml.safe_dump(changed), encoding="utf-8")
    original_config = load_robot_config(source)
    changed_config = load_robot_config(path)
    assert changed_config.config_hash == original_config.config_hash
    assert changed_config.camera_rig.config_hash != original_config.camera_rig.config_hash
    assert raw["robot_calibration_id"] == changed["robot_calibration_id"]


def test_external_camera_rig_overrides_cameras_without_changing_robot_hash() -> None:
    root = Path(__file__).parents[1]
    embedded = load_robot_config(root / "configs" / "robot_config.yaml")
    rig_path = root / "configs" / "camera_rig_dual.example.yaml"
    rig = load_camera_rig_config(rig_path)
    overridden = load_robot_config(
        root / "configs" / "robot_config.yaml",
        camera_rig_path=rig_path,
    )
    assert tuple(rig.cameras) == ("wrist", "global")
    assert overridden.camera_rig == rig
    assert overridden.config_hash == embedded.config_hash
    assert overridden.camera_rig.config_hash != embedded.camera_rig.config_hash

