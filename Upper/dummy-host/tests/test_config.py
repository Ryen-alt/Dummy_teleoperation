from __future__ import annotations

from pathlib import Path

import yaml

from dummy_host.schema import ConfigError, load_robot_config


def test_config_is_deterministic_and_single_d435(config) -> None:
    again = load_robot_config(Path(__file__).parents[1] / "configs" / "robot_config.yaml")
    assert config.config_hash == again.config_hash
    assert len(config.config_hash) == 64
    assert tuple(config.cameras) == ("wrist",)
    assert config.cameras["wrist"].model == "D435"
    assert not config.hardware_parameters_verified


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

