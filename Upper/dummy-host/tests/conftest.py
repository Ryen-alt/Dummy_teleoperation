from __future__ import annotations

from pathlib import Path

import pytest

from dummy_host.schema import RobotConfig, load_robot_config


@pytest.fixture
def config() -> RobotConfig:
    return load_robot_config(Path(__file__).parents[1] / "configs" / "robot_config.yaml")


@pytest.fixture
def config_path() -> Path:
    return Path(__file__).parents[1] / "configs" / "robot_config.yaml"

