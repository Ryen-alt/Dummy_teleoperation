from __future__ import annotations

from pathlib import Path

import yaml
import pytest

from dummy_host.schema import (
    ConfigError,
    load_camera_calibration,
    load_camera_rig_config,
    load_robot_config,
    validate_camera_rig_for_formal_collection,
)


def test_config_is_deterministic_and_calibrated(config) -> None:
    again = load_robot_config(Path(__file__).parents[1] / "configs" / "robot_config.yaml")
    assert config.config_hash == again.config_hash
    assert len(config.config_hash) == 64
    assert tuple(config.cameras) == ("wrist",)
    assert config.cameras["wrist"].model == "D435"
    assert config.config_version == 9
    assert config.can_scheduler_watchdog_hz == 1000
    assert config.can_target_hz_per_node == 50
    assert config.can_position_hz_per_node == 40
    assert config.can_temperature_hz_per_node == 1
    assert config.coherent_max_skew_ms == 30
    assert config.feedback_fault_ms > config.feedback_hold_ms
    assert config.gripper_velocity_limit_per_s == 0.2
    assert config.robot_calibration_id == "dummy_v2_001-arm-gripper-20260821-v2"
    assert config.joint_reduction.tolist() == [50.0] * 6
    assert config.hardware_parameters_verified
    assert not config.external_target_execution_ready
    assert config.external_target_acceptance_ready
    assert config.can_node_quiet_us == 5_000
    assert config.can_response_timeout_us == 4_000
    assert config.can_tx_abort_timeout_us == 5_000
    assert config.can_target_fanout_timeout_us == 15_000


def test_can_scheduler_watchdog_rate_outside_reviewed_range_is_rejected(tmp_path) -> None:
    source = Path(__file__).parents[1] / "configs" / "robot_config.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["can_scheduler_watchdog_hz"] = 5001
    bad = tmp_path / "bad-feedback-rate.yaml"
    bad.write_text(yaml.safe_dump(raw), encoding="utf-8")
    try:
        load_robot_config(bad)
    except ConfigError as exc:
        assert "can_scheduler_watchdog_hz" in str(exc)
    else:
        raise AssertionError("unsafe feedback polling rate was accepted")


def test_event_driven_can_rates_are_not_limited_by_watchdog_slots(tmp_path) -> None:
    source = Path(__file__).parents[1] / "configs" / "robot_config.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["can_scheduler_watchdog_hz"] = 1000
    raw["can_target_hz_per_node"] = 60
    bad = tmp_path / "uneven-feedback-rate.yaml"
    bad.write_text(yaml.safe_dump(raw), encoding="utf-8")
    config = load_robot_config(bad)
    assert config.can_target_hz_per_node == 60


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
    raw["external_target_acceptance_ready"] = False
    bad = tmp_path / "bad-gate.yaml"
    bad.write_text(yaml.safe_dump(raw), encoding="utf-8")
    try:
        load_robot_config(bad)
    except ConfigError as exc:
        assert "requires hardware_parameters_verified" in str(exc)
    else:
        raise AssertionError("unsafe external execution gate was accepted")


def test_acceptance_ready_requires_verified_hardware(tmp_path) -> None:
    source = Path(__file__).parents[1] / "configs" / "robot_config.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["hardware_parameters_verified"] = False
    bad = tmp_path / "bad-acceptance-gate.yaml"
    bad.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ConfigError, match="external_target_acceptance_ready requires"):
        load_robot_config(bad)


def test_production_and_acceptance_gates_are_mutually_exclusive(tmp_path) -> None:
    source = Path(__file__).parents[1] / "configs" / "robot_config.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["external_target_execution_ready"] = True
    bad = tmp_path / "ambiguous-gates.yaml"
    bad.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ConfigError, match="must be false"):
        load_robot_config(bad)


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
    assert set(rig.calibrations) == {"wrist", "global"}
    assert rig.cameras["wrist"].calibration_hash == rig.calibrations["wrist"].file_hash


def test_versioned_camera_calibration_loads_and_matches_rig() -> None:
    root = Path(__file__).parents[1]
    calibration = load_camera_calibration(
        root / "configs" / "calibrations" / "wrist.example.yaml"
    )
    assert calibration.schema_version == 1
    assert calibration.intrinsic_matrix.shape == (3, 3)
    assert calibration.rotation_xyzw.tolist() == [0.0, 0.0, 0.0, 1.0]
    assert len(calibration.file_hash) == 64


def test_camera_calibration_identity_mismatch_is_rejected(tmp_path) -> None:
    root = Path(__file__).parents[1]
    raw = yaml.safe_load(
        (root / "configs" / "camera_rig_dual.example.yaml").read_text(encoding="utf-8")
    )
    raw["cameras"]["wrist"]["device_serial"] = "wrong-device"
    raw["cameras"]["wrist"]["calibration_file"] = str(
        root / "configs" / "calibrations" / "wrist.example.yaml"
    )
    raw["cameras"]["global"]["calibration_file"] = str(
        root / "configs" / "calibrations" / "global.example.yaml"
    )
    path = tmp_path / "bad-rig.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    try:
        load_camera_rig_config(path)
    except ConfigError as exc:
        assert "device_serial" in str(exc)
    else:
        raise AssertionError("mismatched calibration identity was accepted")


def test_formal_collection_rejects_uncalibrated_embedded_rig(config) -> None:
    try:
        validate_camera_rig_for_formal_collection(config.camera_rig)
    except ConfigError as exc:
        assert "uncalibrated" in str(exc)
        assert "calibration_file" in str(exc)
        assert "fixed color_exposure" in str(exc)
    else:
        raise AssertionError("uncalibrated smoke-test rig was accepted for formal collection")
