from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from dummy_host.config_codegen import (
    ConfigGenerationError,
    generate_firmware_header,
    render_firmware_header,
)
from dummy_host.schema import RobotConfig


def test_render_firmware_header_contains_safety_critical_values(config: RobotConfig) -> None:
    header = render_firmware_header(config)
    assert f'constexpr char kRobotId[] = "{config.robot_id}";' in header
    assert f'constexpr char kRobotCalibrationId[] = "{config.robot_calibration_id}";' in header
    assert f"constexpr uint32_t kConfigVersion = {config.config_version}U;" in header
    assert "constexpr bool kHardwareParametersVerified = true;" in header
    assert "constexpr bool kExternalTargetExecutionReady = true;" in header
    assert "kJointZeroOffsetRad" in header
    assert "kJointSign" in header
    assert "kJointReduction" in header
    assert "kMaxAccelerationRadS2" in header
    expected_hash_prefix = ", ".join(f"0x{value:02x}" for value in config.config_hash_bytes[:4])
    assert expected_hash_prefix in header


def test_urdf_coordinate_mapping_contract(config: RobotConfig) -> None:
    firmware_rest_rad = np.deg2rad(np.asarray([0.0, -70.0, 180.0, 0.0, 0.0, 0.0]))
    urdf_rest_rad = config.joint_sign * (firmware_rest_rad - config.joint_zero_offset_rad)

    assert config.config_version == 3
    np.testing.assert_allclose(urdf_rest_rad, config.initial_pose_rad, atol=1e-6)
    np.testing.assert_allclose(
        config.initial_pose_rad,
        np.asarray([0.0, 0.052359878, 0.0, 0.0, 0.0, 0.0]),
        atol=1e-9,
    )
    np.testing.assert_allclose(
        config.joint_limit_min_rad,
        np.asarray(
            [-2.967059728, 0.052359878, -2.617993878, -3.141592654, -1.570796327, -3.490658504]
        ),
        atol=1e-6,
    )
    np.testing.assert_allclose(
        config.joint_limit_max_rad,
        np.asarray(
            [2.967059728, 3.019419606, 0.0, 3.141592654, 1.570796327, 6.283185307]
        ),
        atol=1e-6,
    )


def test_confirmed_legacy_limits_and_urdf_are_synchronized(config: RobotConfig) -> None:
    firmware_endpoints_rad = [
        config.joint_zero_offset_rad + config.joint_sign * config.joint_limit_min_rad,
        config.joint_zero_offset_rad + config.joint_sign * config.joint_limit_max_rad,
    ]
    firmware_min_deg = np.rad2deg(np.minimum(*firmware_endpoints_rad))
    firmware_max_deg = np.rad2deg(np.maximum(*firmware_endpoints_rad))
    np.testing.assert_allclose(
        firmware_min_deg,
        np.asarray([-170.0, -70.0, 30.0, -180.0, -90.0, -360.0]),
        atol=5e-5,
    )
    np.testing.assert_allclose(
        firmware_max_deg,
        np.asarray([170.0, 100.0, 180.0, 180.0, 90.0, 200.0]),
        atol=5e-5,
    )

    urdf_path = Path(__file__).resolve().parents[3] / "Dummy_URDF" / "dummy.urdf"
    root = ET.parse(urdf_path).getroot()
    urdf_min = []
    urdf_max = []
    for joint_number in range(1, 7):
        joint = root.find(f"./joint[@name='joint_{joint_number}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        urdf_min.append(float(limit.attrib["lower"]))
        urdf_max.append(float(limit.attrib["upper"]))
    np.testing.assert_allclose(urdf_min, config.joint_limit_min_rad, atol=5e-7)
    np.testing.assert_allclose(urdf_max, config.joint_limit_max_rad, atol=5e-7)


def test_generate_and_check_detects_stale_header(
    config_path: Path,
    tmp_path: Path,
) -> None:
    output = tmp_path / "robot_config_generated.hpp"
    assert generate_firmware_header(config_path, output)
    assert not generate_firmware_header(config_path, output, check=True)
    output.write_text("stale\n", encoding="utf-8")
    with pytest.raises(ConfigGenerationError, match="is stale"):
        generate_firmware_header(config_path, output, check=True)
