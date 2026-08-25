from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from dummy_host.kinematics import (
    DummyUrdfKinematics,
    KinematicsError,
    load_cartesian_calibration,
)
from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.recording import SessionRecorder
from dummy_host.apps.session_check import check_session
from dummy_host.robot_driver import DummyRobot
from dummy_host.schema import RobotConfig
from dummy_host.teleop import TeleopCommand, TeleopError, load_teleop_profile
from dummy_host.teleop_runtime import run_teleop_collection


PROJECT = Path(__file__).parents[1]
URDF = PROJECT.parents[1] / "Dummy_URDF" / "dummy.urdf"


def _profile():
    return load_teleop_profile(PROJECT / "configs" / "teleop_inputs.yaml")


def _valid_document(config: RobotConfig) -> dict[str, object]:
    ready = (config.joint_limit_min_rad + config.joint_limit_max_rad) / 2.0
    return {
        "version": 1,
        "calibration_id": "cartesian-site-acceptance-001",
        "robot_id": config.robot_id,
        "validated": True,
        "validated_utc": "2026-08-25T10:00:00+08:00",
        "urdf_sha256": hashlib.sha256(URDF.read_bytes()).hexdigest(),
        "ready_pose_rad": ready.tolist(),
        "ready_tolerance_rad": [0.01] * 6,
        "tip_frame": "tcp",
        "tool0_T_tcp": {
            "translation_m": [0.02, -0.01, 0.08],
            "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "evidence_session": "raw/session_cartesian_acceptance_001",
    }


def _write_yaml(path: Path, document: dict[str, object]) -> Path:
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def _backend(config: RobotConfig, calibration=None) -> DummyUrdfKinematics:
    cartesian = _profile().cartesian
    assert cartesian is not None
    return DummyUrdfKinematics(
        URDF,
        joint_min_rad=config.joint_limit_min_rad,
        joint_max_rad=config.joint_limit_max_rad,
        joint_limit_margin_rad=cartesian.joint_limit_margin_rad,
        position_tolerance_m=cartesian.position_tolerance_m,
        orientation_tolerance_rad=cartesian.orientation_tolerance_rad,
        max_iterations=cartesian.max_iterations,
        damping=cartesian.damping,
        finite_difference_rad=cartesian.finite_difference_rad,
        max_solver_step_rad=cartesian.max_solver_step_rad,
        max_solution_step_rad=cartesian.max_solution_step_rad,
        translation_scale_m=cartesian.translation_scale_m,
        tool0_T_tip=None if calibration is None else calibration.tool0_T_tcp,
        tip_frame="tool0" if calibration is None else calibration.tip_frame,
        calibration_hash=None if calibration is None else calibration.file_hash,
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda value: value.pop("ready_pose_rad"), "ready_pose_rad"),
        (
            lambda value: value["tool0_T_tcp"].update(
                {"rotation_xyzw": [0.0, 0.0, 0.0, 2.0]}
            ),
            "unit quaternion",
        ),
        (lambda value: value.update({"validated_utc": None}), "validated_utc"),
    ],
)
def test_cartesian_calibration_rejects_missing_or_invalid_fields(
    config: RobotConfig,
    tmp_path: Path,
    mutation,
    message: str,
) -> None:
    document = _valid_document(config)
    mutation(document)
    path = _write_yaml(tmp_path / "invalid.yaml", document)
    with pytest.raises(KinematicsError, match=message):
        load_cartesian_calibration(path)


def test_cartesian_calibration_rejects_identity_and_ready_pose_mismatches(
    config: RobotConfig,
    tmp_path: Path,
) -> None:
    document = _valid_document(config)
    document["robot_id"] = "different_robot"
    calibration = load_cartesian_calibration(
        _write_yaml(tmp_path / "wrong_robot.yaml", document)
    )
    with pytest.raises(KinematicsError, match="robot_id"):
        calibration.validate_for(config, URDF, require_validated=True)

    document = _valid_document(config)
    document["urdf_sha256"] = "0" * 64
    calibration = load_cartesian_calibration(
        _write_yaml(tmp_path / "wrong_urdf.yaml", document)
    )
    with pytest.raises(KinematicsError, match="URDF hash"):
        calibration.validate_for(config, URDF, require_validated=True)

    document = _valid_document(config)
    document["ready_pose_rad"] = (config.joint_limit_max_rad + 0.1).tolist()
    calibration = load_cartesian_calibration(
        _write_yaml(tmp_path / "bad_ready.yaml", document)
    )
    with pytest.raises(KinematicsError, match="outside configured joint limits"):
        calibration.validate_for(config, URDF, require_validated=True)


def test_unvalidated_example_blocks_real_execution(config: RobotConfig) -> None:
    calibration = load_cartesian_calibration(
        PROJECT / "configs" / "cartesian_calibration.example.yaml"
    )
    calibration.validate_for(config, URDF, require_validated=False)
    assert not calibration.validated
    assert calibration.tool0_T_tcp is None
    with pytest.raises(KinematicsError, match="requires a validated calibration"):
        calibration.validate_for(config, URDF, require_validated=True)


def test_tcp_transform_participates_in_fk_ik_and_model_identity(
    config: RobotConfig,
    tmp_path: Path,
) -> None:
    calibration = load_cartesian_calibration(
        _write_yaml(tmp_path / "validated.yaml", _valid_document(config))
    )
    calibration.validate_for(config, URDF, require_validated=True)
    tool0_backend = _backend(config)
    tcp_backend = _backend(config, calibration)
    joints = np.asarray([0.32, 1.02, -1.05, 0.43, 0.24, 0.54])
    tool0_pose = tool0_backend.forward(joints)
    tcp_pose = tcp_backend.forward(joints)

    expected_offset = tool0_pose.rotation @ np.asarray([0.02, -0.01, 0.08])
    assert tcp_pose.tip_frame == "tcp"
    assert np.allclose(tcp_pose.position_m, tool0_pose.position_m + expected_offset)
    assert tcp_backend.model_hash != tool0_backend.model_hash
    result = tcp_backend.inverse(tcp_pose, joints + 0.01, joints)
    assert result.success
    assert result.joint_position_rad is not None
    assert np.allclose(
        tcp_backend.forward(result.joint_position_rad).position_m,
        tcp_pose.position_m,
        atol=3e-4,
    )


def test_raw_session_archives_exact_cartesian_calibration(
    config: RobotConfig,
    tmp_path: Path,
) -> None:
    source = PROJECT / "configs" / "cartesian_calibration.example.yaml"
    calibration = load_cartesian_calibration(source)
    recorder = SessionRecorder(
        tmp_path,
        config,
        _profile(),
        source="gamepad",
        session_name="session_cartesian_calibration_archive",
    )
    recorder.archive_cartesian_calibration(calibration)
    recorder.close()

    manifest = json.loads(recorder.manifest_path.read_text(encoding="utf-8"))
    record = manifest["cartesian_calibration"]
    archived = recorder.session_dir / record["archive_path"]
    assert archived.read_bytes() == source.read_bytes()
    assert record["sha256"] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert check_session(recorder.session_dir).ok


def test_real_cartesian_runtime_rejects_missing_calibration_before_connect(
    config: RobotConfig,
    tmp_path: Path,
) -> None:
    class RealLabelledFakeMcu(FakeMcuTransport):
        is_simulated = False
        firmware_version = "dummy-ref-v2.1"

    class NeverPolled:
        closed = False

        def poll(self, now_ns: int | None = None) -> TeleopCommand:
            raise AssertionError("calibration gate must run before input polling")

        def close(self) -> None:
            self.closed = True

    profile = _profile()
    source = NeverPolled()
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="gamepad",
        session_name="session_missing_real_cartesian_calibration",
    )
    robot = DummyRobot(config, RealLabelledFakeMcu(config))
    with pytest.raises(ValueError, match="requires a validated Cartesian calibration"):
        run_teleop_collection(
            robot,
            source,
            recorder,
            profile,
            teleop_mode="cartesian",
            kinematics=_backend(config),
        )
    recorder.close(clean_shutdown=False)
    assert source.closed
    assert not robot.is_connected


def test_real_cartesian_runtime_requires_three_ready_pose_sweeps(
    config: RobotConfig,
    tmp_path: Path,
) -> None:
    class RealLabelledFakeMcu(FakeMcuTransport):
        is_simulated = False
        firmware_version = "dummy-ref-v2.1"

    class NeverPolled:
        closed = False

        def poll(self, now_ns: int | None = None) -> TeleopCommand:
            raise AssertionError("ready-pose gate must run before input polling")

        def close(self) -> None:
            self.closed = True

    calibration = load_cartesian_calibration(
        _write_yaml(tmp_path / "validated_ready.yaml", _valid_document(config))
    )
    backend = _backend(config, calibration)
    profile = _profile()
    source = NeverPolled()
    recorder = SessionRecorder(
        tmp_path,
        config,
        profile,
        source="gamepad",
        session_name="session_not_at_cartesian_ready",
    )
    robot = DummyRobot(
        config,
        RealLabelledFakeMcu(config),
        connect_timeout_s=0.2,
    )
    with pytest.raises(TeleopError, match="three coherent sweeps"):
        run_teleop_collection(
            robot,
            source,
            recorder,
            profile,
            teleop_mode="cartesian",
            kinematics=backend,
            cartesian_calibration=calibration,
        )
    recorder.close(clean_shutdown=False)
    assert source.closed
    assert not robot.is_connected
    assert '"event":"cartesian_ready_pose_rejected"' in recorder.events_path.read_text(
        encoding="utf-8"
    )
