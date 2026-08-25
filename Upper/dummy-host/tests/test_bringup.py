from __future__ import annotations

import sys
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

import dummy_host.bringup as bringup_module
from dummy_host.bringup import (
    BringupError,
    EvdevDeadman,
    make_joint_bringup_plan,
    run_real_bringup,
    run_simulated_bringup,
)
from dummy_host.safety import SafetyError
from dummy_host.schema import ControlMode, RobotConfig


def test_bringup_plan_is_single_joint_and_bounded(config: RobotConfig) -> None:
    plan = make_joint_bringup_plan(config, joint=2, delta_deg=0.2)
    changed = [
        index
        for index, (start, final) in enumerate(zip(plan.start_action, plan.final_requested_action))
        if start != final
    ]
    assert changed == [1]
    assert plan.delta_deg == 0.2


@pytest.mark.parametrize("delta", [0.0, 1.01, -1.01, float("nan")])
def test_bringup_plan_rejects_unsafe_delta(config: RobotConfig, delta: float) -> None:
    with pytest.raises(BringupError):
        make_joint_bringup_plan(config, joint=1, delta_deg=delta)


@pytest.mark.parametrize(("joint", "delta"), [(2, -0.2), (3, 0.2)])
def test_real_plan_defers_soft_limit_check_to_live_state(
    config: RobotConfig,
    joint: int,
    delta: float,
) -> None:
    with pytest.raises(BringupError, match="configured soft limit"):
        make_joint_bringup_plan(config, joint=joint, delta_deg=delta)

    plan = make_joint_bringup_plan(
        config,
        joint=joint,
        delta_deg=delta,
        validate_initial_pose_limits=False,
    )
    assert plan.joint == joint
    assert plan.delta_deg == delta


def test_simulated_bringup_closes_in_hold(config: RobotConfig) -> None:
    plan = make_joint_bringup_plan(config, joint=1, delta_deg=0.2, duration_s=0.12)
    result = run_simulated_bringup(config, plan)
    assert result.actions_sent >= 1
    assert result.last_post_command_feedback_sequence == result.last_sequence
    assert result.final_mode == "HOLD"


def test_real_bringup_is_blocked_and_closes_deadman_when_unverified(
    config: RobotConfig,
    tmp_path,
) -> None:
    class FakeDeadman:
        closed = False

        def is_pressed(self) -> bool:
            return True

        def close(self) -> None:
            self.closed = True

    deadman = FakeDeadman()
    plan = make_joint_bringup_plan(config, joint=1)
    unverified = replace(
        config,
        hardware_parameters_verified=False,
        external_target_execution_ready=False,
    )
    with pytest.raises(BringupError, match="hardware_parameters_verified"):
        run_real_bringup(
            unverified,
            plan,
            port="unused",
            deadman=deadman,
            log_jsonl=tmp_path / "unused.jsonl",
        )
    assert deadman.closed


def test_real_bringup_refuses_to_overwrite_log_and_closes_deadman(
    config: RobotConfig,
    tmp_path,
) -> None:
    class FakeDeadman:
        closed = False

        def is_pressed(self) -> bool:
            return True

        def close(self) -> None:
            self.closed = True

    log_path = tmp_path / "j1_pos.jsonl"
    log_path.write_text("existing evidence\n", encoding="utf-8")
    deadman = FakeDeadman()
    plan = make_joint_bringup_plan(config, joint=1)

    with pytest.raises(BringupError, match="refusing to overwrite"):
        run_real_bringup(
            config,
            plan,
            port="unused",
            deadman=deadman,
            log_jsonl=log_path,
        )

    assert deadman.closed
    assert log_path.read_text(encoding="utf-8") == "existing evidence\n"


def test_real_bringup_preserves_primary_error_when_release_cleanup_fails(
    config: RobotConfig,
    tmp_path,
    monkeypatch,
) -> None:
    class FakeDeadman:
        closed = False

        def is_pressed(self) -> bool:
            return True

        def close(self) -> None:
            self.closed = True

    class FakeRobot:
        instance = None

        def __init__(self, *_args, **_kwargs) -> None:
            FakeRobot.instance = self
            self.is_connected = False
            self.state = SimpleNamespace(
                position=np.concatenate(
                    (config.initial_pose_rad.copy(), np.asarray([0.5], dtype=np.float32))
                ),
                position_valid=True,
                gripper_valid=True,
                mode=ControlMode.TELEOP,
            )

        def connect(self) -> None:
            self.is_connected = True

        def acquire_control(self, _mode: ControlMode) -> None:
            pass

        def read_state(self):
            return self.state

        def send_action(self, *_args, **_kwargs):
            raise SafetyError("primary motion-mode failure")

        def hold(self) -> None:
            self.state.mode = ControlMode.HOLD

        def release_control(self) -> None:
            raise RuntimeError("secondary NO_LEASE cleanup failure")

        def disconnect(self) -> None:
            self.is_connected = False

    monkeypatch.setattr(bringup_module, "DummyRobot", FakeRobot)
    deadman = FakeDeadman()
    plan = make_joint_bringup_plan(config, joint=1, duration_s=0.01)

    with pytest.raises(SafetyError, match="primary motion-mode failure"):
        run_real_bringup(
            config,
            plan,
            port="unused",
            deadman=deadman,
            log_jsonl=tmp_path / "failed.jsonl",
        )

    assert deadman.closed
    assert FakeRobot.instance is not None
    assert not FakeRobot.instance.is_connected


def test_evdev_deadman_system_error_is_safe_release(monkeypatch) -> None:
    class UnpluggedInputDevice:
        def __init__(self, path: str) -> None:
            self.path = path

        def active_keys(self):
            raise SystemError(
                "<built-in function ioctl_EVIOCG_bits> returned NULL "
                "without setting an exception"
            )

        def close(self) -> None:
            pass

    monkeypatch.setitem(
        sys.modules,
        "evdev",
        SimpleNamespace(
            InputDevice=UnpluggedInputDevice,
            ecodes=SimpleNamespace(ecodes={"KEY_SPACE": 57}),
        ),
    )
    deadman = EvdevDeadman("/dev/input/event3")
    assert not deadman.is_pressed()
    assert deadman.last_error is not None
