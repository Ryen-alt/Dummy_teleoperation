from __future__ import annotations

import sys
from dataclasses import replace
from types import SimpleNamespace

import pytest

from dummy_host.bringup import (
    BringupError,
    EvdevDeadman,
    make_joint_bringup_plan,
    run_real_bringup,
    run_simulated_bringup,
)
from dummy_host.schema import RobotConfig


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


def test_simulated_bringup_closes_in_hold(config: RobotConfig) -> None:
    plan = make_joint_bringup_plan(config, joint=1, delta_deg=0.2, duration_s=0.12)
    result = run_simulated_bringup(config, plan)
    assert result.actions_sent >= 1
    assert result.last_applied_sequence == result.last_sequence
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
