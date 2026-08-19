from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .schema import ControlMode, RobotConfig, RobotState


class SafetyError(RuntimeError):
    pass


@dataclass(frozen=True)
class SafetyResult:
    requested: np.ndarray
    applied: np.ndarray
    clipped: bool
    reasons: tuple[str, ...]


class SafetyFilter:
    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self._last_applied: np.ndarray | None = None
        self._last_velocity = np.zeros(6, dtype=np.float32)
        self._last_time_ns: int | None = None

    def reset(self) -> None:
        self._last_applied = None
        self._last_velocity.fill(0)
        self._last_time_ns = None

    def apply(
        self,
        action: np.ndarray,
        state: RobotState,
        now_ns: int,
        *,
        velocity_limit_rad_s: np.ndarray | None = None,
    ) -> SafetyResult:
        if not isinstance(action, np.ndarray):
            raise SafetyError("action must be a numpy.ndarray")
        if action.dtype != np.float32:
            raise SafetyError("action dtype must be float32")
        if action.shape != (7,):
            raise SafetyError("action shape must be (7,)")
        if not np.isfinite(action).all():
            raise SafetyError("action contains NaN or Inf")
        if not state.position_valid or state.position.shape != (7,):
            raise SafetyError("robot position is invalid")
        state_age_ms = (now_ns - state.monotonic_ns) / 1_000_000
        if state_age_ms < 0 or state_age_ms > self.config.max_state_age_ms:
            raise SafetyError(f"robot state is stale ({state_age_ms:.1f} ms)")
        if state.fault_bits or state.mode.value >= 6:
            raise SafetyError(f"robot fault is active: 0x{state.fault_bits:04x}")
        if state.mode not in (ControlMode.TELEOP, ControlMode.POLICY):
            raise SafetyError(f"robot is not in a motion mode: {state.mode.name}")
        if state.config_hash != self.config.config_hash:
            raise SafetyError("robot state configuration hash mismatch")

        velocity_limit = self.config.joint_velocity_limit_rad_s
        if velocity_limit_rad_s is not None:
            if not isinstance(velocity_limit_rad_s, np.ndarray):
                raise SafetyError("velocity_limit_rad_s must be a numpy.ndarray")
            if velocity_limit_rad_s.dtype != np.float32 or velocity_limit_rad_s.shape != (6,):
                raise SafetyError("velocity_limit_rad_s must be float32[6]")
            if (
                not np.isfinite(velocity_limit_rad_s).all()
                or np.any(velocity_limit_rad_s <= 0)
                or np.any(velocity_limit_rad_s > self.config.joint_velocity_limit_rad_s)
            ):
                raise SafetyError("velocity_limit_rad_s is outside configured limits")
            velocity_limit = velocity_limit_rad_s

        requested = action.copy()
        applied = action.copy()
        reasons: list[str] = []

        mins = self.config.joint_limit_min_rad
        maxs = self.config.joint_limit_max_rad
        overshoot = self.config.max_target_overshoot_rad
        if np.any(applied[:6] < mins - overshoot) or np.any(applied[:6] > maxs + overshoot):
            raise SafetyError("joint target is far outside configured soft limits")
        limited = np.clip(applied[:6], mins, maxs)
        if not np.array_equal(limited, applied[:6]):
            applied[:6] = limited
            reasons.append("joint_soft_limit")

        grip_min, grip_max = self.config.gripper_range
        if applied[6] < grip_min - 0.1 or applied[6] > grip_max + 0.1:
            raise SafetyError("gripper target is far outside configured range")
        gripper = float(np.clip(applied[6], grip_min, grip_max))
        if gripper != applied[6]:
            applied[6] = gripper
            reasons.append("gripper_range")

        period_s = 1.0 / self.config.control_rate_hz
        if self._last_time_ns is not None:
            measured_dt = (now_ns - self._last_time_ns) / 1e9
            if measured_dt <= 0 or measured_dt > period_s * 2.5:
                self.reset()
        dt = period_s if self._last_time_ns is None else max(
            period_s * 0.5, min(period_s * 1.5, (now_ns - self._last_time_ns) / 1e9)
        )
        base = state.position[:6] if self._last_applied is None else self._last_applied[:6]
        requested_velocity = (applied[:6] - base) / dt
        velocity = np.clip(
            requested_velocity,
            -velocity_limit,
            velocity_limit,
        )
        max_dv = self.config.joint_acceleration_limit_rad_s2 * dt
        velocity = np.clip(velocity, self._last_velocity - max_dv, self._last_velocity + max_dv)
        position = base + velocity * dt
        if not np.allclose(position, applied[:6], rtol=0, atol=1e-7):
            applied[:6] = position
            reasons.append("velocity_or_acceleration_limit")
        applied[:6] = np.clip(applied[:6], mins, maxs)

        self._last_applied = applied.copy()
        self._last_velocity = velocity.astype(np.float32, copy=True)
        self._last_time_ns = now_ns
        return SafetyResult(requested, applied, bool(reasons), tuple(reasons))
