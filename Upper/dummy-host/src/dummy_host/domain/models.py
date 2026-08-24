from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from types import MappingProxyType
from typing import Mapping

import numpy as np

from .._compat import StrEnum


class ControlMode(IntEnum):
    DISABLED = 1
    HOLD = 2
    TELEOP = 3
    POLICY = 4
    GRAVITY = 5
    FAULT = 6


class HoldReasonBits(IntEnum):
    TARGET_TIMEOUT = 1 << 0
    LEASE_TIMEOUT = 1 << 1
    FOLLOWING_ERROR = 1 << 2
    FEEDBACK_STALE = 1 << 3
    OPERATOR = 1 << 4
    RUNTIME_LIMIT = 1 << 5


class FaultBits(IntEnum):
    EMERGENCY_STOP = 1 << 0
    FEEDBACK_LOST = 1 << 1
    OVER_TEMPERATURE = 1 << 2
    ENCODER = 1 << 3
    STALL = 1 << 4
    OVER_CURRENT = 1 << 5


class TelemetryValidityBits(IntEnum):
    FOLLOWING_ERROR = 1 << 0
    CAN_FEEDBACK = 1 << 1
    TEMPERATURE = 1 << 2


class NodeFaultBits(IntEnum):
    FEEDBACK_STALE = 1 << 0
    FOLLOWING_ERROR = 1 << 1
    OVER_TEMPERATURE = 1 << 2
    ENCODER = 1 << 3
    STALL = 1 << 4
    OVER_CURRENT = 1 << 5


class NodeValidityBits(IntEnum):
    POSITION = 1 << 0
    TEMPERATURE = 1 << 1
    ENCODER_FAULT_SOURCE = 1 << 2
    STALL_SOURCE = 1 << 3
    CURRENT_SOURCE = 1 << 4


class ActionSpace(StrEnum):
    JOINT_POSITION_ABSOLUTE = "joint_position_absolute"
    JOINT_POSITION_DELTA = "joint_position_delta"
    JOINT_VELOCITY = "joint_velocity"
    CARTESIAN_POSE = "cartesian_pose"
    CARTESIAN_TWIST = "cartesian_twist"


def _frozen_array(value: np.ndarray, *, shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value)
    if array.dtype != np.float32:
        raise ValueError(f"{name} must have dtype float32")
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} contains NaN or Inf")
    frozen = array.copy()
    frozen.flags.writeable = False
    return frozen


@dataclass(frozen=True)
class RobotState:
    position: np.ndarray
    velocity: np.ndarray
    monotonic_ns: int
    mcu_time_us: int
    mode: ControlMode
    fault_bits: int
    position_valid: bool
    velocity_valid: bool
    gripper_valid: bool
    last_received_sequence: int
    last_applied_sequence: int
    target_age_ms: int
    config_hash: str
    following_error: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=np.float32)
    )
    following_error_duration_ms: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=np.uint32)
    )
    feedback_age_ms: np.ndarray = field(
        default_factory=lambda: np.full(7, np.iinfo(np.uint32).max, dtype=np.uint32)
    )
    feedback_loss_count: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=np.uint32)
    )
    consecutive_feedback_loss: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=np.uint16)
    )
    node_fault_bits: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=np.uint16)
    )
    node_validity: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=np.uint8)
    )
    hold_reason_bits: int = 0
    telemetry_validity: int = 0

    def __post_init__(self) -> None:
        position = _frozen_array(self.position, shape=(7,), name="robot position")
        velocity = _frozen_array(self.velocity, shape=(7,), name="robot velocity")
        following_error = _frozen_array(
            self.following_error, shape=(7,), name="following error"
        )
        integer_fields = (
            ("following_error_duration_ms", self.following_error_duration_ms, np.uint32),
            ("feedback_age_ms", self.feedback_age_ms, np.uint32),
            ("feedback_loss_count", self.feedback_loss_count, np.uint32),
            ("consecutive_feedback_loss", self.consecutive_feedback_loss, np.uint16),
            ("node_fault_bits", self.node_fault_bits, np.uint16),
            ("node_validity", self.node_validity, np.uint8),
        )
        if (
            self.monotonic_ns < 0
            or self.mcu_time_us < 0
            or self.fault_bits < 0
            or self.last_received_sequence < 0
            or self.last_applied_sequence < 0
            or self.target_age_ms < 0
            or self.hold_reason_bits < 0
            or self.telemetry_validity < 0
        ):
            raise ValueError("robot timestamps, faults, sequences and ages must be non-negative")
        try:
            hash_bytes = bytes.fromhex(self.config_hash)
        except ValueError as exc:
            raise ValueError("robot config_hash must be hexadecimal") from exc
        if len(hash_bytes) != 32:
            raise ValueError("robot config_hash must contain 32 bytes")
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "velocity", velocity)
        object.__setattr__(self, "following_error", following_error)
        for name, value, dtype in integer_fields:
            array = np.asarray(value)
            if array.shape != (7,) or array.dtype != dtype:
                raise ValueError(f"{name} must have shape (7,) and dtype {dtype}")
            frozen = array.copy()
            frozen.flags.writeable = False
            object.__setattr__(self, name, frozen)


@dataclass(frozen=True)
class ActionProposal:
    source: str
    action_space: ActionSpace
    values: np.ndarray
    generated_at_ns: int
    valid_until_ns: int
    observation_sequence: int = 0
    chunk_id: str | None = None
    model_id: str | None = None
    task_id: str | None = None
    metadata: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.source:
            raise ValueError("action proposal source must be non-empty")
        if self.generated_at_ns < 0 or self.valid_until_ns <= self.generated_at_ns:
            raise ValueError("action proposal validity window is invalid")
        if self.observation_sequence < 0:
            raise ValueError("observation_sequence must be non-negative")
        values = np.asarray(self.values)
        if values.dtype != np.float32 or values.ndim != 1 or not np.isfinite(values).all():
            raise ValueError("action proposal values must be a finite float32 vector")
        frozen = values.copy()
        frozen.flags.writeable = False
        object.__setattr__(self, "values", frozen)
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))


@dataclass(frozen=True)
class AppliedAction:
    requested: np.ndarray
    applied: np.ndarray
    sequence: int
    monotonic_ns: int
    clipped: bool
    reasons: tuple[str, ...]
    canonical: np.ndarray | None = None
    source: str = "direct"

    def __post_init__(self) -> None:
        requested = _frozen_array(self.requested, shape=(7,), name="requested action")
        applied = _frozen_array(self.applied, shape=(7,), name="applied action")
        canonical_value = applied if self.canonical is None else self.canonical
        canonical = _frozen_array(canonical_value, shape=(7,), name="canonical action")
        if self.sequence < 0 or self.monotonic_ns < 0:
            raise ValueError("action sequence and timestamp must be non-negative")
        if not self.source:
            raise ValueError("applied action source must be non-empty")
        object.__setattr__(self, "requested", requested)
        object.__setattr__(self, "canonical", canonical)
        object.__setattr__(self, "applied", applied)


@dataclass(frozen=True)
class ObservationBundle:
    state: RobotState
    images: Mapping[str, object]
    target_time_ns: int
    task: str | None = None
    valid: bool = True
    invalid_reasons: tuple[str, ...] = ()
    observation_sequence: int = 0
    schema_id: str = ""

    def __post_init__(self) -> None:
        if self.target_time_ns < 0 or self.observation_sequence < 0:
            raise ValueError("observation timestamps and sequence must be non-negative")
        if self.valid and self.invalid_reasons:
            raise ValueError("a valid observation cannot have invalid reasons")
        object.__setattr__(self, "images", MappingProxyType(dict(self.images)))


@dataclass(frozen=True)
class RobotHealth:
    connected: bool
    state_fresh: bool
    mode: ControlMode | None
    fault_bits: int
    state_age_ms: float | None
    details: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "details", MappingProxyType(dict(self.details)))
