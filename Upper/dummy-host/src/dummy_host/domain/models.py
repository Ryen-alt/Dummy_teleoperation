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


class CanRuntimeStatusBits(IntEnum):
    DISPATCHER_ALIVE = 1 << 0
    TX_QUEUED = 1 << 1
    POSITION_REQUESTED = 1 << 2
    POSITION_RESPONDED = 1 << 3
    TX_DEFERRED = 1 << 4
    QUERY_PENDING = 1 << 5
    FEEDBACK_READY = 1 << 6
    TX_RECOVERED = 1 << 7


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


class ActionStage(StrEnum):
    RECEIVED = "received"
    SAFETY_ACCEPTED = "safety_accepted"
    SEND_ENQUEUED = "send_enqueued"
    SERIAL_SEND_STARTED = "serial_send_started"
    SERIAL_SEND_FINISHED = "serial_send_finished"
    ACKNOWLEDGED = "acknowledged"
    CAN_QUEUED_EXACT = "can_queued_exact"
    CAN_TX_COMPLETE_EXACT = "can_tx_complete_exact"
    POST_COMMAND_FEEDBACK = "post_command_feedback"
    SUPERSEDED = "superseded"
    PREEMPTED_BY_SAFETY = "preempted_by_safety"
    REJECTED = "rejected"
    FAILED = "failed"


@dataclass(frozen=True)
class ActionLifecycleUpdate:
    sequence: int
    stage: ActionStage
    host_time_ns: int
    mcu_time_us: int = 0
    detail: str | None = None
    session_epoch: int = 0
    control_tick_id: int = 0
    measurement_us: int = 0

    def __post_init__(self) -> None:
        if (
            self.sequence <= 0
            or self.host_time_ns < 0
            or self.mcu_time_us < 0
            or self.session_epoch < 0
            or self.control_tick_id < 0
            or self.measurement_us < 0
            or self.session_epoch > 0xFFFFFFFF
            or self.control_tick_id > 0xFFFFFFFF
            or self.measurement_us > 0xFFFFFFFF
        ):
            raise ValueError("action lifecycle identifiers and timestamps are invalid")


class ActionProgressFlags(IntEnum):
    CAN_QUEUED_EXACT = 1 << 0
    CAN_TX_COMPLETE_EXACT = 1 << 1
    POST_COMMAND_FEEDBACK = 1 << 2
    SUPERSEDED = 1 << 3
    PREEMPTED_BY_SAFETY = 1 << 4
    FAILED = 1 << 5


@dataclass(frozen=True)
class ActionProgressRecord:
    sequence: int
    flags: int
    can_queued_mcu_us: int = 0
    can_tx_complete_mcu_us: int = 0
    post_feedback_mcu_us: int = 0
    feedback_sweep_id: int = 0

    def __post_init__(self) -> None:
        if min(
            self.sequence,
            self.flags,
            self.can_queued_mcu_us,
            self.can_tx_complete_mcu_us,
            self.post_feedback_mcu_us,
            self.feedback_sweep_id,
        ) < 0:
            raise ValueError("action progress fields must be non-negative")


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
    can_transport_status: int = 0
    feedback_sample_mcu_us: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=np.uint64)
    )
    feedback_sweep_id: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=np.uint32)
    )
    coherent_sweep_id: int = 0
    feedback_max_skew_us: int = 0
    coherent_reference_mcu_us: int = 0
    state_repeated: bool = False
    action_progress: tuple[ActionProgressRecord, ...] = ()

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
            ("feedback_sample_mcu_us", self.feedback_sample_mcu_us, np.uint64),
            ("feedback_sweep_id", self.feedback_sweep_id, np.uint32),
        )
        if (
            self.monotonic_ns < 0
            or self.mcu_time_us < 0
            or self.fault_bits < 0
            or self.last_received_sequence < 0
            or self.target_age_ms < 0
            or self.hold_reason_bits < 0
            or self.telemetry_validity < 0
            or not 0 <= self.can_transport_status <= 0xFF
            or self.coherent_sweep_id < 0
            or self.feedback_max_skew_us < 0
            or self.coherent_reference_mcu_us < 0
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
        progress = tuple(self.action_progress)
        if len(progress) > 6 or any(
            not isinstance(item, ActionProgressRecord) for item in progress
        ):
            raise ValueError("action_progress must contain at most six records")
        object.__setattr__(self, "action_progress", progress)

    @property
    def coherent(self) -> bool:
        return (
            self.coherent_sweep_id > 0
            and self.position_valid
            and np.all(self.feedback_sweep_id == self.coherent_sweep_id)
        )

    @property
    def last_can_queued_exact_sequence(self) -> int:
        return next(
            (
                record.sequence
                for record in reversed(self.action_progress)
                if record.flags & int(ActionProgressFlags.CAN_QUEUED_EXACT)
            ),
            0,
        )

    @property
    def last_can_queued_mcu_us(self) -> int:
        return next(
            (
                record.can_queued_mcu_us
                for record in reversed(self.action_progress)
                if record.flags & int(ActionProgressFlags.CAN_QUEUED_EXACT)
            ),
            0,
        )

    @property
    def last_can_tx_complete_exact_sequence(self) -> int:
        return next(
            (
                record.sequence
                for record in reversed(self.action_progress)
                if record.flags & int(ActionProgressFlags.CAN_TX_COMPLETE_EXACT)
            ),
            0,
        )

    @property
    def last_can_tx_complete_mcu_us(self) -> int:
        return next(
            (
                record.can_tx_complete_mcu_us
                for record in reversed(self.action_progress)
                if record.flags & int(ActionProgressFlags.CAN_TX_COMPLETE_EXACT)
            ),
            0,
        )

    @property
    def last_post_command_feedback_sequence(self) -> int:
        return next(
            (
                record.sequence
                for record in reversed(self.action_progress)
                if record.flags & int(ActionProgressFlags.POST_COMMAND_FEEDBACK)
            ),
            0,
        )

    @property
    def last_post_command_feedback_mcu_us(self) -> int:
        return next(
            (
                record.post_feedback_mcu_us
                for record in reversed(self.action_progress)
                if record.flags & int(ActionProgressFlags.POST_COMMAND_FEEDBACK)
            ),
            0,
        )


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
    session_epoch: int = 0
    control_tick_id: int = 0

    def __post_init__(self) -> None:
        requested = _frozen_array(self.requested, shape=(7,), name="requested action")
        applied = _frozen_array(self.applied, shape=(7,), name="applied action")
        canonical_value = applied if self.canonical is None else self.canonical
        canonical = _frozen_array(canonical_value, shape=(7,), name="canonical action")
        if (
            self.sequence < 0
            or self.monotonic_ns < 0
            or self.session_epoch < 0
            or self.control_tick_id < 0
            or self.session_epoch > 0xFFFFFFFF
            or self.control_tick_id > 0xFFFFFFFF
        ):
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
