from __future__ import annotations

import math
from dataclasses import asdict, dataclass

from .protocol import (
    CAN_TIMING_PROFILE_EPOCH_STABLE,
    CAN_TIMING_PROFILE_LATENCY_SAMPLES_VALID,
    CAN_TIMING_PROFILE_MOTOR_PAGES_COMPLETE,
    CAN_TIMING_PROFILE_WINDOW_ACTIVE,
    CanTimingProfile,
)


@dataclass(frozen=True)
class CanA9Evaluation:
    result: str
    passed: bool
    collecting: bool
    evidence_complete: bool
    recommended_node_quiet_us: int
    measured_node_quiet_us: int
    recommended_response_timeout_us: int
    max_motor_can_service_p999_us: float
    max_response_p999_us: int
    response_latency_us: dict[str, dict[str, int]]
    failures: tuple[str, ...]
    profile: dict[str, object]


def evaluate_can_a9(profile: CanTimingProfile) -> CanA9Evaluation:
    failures: list[str] = []
    collecting = bool(profile.window_flags & CAN_TIMING_PROFILE_WINDOW_ACTIVE)
    epoch_stable = bool(profile.window_flags & CAN_TIMING_PROFILE_EPOCH_STABLE)
    pages_complete = bool(
        profile.window_flags & CAN_TIMING_PROFILE_MOTOR_PAGES_COMPLETE
    )
    samples_valid = bool(
        profile.window_flags & CAN_TIMING_PROFILE_LATENCY_SAMPLES_VALID
    )
    if not collecting:
        failures.append("timing window is not currently active; capture during Stream")
    if not epoch_stable:
        failures.append("session epoch changed during the timing window")
    if not pages_complete or profile.motor_page_valid_mask != (0x7F,) * 4:
        failures.append("not all four timing pages were read from all seven motors")
    if not samples_valid:
        failures.append("latency sample minimum is not reached (position 1000, temperature 100 per node)")

    required_motor_flags = 0x0F
    invalid_nodes = [
        index + 1
        for index, flags in enumerate(profile.motor_flags)
        if flags & required_motor_flags != required_motor_flags
    ]
    if invalid_nodes:
        failures.append(f"motor profiler validity flags missing on nodes {invalid_nodes}")
    missed_nodes = [
        index + 1
        for index, count in enumerate(profile.motor_missed_ticks)
        if count != 0
    ]
    if missed_nodes:
        failures.append(f"20 kHz control ticks were missed on nodes {missed_nodes}")
    timeout_nodes = [
        index + 1
        for index, count in enumerate(profile.timing_timeout)
        if count != 0
    ]
    if timeout_nodes:
        failures.append(f"timing-profile CAN queries timed out on nodes {timeout_nodes}")

    max_motor_p999_us = max(profile.motor_can_p999_x10_us, default=0) / 10.0
    max_response_p999_us = max(
        (*profile.position_p999_us, *profile.temperature_p999_us),
        default=0,
    )
    measured_quiet_us = math.ceil(max_motor_p999_us + 100.0)
    response_timeout_us = max_response_p999_us + 200
    # RobotConfig currently requires response_timeout <= node_quiet. Preserve
    # that invariant explicitly instead of generating an invalid configuration.
    recommended_quiet_us = max(measured_quiet_us, response_timeout_us)
    if recommended_quiet_us > 500:
        failures.append(
            f"recommended can_node_quiet_us={recommended_quiet_us} exceeds the A9 500 us limit"
        )
    if response_timeout_us > 1000:
        failures.append(
            f"recommended can_response_timeout_us={response_timeout_us} exceeds the A9 1000 us limit"
        )

    evidence_complete = collecting and epoch_stable and pages_complete and samples_valid
    response_latency_us = {
        "position": {
            "p50": max(profile.position_p50_us, default=0),
            "p99": max(profile.position_p99_us, default=0),
            "p99_9": max(profile.position_p999_us, default=0),
            "max": max(profile.position_max_us, default=0),
        },
        "temperature": {
            "p50": max(profile.temperature_p50_us, default=0),
            "p99": max(profile.temperature_p99_us, default=0),
            "p99_9": max(profile.temperature_p999_us, default=0),
            "max": max(profile.temperature_max_us, default=0),
        },
    }
    passed = evidence_complete and not failures
    return CanA9Evaluation(
        result="PASS" if passed else "FAIL",
        passed=passed,
        collecting=collecting,
        evidence_complete=evidence_complete,
        recommended_node_quiet_us=recommended_quiet_us,
        measured_node_quiet_us=measured_quiet_us,
        recommended_response_timeout_us=response_timeout_us,
        max_motor_can_service_p999_us=max_motor_p999_us,
        max_response_p999_us=max_response_p999_us,
        response_latency_us=response_latency_us,
        failures=tuple(failures),
        profile=asdict(profile),
    )
