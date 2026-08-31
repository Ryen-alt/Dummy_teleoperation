from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

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
    if profile.session_epoch == 0:
        failures.append(
            "maintenance timing window is measurement-only and cannot pass the formal Stream gate"
        )
    if not collecting:
        failures.append("timing window is not currently active; capture during Stream")
    if not epoch_stable:
        failures.append("session epoch changed during the timing window")
    if not pages_complete or profile.motor_page_valid_mask != (0x7F,) * 4:
        failures.append("not all four timing pages were read from all seven motors")
    if not samples_valid:
        failures.append("latency sample minimum is not reached (position 1000, temperature 100 per node)")
    if any(count < 1000 for count in profile.position_samples) or any(
        count < 100 for count in profile.temperature_samples
    ):
        failures.append(
            "reported latency sample counts are below the A9 per-node minimum"
        )
    if any(count < 100 for count in profile.motor_can_samples):
        failures.append("motor 0x05 profiler has fewer than 100 samples on a node")

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


def load_can_timing_profile_events(
    path: str | Path,
    *,
    minimum_monotonic_ns: int | None = None,
    maximum_monotonic_ns: int | None = None,
) -> CanTimingProfile:
    """Load the latest active A9 snapshot from a session events file."""

    if (
        minimum_monotonic_ns is not None
        and maximum_monotonic_ns is not None
        and maximum_monotonic_ns < minimum_monotonic_ns
    ):
        raise ValueError("timing-profile event bounds are reversed")

    latest: dict[str, object] | None = None
    latest_active: dict[str, object] | None = None
    for line_number, line in enumerate(
        Path(path).read_text(encoding="utf-8").splitlines(), 1
    ):
        if not line:
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"invalid events JSON on line {line_number}: {exc}"
            ) from exc
        if record.get("event") != "can_timing_profile":
            continue
        if minimum_monotonic_ns is not None or maximum_monotonic_ns is not None:
            monotonic_ns = record.get("monotonic_ns")
            if (
                isinstance(monotonic_ns, bool)
                or not isinstance(monotonic_ns, int)
                or monotonic_ns < 0
            ):
                raise ValueError(
                    "bounded can_timing_profile event has invalid monotonic_ns"
                )
            if (
                minimum_monotonic_ns is not None
                and monotonic_ns < minimum_monotonic_ns
            ) or (
                maximum_monotonic_ns is not None
                and monotonic_ns > maximum_monotonic_ns
            ):
                continue
        payload = record.get("payload")
        if not isinstance(payload, dict):
            raise ValueError("can_timing_profile event payload is not an object")
        latest = payload
        if int(payload.get("window_flags", 0)) & CAN_TIMING_PROFILE_WINDOW_ACTIVE:
            latest_active = payload
    selected = latest_active if latest_active is not None else latest
    if selected is None:
        raise ValueError("events file contains no can_timing_profile record")
    converted = {
        key: tuple(value) if isinstance(value, list) else value
        for key, value in selected.items()
    }
    return CanTimingProfile(**converted)
