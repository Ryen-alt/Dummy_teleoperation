from __future__ import annotations

from dataclasses import asdict, dataclass

from .can_a9 import CanA9Evaluation, evaluate_can_a9
from .protocol import CanTimingProfile


@dataclass(frozen=True)
class CanR5Thresholds:
    minimum_duration_s: float = 600.0
    minimum_fanout_samples: int = 1000
    maximum_coherent_sweep_p99_ms: float = 50.0
    maximum_exact_fanout_p99_ms: float = 15.0
    branch_b_warning_transaction_us: int = 3000
    branch_b_limit_transaction_us: int = 3571

    def __post_init__(self) -> None:
        if (
            self.minimum_duration_s <= 0
            or self.minimum_fanout_samples <= 0
            or self.maximum_coherent_sweep_p99_ms <= 0
            or self.maximum_exact_fanout_p99_ms <= 0
            or self.branch_b_warning_transaction_us <= 0
            or self.branch_b_limit_transaction_us
            <= self.branch_b_warning_transaction_us
        ):
            raise ValueError("R5 thresholds must be positive and ordered")


@dataclass(frozen=True)
class CanR5RuntimeEvidence:
    duration_s: float
    soak_passed: bool
    soak_failures: tuple[str, ...]
    coherent_sweep_p99_ms: float
    exact_fanout_p99_ms: float
    exact_fanout_samples: int
    expected_exact_fanout_samples: int
    position_timeout_count: int
    temperature_timeout_count: int
    can_abort_error_count: int
    can_recovery_count: int
    can_busoff_count: int
    can_rx_overflow_count: int
    can_completion_overflow_count: int
    motor_tx_drop_count: int
    motor_rx_error_count: int
    motor_busoff_count: int
    can_unexpected_response_count: int
    target_retry_count: int
    target_deadline_failure_count: int
    transition_failure_count: int
    action_credit_miss_events: int


@dataclass(frozen=True)
class CanR5Decision:
    result: str
    passed: bool
    branch: str
    configuration_ready: bool
    recommended_node_quiet_us: int
    recommended_response_timeout_us: int
    recommended_position_hz_per_node: int
    control_rate_hz: int
    retry_policy: str
    single_transaction_p99_us: int
    single_transaction_budget_us: int
    formulas: dict[str, str]
    current_configuration: dict[str, int]
    a9: CanA9Evaluation
    runtime: CanR5RuntimeEvidence
    failures: tuple[str, ...]


def evaluate_can_r5(
    profile: CanTimingProfile,
    runtime: CanR5RuntimeEvidence,
    *,
    current_node_quiet_us: int,
    current_response_timeout_us: int,
    current_position_hz_per_node: int,
    control_rate_hz: int = 20,
    minimum_required_position_hz: int = 20,
    thresholds: CanR5Thresholds = CanR5Thresholds(),
) -> CanR5Decision:
    """Apply the measurement-driven R5 A/B/C decision without changing config."""

    if current_position_hz_per_node not in {25, 30, 40}:
        raise ValueError("R5 position rate must be one of 25, 30, or 40 Hz")
    if not 1 <= minimum_required_position_hz <= 40:
        raise ValueError("minimum required position rate must be in 1..40 Hz")

    a9 = evaluate_can_a9(profile)
    transaction_p99_us = max(
        (*profile.position_p99_us, *profile.temperature_p99_us), default=0
    )
    recommended_position_hz = current_position_hz_per_node
    if transaction_p99_us >= thresholds.branch_b_limit_transaction_us:
        recommended_position_hz = 25
    elif transaction_p99_us >= thresholds.branch_b_warning_transaction_us:
        recommended_position_hz = 30

    if current_position_hz_per_node == 40:
        branch = (
            f"B_REDUCE_TO_{recommended_position_hz}_HZ"
            if recommended_position_hz < 40
            else "A_KEEP_40_HZ"
        )
    else:
        branch = f"B_VERIFY_{recommended_position_hz}_HZ"

    if minimum_required_position_hz > recommended_position_hz:
        branch = "C_BOUNDED_CONCURRENCY_REVIEW"

    transaction_budget_us = 1_000_000 // (
        recommended_position_hz * 7
    )

    failures: list[str] = list(runtime.soak_failures)
    if control_rate_hz != 20:
        failures.append(
            f"R5 requires the control loop to remain at 20 Hz, got {control_rate_hz}"
        )
    if runtime.duration_s < thresholds.minimum_duration_s:
        failures.append(
            f"R5 window {runtime.duration_s:.3f} s is below "
            f"{thresholds.minimum_duration_s:.3f} s"
        )
    if not runtime.soak_passed and not runtime.soak_failures:
        failures.append("strict soak evaluation did not pass")
    if runtime.coherent_sweep_p99_ms >= thresholds.maximum_coherent_sweep_p99_ms:
        failures.append(
            f"coherent sweep p99 {runtime.coherent_sweep_p99_ms:.3f} ms "
            f"is not below {thresholds.maximum_coherent_sweep_p99_ms:.3f} ms"
        )
    if runtime.exact_fanout_samples < thresholds.minimum_fanout_samples:
        failures.append(
            f"exact fanout evidence has {runtime.exact_fanout_samples} samples; "
            f"requires {thresholds.minimum_fanout_samples}"
        )
    if runtime.exact_fanout_samples != runtime.expected_exact_fanout_samples:
        failures.append(
            f"exact fanout evidence has {runtime.exact_fanout_samples} events "
            f"for {runtime.expected_exact_fanout_samples} action lifecycles"
        )
    if runtime.exact_fanout_p99_ms >= thresholds.maximum_exact_fanout_p99_ms:
        failures.append(
            f"exact target fanout p99 {runtime.exact_fanout_p99_ms:.3f} ms "
            f"is not below {thresholds.maximum_exact_fanout_p99_ms:.3f} ms"
        )
    zero_gates = {
        "position timeout": runtime.position_timeout_count,
        "temperature timeout": runtime.temperature_timeout_count,
        "CAN abort/error": runtime.can_abort_error_count,
        "CAN recovery": runtime.can_recovery_count,
        "CAN bus-off": runtime.can_busoff_count,
        "CAN RX overflow": runtime.can_rx_overflow_count,
        "CAN completion overflow": runtime.can_completion_overflow_count,
        "motor TX drop": runtime.motor_tx_drop_count,
        "motor RX error": runtime.motor_rx_error_count,
        "motor bus-off": runtime.motor_busoff_count,
        "unexpected CAN response": runtime.can_unexpected_response_count,
        "target retry": runtime.target_retry_count,
        "target deadline failure": runtime.target_deadline_failure_count,
        "stream transition failure": runtime.transition_failure_count,
        "action-credit miss/defer": runtime.action_credit_miss_events,
    }
    for label, count in zero_gates.items():
        if count != 0:
            failures.append(f"{label} count is {count}, expected zero")

    # A9's 500/1000 us margin failures select branch B; every other A9
    # failure invalidates the evidence itself and cannot be scheduled around.
    a9_integrity_failures = tuple(
        item
        for item in a9.failures
        if not item.startswith("recommended can_node_quiet_us=")
        and not item.startswith("recommended can_response_timeout_us=")
    )
    failures.extend(a9_integrity_failures)
    if recommended_position_hz == 40:
        failures.extend(
            item
            for item in a9.failures
            if item.startswith("recommended can_node_quiet_us=")
            or item.startswith("recommended can_response_timeout_us=")
        )
    if (
        recommended_position_hz < 40
        and a9.recommended_response_timeout_us >= transaction_budget_us
    ):
        failures.append(
            f"measured response timeout {a9.recommended_response_timeout_us} us "
            f"does not fit the {transaction_budget_us} us single-outstanding "
            f"slot at {recommended_position_hz} Hz"
        )

    config_covers_measurement = (
        not a9_integrity_failures
        and current_node_quiet_us == a9.recommended_node_quiet_us
        and current_response_timeout_us
        == a9.recommended_response_timeout_us
        and current_response_timeout_us <= current_node_quiet_us
        and current_position_hz_per_node == recommended_position_hz
        and (
            (
                recommended_position_hz == 40
                and current_node_quiet_us <= 500
                and current_response_timeout_us <= 1000
            )
            or (
                recommended_position_hz < 40
                and current_response_timeout_us < transaction_budget_us
            )
        )
    )
    runtime_passed = not failures
    configuration_ready = runtime_passed and config_covers_measurement

    if a9_integrity_failures:
        result = "FAIL"
    elif not runtime_passed:
        result = "FAIL"
    elif branch == "C_BOUNDED_CONCURRENCY_REVIEW":
        result = "REVIEW_C"
        failures.append(
            "the required feedback rate exceeds the measured single-outstanding "
            "rate; a separately reviewed bounded-concurrency design is required "
            "and unlimited outstanding is forbidden"
        )
    elif branch.startswith("B_REDUCE"):
        result = "RETEST_B"
        failures.append(
            f"set position feedback to {recommended_position_hz} Hz with the "
            "measured quiet/timeout values, then collect a fresh 10-minute window"
        )
    elif not config_covers_measurement:
        result = "RECONFIGURE"
        failures.append(
            "current CAN configuration is not frozen to the measured "
            "quiet/timeout/rate values"
        )
    else:
        result = "PASS"

    unique_failures = tuple(dict.fromkeys(failures))
    passed = result == "PASS" and not unique_failures
    return CanR5Decision(
        result=result,
        passed=passed,
        branch=branch,
        configuration_ready=passed and configuration_ready,
        recommended_node_quiet_us=a9.recommended_node_quiet_us,
        recommended_response_timeout_us=a9.recommended_response_timeout_us,
        recommended_position_hz_per_node=recommended_position_hz,
        control_rate_hz=control_rate_hz,
        retry_policy="one bounded position retry; do not increase",
        single_transaction_p99_us=transaction_p99_us,
        single_transaction_budget_us=transaction_budget_us,
        formulas={
            "node_quiet": "ceil(max motor 0x05 p99.9 + 100 us), then preserve response<=quiet",
            "response_timeout": "max(position/temperature p99.9) + 200 us",
            "branch_b": "single-outstanding slot = floor(1e6/(position_hz*7)) us",
        },
        current_configuration={
            "can_node_quiet_us": current_node_quiet_us,
            "can_response_timeout_us": current_response_timeout_us,
            "can_position_hz_per_node": current_position_hz_per_node,
        },
        a9=a9,
        runtime=runtime,
        failures=unique_failures,
    )


def decision_as_dict(value: CanR5Decision) -> dict[str, object]:
    return asdict(value)
