from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class TimeSyncExchange:
    host_t0_ns: int
    mcu_rx_us: int
    mcu_tx_us: int
    host_t3_ns: int

    def __post_init__(self) -> None:
        if min(self.host_t0_ns, self.mcu_rx_us, self.mcu_tx_us, self.host_t3_ns) < 0:
            raise ValueError("time-sync timestamps must be non-negative")
        if self.host_t3_ns < self.host_t0_ns or self.mcu_tx_us < self.mcu_rx_us:
            raise ValueError("time-sync timestamps must be ordered")

    @property
    def rtt_ns(self) -> int:
        remote_work_ns = (self.mcu_tx_us - self.mcu_rx_us) * 1_000
        return max(0, self.host_t3_ns - self.host_t0_ns - remote_work_ns)

    @property
    def mcu_mid_us(self) -> float:
        return (self.mcu_rx_us + self.mcu_tx_us) / 2.0

    @property
    def host_mid_ns(self) -> float:
        return (self.host_t0_ns + self.host_t3_ns) / 2.0


@dataclass(frozen=True)
class TimeSyncModel:
    model_id: int
    segment_id: int
    slope_ns_per_us: float
    intercept_ns: float
    rtt_ns: int
    residual_ns: float
    sample_count: int
    created_host_ns: int

    def __post_init__(self) -> None:
        if (
            self.model_id <= 0
            or self.segment_id <= 0
            or self.rtt_ns < 0
            or self.residual_ns < 0
            or self.sample_count < 2
            or self.created_host_ns < 0
            or not math.isfinite(self.slope_ns_per_us)
            or not math.isfinite(self.intercept_ns)
            or not math.isfinite(self.residual_ns)
            or self.slope_ns_per_us <= 0
        ):
            raise ValueError("invalid affine time-sync model")

    def host_time_ns(self, mcu_time_us: int) -> int:
        if mcu_time_us < 0:
            raise ValueError("MCU timestamp must be non-negative")
        return round(self.slope_ns_per_us * mcu_time_us + self.intercept_ns)


class AffineTimeSyncEstimator:
    """Fits MCU microseconds to host monotonic nanoseconds from four timestamps."""

    def __init__(
        self,
        *,
        window_size: int = 32,
        minimum_samples: int = 3,
        maximum_rtt_ns: int = 20_000_000,
        rtt_median_factor: float = 1.5,
        maximum_model_jump_ns: int = 10_000_000,
    ) -> None:
        if window_size < minimum_samples or minimum_samples < 2:
            raise ValueError("time-sync fitting window is too small")
        if (
            maximum_rtt_ns <= 0
            or rtt_median_factor < 1.0
            or maximum_model_jump_ns <= 0
        ):
            raise ValueError("time-sync RTT filter is invalid")
        self._samples: deque[TimeSyncExchange] = deque(maxlen=window_size)
        self._minimum_samples = minimum_samples
        self._maximum_rtt_ns = maximum_rtt_ns
        self._rtt_median_factor = rtt_median_factor
        self._maximum_model_jump_ns = maximum_model_jump_ns
        self._next_model_id = 1
        self._segment_id = 1
        self._last_mcu_us: float | None = None
        self._last_host_ns: float | None = None
        self._last_model: TimeSyncModel | None = None

    @property
    def segment_id(self) -> int:
        return self._segment_id

    def observe(self, exchange: TimeSyncExchange) -> TimeSyncModel | None:
        mcu_mid = exchange.mcu_mid_us
        host_mid = exchange.host_mid_ns
        if (
            (self._last_mcu_us is not None and mcu_mid < self._last_mcu_us)
            or (self._last_host_ns is not None and host_mid < self._last_host_ns)
        ):
            self._samples.clear()
            self._segment_id += 1
            self._last_model = None
        self._last_mcu_us = mcu_mid
        self._last_host_ns = host_mid
        self._samples.append(exchange)
        if len(self._samples) < self._minimum_samples:
            return None

        ordered_rtt = sorted(item.rtt_ns for item in self._samples)
        median_rtt = ordered_rtt[len(ordered_rtt) // 2]
        rtt_limit = min(
            self._maximum_rtt_ns,
            max(1, round(median_rtt * self._rtt_median_factor)),
        )
        retained = [item for item in self._samples if item.rtt_ns <= rtt_limit]
        if len(retained) < self._minimum_samples:
            return None

        x_mean = sum(item.mcu_mid_us for item in retained) / len(retained)
        y_mean = sum(item.host_mid_ns for item in retained) / len(retained)
        denominator = sum((item.mcu_mid_us - x_mean) ** 2 for item in retained)
        if denominator <= 0:
            return None
        slope = sum(
            (item.mcu_mid_us - x_mean) * (item.host_mid_ns - y_mean)
            for item in retained
        ) / denominator
        if not math.isfinite(slope) or slope <= 0:
            return None
        intercept = y_mean - slope * x_mean
        residual = math.sqrt(
            sum(
                (
                    item.host_mid_ns
                    - (slope * item.mcu_mid_us + intercept)
                )
                ** 2
                for item in retained
            )
            / len(retained)
        )
        if self._last_model is not None:
            old_prediction = (
                self._last_model.slope_ns_per_us * mcu_mid
                + self._last_model.intercept_ns
            )
            new_prediction = slope * mcu_mid + intercept
            if abs(new_prediction - old_prediction) > self._maximum_model_jump_ns:
                self._samples.clear()
                self._samples.append(exchange)
                self._segment_id += 1
                self._last_model = None
                return None
        model = TimeSyncModel(
            model_id=self._next_model_id,
            segment_id=self._segment_id,
            slope_ns_per_us=slope,
            intercept_ns=intercept,
            rtt_ns=exchange.rtt_ns,
            residual_ns=residual,
            sample_count=len(retained),
            created_host_ns=exchange.host_t3_ns,
        )
        self._next_model_id += 1
        self._last_model = model
        return model
