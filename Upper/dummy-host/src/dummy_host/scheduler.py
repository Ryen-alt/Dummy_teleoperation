from __future__ import annotations

import statistics
import time
from dataclasses import dataclass
from threading import Event
from typing import Callable


@dataclass(frozen=True)
class SchedulerStats:
    ticks: int
    overruns: int
    mean_jitter_ms: float
    max_abs_jitter_ms: float


class FixedRateScheduler:
    def __init__(self, rate_hz: int, clock_ns: Callable[[], int] = time.monotonic_ns) -> None:
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self.period_ns = int(1e9 / rate_hz)
        self.clock_ns = clock_ns
        self._jitter_ns: list[int] = []
        self._overruns = 0

    def run(self, callback: Callable[[int], None], stop: Event) -> SchedulerStats:
        deadline = self.clock_ns()
        while not stop.is_set():
            now = self.clock_ns()
            wait_ns = deadline - now
            if wait_ns > 0:
                stop.wait(wait_ns / 1e9)
                if stop.is_set():
                    break
                now = self.clock_ns()
            jitter = now - deadline
            self._jitter_ns.append(jitter)
            if jitter > self.period_ns:
                self._overruns += 1
                deadline = now
            callback(now)
            deadline += self.period_ns
        return self.stats()

    def stats(self) -> SchedulerStats:
        if not self._jitter_ns:
            return SchedulerStats(0, self._overruns, 0.0, 0.0)
        return SchedulerStats(
            len(self._jitter_ns),
            self._overruns,
            statistics.fmean(self._jitter_ns) / 1e6,
            max(abs(value) for value in self._jitter_ns) / 1e6,
        )
