from __future__ import annotations

import statistics
import time
from collections import deque
from dataclasses import dataclass
from threading import Event
from typing import Callable


@dataclass(frozen=True)
class SchedulerStats:
    ticks: int
    overruns: int
    mean_jitter_ms: float
    max_abs_jitter_ms: float
    p99_abs_jitter_ms: float = 0.0


class FixedRateScheduler:
    def __init__(
        self,
        rate_hz: int,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        *,
        jitter_sample_window: int = 10_000,
    ) -> None:
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        if jitter_sample_window <= 0:
            raise ValueError("jitter_sample_window must be positive")
        self.period_ns = int(1e9 / rate_hz)
        self.clock_ns = clock_ns
        self._jitter_ns: deque[int] = deque(maxlen=jitter_sample_window)
        self._ticks = 0
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
            self._ticks += 1
            if jitter > self.period_ns:
                self._overruns += 1
                deadline = now
            callback(now)
            deadline += self.period_ns
        return self.stats()

    def stats(self) -> SchedulerStats:
        if not self._jitter_ns:
            return SchedulerStats(0, self._overruns, 0.0, 0.0, 0.0)
        absolute = sorted(abs(value) for value in self._jitter_ns)
        p99_index = max(0, min(len(absolute) - 1, int(len(absolute) * 0.99)))
        return SchedulerStats(
            self._ticks,
            self._overruns,
            statistics.fmean(self._jitter_ns) / 1e6,
            absolute[-1] / 1e6,
            absolute[p99_index] / 1e6,
        )
