from __future__ import annotations

from dummy_host.scheduler import FixedRateScheduler


def test_scheduler_jitter_history_is_bounded() -> None:
    scheduler = FixedRateScheduler(20, jitter_sample_window=4)
    scheduler._jitter_ns.extend(range(10))
    scheduler._ticks = 10
    stats = scheduler.stats()
    assert stats.ticks == 10
    assert len(scheduler._jitter_ns) == 4
    assert list(scheduler._jitter_ns) == [6, 7, 8, 9]
