from __future__ import annotations

from threading import Event

from dummy_host.scheduler import FixedRateScheduler


def test_scheduler_jitter_history_is_bounded() -> None:
    scheduler = FixedRateScheduler(20, jitter_sample_window=4)
    scheduler._jitter_ns.extend(range(10))
    scheduler._ticks = 10
    stats = scheduler.stats()
    assert stats.ticks == 10
    assert len(scheduler._jitter_ns) == 4
    assert list(scheduler._jitter_ns) == [6, 7, 8, 9]


def test_overrun_preserves_original_deadline_and_records_rebase() -> None:
    times = iter((0, 0, 180_000_000))
    scheduler = FixedRateScheduler(20, clock_ns=lambda: next(times))
    stop = Event()
    ticks = []

    def capture(tick) -> None:
        ticks.append(tick)
        if len(ticks) == 2:
            stop.set()

    scheduler.run_timed(capture, stop)
    assert ticks[1].planned_ns == 50_000_000
    assert ticks[1].actual_start_ns == 180_000_000
    assert ticks[1].missed_periods == 2
    assert ticks[1].next_rebase_deadline_ns == 200_000_000


def test_exactly_one_period_late_counts_one_missed_tick() -> None:
    times = iter((0, 0, 100_000_000))
    scheduler = FixedRateScheduler(20, clock_ns=lambda: next(times))
    stop = Event()
    ticks = []

    def capture(tick) -> None:
        ticks.append(tick)
        if len(ticks) == 2:
            stop.set()

    scheduler.run_timed(capture, stop)
    assert ticks[1].planned_ns == 50_000_000
    assert ticks[1].missed_periods == 1
    assert ticks[1].next_rebase_deadline_ns == 150_000_000
