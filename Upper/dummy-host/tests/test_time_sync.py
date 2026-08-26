from __future__ import annotations

import pytest

from dummy_host.time_sync import AffineTimeSyncEstimator, TimeSyncExchange


def _exchange(mcu_us: int, *, delay_ns: int = 1_000_000) -> TimeSyncExchange:
    host_mid_ns = 7_000_000_000 + mcu_us * 1_000
    return TimeSyncExchange(
        host_mid_ns - delay_ns // 2,
        mcu_us,
        mcu_us,
        host_mid_ns + delay_ns // 2,
    )


def test_affine_time_sync_filters_high_rtt_and_versions_each_model() -> None:
    estimator = AffineTimeSyncEstimator(minimum_samples=3)
    assert estimator.observe(_exchange(1_000)) is None
    assert estimator.observe(_exchange(2_000)) is None
    first = estimator.observe(_exchange(3_000))
    assert first is not None
    assert first.model_id == 1
    assert first.segment_id == 1
    assert first.slope_ns_per_us == pytest.approx(1_000.0)
    assert first.host_time_ns(4_000) == 7_004_000_000

    second = estimator.observe(_exchange(4_000, delay_ns=100_000_000))
    assert second is not None
    assert second.model_id == 2
    assert second.sample_count == 3
    assert second.slope_ns_per_us == pytest.approx(1_000.0)


def test_affine_time_sync_starts_a_new_segment_after_mcu_clock_reset() -> None:
    estimator = AffineTimeSyncEstimator(minimum_samples=2)
    estimator.observe(_exchange(10_000))
    first = estimator.observe(_exchange(11_000))
    assert first is not None and first.segment_id == 1
    assert estimator.observe(_exchange(100)) is None
    second = estimator.observe(_exchange(200))
    assert second is not None
    assert second.segment_id == 2
    assert second.model_id == first.model_id + 1
