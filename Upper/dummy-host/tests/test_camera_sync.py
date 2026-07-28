from __future__ import annotations

from dummy_host.cameras import DeviceClockMapper


def test_device_clock_mapper_uses_monotonic_anchor() -> None:
    mapper = DeviceClockMapper()
    assert mapper.map(1000.0, 10_000_000_000) == 10_000_000_000
    assert mapper.map(1010.0, 10_012_000_000) == 10_010_000_000
    # A device clock reset starts a new host monotonic epoch.
    assert mapper.map(2.0, 11_000_000_000) == 11_000_000_000
