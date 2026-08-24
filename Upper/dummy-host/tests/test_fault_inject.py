from __future__ import annotations

from dummy_host.apps.fault_inject import SCENARIOS, run_fault_injection


def test_all_fault_injection_scenarios_pass_against_fake_mcu(config) -> None:
    report = run_fault_injection(config, SCENARIOS)
    assert report["source"] == "fake"
    assert report["passed"] is True
    assert set(report["scenarios"]) == set(SCENARIOS)
    assert all(result["passed"] for result in report["scenarios"].values())
