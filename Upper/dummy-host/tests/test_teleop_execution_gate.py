from __future__ import annotations

from dataclasses import replace

import pytest

from dummy_host.apps.teleop_collect import (
    ACCEPTANCE_RISK_ACKNOWLEDGEMENT,
    validate_execution_authority,
)
from dummy_host.schema import ConfigError


def test_acceptance_authority_requires_explicit_session_and_ack(config) -> None:
    with pytest.raises(ConfigError, match="--acceptance-session"):
        validate_execution_authority(
            config,
            execute=True,
            acceptance_session=False,
            risk_acknowledgement=None,
        )
    with pytest.raises(ConfigError, match="--acknowledge-real-risk"):
        validate_execution_authority(
            config,
            execute=True,
            acceptance_session=True,
            risk_acknowledgement="wrong",
        )

    authority = validate_execution_authority(
        config,
        execute=True,
        acceptance_session=True,
        risk_acknowledgement=ACCEPTANCE_RISK_ACKNOWLEDGEMENT,
    )
    assert authority == "acceptance_teleop"


def test_production_authority_rejects_acceptance_options(config) -> None:
    production = replace(
        config,
        external_target_execution_ready=True,
        external_target_acceptance_ready=False,
    )
    assert (
        validate_execution_authority(
            production,
            execute=True,
            acceptance_session=False,
            risk_acknowledgement=None,
        )
        == "production_teleop"
    )
    with pytest.raises(ConfigError, match="invalid for a production-ready"):
        validate_execution_authority(
            production,
            execute=True,
            acceptance_session=True,
            risk_acknowledgement=ACCEPTANCE_RISK_ACKNOWLEDGEMENT,
        )


def test_simulation_rejects_real_acceptance_options(config) -> None:
    assert (
        validate_execution_authority(
            config,
            execute=False,
            acceptance_session=False,
            risk_acknowledgement=None,
        )
        == "simulation"
    )
    with pytest.raises(ConfigError, match="require real execution"):
        validate_execution_authority(
            config,
            execute=False,
            acceptance_session=True,
            risk_acknowledgement=ACCEPTANCE_RISK_ACKNOWLEDGEMENT,
        )
