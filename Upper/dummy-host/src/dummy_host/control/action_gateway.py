from __future__ import annotations

import numpy as np

from ..domain import ActionProposal, ActionSpace, RobotState
from ..safety import SafetyError, SafetyFilter, SafetyResult
from ..schema import RobotConfig


class ActionGateway:
    """Convert a versioned proposal into one canonical, safety-checked action."""

    def __init__(self, config: RobotConfig, safety: SafetyFilter | None = None) -> None:
        self.config = config
        self.safety = SafetyFilter(config) if safety is None else safety

    def reset(self) -> None:
        self.safety.reset()

    def evaluate(
        self,
        proposal: ActionProposal,
        state: RobotState,
        now_ns: int,
        *,
        velocity_limit_rad_s: np.ndarray | None = None,
    ) -> SafetyResult:
        if proposal.action_space is not ActionSpace.JOINT_POSITION_ABSOLUTE:
            raise SafetyError(
                f"ActionGateway requires absolute joint actions, received {proposal.action_space.value}"
            )
        if proposal.values.shape != (7,):
            raise SafetyError("absolute joint action must have shape (7,)")
        if now_ns < proposal.generated_at_ns or now_ns > proposal.valid_until_ns:
            raise SafetyError("action proposal is not valid at the current monotonic time")
        return self.safety.apply(
            proposal.values.copy(),
            state,
            now_ns,
            velocity_limit_rad_s=velocity_limit_rad_s,
        )
