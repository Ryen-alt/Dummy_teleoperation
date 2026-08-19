from __future__ import annotations

import pytest

from dummy_host.domain import EpisodeError, EpisodeManager, EpisodeStatus


def test_episode_manager_supports_multiple_explicit_episodes() -> None:
    ids = iter(("episode-a", "episode-b"))
    manager = EpisodeManager(id_factory=lambda: next(ids))
    first = manager.begin(task_id="pick", task="Pick the cube", now_ns=10)
    assert first.episode_id == "episode-a"
    assert first.status is EpisodeStatus.RECORDING
    accepted = manager.finish(EpisodeStatus.ACCEPTED, now_ns=20)
    assert accepted.ended_ns == 20

    second = manager.begin(task_id="pick", task="Pick the cube", now_ns=30)
    assert second.episode_id == "episode-b"
    failed = manager.finish(EpisodeStatus.FAILED, now_ns=40, failure_reason="dropped")
    assert failed.failure_reason == "dropped"


def test_episode_manager_rejects_invalid_transitions() -> None:
    manager = EpisodeManager()
    with pytest.raises(EpisodeError, match="armed"):
        manager.start(now_ns=1)
    manager.begin(task_id="pick", task="Pick", now_ns=2)
    with pytest.raises(EpisodeError, match="cannot arm"):
        manager.arm(task_id="other", task="Other", now_ns=3)
