from __future__ import annotations

from types import SimpleNamespace

import pytest

from dummy_host.apps import camera_check
from dummy_host.cameras import CameraError


class StartupCameraManager:
    def __init__(self, *, failures: int | None, capture_error: str | None = None) -> None:
        self.failures = failures
        self.capture_error = capture_error
        self.calls = 0
        self.frame = object()

    def latest_all(self):
        self.calls += 1
        if self.failures is None or self.calls <= self.failures:
            raise CameraError("wrist: no D435 frame is available")
        return {"wrist": self.frame}

    def stats(self):
        return {"wrist": SimpleNamespace(last_error=self.capture_error)}


@pytest.fixture
def fake_clock(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(camera_check.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(
        camera_check.time,
        "sleep",
        lambda seconds: now.__setitem__(0, now[0] + seconds),
    )
    return now


def test_wait_for_first_frames_retries_normal_camera_warmup(fake_clock) -> None:
    cameras = StartupCameraManager(failures=2)

    frames = camera_check.wait_for_first_frames(
        cameras,  # type: ignore[arg-type]
        timeout_s=1.0,
        poll_interval_s=0.1,
    )

    assert frames == {"wrist": cameras.frame}
    assert cameras.calls == 3
    assert fake_clock[0] == pytest.approx(0.2)


def test_wait_for_first_frames_reports_timeout(fake_clock) -> None:
    cameras = StartupCameraManager(failures=None)

    with pytest.raises(CameraError, match="timed out after 0.3s"):
        camera_check.wait_for_first_frames(
            cameras,  # type: ignore[arg-type]
            timeout_s=0.3,
            poll_interval_s=0.1,
        )


def test_wait_for_first_frames_surfaces_capture_thread_error(fake_clock) -> None:
    cameras = StartupCameraManager(
        failures=None,
        capture_error="Frame didn't arrive within 1000",
    )

    with pytest.raises(
        CameraError,
        match="camera capture stopped.*Frame didn't arrive within 1000",
    ):
        camera_check.wait_for_first_frames(
            cameras,  # type: ignore[arg-type]
            timeout_s=1.0,
        )
