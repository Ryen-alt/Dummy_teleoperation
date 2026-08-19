from __future__ import annotations

import json

import numpy as np

from dummy_host.dataset import DatasetFrame
from lerobot_robot_dummy import LeRobotV3DatasetSink


class FakeLeRobotDataset:
    def __init__(self, **create_kwargs) -> None:
        self.create_kwargs = create_kwargs
        self.frames: list[dict[str, object]] = []
        self.saved_episodes = 0
        self.finalized = False

    def add_frame(self, frame) -> None:
        self.frames.append(frame)

    def save_episode(self) -> None:
        self.saved_episodes += 1

    def finalize(self) -> None:
        self.finalized = True


def test_sink_maps_contract_to_pinned_lerobot_v3_api(tmp_path) -> None:
    created: list[FakeLeRobotDataset] = []

    def factory(**kwargs):
        dataset = FakeLeRobotDataset(**kwargs)
        created.append(dataset)
        return dataset

    sink = LeRobotV3DatasetSink(
        repo_id="local/dummy-test",
        root=tmp_path / "dataset",
        fps=20,
        dataset_factory=factory,
    )
    sink.begin_episode(episode_id="ep-1", task_id="pick", task="Pick the cube")
    sink.add_frame(
        DatasetFrame(
            observation_state=np.zeros(7, dtype=np.float32),
            action=np.ones(7, dtype=np.float32),
            images={"wrist": np.zeros((2, 3, 3), dtype=np.uint8)},
            timestamp_s=0.0,
            frame_index=0,
            episode_id="ep-1",
            task_id="pick",
            task="Pick the cube",
            source_sample_index=12,
            source_tick_ns=123_000,
        )
    )
    sink.end_episode(episode_id="ep-1")
    result = sink.finalize(metadata={"robot_config_hash": "abc"})

    dataset = created[0]
    assert dataset.create_kwargs["use_videos"] is True
    assert dataset.create_kwargs["features"]["observation.images.wrist"]["dtype"] == "video"
    assert dataset.frames[0]["source.sample_index"].tolist() == [12]
    assert "timestamp" not in dataset.frames[0]
    assert dataset.saved_episodes == 1
    assert dataset.finalized
    sidecar = json.loads((tmp_path / "dataset" / "dummy_export_metadata.json").read_text())
    assert sidecar["lerobot_version"] == "0.4.0"
    assert sidecar["source"]["robot_config_hash"] == "abc"
    assert result["repo_id"] == "local/dummy-test"
