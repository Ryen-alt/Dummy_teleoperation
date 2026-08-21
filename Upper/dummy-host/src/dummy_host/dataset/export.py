from __future__ import annotations

import hashlib
from itertools import chain

from .contracts import DatasetSink, ExportRecipe, ExportReport
from .raw_session import RawSession


def export_raw_session(
    session_dir,
    recipe: ExportRecipe,
    sink: DatasetSink,
) -> ExportReport:
    session = RawSession(session_dir)
    episodes = session.episodes()
    selected = [episode for episode in episodes if episode.outcome in recipe.accepted_outcomes]
    episodes_exported = 0
    frames_exported = 0
    invalid_excluded = 0
    for episode in selected:
        total_samples, _ = session.sample_counts(episode)
        episode_frames = 0
        frames = session.iter_frames(episode, recipe)
        try:
            first_frame = next(frames)
        except StopIteration:
            invalid_excluded += total_samples
            continue
        sink.begin_episode(
            episode_id=episode.episode_id,
            task_id=episode.task_id,
            task=episode.task,
        )
        for frame in chain((first_frame,), frames):
            sink.add_frame(frame)
            frames_exported += 1
            episode_frames += 1
        sink.end_episode(episode_id=episode.episode_id)
        episodes_exported += 1
        invalid_excluded += total_samples - episode_frames
    metadata = {
        "dataset_format": recipe.dataset_format,
        "export_recipe_id": recipe.recipe_id,
        "export_recipe_version": recipe.version,
        "export_recipe_hash": recipe.config_hash,
        "source_session": str(session.session_dir.resolve()),
        "source_checksums_sha256": hashlib.sha256(
            (session.session_dir / "checksums.json").read_bytes()
        ).hexdigest(),
        "raw_session_schema_version": session.manifest.get("schema_version"),
        "robot_config_hash": session.manifest.get("robot_config_hash"),
        "robot_calibration_id": session.manifest.get("robot_calibration_id"),
        "camera_rig_id": session.manifest.get("camera_rig_id"),
        "camera_rig_version": session.manifest.get("camera_rig_version"),
        "camera_rig_hash": session.manifest.get("camera_rig_hash"),
        "camera_calibrations": session.manifest.get("camera_calibrations", {}),
    }
    result = sink.finalize(metadata=metadata)
    return ExportReport(
        source_session=str(session.session_dir.resolve()),
        recipe_hash=recipe.config_hash,
        episodes_exported=episodes_exported,
        frames_exported=frames_exported,
        invalid_samples_excluded=invalid_excluded,
        incomplete_episodes_excluded=len(episodes) - episodes_exported,
        sink_result=result,
    )
