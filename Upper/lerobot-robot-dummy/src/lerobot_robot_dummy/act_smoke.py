from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from collections.abc import Callable
from dataclasses import asdict, dataclass
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Any

import numpy as np

from .sink import LEROBOT_VERSION

TEMP_CLASSIFICATION = "temporary_uncalibrated_pipeline_test"


class ActSmokeError(RuntimeError):
    pass


@dataclass(frozen=True)
class DatasetAudit:
    repo_id: str
    root: str
    frames: int
    episodes: int
    fps: int
    camera_keys: tuple[str, ...]
    features: tuple[str, ...]
    data_classification: str
    offline_training_only: bool
    real_policy_execution_allowed: bool


def _load_sidecar(root: Path) -> dict[str, Any]:
    path = root / "dummy_export_metadata.json"
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ActSmokeError(f"cannot load dataset provenance sidecar {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ActSmokeError("dataset provenance sidecar must be a JSON object")
    return value


def _require_exact_lerobot() -> None:
    try:
        installed = version("lerobot")
    except PackageNotFoundError as exc:
        raise ActSmokeError("install the isolated lerobot-robot-dummy environment first") from exc
    if installed != LEROBOT_VERSION:
        raise ActSmokeError(f"lerobot {LEROBOT_VERSION} is required, found {installed}")


def audit_local_dataset(
    repo_id: str,
    root: str | Path,
    *,
    require_temporary: bool = True,
    dataset_factory: Callable[..., Any] | None = None,
) -> DatasetAudit:
    dataset_root = Path(root)
    sidecar = _load_sidecar(dataset_root)
    source = sidecar.get("source")
    if not isinstance(source, dict):
        raise ActSmokeError("dataset sidecar is missing source provenance")
    classification = str(source.get("data_classification", "legacy_unspecified"))
    offline_only = source.get("offline_training_only") is True
    real_allowed = source.get("real_policy_execution_allowed") is True
    if require_temporary and (
        classification != TEMP_CLASSIFICATION or not offline_only or real_allowed
    ):
        raise ActSmokeError(
            "ACT TEMP smoke requires a temporary_uncalibrated_pipeline_test dataset "
            "that explicitly forbids real policy execution"
        )
    if dataset_factory is None:
        _require_exact_lerobot()
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise ActSmokeError("cannot import LeRobotDataset") from exc
        dataset_factory = LeRobotDataset
    dataset = dataset_factory(repo_id, root=dataset_root)
    frames = len(dataset)
    if frames <= 0:
        raise ActSmokeError("dataset contains no frames")
    features = dataset.features
    if not isinstance(features, dict):
        raise ActSmokeError("LeRobot dataset features must be a mapping")
    required = {"observation.state", "action"}
    missing = required - features.keys()
    if missing:
        raise ActSmokeError(f"dataset is missing required feature(s): {sorted(missing)}")
    for key in required:
        shape = tuple(features[key].get("shape", ()))
        if shape != (7,):
            raise ActSmokeError(f"{key} must have shape (7,), got {shape}")
    camera_keys = tuple(
        sorted(
            key
            for key, feature in features.items()
            if isinstance(feature, dict) and feature.get("dtype") in {"image", "video"}
        )
    )
    if not camera_keys:
        raise ActSmokeError("ACT dataset requires at least one RGB camera feature")
    image_shapes = {tuple(features[key].get("shape", ())) for key in camera_keys}
    if len(image_shapes) != 1:
        raise ActSmokeError(
            f"LeRobot 0.4.0 ACT requires equal image shapes, got {sorted(image_shapes)}"
        )
    first = dataset[0]
    for key in required:
        value = first[key]
        array = value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)
        if array.shape[-1:] != (7,) or not np.isfinite(array).all():
            raise ActSmokeError(f"first frame {key} is not a finite 7D vector")
    episodes = int(getattr(dataset.meta, "total_episodes", 0))
    fps = int(getattr(dataset.meta, "fps", 0))
    if episodes <= 0 or fps <= 0:
        raise ActSmokeError("dataset metadata has invalid episode count or FPS")
    return DatasetAudit(
        repo_id=repo_id,
        root=str(dataset_root.resolve()),
        frames=frames,
        episodes=episodes,
        fps=fps,
        camera_keys=camera_keys,
        features=tuple(sorted(features)),
        data_classification=classification,
        offline_training_only=offline_only,
        real_policy_execution_allowed=real_allowed,
    )


def build_train_command(
    *,
    repo_id: str,
    dataset_root: str | Path,
    output_dir: str | Path,
    device: str,
    steps: int,
    batch_size: int,
    num_workers: int,
    compact_model: bool,
    executable: str | None = None,
) -> list[str]:
    if steps <= 0 or batch_size <= 0 or num_workers < 0:
        raise ValueError("steps/batch_size must be positive and num_workers non-negative")
    if device not in {"cpu", "cuda", "mps", "xpu"}:
        raise ValueError("device must be cpu, cuda, mps or xpu")
    train = executable or shutil.which("lerobot-train")
    if not train:
        candidate = Path(sys.executable).parent / "lerobot-train"
        if not candidate.is_file():
            raise ActSmokeError("lerobot-train is not installed in the active environment")
        train = str(candidate)
    output = Path(output_dir)
    command = [
        train,
        f"--dataset.repo_id={repo_id}",
        f"--dataset.root={Path(dataset_root).resolve()}",
        "--policy.type=act",
        f"--policy.device={device}",
        "--policy.push_to_hub=false",
        "--policy.pretrained_backbone_weights=null",
        f"--output_dir={output.resolve()}",
        f"--job_name={output.name}",
        f"--steps={steps}",
        f"--batch_size={batch_size}",
        f"--num_workers={num_workers}",
        "--eval_freq=0",
        "--log_freq=1",
        "--save_checkpoint=true",
        f"--save_freq={steps}",
        "--wandb.enable=false",
        "--policy.chunk_size=8",
        "--policy.n_action_steps=8",
    ]
    if compact_model:
        command.extend(
            (
                "--policy.dim_model=128",
                "--policy.n_heads=4",
                "--policy.dim_feedforward=256",
                "--policy.n_encoder_layers=1",
                "--policy.n_decoder_layers=1",
                "--policy.n_vae_encoder_layers=1",
            )
        )
    return command


def find_pretrained_model(output_dir: str | Path) -> Path:
    output = Path(output_dir)
    preferred = output / "checkpoints" / "last" / "pretrained_model"
    if preferred.is_dir():
        return preferred.resolve()
    candidates = sorted(output.glob("checkpoints/*/pretrained_model"))
    if not candidates:
        raise ActSmokeError(f"no pretrained_model checkpoint found under {output}")
    return candidates[-1].resolve()


def offline_inference_check(
    *,
    repo_id: str,
    dataset_root: str | Path,
    checkpoint: str | Path,
    device: str,
) -> dict[str, object]:
    _require_exact_lerobot()
    try:
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors
    except ImportError as exc:
        raise ActSmokeError(f"cannot import LeRobot inference API: {exc}") from exc
    checkpoint_path = Path(checkpoint)
    dataset = LeRobotDataset(repo_id, root=Path(dataset_root))
    config = PreTrainedConfig.from_pretrained(checkpoint_path)
    config.device = device
    policy_class = get_policy_class(config.type)
    policy = policy_class.from_pretrained(checkpoint_path, config=config)
    policy.to(device)
    policy.eval()
    policy.reset()
    preprocessor, postprocessor = make_pre_post_processors(
        config,
        pretrained_path=str(checkpoint_path),
        preprocessor_overrides={"device_processor": {"device": device}},
    )
    sample = dataset[0]
    batch = {
        key: value.unsqueeze(0) if hasattr(value, "unsqueeze") else value
        for key, value in sample.items()
        if key.startswith("observation.")
    }
    with torch.no_grad():
        processed = preprocessor(batch)
        action = policy.select_action(processed)
        action = postprocessor(action)
    action_array = action.detach().cpu().numpy() if hasattr(action, "detach") else np.asarray(action)
    if action_array.shape[-1:] != (7,) or not np.isfinite(action_array).all():
        raise ActSmokeError(f"checkpoint returned an invalid action shape/value: {action_array.shape}")
    return {
        "checkpoint": str(checkpoint_path.resolve()),
        "device": device,
        "action_shape": list(action_array.shape),
        "action": action_array.reshape(-1, 7)[0].tolist(),
        "finite": True,
        "real_policy_execution_allowed": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Audit a TEMP LeRobot dataset, run a tiny ACT train, and check offline inference"
    )
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda", "mps", "xpu"), default="auto")
    parser.add_argument("--steps", type=int, default=2)
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--full-model", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        _require_exact_lerobot()
        import torch

        device = args.device
        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        audit = audit_local_dataset(args.repo_id, args.dataset_root)
        command = build_train_command(
            repo_id=args.repo_id,
            dataset_root=args.dataset_root,
            output_dir=args.output_dir,
            device=device,
            steps=args.steps,
            batch_size=args.batch_size,
            num_workers=args.num_workers,
            compact_model=not args.full_model,
        )
        request = {
            "schema_version": 1,
            "data_classification": TEMP_CLASSIFICATION,
            "real_policy_execution_allowed": False,
            "dataset": asdict(audit),
            "train_command": command,
            "dry_run": args.dry_run,
        }
        output = Path(args.output_dir)
        output.parent.mkdir(parents=True, exist_ok=True)
        request_path = output.parent / f"{output.name}.request.json"
        request_path.write_text(
            json.dumps(request, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        if args.dry_run:
            print(json.dumps({**request, "request": str(request_path.resolve())}, indent=2))
            return
        if output.exists():
            raise ActSmokeError(f"output directory already exists: {output}")
        subprocess.run(command, check=True)
        checkpoint = find_pretrained_model(output)
        inference = offline_inference_check(
            repo_id=args.repo_id,
            dataset_root=args.dataset_root,
            checkpoint=checkpoint,
            device=device,
        )
        report = {
            **request,
            "request": str(request_path.resolve()),
            "checkpoint": str(checkpoint),
            "offline_inference": inference,
        }
        report_path = output / "dummy_temp_act_smoke.json"
        report_path.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print(json.dumps({"ok": True, "report": str(report_path.resolve()), **inference}, indent=2))
    except (ActSmokeError, OSError, subprocess.CalledProcessError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
