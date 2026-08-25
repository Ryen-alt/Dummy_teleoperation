from __future__ import annotations

import argparse
import json
import platform
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from dummy_host.kinematics import DummyUrdfKinematics
from dummy_host.schema import load_robot_config
from dummy_host.teleop import load_teleop_profile, validate_profile_for_robot


def _percentiles(values_ms: list[float]) -> dict[str, float | None]:
    if not values_ms:
        return {"p50_ms": None, "p95_ms": None, "p99_ms": None, "max_ms": None}
    values = np.asarray(values_ms, dtype=np.float64)
    return {
        "p50_ms": float(np.percentile(values, 50)),
        "p95_ms": float(np.percentile(values, 95)),
        "p99_ms": float(np.percentile(values, 99)),
        "max_ms": float(np.max(values)),
    }


def run_benchmark(
    *,
    config_path: str | Path,
    input_config_path: str | Path,
    urdf_path: str | Path,
    samples: int,
    random_seed: int,
    stress_fraction: float = 0.2,
) -> dict[str, object]:
    if samples <= 0:
        raise ValueError("samples must be positive")
    if not 0.0 <= stress_fraction <= 1.0:
        raise ValueError("stress_fraction must be within [0, 1]")
    config = load_robot_config(config_path)
    profile = load_teleop_profile(input_config_path)
    validate_profile_for_robot(profile, config)
    cartesian = profile.cartesian
    if cartesian is None:
        raise ValueError("input configuration has no Cartesian profile")
    backend = DummyUrdfKinematics(
        urdf_path,
        joint_min_rad=config.joint_limit_min_rad,
        joint_max_rad=config.joint_limit_max_rad,
        joint_limit_margin_rad=cartesian.joint_limit_margin_rad,
        position_tolerance_m=cartesian.position_tolerance_m,
        orientation_tolerance_rad=cartesian.orientation_tolerance_rad,
        max_iterations=cartesian.max_iterations,
        damping=cartesian.damping,
        finite_difference_rad=cartesian.finite_difference_rad,
        max_solver_step_rad=cartesian.max_solver_step_rad,
        max_solution_step_rad=cartesian.max_solution_step_rad,
        translation_scale_m=cartesian.translation_scale_m,
    )
    rng = np.random.default_rng(random_seed)
    span = config.joint_limit_max_rad - config.joint_limit_min_rad
    lower = config.joint_limit_min_rad + span * 0.05
    upper = config.joint_limit_max_rad - span * 0.05
    hard_budget_ns = int(cartesian.hard_budget_ms * 1_000_000)
    all_durations_ms: list[float] = []
    success_durations_ms: list[float] = []
    failures: list[dict[str, object]] = []
    stress_samples = 0
    for index in range(samples):
        target_joint = rng.uniform(lower, upper)
        stress = bool(rng.random() < stress_fraction)
        if stress:
            measured_joint = rng.uniform(lower, upper)
            stress_samples += 1
        else:
            measured_joint = np.clip(
                target_joint + rng.normal(0.0, 0.02, size=6), lower, upper
            )
        result = backend.inverse(
            backend.forward(target_joint),
            measured_joint,
            measured_joint,
            hard_budget_ns=hard_budget_ns,
        )
        duration_ms = result.solve_duration_ns / 1e6
        all_durations_ms.append(duration_ms)
        if result.success:
            success_durations_ms.append(duration_ms)
        else:
            failures.append(
                {
                    "sample_index": index,
                    "scenario": "independent_seed_stress" if stress else "local_seed",
                    "solve_duration_ms": duration_ms,
                    "failure_reason": result.failure_reason,
                    "timed_out": result.timed_out,
                    "timeout_stage": result.timeout_stage,
                    "iterations": result.iterations,
                }
            )
    failures.sort(key=lambda value: float(value["solve_duration_ms"]), reverse=True)
    maximum_duration_ms = max(all_durations_ms)
    return {
        "benchmark_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "host": platform.platform(),
        "python": platform.python_version(),
        "samples": samples,
        "random_seed": random_seed,
        "stress_fraction": stress_fraction,
        "stress_samples": stress_samples,
        "soft_budget_ms": cartesian.soft_budget_ms,
        "hard_budget_ms": cartesian.hard_budget_ms,
        "hard_timeout_processing_target_ms": 25.0,
        "hard_timeout_processing_target_met": maximum_duration_ms <= 25.0,
        "successes": samples - len(failures),
        "failures": len(failures),
        "timeouts": sum(bool(value["timed_out"]) for value in failures),
        "all_solves": _percentiles(all_durations_ms),
        "successful_solves": _percentiles(success_durations_ms),
        "failure_tail": failures[:20],
        "kinematics": backend.describe(),
        "robot_config_hash": config.config_hash,
        "teleop_config_hash": profile.config_hash,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark Cartesian URDF IK latency with the configured hard budget"
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--input-config", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--samples", type=int, default=300)
    parser.add_argument("--seed", type=int, default=20260825)
    parser.add_argument("--stress-fraction", type=float, default=0.2)
    parser.add_argument("--output")
    args = parser.parse_args()
    report = run_benchmark(
        config_path=args.config,
        input_config_path=args.input_config,
        urdf_path=args.urdf,
        samples=args.samples,
        random_seed=args.seed,
        stress_fraction=args.stress_fraction,
    )
    rendered = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
