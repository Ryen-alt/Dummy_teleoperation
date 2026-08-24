from __future__ import annotations

import html
import json
from pathlib import Path
from typing import Iterable

import numpy as np

from .geometry import validate_transform


def _svg_axes(frames: list[tuple[str, np.ndarray]], *, axis_length_m: float) -> str:
    width = 1120
    height = 390
    panel = 340
    margin_x = 25
    margin_y = 25
    projections = ((0, 1, "X", "Y"), (0, 2, "X", "Z"), (1, 2, "Y", "Z"))
    points: list[np.ndarray] = []
    for _, transform in frames:
        origin = transform[:3, 3]
        points.append(origin)
        for axis in range(3):
            points.append(origin + transform[:3, axis] * axis_length_m)
    cloud = np.asarray(points)
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
    ]
    colors = ("#d62728", "#2ca02c", "#1f77b4")
    for panel_index, (u, v, u_name, v_name) in enumerate(projections):
        x0 = margin_x + panel_index * 370
        y0 = margin_y
        values_u = cloud[:, u]
        values_v = cloud[:, v]
        center_u = float((values_u.min() + values_u.max()) / 2.0)
        center_v = float((values_v.min() + values_v.max()) / 2.0)
        span = max(float(np.ptp(values_u)), float(np.ptp(values_v)), axis_length_m * 2.0)
        span *= 1.25

        def project(point: np.ndarray) -> tuple[float, float]:
            px = x0 + panel / 2 + (float(point[u]) - center_u) / span * panel
            py = y0 + panel / 2 - (float(point[v]) - center_v) / span * panel
            return px, py

        elements.append(
            f'<rect x="{x0}" y="{y0}" width="{panel}" height="{panel}" '
            'fill="white" stroke="#bbb"/>'
        )
        elements.append(
            f'<text x="{x0 + 8}" y="{y0 + 18}" font-family="sans-serif" font-size="14">'
            f'{u_name}{v_name} projection (metres)</text>'
        )
        for frame_index, (label, transform) in enumerate(frames):
            origin = transform[:3, 3]
            ox, oy = project(origin)
            for axis in range(3):
                endpoint = origin + transform[:3, axis] * axis_length_m
                ex, ey = project(endpoint)
                elements.append(
                    f'<line x1="{ox:.2f}" y1="{oy:.2f}" x2="{ex:.2f}" y2="{ey:.2f}" '
                    f'stroke="{colors[axis]}" stroke-width="2"/>'
                )
            elements.append(f'<circle cx="{ox:.2f}" cy="{oy:.2f}" r="2.5" fill="#222"/>')
            if frame_index < 12 or label in {"base_link", "camera"}:
                elements.append(
                    f'<text x="{ox + 4:.2f}" y="{oy - 4:.2f}" font-family="sans-serif" '
                    f'font-size="9" fill="#333">{html.escape(label)}</text>'
                )
    elements.append(
        '<text x="25" y="382" font-family="sans-serif" font-size="12">'
        '<tspan fill="#d62728">X red</tspan> · '
        '<tspan fill="#2ca02c">Y green</tspan> · '
        '<tspan fill="#1f77b4">Z blue</tspan></text>'
    )
    elements.append("</svg>")
    return "\n".join(elements) + "\n"


def write_axis_visualization(
    frames: Iterable[tuple[str, np.ndarray]],
    output_path: str | Path,
    *,
    axis_length_m: float = 0.05,
) -> str:
    values = [(label, validate_transform(transform)) for label, transform in frames]
    if not values:
        raise ValueError("at least one frame is required for axis visualization")
    if not np.isfinite(axis_length_m) or axis_length_m <= 0:
        raise ValueError("axis_length_m must be positive")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(_svg_axes(values, axis_length_m=axis_length_m), encoding="utf-8")
    return str(output.resolve())


def write_hand_eye_html(report: dict[str, object], output_path: str | Path) -> str:
    output = Path(output_path)
    metrics = report.get("metrics", {})
    rows: list[str] = []
    if isinstance(metrics, dict):
        for split in ("train", "holdout"):
            value = metrics.get(split)
            if isinstance(value, dict):
                rows.append(
                    "<tr>"
                    f"<td>{html.escape(split)}</td>"
                    f"<td>{int(value.get('count', 0))}</td>"
                    f"<td>{float(value.get('translation_mean_mm', 0.0)):.3f}</td>"
                    f"<td>{float(value.get('translation_p95_mm', 0.0)):.3f}</td>"
                    f"<td>{float(value.get('rotation_mean_deg', 0.0)):.3f}</td>"
                    f"<td>{float(value.get('rotation_p95_deg', 0.0)):.3f}</td>"
                    "</tr>"
                )
    axis_file = html.escape(Path(str(report.get("axis_visualization", ""))).name)
    json_text = html.escape(json.dumps(report, indent=2, ensure_ascii=False))
    document = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>Dummy hand-eye calibration report</title>
<style>body{{font-family:sans-serif;max-width:1180px;margin:2rem auto;color:#222}}
table{{border-collapse:collapse}}th,td{{border:1px solid #bbb;padding:.45rem .7rem;text-align:right}}
th:first-child,td:first-child{{text-align:left}}img{{max-width:100%;border:1px solid #ddd}}
pre{{background:#f5f5f5;padding:1rem;overflow:auto}}</style></head>
<body><h1>Dummy hand-eye calibration report</h1>
<p>Mode: <strong>{html.escape(str(report.get('mode', 'unknown')))}</strong> ·
Calibration: <strong>{html.escape(str(report.get('calibration_id', 'unknown')))}</strong></p>
<table><thead><tr><th>split</th><th>poses</th><th>translation mean (mm)</th>
<th>translation p95 (mm)</th><th>rotation mean (deg)</th><th>rotation p95 (deg)</th>
</tr></thead><tbody>{''.join(rows)}</tbody></table>
<h2>Coordinate axes</h2><img src="{axis_file}" alt="coordinate-axis projections">
<h2>Machine-readable report</h2><pre>{json_text}</pre></body></html>
"""
    output.write_text(document, encoding="utf-8")
    return str(output.resolve())
