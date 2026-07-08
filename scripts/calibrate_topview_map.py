#!/usr/bin/env python3
"""Build a top-view image alignment from pixel/map control points."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


def affine_from_points(points: list[dict]):
    if len(points) < 3:
        raise ValueError("at least 3 control points are required for affine calibration")

    rows = []
    targets = []
    for point in points:
        px, py = point["pixel"]
        mx, my = point["map"]
        rows.append([float(px), float(py), 1.0, 0.0, 0.0, 0.0])
        rows.append([0.0, 0.0, 0.0, float(px), float(py), 1.0])
        targets.extend([float(mx), float(my)])

    solution, *_ = np.linalg.lstsq(np.array(rows), np.array(targets), rcond=None)
    a, b, c, d, e, f = [float(value) for value in solution]
    return [[a, b, c], [d, e, f]]


def similarity_from_points(points: list[dict]):
    if len(points) < 2:
        raise ValueError("at least 2 control points are required for similarity calibration")

    pixels = np.array([[float(point["pixel"][0]), float(point["pixel"][1])] for point in points])
    maps = np.array([[float(point["map"][0]), float(point["map"][1])] for point in points])
    p_mean = pixels.mean(axis=0)
    m_mean = maps.mean(axis=0)
    p_centered = pixels - p_mean
    m_centered = maps - m_mean
    denom = float((p_centered * p_centered).sum())
    if denom <= 1e-12:
        raise ValueError("control point pixels are degenerate")

    # Solve map ~= s * R * pixel + t with a proper rotation and uniform scale.
    covariance = p_centered.T @ m_centered
    u, singular_values, vt = np.linalg.svd(covariance)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = u @ vt
    scale = float(singular_values.sum() / denom)
    transform = scale * rotation
    translation = m_mean - p_mean @ transform
    a, d = transform[0, 0], transform[0, 1]
    b, e = transform[1, 0], transform[1, 1]
    c, f = translation[0], translation[1]
    return [[float(a), float(b), float(c)], [float(d), float(e), float(f)]]


def invert_affine(matrix: list[list[float]]):
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    det = a * e - b * d
    if abs(det) < 1e-12:
        raise ValueError("affine matrix is singular")
    inv_a = e / det
    inv_b = -b / det
    inv_d = -d / det
    inv_e = a / det
    inv_c = -(inv_a * c + inv_b * f)
    inv_f = -(inv_d * c + inv_e * f)
    return [[inv_a, inv_b, inv_c], [inv_d, inv_e, inv_f]]


def apply_affine(matrix: list[list[float]], x: float, y: float):
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2],
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2],
    )


def error_stats(points: list[dict], matrix: list[list[float]]):
    errors = []
    residuals = []
    for point in points:
        px, py = point["pixel"]
        mx, my = point["map"]
        pred_x, pred_y = apply_affine(matrix, float(px), float(py))
        error_m = math.hypot(pred_x - float(mx), pred_y - float(my))
        errors.append(error_m)
        residuals.append(
            {
                "name": point.get("name", ""),
                "pixel": [px, py],
                "map": [mx, my],
                "predicted_map": [pred_x, pred_y],
                "error_m": error_m,
            }
        )
    rms = math.sqrt(sum(error * error for error in errors) / len(errors)) if errors else 0.0
    return {
        "rms_error_m": rms,
        "max_error_m": max(errors) if errors else 0.0,
        "mean_error_m": sum(errors) / len(errors) if errors else 0.0,
        "residuals": residuals,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-points", type=Path, required=True)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("recordings/maps/topview_alignment.json"))
    parser.add_argument("--name", default="topview_alignment")
    parser.add_argument(
        "--method",
        choices=("similarity", "affine"),
        default="similarity",
        help="similarity avoids image shear; affine can skew/distort the photo",
    )
    args = parser.parse_args()

    points = json.loads(args.control_points.expanduser().read_text(encoding="utf-8"))
    if isinstance(points, dict):
        points = points.get("control_points", [])
    matrix = affine_from_points(points) if args.method == "affine" else similarity_from_points(points)
    inverse = invert_affine(matrix)
    stats = error_stats(points, matrix)

    with Image.open(args.image.expanduser()) as image:
        width, height = image.size

    payload = {
        "schema": "aerion_topview_alignment_v1",
        "name": args.name,
        "image": str(args.image),
        "image_size": {"width": width, "height": height},
        "method": args.method,
        "pixel_to_map": matrix,
        "map_to_pixel": inverse,
        "rms_error_m": stats["rms_error_m"],
        "max_error_m": stats["max_error_m"],
        "mean_error_m": stats["mean_error_m"],
        "control_points": points,
        "residuals": stats["residuals"],
    }

    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(
        f"Wrote {output.resolve()} "
        f"(method={args.method}, points={len(points)}, rms={payload['rms_error_m']:.2f}m, max={payload['max_error_m']:.2f}m)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
