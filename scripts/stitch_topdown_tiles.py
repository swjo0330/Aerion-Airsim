#!/usr/bin/env python3
"""Stitch overscanned top-down tiles using conservative overlap compositing."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


def read_rgb(path: Path) -> np.ndarray:
    return np.asarray(Image.open(path).convert("RGB"), dtype=np.float32) / 255.0


def to_gray(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor((image * 255).astype(np.uint8), cv2.COLOR_RGB2GRAY)
    return gray.astype(np.float32)


def estimate_translation(a: np.ndarray, b: np.ndarray) -> tuple[float, float, float]:
    ga = to_gray(a)
    gb = to_gray(b)
    window = cv2.createHanningWindow((ga.shape[1], ga.shape[0]), cv2.CV_32F)
    shift, response = cv2.phaseCorrelate(ga, gb, window)
    return float(shift[0]), float(shift[1]), float(response)


def feather_mask(height: int, width: int, feather: int) -> np.ndarray:
    feather = max(1, int(feather))
    y = np.minimum(np.arange(height), np.arange(height)[::-1]).astype(np.float32)
    x = np.minimum(np.arange(width), np.arange(width)[::-1]).astype(np.float32)
    yy, xx = np.meshgrid(y, x, indexing="ij")
    return np.clip(np.minimum(xx, yy) / feather, 0.0, 1.0)


def center_crop_array(image: np.ndarray, side: int) -> np.ndarray:
    height, width = image.shape[:2]
    side = min(side, height, width)
    y0 = (height - side) // 2
    x0 = (width - side) // 2
    return image[y0 : y0 + side, x0 : x0 + side]


def paste(canvas: np.ndarray, weights: np.ndarray, image: np.ndarray, x: int, y: int, mask: np.ndarray) -> None:
    height, width = image.shape[:2]
    canvas_height, canvas_width = canvas.shape[:2]
    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(canvas_width, x + width)
    y1 = min(canvas_height, y + height)
    if x1 <= x0 or y1 <= y0:
        return
    sx0 = x0 - x
    sy0 = y0 - y
    sx1 = sx0 + (x1 - x0)
    sy1 = sy0 + (y1 - y0)
    m = mask[sy0:sy1, sx0:sx1, None]
    canvas[y0:y1, x0:x1] += image[sy0:sy1, sx0:sx1] * m
    weights[y0:y1, x0:x1] += m[:, :, 0]


def clamp_position(value: float, nominal: float, max_adjust: float) -> float:
    return max(nominal - max_adjust, min(nominal + max_adjust, value))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--tile-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata-output", type=Path, default=None)
    parser.add_argument("--max-adjust-px", type=float, default=32.0)
    parser.add_argument("--min-response", type=float, default=0.03)
    parser.add_argument("--feather-px", type=int, default=80)
    parser.add_argument(
        "--mode",
        choices=("full-register", "central-overlap"),
        default="central-overlap",
        help=(
            "central-overlap uses only a narrow central overlap to avoid parallax ghosts; "
            "full-register is an experimental full-tile phase-correlation blend"
        ),
    )
    parser.add_argument(
        "--central-overlap-px",
        type=int,
        default=64,
        help="extra pixels to keep around each logical tile in central-overlap mode",
    )
    args = parser.parse_args()

    meta = json.loads(args.metadata.read_text(encoding="utf-8"))
    rows = int(meta["rows"])
    cols = int(meta["cols"])
    tile_px = int(meta["tile_px"])
    capture_tile_px = int(meta.get("capture_tile_px", tile_px))
    overlap = max(0, capture_tile_px - tile_px)
    margin = overlap // 2
    strip = max(24, overlap)

    tiles: list[list[np.ndarray]] = []
    for row in range(rows):
        tile_row = []
        for col in range(cols):
            path = args.tile_dir / f"tile_r{row:02d}_c{col:02d}.png"
            if not path.exists():
                raise FileNotFoundError(path)
            tile_row.append(read_rgb(path))
        tiles.append(tile_row)

    positions: list[list[tuple[float, float]]] = []
    diagnostics = []
    canvas_width = cols * tile_px
    canvas_height = rows * tile_px

    if args.mode == "central-overlap":
        overlap_px = max(0, min(args.central_overlap_px, capture_tile_px - tile_px))
        crop_side = tile_px + overlap_px
        crop_margin = overlap_px // 2
        canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.float32)
        weights = np.zeros((canvas_height, canvas_width), dtype=np.float32)
        mask = feather_mask(crop_side, crop_side, max(1, min(args.feather_px, max(1, crop_margin))))

        for row in range(rows):
            pos_row: list[tuple[float, float]] = []
            for col in range(cols):
                x = col * tile_px - crop_margin
                y = row * tile_px - crop_margin
                tile = center_crop_array(tiles[row][col], crop_side)
                paste(canvas, weights, tile, x, y, mask)
                pos_row.append((float(x), float(y)))
            positions.append(pos_row)

        output = canvas / np.maximum(weights[:, :, None], 1e-6)
        output = np.clip(output * 255, 0, 255).astype(np.uint8)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(output, "RGB").save(args.output)

        metadata_output = args.metadata_output or args.output.with_suffix(".json")
        metadata = {
            "schema": "aerion_stitched_topdown_mosaic_v1",
            "mode": args.mode,
            "source_metadata": str(args.metadata),
            "tile_dir": str(args.tile_dir),
            "output": str(args.output),
            "bounds": meta["bounds"],
            "width": canvas_width,
            "height": canvas_height,
            "rows": rows,
            "cols": cols,
            "tile_px": tile_px,
            "capture_tile_px": capture_tile_px,
            "central_overlap_px": overlap_px,
            "positions": positions,
            "diagnostics": diagnostics,
        }
        metadata_output.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        print(f"Wrote {args.output}")
        print(f"Wrote {metadata_output}")
        print(f"Composited central overlap: {overlap_px}px")
        return 0

    for row in range(rows):
        pos_row: list[tuple[float, float]] = []
        for col in range(cols):
            nominal_x = col * tile_px - margin
            nominal_y = row * tile_px - margin
            if row == 0 and col == 0:
                pos_row.append((nominal_x, nominal_y))
                continue

            candidates = []
            if col > 0:
                left = tiles[row][col - 1]
                tile = tiles[row][col]
                a = left[:, -strip:, :]
                b = tile[:, :strip, :]
                dx, dy, response = estimate_translation(a, b)
                expected_x = pos_row[col - 1][0] + tile_px
                expected_y = pos_row[col - 1][1]
                used = response >= args.min_response and abs(dx) <= args.max_adjust_px and abs(dy) <= args.max_adjust_px
                diagnostics.append({"row": row, "col": col, "from": "left", "dx": dx, "dy": dy, "response": response, "used": used})
                if used:
                    candidates.append((expected_x + dx, expected_y + dy, response))

            if row > 0:
                top = tiles[row - 1][col]
                tile = tiles[row][col]
                a = top[-strip:, :, :]
                b = tile[:strip, :, :]
                dx, dy, response = estimate_translation(a, b)
                expected_x = positions[row - 1][col][0]
                expected_y = positions[row - 1][col][1] + tile_px
                used = response >= args.min_response and abs(dx) <= args.max_adjust_px and abs(dy) <= args.max_adjust_px
                diagnostics.append({"row": row, "col": col, "from": "top", "dx": dx, "dy": dy, "response": response, "used": used})
                if used:
                    candidates.append((expected_x + dx, expected_y + dy, response))

            if candidates:
                weight = sum(max(c[2], 1e-6) for c in candidates)
                x = sum(c[0] * max(c[2], 1e-6) for c in candidates) / weight
                y = sum(c[1] * max(c[2], 1e-6) for c in candidates) / weight
            else:
                x = nominal_x
                y = nominal_y

            pos_row.append(
                (
                    clamp_position(x, nominal_x, args.max_adjust_px),
                    clamp_position(y, nominal_y, args.max_adjust_px),
                )
            )
        positions.append(pos_row)

    canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.float32)
    weights = np.zeros((canvas_height, canvas_width), dtype=np.float32)
    mask = feather_mask(capture_tile_px, capture_tile_px, args.feather_px)

    for row in range(rows):
        for col in range(cols):
            x, y = positions[row][col]
            paste(canvas, weights, tiles[row][col], int(round(x)), int(round(y)), mask)

    output = canvas / np.maximum(weights[:, :, None], 1e-6)
    output = np.clip(output * 255, 0, 255).astype(np.uint8)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(output, "RGB").save(args.output)

    metadata_output = args.metadata_output or args.output.with_suffix(".json")
    metadata = {
        "schema": "aerion_stitched_topdown_mosaic_v1",
        "mode": args.mode,
        "source_metadata": str(args.metadata),
        "tile_dir": str(args.tile_dir),
        "output": str(args.output),
        "bounds": meta["bounds"],
        "width": canvas_width,
        "height": canvas_height,
        "rows": rows,
        "cols": cols,
        "tile_px": tile_px,
        "capture_tile_px": capture_tile_px,
        "overlap_px": overlap,
        "positions": positions,
        "diagnostics": diagnostics,
    }
    metadata_output.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Wrote {args.output}")
    print(f"Wrote {metadata_output}")
    print(f"Registered {sum(1 for d in diagnostics if d.get('used'))} overlap constraints")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
