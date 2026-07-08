#!/usr/bin/env python3
"""Warp a map-referenced image into a reference top-view image pixel frame."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def apply_affine(matrix: list[list[float]], x, y):
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2],
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2],
    )


def sample_nearest(image: np.ndarray, px: np.ndarray, py: np.ndarray, fill: tuple[int, int, int]):
    height, width, channels = image.shape
    ix = np.rint(px).astype(np.int64)
    iy = np.rint(py).astype(np.int64)
    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    out = np.empty((py.shape[0], px.shape[1], channels), dtype=np.uint8)
    out[:, :] = fill
    out[valid] = image[iy[valid], ix[valid]]
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-image", type=Path, required=True)
    parser.add_argument("--source-metadata", type=Path, required=True)
    parser.add_argument("--reference-image", type=Path, required=True)
    parser.add_argument("--reference-alignment", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata-output", type=Path, default=None)
    parser.add_argument("--fill", nargs=3, type=int, default=(0, 0, 0), metavar=("R", "G", "B"))
    parser.add_argument("--chunk-rows", type=int, default=256)
    args = parser.parse_args()

    source_meta = json.loads(args.source_metadata.read_text(encoding="utf-8"))
    alignment = json.loads(args.reference_alignment.read_text(encoding="utf-8"))
    bounds = source_meta["bounds"]
    matrix = alignment["pixel_to_map"]

    source = np.asarray(Image.open(args.source_image).convert("RGB"))
    with Image.open(args.reference_image) as ref_image:
        ref_width, ref_height = ref_image.size

    src_height, src_width = source.shape[:2]
    result = np.empty((ref_height, ref_width, 3), dtype=np.uint8)

    xs = np.arange(ref_width, dtype=np.float64)[None, :]
    for row0 in range(0, ref_height, args.chunk_rows):
        row1 = min(ref_height, row0 + args.chunk_rows)
        ys = np.arange(row0, row1, dtype=np.float64)[:, None]
        map_x, map_y = apply_affine(matrix, xs, ys)
        src_px = ((map_x - bounds["west"]) / (bounds["east"] - bounds["west"])) * (src_width - 1)
        src_py = ((bounds["north"] - map_y) / (bounds["north"] - bounds["south"])) * (src_height - 1)
        result[row0:row1] = sample_nearest(source, src_px, src_py, tuple(args.fill))

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(result, "RGB").save(output)

    if args.metadata_output:
        payload = {
            "schema": "aerion_warped_map_image_v1",
            "image": str(output),
            "source_image": str(args.source_image.expanduser().resolve()),
            "source_metadata": str(args.source_metadata.expanduser().resolve()),
            "reference_image": str(args.reference_image.expanduser().resolve()),
            "reference_alignment": str(args.reference_alignment.expanduser().resolve()),
            "width": ref_width,
            "height": ref_height,
            "source_bounds": bounds,
            "reference_alignment_method": alignment.get("method"),
            "reference_alignment_rms_error_m": alignment.get("rms_error_m"),
            "pixel_frame": "reference_image_pixels",
            "note": "Each output pixel was sampled at the map coordinate defined by the reference image alignment.",
        }
        args.metadata_output.expanduser().resolve().write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
