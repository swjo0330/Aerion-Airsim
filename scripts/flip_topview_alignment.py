#!/usr/bin/env python3
"""Compose a top-view alignment with an image-space flip."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--flip-y", action="store_true")
    parser.add_argument("--name", default=None)
    args = parser.parse_args()

    data = json.loads(args.input.expanduser().read_text(encoding="utf-8"))
    if data.get("schema") != "aerion_topview_alignment_v1":
        raise ValueError(f"unsupported alignment schema: {data.get('schema')}")
    width = float(data["image_size"]["width"])
    height = float(data["image_size"]["height"])
    pixel_to_map = [row[:] for row in data["pixel_to_map"]]
    map_to_pixel = [row[:] for row in data["map_to_pixel"]]

    transforms = []
    if args.flip_y:
        # New image pixel py maps to old image pixel height - py.
        for row in pixel_to_map:
            row[2] += row[1] * height
            row[1] = -row[1]
        # Inverse: new py = height - old py.
        map_to_pixel[1] = [-map_to_pixel[1][0], -map_to_pixel[1][1], height - map_to_pixel[1][2]]
        transforms.append("image_flip_y")

    if not transforms:
        raise ValueError("no transform requested")

    metadata = dict(data.get("metadata") or {})
    metadata["alignment_transform"] = {
        "source": str(args.input),
        "transforms": transforms,
        "note": "Image-space transform only; map screen axes remain unchanged.",
    }
    data.update(
        {
            "name": args.name or f"{data.get('name', args.input.stem)}_{'_'.join(transforms)}",
            "pixel_to_map": pixel_to_map,
            "map_to_pixel": map_to_pixel,
            "metadata": metadata,
        }
    )

    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"Wrote {output.resolve()} ({', '.join(transforms)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
