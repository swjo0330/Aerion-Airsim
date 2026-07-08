#!/usr/bin/env python3
"""Render an aerion_heightmap_v1 JSON scan into a top-down PNG map."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw


SEMANTIC_COLORS = {
    "Roads": (62, 67, 73, 255),
    "RoadLines": (232, 206, 94, 255),
    "Sidewalks": (166, 170, 164, 255),
    "Buildings": (166, 139, 91, 255),
    "Walls": (145, 130, 112, 255),
    "Fences": (137, 122, 104, 255),
    "Poles": (99, 99, 93, 255),
    "TrafficSigns": (210, 86, 65, 255),
    "TrafficLight": (210, 86, 65, 255),
    "Vegetation": (83, 121, 87, 255),
    "Terrain": (118, 144, 96, 255),
    "Sky": (226, 229, 221, 255),
    "Ground": (118, 144, 96, 255),
    "Static": (126, 150, 106, 255),
    "Other": (154, 158, 148, 255),
}


def color_for_height(value: float | None, min_h: float, max_h: float) -> tuple[int, int, int, int]:
    if value is None or not math.isfinite(value):
        return (226, 229, 221, 255)
    t = 0.0 if max_h <= min_h else max(0.0, min(1.0, (value - min_h) / (max_h - min_h)))
    if t < 0.45:
        q = t / 0.45
        return (
            int(72 + 42 * q),
            int(108 + 66 * q),
            int(86 + 28 * q),
            255,
        )
    q = (t - 0.45) / 0.55
    return (
        int(114 + 128 * q),
        int(174 - 71 * q),
        int(114 - 66 * q),
        255,
    )


def color_for_semantic(label: str | None, value: float | None, min_h: float, max_h: float):
    if value is None or not math.isfinite(value):
        return (226, 229, 221, 255)
    if label in SEMANTIC_COLORS:
        return SEMANTIC_COLORS[label]
    return SEMANTIC_COLORS["Other"]


def render_heightmap(
    heightmap: dict,
    output: Path,
    scale: int,
    draw_grid: bool,
    draw_label: bool,
    mode: str,
) -> None:
    width = int(heightmap["width"])
    height = int(heightmap["height"])
    values = heightmap["values"]
    known = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    min_h = min(known) if known else 0.0
    max_h = max(known) if known else 1.0

    image = Image.new("RGBA", (width, height), (226, 229, 221, 255))
    pixels = image.load()
    labels = heightmap.get("labels") or [None] * len(values)
    if mode == "auto":
        mode = "semantic" if any(label is not None for label in labels) else "height"
    for row in range(height):
        for col in range(width):
            index = row * width + col
            if mode == "semantic":
                pixels[col, row] = color_for_semantic(labels[index], values[index], min_h, max_h)
            else:
                pixels[col, row] = color_for_height(values[index], min_h, max_h)

    if scale > 1:
        nearest = getattr(getattr(Image, "Resampling", Image), "NEAREST")
        image = image.resize((width * scale, height * scale), nearest)

    draw = ImageDraw.Draw(image)
    if draw_grid and scale >= 3:
        for x in range(0, image.width, scale):
            draw.line((x, 0, x, image.height), fill=(255, 255, 255, 45))
        for y in range(0, image.height, scale):
            draw.line((0, y, image.width, y), fill=(255, 255, 255, 45))

    if draw_label:
        coverage = heightmap.get("coverage", {})
        label = (
            f"heightmap {width}x{height} res={heightmap.get('resolution_m')}m "
            f"coverage={coverage.get('ratio', 0.0) * 100:.1f}% "
            f"height={min_h:.1f}..{max_h:.1f}m"
        )
        draw.rectangle((8, 8, min(image.width - 8, 8 + len(label) * 7), 31), fill=(255, 255, 255, 210))
        draw.text((14, 13), label, fill=(32, 33, 36, 255))

    output.parent.mkdir(parents=True, exist_ok=True)
    image.convert("RGB").save(output)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--heightmap", type=Path, default=Path("recordings/maps/heightmap.json"))
    parser.add_argument("--output", type=Path, default=Path("recordings/maps/heightmap_map.png"))
    parser.add_argument("--scale", type=int, default=3)
    parser.add_argument("--grid", action="store_true")
    parser.add_argument("--label", action="store_true")
    parser.add_argument("--mode", choices=("auto", "semantic", "height"), default="auto")
    args = parser.parse_args()

    heightmap = json.loads(args.heightmap.expanduser().read_text(encoding="utf-8"))
    if heightmap.get("schema") != "aerion_heightmap_v1":
        raise ValueError(f"unsupported heightmap schema: {heightmap.get('schema')}")
    render_heightmap(heightmap, args.output.expanduser(), max(1, args.scale), args.grid, args.label, args.mode)
    print(f"Wrote {args.output.expanduser().resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
