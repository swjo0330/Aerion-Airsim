#!/usr/bin/env python3
"""Capture a CARLA top-down mosaic with map-coordinate bounds metadata."""

from __future__ import annotations

import argparse
import json
import math
import queue
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import carla
from PIL import Image

RESAMPLE_BILINEAR = getattr(getattr(Image, "Resampling", Image), "BILINEAR")


DEFAULT_XODR = (
    Path.home()
    / "workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr"
)
DEFAULT_OUTPUT = Path("recordings/maps/carla_topdown_mosaic.png")


def xodr_bounds(path: Path) -> dict[str, float]:
    root = ET.parse(path).getroot()
    header = root.find("header")
    if header is None:
        raise ValueError(f"OpenDRIVE header not found: {path}")
    return {key: float(header.attrib[key]) for key in ("west", "east", "south", "north")}


def parse_bounds(args) -> dict[str, float]:
    if args.bounds:
        west, east, south, north = args.bounds
        return {"west": west, "east": east, "south": south, "north": north}
    bounds = xodr_bounds(args.xodr.expanduser().resolve())
    if args.crop_to_known_heightmap:
        data = json.loads(args.crop_to_known_heightmap.expanduser().read_text(encoding="utf-8"))
        values = data["values"]
        hm_bounds = data["bounds"]
        res = float(data["resolution_m"])
        width = int(data["width"])
        xs = []
        ys = []
        for row in range(int(data["height"])):
            y = hm_bounds["north"] - (row + 0.5) * res
            for col in range(width):
                if values[row * width + col] is not None:
                    xs.append(hm_bounds["west"] + (col + 0.5) * res)
                    ys.append(y)
        if xs and ys:
            pad = args.crop_pad_m
            bounds = {"west": min(xs) - pad, "east": max(xs) + pad, "south": min(ys) - pad, "north": max(ys) + pad}
    return bounds


def fov_for_tile(tile_m: float, altitude_m: float) -> float:
    return math.degrees(2.0 * math.atan((tile_m * 0.5) / altitude_m))


def image_from_carla(raw, color_converter=None) -> Image.Image:
    if color_converter is not None:
        raw.convert(color_converter)
    image = Image.frombytes("RGBA", (raw.width, raw.height), bytes(raw.raw_data), "raw", "BGRA")
    return image.convert("RGB")


def orient_tile(image: Image.Image, rotate: int, flip_x: bool, flip_y: bool) -> Image.Image:
    if rotate:
        image = image.rotate(rotate, expand=True)
    if flip_x:
        image = image.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
    if flip_y:
        image = image.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
    return image


def mean_luma(image: Image.Image) -> float:
    stat = image.convert("L").resize((1, 1), RESAMPLE_BILINEAR)
    return float(stat.getpixel((0, 0)))


def dark_pixel_ratio(image: Image.Image, threshold: int) -> float:
    if threshold <= 0:
        return 0.0
    gray = image.convert("L")
    histogram = gray.histogram()
    dark = sum(histogram[: min(threshold, len(histogram))])
    total = gray.width * gray.height
    return dark / total if total else 0.0


def capture_tile(
    world,
    blueprint,
    transform,
    timeout: float,
    *,
    warmup_frames: int,
    color_converter=None,
) -> Image.Image:
    images: queue.Queue = queue.Queue(maxsize=1)
    camera = world.spawn_actor(blueprint, transform)
    try:
        remaining = max(1, warmup_frames)

        def on_image(image):
            nonlocal remaining
            remaining -= 1
            if remaining <= 0 and images.empty():
                images.put(image, block=False)

        camera.listen(on_image)
        raw = images.get(timeout=timeout)
        return image_from_carla(raw, color_converter=color_converter)
    finally:
        camera.stop()
        camera.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--xodr", type=Path, default=DEFAULT_XODR)
    parser.add_argument("--bounds", nargs=4, type=float, metavar=("WEST", "EAST", "SOUTH", "NORTH"))
    parser.add_argument("--crop-to-known-heightmap", type=Path, default=None)
    parser.add_argument("--crop-pad-m", type=float, default=30.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--metadata", type=Path, default=None)
    parser.add_argument("--camera-type", choices=("rgb", "semantic"), default="rgb")
    parser.add_argument("--tile-m", type=float, default=120.0)
    parser.add_argument("--pixels-per-meter", type=float, default=4.0)
    parser.add_argument("--altitude-m", type=float, default=450.0)
    parser.add_argument("--camera-z-offset", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--fov-deg", type=float, default=None)
    parser.add_argument("--settle-sec", type=float, default=0.05)
    parser.add_argument("--warmup-frames", type=int, default=3)
    parser.add_argument("--min-luma", type=float, default=3.0)
    parser.add_argument("--dark-pixel-threshold", type=int, default=8)
    parser.add_argument("--dark-retries", type=int, default=1)
    parser.add_argument("--allow-dark", action="store_true")
    parser.add_argument("--raw-rotate", type=int, choices=(0, 90, 180, 270), default=0)
    parser.add_argument("--raw-flip-x", action="store_true")
    parser.add_argument("--raw-flip-y", action="store_true")
    args = parser.parse_args()

    bounds = parse_bounds(args)
    tile_px = max(64, int(round(args.tile_m * args.pixels_per_meter)))
    cols = int(math.ceil((bounds["east"] - bounds["west"]) / args.tile_m))
    rows = int(math.ceil((bounds["north"] - bounds["south"]) / args.tile_m))
    mosaic = Image.new("RGB", (cols * tile_px, rows * tile_px), (230, 232, 224))

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    blueprints = world.get_blueprint_library()
    camera_blueprint = "sensor.camera.semantic_segmentation" if args.camera_type == "semantic" else "sensor.camera.rgb"
    blueprint = blueprints.find(camera_blueprint)
    blueprint.set_attribute("image_size_x", str(tile_px))
    blueprint.set_attribute("image_size_y", str(tile_px))
    blueprint.set_attribute("fov", str(args.fov_deg or fov_for_tile(args.tile_m, args.altitude_m)))
    color_converter = carla.ColorConverter.CityScapesPalette if args.camera_type == "semantic" else None

    print(
        f"Capturing {cols}x{rows} tile(s), camera={args.camera_type}, tile={args.tile_m}m/{tile_px}px, "
        f"bounds={bounds}, fov={blueprint.get_attribute('fov').as_float():.2f}",
        flush=True,
    )
    started = time.time()
    for row in range(rows):
        y = bounds["north"] - (row + 0.5) * args.tile_m
        for col in range(cols):
            x = bounds["west"] + (col + 0.5) * args.tile_m
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=args.altitude_m + args.camera_z_offset),
                carla.Rotation(pitch=-90.0, yaw=args.yaw_deg, roll=0.0),
            )
            tile = None
            last_luma = 0.0
            for attempt in range(max(1, args.dark_retries + 1)):
                tile = capture_tile(
                    world,
                    blueprint,
                    transform,
                    args.timeout,
                    warmup_frames=args.warmup_frames,
                    color_converter=color_converter,
                )
                last_luma = mean_luma(tile)
                if args.allow_dark or last_luma >= args.min_luma:
                    break
                print(
                    f"  tile {row + 1}/{rows}, {col + 1}/{cols} dark "
                    f"(luma={last_luma:.1f}), retry {attempt + 1}/{args.dark_retries + 1}",
                    flush=True,
                )
                time.sleep(max(args.settle_sec, 0.2))
            if tile is None:
                raise RuntimeError("tile capture returned no image")
            if not args.allow_dark and last_luma < args.min_luma:
                raise RuntimeError(
                    f"captured tile is too dark: row={row} col={col} luma={last_luma:.1f}; "
                    "try --camera-type semantic or --allow-dark"
                )
            tile = orient_tile(tile, args.raw_rotate, args.raw_flip_x, args.raw_flip_y)
            if tile.size != (tile_px, tile_px):
                tile = tile.resize((tile_px, tile_px), RESAMPLE_BILINEAR)
            mosaic.paste(tile, (col * tile_px, row * tile_px))
            print(f"  tile {row + 1}/{rows}, {col + 1}/{cols}, luma={last_luma:.1f}", flush=True)
            if args.settle_sec > 0:
                time.sleep(args.settle_sec)

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    mosaic.save(output)
    black_ratio = dark_pixel_ratio(mosaic, args.dark_pixel_threshold)
    metadata_path = args.metadata.expanduser().resolve() if args.metadata else output.with_suffix(".json")
    payload = {
        "schema": "aerion_carla_topdown_mosaic_v1",
        "image": str(output),
        "bounds": bounds,
        "width": mosaic.width,
        "height": mosaic.height,
        "tile_m": args.tile_m,
        "tile_px": tile_px,
        "pixels_per_meter": tile_px / args.tile_m,
        "rows": rows,
        "cols": cols,
        "camera": {
            "type": args.camera_type,
            "blueprint": camera_blueprint,
            "altitude_m": args.altitude_m,
            "yaw_deg": args.yaw_deg,
            "fov_deg": blueprint.get_attribute("fov").as_float(),
            "warmup_frames": args.warmup_frames,
            "min_luma": args.min_luma,
            "dark_pixel_threshold": args.dark_pixel_threshold,
            "dark_pixel_ratio": black_ratio,
            "dark_retries": args.dark_retries,
            "allow_dark": args.allow_dark,
            "raw_rotate": args.raw_rotate,
            "raw_flip_x": args.raw_flip_x,
            "raw_flip_y": args.raw_flip_y,
        },
        "elapsed_sec": time.time() - started,
    }
    metadata_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Wrote {output}")
    print(f"Wrote {metadata_path}")
    print(f"Dark pixel ratio: {black_ratio * 100:.2f}%")
    print(
        "Editor bounds: "
        f"{bounds['west']} {bounds['east']} {bounds['south']} {bounds['north']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
