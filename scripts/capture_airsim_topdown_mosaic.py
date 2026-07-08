#!/usr/bin/env python3
"""Capture a top-down RGB mosaic through an AirSim camera."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import airsim
import numpy as np
from PIL import Image

RESAMPLE_BILINEAR = getattr(getattr(Image, "Resampling", Image), "BILINEAR")

DEFAULT_XODR = (
    Path.home()
    / "workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr"
)
DEFAULT_OUTPUT = Path("recordings/maps/airsim_topdown_mosaic.png")


def xodr_bounds(path: Path) -> dict[str, float]:
    root = ET.parse(path).getroot()
    header = root.find("header")
    if header is None:
        raise ValueError(f"OpenDRIVE header not found: {path}")
    return {key: float(header.attrib[key]) for key in ("west", "east", "south", "north")}


def fov_for_square_crop(tile_m: float, altitude_m: float, width: int, height: int) -> float:
    vertical_fov = 2.0 * math.atan((tile_m * 0.5) / altitude_m)
    horizontal_fov = 2.0 * math.atan(math.tan(vertical_fov * 0.5) * (width / height))
    return math.degrees(horizontal_fov)


def center_square(image: Image.Image) -> Image.Image:
    side = min(image.width, image.height)
    left = (image.width - side) // 2
    top = (image.height - side) // 2
    return image.crop((left, top, left + side, top + side))


def center_fraction(image: Image.Image, fraction: float) -> Image.Image:
    fraction = max(0.05, min(1.0, fraction))
    side = int(round(min(image.width, image.height) * fraction))
    left = (image.width - side) // 2
    top = (image.height - side) // 2
    return image.crop((left, top, left + side, top + side))


def response_to_image(response) -> Image.Image:
    if response.compress:
        return Image.open(Path(response.image_data_uint8)).convert("RGB")
    array = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    array = array.reshape(response.height, response.width, 3)
    return Image.fromarray(array, "RGB")


def map_to_airsim_ned(map_x: float, map_y: float, *, map_x_offset: float, map_y_offset: float):
    # OpenDRIVE/map: x=east, y=north. AirSim NED: x=north, y=east, z=down.
    return map_y - map_y_offset, map_x - map_x_offset


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--airsim-ip", default="127.0.0.1")
    parser.add_argument("--airsim-port", type=int, default=41451)
    parser.add_argument("--airsim-timeout", type=float, default=10.0)
    parser.add_argument("--vehicle", default="drone1")
    parser.add_argument("--camera", default="front_center")
    parser.add_argument("--xodr", type=Path, default=DEFAULT_XODR)
    parser.add_argument("--bounds", nargs=4, type=float, metavar=("WEST", "EAST", "SOUTH", "NORTH"))
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--metadata", type=Path, default=None)
    parser.add_argument("--tile-output-dir", type=Path, default=None, help="optional directory for overscanned tile images")
    parser.add_argument("--tile-m", type=float, default=80.0)
    parser.add_argument("--pixels-per-meter", type=float, default=6.0)
    parser.add_argument("--altitude-m", type=float, default=180.0)
    parser.add_argument(
        "--overscan",
        type=float,
        default=1.0,
        help="capture this many tile widths and crop the center tile to reduce perspective edge seams",
    )
    parser.add_argument("--map-x-offset", type=float, default=0.0)
    parser.add_argument("--map-y-offset", type=float, default=0.0)
    parser.add_argument("--settle-sec", type=float, default=0.05)
    parser.add_argument("--raw-rotate", type=int, choices=(0, 90, 180, 270), default=0)
    parser.add_argument("--raw-flip-x", action="store_true")
    parser.add_argument("--raw-flip-y", action="store_true")
    args = parser.parse_args()

    if args.bounds:
        west, east, south, north = args.bounds
        bounds = {"west": west, "east": east, "south": south, "north": north}
    else:
        bounds = xodr_bounds(args.xodr.expanduser().resolve())

    tile_px = max(64, int(round(args.tile_m * args.pixels_per_meter)))
    cols = int(math.ceil((bounds["east"] - bounds["west"]) / args.tile_m))
    rows = int(math.ceil((bounds["north"] - bounds["south"]) / args.tile_m))
    mosaic = Image.new("RGB", (cols * tile_px, rows * tile_px), (0, 0, 0))

    client = airsim.MultirotorClient(
        ip=args.airsim_ip,
        port=args.airsim_port,
        timeout_value=args.airsim_timeout,
    )
    client.confirmConnection()
    vehicle_pose = client.simGetVehiclePose(vehicle_name=args.vehicle)
    camera_info = client.simGetCameraInfo(args.camera, vehicle_name=args.vehicle)
    probe = client.simGetImages(
        [airsim.ImageRequest(args.camera, airsim.ImageType.Scene, False, False)],
        vehicle_name=args.vehicle,
    )[0]
    if probe.width <= 0 or probe.height <= 0:
        raise RuntimeError(f"camera returned empty probe image: {args.camera}")

    capture_tile_m = args.tile_m * max(1.0, args.overscan)
    capture_tile_px = max(tile_px, int(round(capture_tile_m * args.pixels_per_meter)))
    fov_deg = fov_for_square_crop(capture_tile_m, args.altitude_m, probe.width, probe.height)
    client.simSetCameraFov(args.camera, fov_deg, vehicle_name=args.vehicle)
    orientation = airsim.to_quaternion(-math.pi / 2.0, 0.0, 0.0)

    print(
        f"Capturing {cols}x{rows} AirSim tile(s), camera={args.camera}, "
        f"tile={args.tile_m}m/{tile_px}px, overscan={args.overscan:.2f}, "
        f"altitude={args.altitude_m}m, fov={fov_deg:.2f}, bounds={bounds}",
        flush=True,
    )
    started = time.time()
    try:
        for row in range(rows):
            map_y = bounds["north"] - (row + 0.5) * args.tile_m
            for col in range(cols):
                map_x = bounds["west"] + (col + 0.5) * args.tile_m
                ned_x, ned_y = map_to_airsim_ned(
                    map_x,
                    map_y,
                    map_x_offset=args.map_x_offset,
                    map_y_offset=args.map_y_offset,
                )
                relative = airsim.Vector3r(
                    ned_x - vehicle_pose.position.x_val,
                    ned_y - vehicle_pose.position.y_val,
                    -args.altitude_m - vehicle_pose.position.z_val,
                )
                client.simSetCameraPose(args.camera, airsim.Pose(relative, orientation), vehicle_name=args.vehicle)
                if args.settle_sec > 0:
                    time.sleep(args.settle_sec)
                response = client.simGetImages(
                    [airsim.ImageRequest(args.camera, airsim.ImageType.Scene, False, False)],
                    vehicle_name=args.vehicle,
                )[0]
                tile = center_square(response_to_image(response))
                if args.raw_rotate:
                    tile = tile.rotate(args.raw_rotate, expand=True)
                if args.raw_flip_x:
                    tile = tile.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
                if args.raw_flip_y:
                    tile = tile.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
                if args.overscan > 1.0:
                    tile = tile.resize((capture_tile_px, capture_tile_px), RESAMPLE_BILINEAR)
                    if args.tile_output_dir is not None:
                        tile_dir = args.tile_output_dir.expanduser().resolve()
                        tile_dir.mkdir(parents=True, exist_ok=True)
                        tile.save(tile_dir / f"tile_r{row:02d}_c{col:02d}.png")
                    tile = center_fraction(tile, 1.0 / args.overscan)
                else:
                    if args.tile_output_dir is not None:
                        tile_dir = args.tile_output_dir.expanduser().resolve()
                        tile_dir.mkdir(parents=True, exist_ok=True)
                        tile.save(tile_dir / f"tile_r{row:02d}_c{col:02d}.png")
                if tile.size != (tile_px, tile_px):
                    tile = tile.resize((tile_px, tile_px), RESAMPLE_BILINEAR)
                mosaic.paste(tile, (col * tile_px, row * tile_px))
                print(f"  tile {row + 1}/{rows}, {col + 1}/{cols}", flush=True)
    finally:
        client.simSetCameraPose(args.camera, camera_info.pose, vehicle_name=args.vehicle)

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    mosaic.save(output)
    metadata_path = args.metadata.expanduser().resolve() if args.metadata else output.with_suffix(".json")
    payload = {
        "schema": "aerion_airsim_topdown_mosaic_v1",
        "image": str(output),
        "bounds": bounds,
        "width": mosaic.width,
        "height": mosaic.height,
        "tile_m": args.tile_m,
        "tile_px": tile_px,
        "capture_tile_px": capture_tile_px,
        "pixels_per_meter": tile_px / args.tile_m,
        "rows": rows,
        "cols": cols,
        "camera": {
            "vehicle": args.vehicle,
            "name": args.camera,
            "altitude_m": args.altitude_m,
            "fov_deg": fov_deg,
            "overscan": args.overscan,
            "capture_tile_m": capture_tile_m,
            "raw_rotate": args.raw_rotate,
            "raw_flip_x": args.raw_flip_x,
            "raw_flip_y": args.raw_flip_y,
        },
        "tiles": {
            "output_dir": str(args.tile_output_dir.expanduser().resolve()) if args.tile_output_dir else None,
            "filename_pattern": "tile_r{row:02d}_c{col:02d}.png",
        },
        "coordinate_transform": {
            "input_frame": "opendrive_map_xy",
            "output_frame": "airsim_ned",
            "map_x_to": "airsim_y",
            "map_y_to": "airsim_x",
            "map_x_offset": args.map_x_offset,
            "map_y_offset": args.map_y_offset,
        },
        "elapsed_sec": time.time() - started,
    }
    metadata_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Wrote {output}")
    print(f"Wrote {metadata_path}")
    print(
        "Editor bounds: "
        f"{bounds['west']} {bounds['east']} {bounds['south']} {bounds['north']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
