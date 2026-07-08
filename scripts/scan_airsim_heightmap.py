#!/usr/bin/env python3
"""Scan AirSim/Colosseum scene meshes into a 2.5D heightmap JSON."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import sys
from typing import Iterable
import xml.etree.ElementTree as ET

import airsim


DEFAULT_XODR = (
    Path.home()
    / "workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr"
)
DEFAULT_OUTPUT = Path("recordings/maps/heightmap.json")


def xodr_bounds(path: Path) -> dict[str, float]:
    root = ET.parse(path).getroot()
    header = root.find("header")
    if header is None:
        raise ValueError(f"OpenDRIVE header not found: {path}")
    return {key: float(header.attrib[key]) for key in ("west", "east", "south", "north")}


def rotate_vector(q, x: float, y: float, z: float) -> tuple[float, float, float]:
    # Quaternion vector rotation: v' = v + 2*w*(q.xyz x v) + 2*(q.xyz x (q.xyz x v)).
    qx, qy, qz, qw = q.x_val, q.y_val, q.z_val, q.w_val
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def chunked_vertices(values: Iterable[float]):
    values = list(values)
    for index in range(0, len(values) - 2, 3):
        yield float(values[index]), float(values[index + 1]), float(values[index + 2])


def mesh_vertices_world(mesh, coordinate_frame: str, map_x_offset: float, map_y_offset: float, up_offset: float):
    out = []
    pos = mesh.position
    for x, y, z in chunked_vertices(mesh.vertices):
        wx, wy, wz = rotate_vector(mesh.orientation, x, y, z)
        wx += pos.x_val
        wy += pos.y_val
        wz += pos.z_val

        if coordinate_frame == "airsim_ned":
            # AirSim NED: x=north, y=east, z=down -> map/OpenDRIVE: x=east, y=north, up=-down.
            map_x = wy + map_x_offset
            map_y = wx + map_y_offset
            up = -wz + up_offset
        elif coordinate_frame == "opendrive":
            map_x = wx + map_x_offset
            map_y = wy + map_y_offset
            up = wz + up_offset
        else:
            raise ValueError(f"unsupported coordinate frame: {coordinate_frame}")
        out.append((map_x, map_y, up))
    return out


def cell_index(x: float, y: float, bounds: dict[str, float], resolution: float, width: int, height: int):
    col = int(math.floor((x - bounds["west"]) / resolution))
    row = int(math.floor((bounds["north"] - y) / resolution))
    if 0 <= col < width and 0 <= row < height:
        return row * width + col
    return None


def barycentric(px: float, py: float, a, b, c):
    ax, ay, _ = a
    bx, by, _ = b
    cx, cy, _ = c
    denom = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
    if abs(denom) < 1e-9:
        return None
    w1 = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / denom
    w2 = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / denom
    w3 = 1.0 - w1 - w2
    if w1 < -1e-9 or w2 < -1e-9 or w3 < -1e-9:
        return None
    return w1, w2, w3


def stamp_height(grid: list[float | None], index: int | None, up: float):
    if index is None:
        return
    current = grid[index]
    if current is None or up > current:
        grid[index] = up


def rasterize_vertices(grid, vertices, bounds, resolution, width, height):
    for x, y, up in vertices:
        stamp_height(grid, cell_index(x, y, bounds, resolution, width, height), up)


def rasterize_triangles(grid, vertices, indices, bounds, resolution, width, height, max_triangle_edge: float):
    if not indices:
        return 0
    count = 0
    int_indices = [int(value) for value in indices]
    for offset in range(0, len(int_indices) - 2, 3):
        try:
            tri = [vertices[int_indices[offset]], vertices[int_indices[offset + 1]], vertices[int_indices[offset + 2]]]
        except IndexError:
            continue
        edges = [
            math.dist(tri[0], tri[1]),
            math.dist(tri[1], tri[2]),
            math.dist(tri[2], tri[0]),
        ]
        if max(edges) > max_triangle_edge:
            rasterize_vertices(grid, tri, bounds, resolution, width, height)
            continue
        min_x = max(bounds["west"], min(point[0] for point in tri))
        max_x = min(bounds["east"], max(point[0] for point in tri))
        min_y = max(bounds["south"], min(point[1] for point in tri))
        max_y = min(bounds["north"], max(point[1] for point in tri))
        col0 = max(0, int(math.floor((min_x - bounds["west"]) / resolution)))
        col1 = min(width - 1, int(math.floor((max_x - bounds["west"]) / resolution)))
        row0 = max(0, int(math.floor((bounds["north"] - max_y) / resolution)))
        row1 = min(height - 1, int(math.floor((bounds["north"] - min_y) / resolution)))
        for row in range(row0, row1 + 1):
            y = bounds["north"] - (row + 0.5) * resolution
            for col in range(col0, col1 + 1):
                x = bounds["west"] + (col + 0.5) * resolution
                weights = barycentric(x, y, tri[0], tri[1], tri[2])
                if weights is None:
                    continue
                up = weights[0] * tri[0][2] + weights[1] * tri[1][2] + weights[2] * tri[2][2]
                stamp_height(grid, row * width + col, up)
                count += 1
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--xodr", type=Path, default=DEFAULT_XODR, help="Use OpenDRIVE header bounds")
    parser.add_argument("--bounds", nargs=4, type=float, metavar=("WEST", "EAST", "SOUTH", "NORTH"))
    parser.add_argument("--resolution", type=float, default=2.0, help="Grid cell size in meters")
    parser.add_argument("--coordinate-frame", choices=("airsim_ned", "opendrive"), default="airsim_ned")
    parser.add_argument("--map-x-offset", type=float, default=0.0)
    parser.add_argument("--map-y-offset", type=float, default=0.0)
    parser.add_argument("--up-offset", type=float, default=0.0)
    parser.add_argument("--mesh-regex", default=".*", help="Include mesh names matching this regex")
    parser.add_argument("--ignore-regex", default="(?i)(sky|cloud|weather|sun|fog)", help="Exclude mesh names")
    parser.add_argument("--vertices-only", action="store_true", help="Skip triangle rasterization")
    parser.add_argument("--max-triangle-edge", type=float, default=80.0)
    parser.add_argument("--airsim-ip", default="127.0.0.1")
    parser.add_argument("--airsim-port", type=int, default=41451)
    parser.add_argument("--airsim-timeout", type=float, default=10.0)
    args = parser.parse_args()

    if args.bounds:
        west, east, south, north = args.bounds
        bounds = {"west": west, "east": east, "south": south, "north": north}
    else:
        bounds = xodr_bounds(args.xodr.expanduser().resolve())

    if args.resolution <= 0:
        raise ValueError("--resolution must be positive")
    width = int(math.ceil((bounds["east"] - bounds["west"]) / args.resolution))
    height = int(math.ceil((bounds["north"] - bounds["south"]) / args.resolution))
    grid: list[float | None] = [None] * (width * height)

    include_re = re.compile(args.mesh_regex)
    ignore_re = re.compile(args.ignore_regex) if args.ignore_regex else None

    client = airsim.MultirotorClient(ip=args.airsim_ip, port=args.airsim_port, timeout_value=args.airsim_timeout)
    try:
        client.confirmConnection()
        meshes = client.simGetMeshPositionVertexBuffers()
    except Exception as exc:
        print(f"AirSim heightmap scan failed: {exc}", file=sys.stderr)
        return 2

    included = []
    skipped = []
    stamped = 0
    for mesh in meshes:
        name = str(mesh.name)
        if not include_re.search(name) or (ignore_re and ignore_re.search(name)):
            skipped.append(name)
            continue
        vertices = mesh_vertices_world(
            mesh,
            coordinate_frame=args.coordinate_frame,
            map_x_offset=args.map_x_offset,
            map_y_offset=args.map_y_offset,
            up_offset=args.up_offset,
        )
        if not vertices:
            skipped.append(name)
            continue
        if args.vertices_only:
            before = sum(1 for value in grid if value is not None)
            rasterize_vertices(grid, vertices, bounds, args.resolution, width, height)
            stamped += sum(1 for value in grid if value is not None) - before
        else:
            stamped += rasterize_triangles(
                grid,
                vertices,
                mesh.indices,
                bounds,
                args.resolution,
                width,
                height,
                args.max_triangle_edge,
            )
        included.append({"name": name, "vertices": len(vertices), "indices": len(mesh.indices or [])})

    known_cells = sum(1 for value in grid if value is not None)
    payload = {
        "schema": "aerion_heightmap_v1",
        "source": "airsim_mesh_position_vertex_buffers",
        "coordinate_frame": "opendrive_map_xy",
        "height_reference": "up_m",
        "resolution_m": args.resolution,
        "width": width,
        "height": height,
        "bounds": bounds,
        "no_data": None,
        "values": grid,
        "coverage": {
            "known_cells": known_cells,
            "total_cells": len(grid),
            "ratio": known_cells / len(grid) if grid else 0.0,
            "stamped_samples": stamped,
        },
        "scan": {
            "coordinate_frame_input": args.coordinate_frame,
            "map_x_offset": args.map_x_offset,
            "map_y_offset": args.map_y_offset,
            "up_offset": args.up_offset,
            "included_mesh_count": len(included),
            "skipped_mesh_count": len(skipped),
            "included_meshes": included[:200],
        },
    }

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
    print(
        f"Wrote {output} ({width}x{height}, known={known_cells}/{len(grid)}, "
        f"meshes={len(included)}/{len(meshes)})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
