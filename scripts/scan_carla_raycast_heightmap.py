#!/usr/bin/env python3
"""Scan a CARLA world into a 2.5D heightmap using vertical ray casts."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor, as_completed

import carla


DEFAULT_XODR = (
    Path.home()
    / "workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr"
)
DEFAULT_OUTPUT = Path("recordings/maps/heightmap.json")
_THREAD_LOCAL = threading.local()


def xodr_bounds(path: Path) -> dict[str, float]:
    root = ET.parse(path).getroot()
    header = root.find("header")
    if header is None:
        raise ValueError(f"OpenDRIVE header not found: {path}")
    return {key: float(header.attrib[key]) for key in ("west", "east", "south", "north")}


def cast_height(world, x: float, y: float, z_top: float, z_bottom: float):
    start = carla.Location(x=x, y=y, z=z_top)
    end = carla.Location(x=x, y=y, z=z_bottom)
    hits = world.cast_ray(start, end)
    if not hits:
        return None, None
    best = max(hits, key=lambda point: point.location.z)
    label = str(best.label).split(".")[-1]
    return float(best.location.z), label


def thread_world(host: str, port: int, timeout: float):
    world = getattr(_THREAD_LOCAL, "world", None)
    if world is None:
        client = carla.Client(host, port)
        client.set_timeout(timeout)
        world = client.get_world()
        _THREAD_LOCAL.client = client
        _THREAD_LOCAL.world = world
    return world


def scan_row(
    host: str,
    port: int,
    timeout: float,
    row: int,
    width: int,
    bounds: dict[str, float],
    resolution: float,
    z_top: float,
    z_bottom: float,
):
    world = thread_world(host, port, timeout)
    y = bounds["north"] - (row + 0.5) * resolution
    row_values: list[float | None] = [None] * width
    row_labels: list[str | None] = [None] * width
    row_known = 0
    for col in range(width):
        x = bounds["west"] + (col + 0.5) * resolution
        z, label = cast_height(world, x, y, z_top, z_bottom)
        if z is not None:
            row_values[col] = z
            row_labels[col] = label
            row_known += 1
    return row, row_values, row_labels, row_known


def count_known(values: list[float | None]) -> int:
    return sum(1 for value in values if value is not None and math.isfinite(float(value)))


def label_counts(labels: list[str | None]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for label in labels:
        if label is not None:
            counts[label] = counts.get(label, 0) + 1
    return counts


def build_payload(
    *,
    values: list[float | None],
    labels: list[str | None],
    scanned_rows: list[bool],
    bounds: dict[str, float],
    resolution: float,
    width: int,
    height: int,
    z_top: float,
    z_bottom: float,
    host: str,
    port: int,
    workers: int,
    started: float,
) -> dict:
    known = count_known(values)
    total = width * height
    return {
        "schema": "aerion_heightmap_v1",
        "source": "carla_world_cast_ray",
        "coordinate_frame": "opendrive_map_xy",
        "height_reference": "up_m",
        "resolution_m": resolution,
        "width": width,
        "height": height,
        "bounds": bounds,
        "no_data": None,
        "values": values,
        "labels": labels,
        "coverage": {
            "known_cells": known,
            "total_cells": total,
            "ratio": known / total if total else 0.0,
            "stamped_samples": known,
        },
        "scan": {
            "z_top": z_top,
            "z_bottom": z_bottom,
            "host": host,
            "port": port,
            "workers": max(1, workers),
            "label_counts": label_counts(labels),
            "elapsed_sec": time.time() - started,
            "scanned_rows": scanned_rows,
            "completed_rows": sum(1 for row_done in scanned_rows if row_done),
        },
    }


def write_payload(output: Path, payload: dict) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    tmp = output.with_suffix(output.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
    tmp.replace(output)


def can_resume(data: dict, *, bounds: dict[str, float], resolution: float, width: int, height: int) -> bool:
    if data.get("schema") != "aerion_heightmap_v1":
        return False
    if int(data.get("width", -1)) != width or int(data.get("height", -1)) != height:
        return False
    if abs(float(data.get("resolution_m", -1)) - resolution) > 1e-9:
        return False
    existing_bounds = data.get("bounds") or {}
    return all(abs(float(existing_bounds.get(key, math.nan)) - bounds[key]) < 1e-6 for key in bounds)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--xodr", type=Path, default=DEFAULT_XODR, help="Use OpenDRIVE header bounds")
    parser.add_argument("--bounds", nargs=4, type=float, metavar=("WEST", "EAST", "SOUTH", "NORTH"))
    parser.add_argument("--resolution", type=float, default=3.0)
    parser.add_argument("--z-top", type=float, default=250.0)
    parser.add_argument("--z-bottom", type=float, default=-50.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--workers", type=int, default=1, help="Parallel CARLA client workers")
    parser.add_argument("--progress-every", type=int, default=25)
    parser.add_argument("--checkpoint-every", type=int, default=0, help="Write partial output after this many completed rows")
    parser.add_argument("--resume", action="store_true", help="Resume compatible partial output")
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
    values: list[float | None] = [None] * (width * height)
    labels: list[str | None] = [None] * (width * height)
    scanned_rows: list[bool] = [False] * height
    output = args.output.expanduser().resolve()
    if args.resume and output.exists():
        existing = json.loads(output.read_text(encoding="utf-8"))
        if can_resume(existing, bounds=bounds, resolution=args.resolution, width=width, height=height):
            values = existing.get("values", values)
            labels = existing.get("labels", labels)
            existing_rows = (existing.get("scan") or {}).get("scanned_rows")
            if isinstance(existing_rows, list) and len(existing_rows) == height:
                scanned_rows = [bool(row_done) for row_done in existing_rows]
            print(
                f"Resuming {output} "
                f"(completed_rows={sum(1 for row_done in scanned_rows if row_done)}/{height})",
                flush=True,
            )
        else:
            print(f"Cannot resume incompatible output, starting fresh: {output}", flush=True)

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        world = client.get_world()
    except Exception as exc:
        print(f"CARLA raycast heightmap scan failed to connect: {exc}", file=sys.stderr)
        return 2

    started = time.time()
    total = width * height
    known = count_known(values)
    scanned_at_start = sum(1 for row_done in scanned_rows if row_done)

    if args.workers <= 1:
        for row in range(height):
            if scanned_rows[row]:
                continue
            y = bounds["north"] - (row + 0.5) * args.resolution
            row_known = 0
            for col in range(width):
                x = bounds["west"] + (col + 0.5) * args.resolution
                try:
                    z, label = cast_height(world, x, y, args.z_top, args.z_bottom)
                except Exception as exc:
                    print(f"CARLA raycast failed at row={row} col={col}: {exc}", file=sys.stderr)
                    return 2
                if z is not None:
                    index = row * width + col
                    values[index] = z
                    labels[index] = label
                    known += 1
                    row_known += 1
            scanned_rows[row] = True
            completed_rows = sum(1 for row_done in scanned_rows if row_done)
            if args.checkpoint_every > 0 and completed_rows % args.checkpoint_every == 0:
                write_payload(
                    output,
                    build_payload(
                        values=values,
                        labels=labels,
                        scanned_rows=scanned_rows,
                        bounds=bounds,
                        resolution=args.resolution,
                        width=width,
                        height=height,
                        z_top=args.z_top,
                        z_bottom=args.z_bottom,
                        host=args.host,
                        port=args.port,
                        workers=args.workers,
                        started=started,
                    ),
                )
            if args.progress_every > 0 and (completed_rows - scanned_at_start) % args.progress_every == 0:
                elapsed = time.time() - started
                print(f"  scanned rows {completed_rows}/{height} known={known}/{completed_rows * width} elapsed={elapsed:.1f}s", flush=True)
    else:
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            tasks = []
            for row in range(height):
                if scanned_rows[row]:
                    continue
                future = executor.submit(
                    scan_row,
                    args.host,
                    args.port,
                    args.timeout,
                    row,
                    width,
                    bounds,
                    args.resolution,
                    args.z_top,
                    args.z_bottom,
                )
                tasks.append(future)

            completed_this_run = 0
            for future in as_completed(tasks):
                try:
                    row, row_values, row_labels, row_known = future.result()
                except Exception as exc:
                    print(f"CARLA raycast failed: {exc}", file=sys.stderr)
                    return 2
                start_index = row * width
                values[start_index : start_index + width] = row_values
                labels[start_index : start_index + width] = row_labels
                known += row_known
                scanned_rows[row] = True
                completed_this_run += 1
                completed_rows = scanned_at_start + completed_this_run
                if args.checkpoint_every > 0 and completed_rows % args.checkpoint_every == 0:
                    write_payload(
                        output,
                        build_payload(
                            values=values,
                            labels=labels,
                            scanned_rows=scanned_rows,
                            bounds=bounds,
                            resolution=args.resolution,
                            width=width,
                            height=height,
                            z_top=args.z_top,
                            z_bottom=args.z_bottom,
                            host=args.host,
                            port=args.port,
                            workers=args.workers,
                            started=started,
                        ),
                    )
                if args.progress_every > 0 and completed_this_run % args.progress_every == 0:
                    elapsed = time.time() - started
                    print(
                        f"  scanned rows {completed_rows}/{height} known={known}/{completed_rows * width} elapsed={elapsed:.1f}s",
                        flush=True,
                    )

    payload = build_payload(
        values=values,
        labels=labels,
        scanned_rows=scanned_rows,
        bounds=bounds,
        resolution=args.resolution,
        width=width,
        height=height,
        z_top=args.z_top,
        z_bottom=args.z_bottom,
        host=args.host,
        port=args.port,
        workers=args.workers,
        started=started,
    )
    write_payload(output, payload)
    print(f"Wrote {output} ({width}x{height}, known={known}/{total}, elapsed={time.time() - started:.1f}s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
