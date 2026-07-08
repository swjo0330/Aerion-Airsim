#!/usr/bin/env python3
"""Compute AirSim vehicle spawn coordinates for a clicked mission node."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


def load_airsim_ned_origin(path: Path) -> tuple[dict[str, float], dict]:
    payload = json.loads(path.expanduser().read_text(encoding="utf-8"))
    origin = payload.get("origin_meters")
    if origin is not None:
        origin_meters = {axis: float(origin[axis]) for axis in ("x", "y", "z")}
        return origin_meters, payload

    world_to_meters = float(payload.get("world_to_meters", 100.0))
    origin_ue = payload["origin_ue"]
    return {
        "x": float(origin_ue["x"]) / world_to_meters,
        "y": float(origin_ue["y"]) / world_to_meters,
        "z": float(origin_ue["z"]) / world_to_meters,
    }, payload


def load_node(path: Path, node_index: int) -> dict:
    data = json.loads(path.expanduser().read_text(encoding="utf-8"))
    metadata = data.get("metadata") or {}
    local_source = metadata.get("local_source") if isinstance(metadata, dict) else None
    if isinstance(local_source, dict):
        points = local_source.get("raw_waypoints") or local_source.get("waypoints") or []
    else:
        points = metadata.get("raw_waypoints") or data.get("waypoints") or []
    if not points:
        raise ValueError(f"no waypoints in {path}")
    index = node_index - 1
    if index < 0 or index >= len(points):
        raise ValueError(f"node index out of range: {node_index}")
    return points[index]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--origin", type=Path, required=True, help="recordings/maps/airsim_ned_origin.json")
    parser.add_argument("--node-index", type=int, default=1)
    parser.add_argument("--z", type=float, default=0.2)
    parser.add_argument("--print-shell", action="store_true")
    args = parser.parse_args()

    try:
        node = load_node(args.mission, args.node_index)
        origin, origin_payload = load_airsim_ned_origin(args.origin)
        world_to_meters = float(origin_payload.get("world_to_meters", 100.0))
        node_east = float(node.get("east", node.get("x")))
        node_north = float(node.get("north", node.get("y")))

        # AirSim settings X/Y are global NED meters relative to the UE Play-time
        # NED origin: X is north, Y is east.
        spawn_x = node_north - float(origin["x"])
        spawn_y = node_east - float(origin["y"])
        result = {
            "schema": "aerion_airsim_spawn_for_mission_node_v1",
            "mission": str(args.mission),
            "origin": str(args.origin),
            "node_index": args.node_index,
            "node_map_xy": {
                "east": node_east,
                "north": node_north,
                "up": float(node.get("up", node.get("altitude", 0.0))),
            },
            "airsim_ned_origin_meters": origin,
            "airsim_ned_origin_ue": origin_payload.get("origin_ue"),
            "world_to_meters": world_to_meters,
            "spawn": {"x": spawn_x, "y": spawn_y, "z": args.z},
            "note": "Use spawn.x/y/z as AirSim Vehicles.drone1 X/Y/Z before pressing UE Play.",
        }
    except Exception as exc:
        print(f"spawn computation failed: {exc}", file=sys.stderr)
        return 2

    if args.print_shell:
        print(f"DRONE1_X={result['spawn']['x']:.6f}")
        print(f"DRONE1_Y={result['spawn']['y']:.6f}")
        print(f"DRONE1_Z={result['spawn']['z']:.6f}")
    else:
        print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
