#!/usr/bin/env python3
"""Compute map/OpenDRIVE home coordinates for an AirSim global-NED spawn."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


def load_airsim_ned_origin(path: Path) -> dict[str, float]:
    payload = json.loads(path.expanduser().read_text(encoding="utf-8"))
    origin = payload.get("origin_meters")
    if origin is not None:
        return {axis: float(origin[axis]) for axis in ("x", "y", "z")}

    world_to_meters = float(payload.get("world_to_meters", 100.0))
    origin_ue = payload["origin_ue"]
    return {
        "x": float(origin_ue["x"]) / world_to_meters,
        "y": float(origin_ue["y"]) / world_to_meters,
        "z": float(origin_ue["z"]) / world_to_meters,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--origin", type=Path, required=True, help="recordings/maps/airsim_ned_origin.json")
    parser.add_argument("--airsim-x", type=float, required=True)
    parser.add_argument("--airsim-y", type=float, required=True)
    parser.add_argument("--airsim-z", type=float, default=0.2)
    parser.add_argument("--print-shell", action="store_true")
    args = parser.parse_args()

    try:
        origin = load_airsim_ned_origin(args.origin)
        # AirSim settings X/Y are NED meters: X is north, Y is east.
        home_north = float(origin["x"]) + args.airsim_x
        home_east = float(origin["y"]) + args.airsim_y
        home_up = float(origin["z"]) - args.airsim_z
        result = {
            "schema": "aerion_map_home_for_airsim_spawn_v1",
            "origin": str(args.origin),
            "airsim_spawn": {"x": args.airsim_x, "y": args.airsim_y, "z": args.airsim_z},
            "origin_meters": origin,
            "home_map_enu": {"east": home_east, "north": home_north, "up": home_up},
        }
    except Exception as exc:
        print(f"map home computation failed: {exc}", file=sys.stderr)
        return 2

    if args.print_shell:
        print(f"MISSION_HOME_EAST={result['home_map_enu']['east']:.6f}")
        print(f"MISSION_HOME_NORTH={result['home_map_enu']['north']:.6f}")
        print(f"MISSION_HOME_UP={result['home_map_enu']['up']:.6f}")
    else:
        print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
