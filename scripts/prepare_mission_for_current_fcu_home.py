#!/usr/bin/env python3
"""Prepare an editor mission for upload relative to the current FCU/AirSim home."""

from __future__ import annotations

import argparse
import contextlib
import json
import math
from pathlib import Path
import sys


def component(point: dict, name: str) -> float:
    sign = -1.0 if name.startswith("-") else 1.0
    key = name[1:] if name.startswith("-") else name
    if key not in ("east", "north"):
        raise ValueError(f"unsupported component expression: {name}")
    return sign * float(point[key])


def transform(point: dict, east_expr: str, north_expr: str) -> tuple[float, float]:
    return component(point, east_expr), component(point, north_expr)


def query_airsim_map_origin(args) -> dict:
    import airsim

    client = airsim.MultirotorClient(
        ip=args.airsim_ip,
        port=args.airsim_port,
        timeout_value=args.airsim_timeout,
    )
    with contextlib.redirect_stdout(sys.stderr):
        client.confirmConnection()
    pose = client.simGetVehiclePose(vehicle_name=args.vehicle)
    pos = pose.position
    if args.coordinate_frame == "airsim_ned":
        # AirSim NED: x=north, y=east, z=down -> OpenDRIVE/map: x=east, y=north, up=-down.
        map_east = float(pos.y_val) + args.map_x_offset
        map_north = float(pos.x_val) + args.map_y_offset
        up = -float(pos.z_val) + args.up_offset
    else:
        map_east = float(pos.x_val) + args.map_x_offset
        map_north = float(pos.y_val) + args.map_y_offset
        up = float(pos.z_val) + args.up_offset
    return {
        "east": map_east,
        "north": map_north,
        "up": up,
        "airsim_position": {
            "x": float(pos.x_val),
            "y": float(pos.y_val),
            "z": float(pos.z_val),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--east-from", choices=("east", "north", "-east", "-north"), default="north")
    parser.add_argument("--north-from", choices=("east", "north", "-east", "-north"), default="east")
    parser.add_argument("--origin-east", type=float, default=None)
    parser.add_argument("--origin-north", type=float, default=None)
    parser.add_argument("--origin-from-airsim", action="store_true")
    parser.add_argument("--origin-from-first-waypoint", action="store_true")
    parser.add_argument("--origin-from-mission-home", action="store_true")
    parser.add_argument("--vehicle", default="drone1")
    parser.add_argument("--coordinate-frame", choices=("airsim_ned", "opendrive"), default="airsim_ned")
    parser.add_argument("--map-x-offset", type=float, default=0.0)
    parser.add_argument("--map-y-offset", type=float, default=0.0)
    parser.add_argument("--up-offset", type=float, default=0.0)
    parser.add_argument("--airsim-ip", default="127.0.0.1")
    parser.add_argument("--airsim-port", type=int, default=41451)
    parser.add_argument("--airsim-timeout", type=float, default=5.0)
    parser.add_argument("--name", default="clicked_mission_current_home_fcu_axes")
    args = parser.parse_args()

    data = json.loads(args.input.expanduser().read_text(encoding="utf-8"))
    frame = str(data.get("coordinate_frame", data.get("frame", ""))).lower()
    if frame not in ("local_enu", "enu"):
        raise ValueError(f"only local_enu missions are supported, got {frame!r}")
    waypoints = data.get("waypoints") or []
    if not waypoints:
        raise ValueError("mission has no waypoints")

    origin_modes = sum(
        bool(value)
        for value in (
            args.origin_from_airsim,
            args.origin_from_first_waypoint,
            args.origin_from_mission_home,
            args.origin_east is not None or args.origin_north is not None,
        )
    )
    if origin_modes != 1:
        raise ValueError(
            "select exactly one origin source: --origin-from-airsim, --origin-from-first-waypoint, "
            "--origin-from-mission-home, or both --origin-east/--origin-north"
        )

    if args.origin_from_airsim:
        try:
            origin = query_airsim_map_origin(args)
        except Exception as exc:
            print(
                "AirSim current-home query failed. Make sure UE is in Play and AirSim RPC is responding "
                f"at {args.airsim_ip}:{args.airsim_port}: {exc}",
                file=sys.stderr,
            )
            return 2
    elif args.origin_from_first_waypoint:
        origin = {
            "east": float(waypoints[0]["east"]),
            "north": float(waypoints[0]["north"]),
            "up": float(waypoints[0].get("up", 0.0)),
        }
    elif args.origin_from_mission_home:
        home = (data.get("metadata") or {}).get("home_origin_map_xy") or {}
        if "east" not in home or "north" not in home:
            raise ValueError("mission metadata has no home_origin_map_xy")
        origin = {"east": float(home["east"]), "north": float(home["north"]), "up": 0.0}
    else:
        origin = {"east": args.origin_east, "north": args.origin_north, "up": 0.0}

    origin_fcu_east, origin_fcu_north = transform(origin, args.east_from, args.north_from)
    transformed_waypoints = []
    for point in waypoints:
        fcu_east, fcu_north = transform(point, args.east_from, args.north_from)
        transformed_waypoints.append(
            {
                **point,
                "east": fcu_east - origin_fcu_east,
                "north": fcu_north - origin_fcu_north,
            }
        )

    first = transformed_waypoints[0]
    first_distance = math.hypot(first["east"], first["north"])
    transformed = {
        **data,
        "name": args.name,
        "coordinate_frame": "local_enu",
        "waypoint_count": len(transformed_waypoints),
        "waypoints": transformed_waypoints,
    }
    metadata = dict(transformed.get("metadata") or {})
    metadata["fcu_home_anchor"] = {
        "source": str(args.input),
        "origin_map_enu": {
            "east": origin["east"],
            "north": origin["north"],
            "up": origin.get("up", 0.0),
        },
        "origin_fcu_axes": {
            "east": origin_fcu_east,
            "north": origin_fcu_north,
        },
        "axis_transform": {
            "east_from": args.east_from,
            "north_from": args.north_from,
        },
        "first_waypoint_relative_distance_m": first_distance,
        "note": "Waypoints are relative to the current AirSim/FCU home at upload time.",
    }
    if "airsim_position" in origin:
        metadata["fcu_home_anchor"]["airsim_position"] = origin["airsim_position"]
    transformed["metadata"] = metadata

    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(transformed, indent=2), encoding="utf-8")
    print(
        f"wrote {output} ({len(transformed_waypoints)} waypoint(s), "
        f"origin map E={origin['east']:.2f} N={origin['north']:.2f}, "
        f"first relative={first['east']:.2f}E {first['north']:.2f}N, "
        f"distance={first_distance:.2f}m)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
