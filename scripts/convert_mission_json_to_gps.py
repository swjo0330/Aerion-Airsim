#!/usr/bin/env python3
"""Convert a local mission JSON into GPS waypoints for MAVROS upload."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

EARTH_RADIUS_M = 6_378_137.0


def waypoint_to_local_enu(item: Any, default_frame: str) -> dict[str, float]:
    if isinstance(item, (list, tuple)) and len(item) >= 3:
        return {"east": float(item[0]), "north": float(item[1]), "up": float(item[2])}
    if not isinstance(item, dict):
        raise ValueError(f"unsupported waypoint item: {item!r}")

    frame = str(item.get("coordinate_frame", item.get("frame", default_frame))).lower()
    if frame in ("local_enu", "enu", ""):
        return {
            "east": float(item["east"]),
            "north": float(item["north"]),
            "up": float(item.get("up", item.get("altitude", 0.0))),
        }
    if frame in ("local_ned", "ned"):
        return {
            "east": float(item["east"]),
            "north": float(item["north"]),
            "up": -float(item.get("down", 0.0)),
        }
    raise ValueError(f"input mission is not local ENU/NED: frame={frame!r}")


def local_to_gps(east_m: float, north_m: float, up_m: float, origin: dict[str, float]) -> dict[str, float]:
    origin_lat_rad = math.radians(origin["latitude"])
    lat = origin["latitude"] + math.degrees(north_m / EARTH_RADIUS_M)
    lon = origin["longitude"] + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(origin_lat_rad)))
    return {"latitude": lat, "longitude": lon, "altitude": up_m}


def convert(data: dict[str, Any], origin: dict[str, float]) -> dict[str, Any]:
    frame = str(data.get("coordinate_frame", data.get("frame", "local_enu"))).lower()
    if frame in ("gps", "global", "wgs84"):
        return data

    local_waypoints = [waypoint_to_local_enu(item, frame) for item in data.get("waypoints", [])]
    metadata = dict(data.get("metadata") or {})
    local_source = {
        "coordinate_frame": frame or "local_enu",
        "home_origin_map_xy": metadata.get("home_origin_map_xy", {"east": 0.0, "north": 0.0}),
        "export_axis_transform": metadata.get("export_axis_transform"),
        "waypoints": local_waypoints,
        "raw_waypoints": metadata.get("raw_waypoints"),
        "raw_waypoint_frame": metadata.get("raw_waypoint_frame"),
    }
    metadata["gps_origin"] = origin
    metadata["altitude_reference"] = "relative"
    metadata["local_source"] = local_source

    return {
        "schema": data.get("schema", "aerion_mission_v1"),
        "name": data.get("name", "clicked_mission"),
        "coordinate_frame": "gps",
        "waypoints": [local_to_gps(wp["east"], wp["north"], wp["up"], origin) for wp in local_waypoints],
        "metadata": metadata,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Input local mission JSON")
    parser.add_argument("--output", required=True, type=Path, help="Output GPS mission JSON")
    parser.add_argument("--origin-lat", type=float, default=37.5665)
    parser.add_argument("--origin-lon", type=float, default=126.9780)
    parser.add_argument("--origin-alt", type=float, default=38.0)
    args = parser.parse_args()

    origin = {"latitude": args.origin_lat, "longitude": args.origin_lon, "altitude": args.origin_alt}
    data = json.loads(args.input.expanduser().read_text(encoding="utf-8"))
    converted = convert(data, origin)

    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(converted, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {output} ({len(converted.get('waypoints', []))} GPS waypoints)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
