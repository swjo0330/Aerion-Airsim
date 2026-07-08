#!/usr/bin/env python3
"""Transform local_enu mission axes for coordinate-frame validation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def component(point: dict, name: str) -> float:
    sign = -1.0 if name.startswith("-") else 1.0
    key = name[1:] if name.startswith("-") else name
    if key not in ("east", "north"):
        raise ValueError(f"unsupported component expression: {name}")
    return sign * float(point[key])


def transform_waypoint(point: dict, east_expr: str, north_expr: str) -> dict:
    return {
        **point,
        "east": component(point, east_expr),
        "north": component(point, north_expr),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--east-from", choices=("east", "north", "-east", "-north"), default="east")
    parser.add_argument("--north-from", choices=("east", "north", "-east", "-north"), default="north")
    parser.add_argument("--name", default=None)
    args = parser.parse_args()

    data = json.loads(args.input.expanduser().read_text(encoding="utf-8"))
    frame = str(data.get("coordinate_frame", data.get("frame", ""))).lower()
    if frame not in ("local_enu", "enu"):
        raise ValueError(f"only local_enu missions are supported, got {frame!r}")
    waypoints = data.get("waypoints") or []
    if not waypoints:
        raise ValueError("mission has no waypoints")

    transformed = {
        **data,
        "name": args.name or f"{data.get('name', args.input.stem)}_{args.east_from}_to_east_{args.north_from}_to_north",
        "waypoints": [transform_waypoint(point, args.east_from, args.north_from) for point in waypoints],
    }
    metadata = dict(transformed.get("metadata") or {})
    metadata["axis_transform"] = {
        "source": str(args.input),
        "east_from": args.east_from,
        "north_from": args.north_from,
        "note": "Used to validate editor/map axes against UE/AirSim/PX4 mission motion.",
    }
    transformed["metadata"] = metadata

    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(transformed, indent=2), encoding="utf-8")
    print(
        f"wrote {output} ({len(transformed['waypoints'])} waypoint(s), "
        f"east={args.east_from}, north={args.north_from})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
