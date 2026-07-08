#!/usr/bin/env python3
"""Extract the AirSim global NED origin from a patched UE log."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re


ORIGIN_RE = re.compile(
    r"AERION_NED_ORIGIN_UE\s+"
    r"x=(?P<x>[-+0-9.eE]+)\s+"
    r"y=(?P<y>[-+0-9.eE]+)\s+"
    r"z=(?P<z>[-+0-9.eE]+)\s+"
    r"world_to_meters=(?P<world_to_meters>[-+0-9.eE]+)"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    text = args.log.expanduser().read_text(encoding="utf-8", errors="replace")
    matches = list(ORIGIN_RE.finditer(text))
    if not matches:
        raise SystemExit(f"AERION_NED_ORIGIN_UE not found in {args.log}")

    match = matches[-1]
    origin = {key: float(value) for key, value in match.groupdict().items()}
    payload = {
        "schema": "aerion_airsim_ned_origin_v1",
        "source_log": str(args.log.expanduser()),
        "origin_ue": {
            "x": origin["x"],
            "y": origin["y"],
            "z": origin["z"],
        },
        "origin_meters": {
            "x": origin["x"] / origin["world_to_meters"],
            "y": origin["y"] / origin["world_to_meters"],
            "z": origin["z"] / origin["world_to_meters"],
        },
        "world_to_meters": origin["world_to_meters"],
        "note": "AirSim Vehicles.X/Y/Z are global NED meters relative to origin_meters.",
    }

    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(
        f"wrote {output}: origin UE x={origin['x']:.3f} y={origin['y']:.3f} "
        f"z={origin['z']:.3f}, world_to_meters={origin['world_to_meters']:.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
