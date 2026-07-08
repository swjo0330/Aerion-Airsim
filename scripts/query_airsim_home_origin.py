#!/usr/bin/env python3
"""Query the current AirSim vehicle pose as an OpenDRIVE map origin."""

from __future__ import annotations

import argparse
import contextlib
import json
import sys

import airsim


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", default="drone1")
    parser.add_argument("--coordinate-frame", choices=("airsim_ned", "opendrive"), default="airsim_ned")
    parser.add_argument("--map-x-offset", type=float, default=0.0)
    parser.add_argument("--map-y-offset", type=float, default=0.0)
    parser.add_argument("--up-offset", type=float, default=0.0)
    parser.add_argument("--airsim-ip", default="127.0.0.1")
    parser.add_argument("--airsim-port", type=int, default=41451)
    parser.add_argument("--airsim-timeout", type=float, default=5.0)
    args = parser.parse_args()

    client = airsim.MultirotorClient(ip=args.airsim_ip, port=args.airsim_port, timeout_value=args.airsim_timeout)
    try:
        with contextlib.redirect_stdout(sys.stderr):
            client.confirmConnection()
        pose = client.simGetVehiclePose(vehicle_name=args.vehicle)
    except Exception as exc:
        print(f"AirSim home-origin query failed: {exc}", file=sys.stderr)
        return 2
    pos = pose.position

    if args.coordinate_frame == "airsim_ned":
        # AirSim NED: x=north, y=east, z=down -> OpenDRIVE/map: x=east, y=north, up=-down.
        map_x = pos.y_val + args.map_x_offset
        map_y = pos.x_val + args.map_y_offset
        up = -pos.z_val + args.up_offset
    else:
        map_x = pos.x_val + args.map_x_offset
        map_y = pos.y_val + args.map_y_offset
        up = pos.z_val + args.up_offset

    print(
        json.dumps(
            {
                "schema": "aerion_home_origin_v1",
                "vehicle": args.vehicle,
                "coordinate_frame_input": args.coordinate_frame,
                "map_x": map_x,
                "map_y": map_y,
                "up": up,
                "airsim_position": {
                    "x": pos.x_val,
                    "y": pos.y_val,
                    "z": pos.z_val,
                },
                "offsets": {
                    "map_x_offset": args.map_x_offset,
                    "map_y_offset": args.map_y_offset,
                    "up_offset": args.up_offset,
                },
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
