#!/usr/bin/env python3
"""MAVSDK GPS mission smoke for AirSim + PX4.

This intentionally talks to PX4 through the MAVLink mission protocol only.
AirSim RPC and ROS/MAVROS are not used here; the goal is to prove that the
PX4 offboard/API UDP channel can accept, run, and finish a GPS mission.
"""

from __future__ import annotations

import argparse
import asyncio
import inspect
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

try:
    from mavsdk import System
    from mavsdk.mission import MissionItem, MissionPlan
except ImportError as exc:  # pragma: no cover - depends on operator machine
    raise SystemExit(
        "mavsdk is required. Install once with: python3 -m pip install mavsdk"
    ) from exc


EARTH_RADIUS_M = 6378137.0


@dataclass(frozen=True)
class GeoPoint:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float = 0.0


def parse_waypoints(text: str, altitude_m: float) -> list[GeoPoint]:
    points = []
    for raw in text.split(";"):
        raw = raw.strip()
        if not raw:
            continue
        parts = [part.strip() for part in raw.split(",")]
        if len(parts) not in (2, 3):
            raise ValueError(f"waypoint must be lat,lon or lat,lon,alt: {raw}")
        lat = float(parts[0])
        lon = float(parts[1])
        alt = float(parts[2]) if len(parts) == 3 else altitude_m
        points.append(GeoPoint(lat, lon, alt))
    if not points:
        raise ValueError("--wps did not contain any waypoints")
    return points


def _float_field(item: dict, names: tuple[str, ...], default: float | None = None) -> float:
    for name in names:
        if name in item and item[name] not in (None, ""):
            return float(item[name])
    if default is not None:
        return float(default)
    raise KeyError(names[0])


def load_mission_file(path: str, altitude_m: float) -> list[GeoPoint]:
    mission_path = Path(path).expanduser()
    with mission_path.open(encoding="utf-8") as fh:
        payload = json.load(fh)

    frame = str(payload.get("coordinate_frame", payload.get("frame", "gps"))).lower()
    if frame not in {"gps", "global", "wgs84"}:
        raise ValueError(
            f"{mission_path} is coordinate_frame={frame!r}; mission_smoke requires GPS/global waypoints"
        )

    raw_waypoints = payload.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError(f"{mission_path} does not contain any waypoints")

    points: list[GeoPoint] = []
    for index, item in enumerate(raw_waypoints, start=1):
        if not isinstance(item, dict):
            raise ValueError(f"waypoint #{index} must be an object")
        try:
            lat = _float_field(item, ("latitude", "lat", "x_lat"))
            lon = _float_field(item, ("longitude", "lon", "lng", "y_long"))
            alt = _float_field(item, ("altitude", "alt", "z_alt"), altitude_m)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"waypoint #{index} must contain latitude, longitude, and optional altitude"
            ) from exc
        points.append(GeoPoint(lat, lon, alt))
    return points


def offset_meters(origin: GeoPoint, north_m: float, east_m: float, altitude_m: float) -> GeoPoint:
    lat_rad = math.radians(origin.latitude_deg)
    latitude = origin.latitude_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = origin.longitude_deg + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(lat_rad)))
    return GeoPoint(latitude, longitude, altitude_m)


def meters_between(a: GeoPoint, b: GeoPoint) -> tuple[float, float, float]:
    lat_rad = math.radians(a.latitude_deg)
    north = math.radians(b.latitude_deg - a.latitude_deg) * EARTH_RADIUS_M
    east = math.radians(b.longitude_deg - a.longitude_deg) * EARTH_RADIUS_M * math.cos(lat_rad)
    return north, east, math.hypot(north, east)


def interpolate_segment(start: GeoPoint, end: GeoPoint, spacing_m: float) -> list[GeoPoint]:
    north, east, distance = meters_between(start, end)
    if distance < 0.01:
        return []
    steps = max(1, math.ceil(distance / spacing_m))
    return [
        offset_meters(
            start,
            north * i / steps,
            east * i / steps,
            start.altitude_m + (end.altitude_m - start.altitude_m) * i / steps,
        )
        for i in range(1, steps + 1)
    ]


def build_route(
    home: GeoPoint,
    targets: Iterable[GeoPoint] | None,
    altitude_m: float,
    forward_m: float,
    spacing_m: float,
    bearing_deg: float,
) -> list[GeoPoint]:
    if spacing_m <= 0.0:
        raise ValueError("--interp-m must be positive")
    if targets is None:
        bearing_rad = math.radians(bearing_deg)
        count = max(1, math.ceil(forward_m / spacing_m))
        return [
            offset_meters(
                home,
                min(spacing_m * i, forward_m) * math.cos(bearing_rad),
                min(spacing_m * i, forward_m) * math.sin(bearing_rad),
                altitude_m,
            )
            for i in range(1, count + 1)
        ]

    route: list[GeoPoint] = []
    current = GeoPoint(home.latitude_deg, home.longitude_deg, altitude_m)
    for target in targets:
        target = GeoPoint(target.latitude_deg, target.longitude_deg, target.altitude_m or altitude_m)
        segment = interpolate_segment(current, target, spacing_m)
        route.extend(segment)
        current = target
    if not route:
        raise ValueError("mission has no movement; target is effectively the current home")
    return route


def enum_value(enum_class, name: str, fallback: int = 0):
    return getattr(enum_class, name, fallback) if enum_class is not None else fallback


def make_mission_item(point: GeoPoint, speed_m_s: float, acceptance_radius_m: float):
    kwargs = {
        "latitude_deg": point.latitude_deg,
        "longitude_deg": point.longitude_deg,
        "relative_altitude_m": point.altitude_m,
        "speed_m_s": speed_m_s,
        "is_fly_through": True,
        "gimbal_pitch_deg": float("nan"),
        "gimbal_yaw_deg": float("nan"),
        "loiter_time_s": 0.0,
        "camera_photo_interval_s": 0.0,
        "acceptance_radius_m": acceptance_radius_m,
        "yaw_deg": float("nan"),
        "camera_photo_distance_m": 0.0,
    }
    camera_action = getattr(MissionItem, "CameraAction", None)
    vehicle_action = getattr(MissionItem, "VehicleAction", None)
    kwargs["camera_action"] = enum_value(camera_action, "NONE")
    kwargs["vehicle_action"] = enum_value(vehicle_action, "NONE")
    signature = inspect.signature(MissionItem)
    params = signature.parameters
    filtered = {name: value for name, value in kwargs.items() if name in params}
    try:
        return MissionItem(**filtered)
    except TypeError:
        common_args = [
            kwargs["latitude_deg"],
            kwargs["longitude_deg"],
            kwargs["relative_altitude_m"],
            kwargs["speed_m_s"],
            kwargs["is_fly_through"],
            kwargs["gimbal_pitch_deg"],
            kwargs["gimbal_yaw_deg"],
            kwargs["camera_action"],
            kwargs["loiter_time_s"],
            kwargs["camera_photo_interval_s"],
            kwargs["acceptance_radius_m"],
            kwargs["yaw_deg"],
            kwargs["camera_photo_distance_m"],
            kwargs["vehicle_action"],
        ]
        try:
            return MissionItem(*common_args)
        except TypeError:
            return MissionItem(*common_args[:-1])


def point_from_mission_item(item) -> GeoPoint:
    return GeoPoint(
        float(getattr(item, "latitude_deg")),
        float(getattr(item, "longitude_deg")),
        float(getattr(item, "relative_altitude_m")),
    )


def compare_routes(expected: list[GeoPoint], actual_items, tolerance_m: float) -> None:
    actual = [point_from_mission_item(item) for item in actual_items]
    if len(actual) != len(expected):
        raise RuntimeError(f"download count mismatch: expected {len(expected)}, got {len(actual)}")
    for index, (want, got) in enumerate(zip(expected, actual), start=1):
        north, east, distance = meters_between(want, got)
        alt_error = abs(want.altitude_m - got.altitude_m)
        if distance > tolerance_m or alt_error > 0.2:
            raise RuntimeError(
                "download item mismatch "
                f"#{index}: horizontal={distance:.3f}m north={north:.3f} east={east:.3f} alt={alt_error:.3f}m"
            )


async def next_before(aiter, deadline: float, label: str):
    remaining = deadline - time.monotonic()
    if remaining <= 0.0:
        raise TimeoutError(label)
    try:
        return await asyncio.wait_for(anext(aiter), timeout=remaining)
    except asyncio.TimeoutError as exc:
        raise TimeoutError(label) from exc


async def wait_connected(drone: System, timeout_s: float) -> None:
    print(f"[S1] waiting for heartbeat on MAVLink channel, timeout={timeout_s:.0f}s")
    deadline = time.monotonic() + timeout_s
    states = drone.core.connection_state()
    while True:
        state = await next_before(states, deadline, "heartbeat timeout; check UE Play, PX4 SITL, and UDP bind port")
        if state.is_connected:
            print("[S1] connected")
            return


async def wait_home(drone: System, timeout_s: float) -> GeoPoint:
    print(f"[S2] waiting for GPS/home health, timeout={timeout_s:.0f}s")
    deadline = time.monotonic() + timeout_s
    latest_position = None
    health_stream = drone.telemetry.health()
    position_stream = drone.telemetry.position()
    position_task = asyncio.create_task(next_before(position_stream, deadline, "position timeout"))
    try:
        while True:
            health = await next_before(
                health_stream,
                deadline,
                "GPS/home did not become healthy; check OriginGeopoint and AirSim EKF",
            )
            if position_task.done():
                latest_position = position_task.result()
            if (
                latest_position is not None
                and health.is_global_position_ok
                and health.is_home_position_ok
            ):
                home = GeoPoint(
                    float(latest_position.latitude_deg),
                    float(latest_position.longitude_deg),
                    0.0,
                )
                print(
                    "[S2] health ok; current/home GPS "
                    f"lat={home.latitude_deg:.7f} lon={home.longitude_deg:.7f}"
                )
                return home
            if position_task.done():
                position_task = asyncio.create_task(next_before(position_stream, deadline, "position timeout"))
    finally:
        if not position_task.done():
            position_task.cancel()


async def upload_and_verify(drone: System, items: list, route: list[GeoPoint], tolerance_m: float) -> None:
    print(f"[S4] uploading {len(items)} mission item(s)")
    try:
        await drone.mission.clear_mission()
    except Exception as exc:
        print(f"[S4] clear_mission warning: {exc}")
    await drone.mission.upload_mission(MissionPlan(items))
    downloaded = await drone.mission.download_mission()
    compare_routes(route, downloaded.mission_items, tolerance_m)
    print("[S4] upload/download round-trip matches")


async def start_and_watch(drone: System, timeout_s: float, rtl: bool) -> None:
    print("[S5] arming and starting AUTO.MISSION")
    await drone.action.arm()
    await drone.mission.start_mission()

    deadline = time.monotonic() + timeout_s
    last = None
    progress_stream = drone.mission.mission_progress()
    while True:
        progress = await next_before(progress_stream, deadline, "mission did not complete before timeout")
        current_total = (progress.current, progress.total)
        if current_total != last:
            print(f"[S6] mission progress {progress.current}/{progress.total}")
            last = current_total
        if progress.total > 0 and progress.current >= progress.total:
            print("[S6] mission complete")
            break

    if rtl:
        print("[S6] RTL")
        await drone.action.return_to_launch()
        disarm_deadline = time.monotonic() + 180.0
        armed_stream = drone.telemetry.armed()
        while True:
            try:
                armed = await next_before(armed_stream, disarm_deadline, "disarm timeout")
            except TimeoutError:
                print("[S6] RTL requested; disarm not observed before timeout")
                return
            if not armed:
                print("[S6] disarmed")
                return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AirSim/PX4 MAVSDK GPS mission smoke")
    parser.add_argument("--url", default="", help="team-lead script alias for --system-address, e.g. udpin://0.0.0.0:14540")
    parser.add_argument("--system-address", default="", help="MAVSDK address, e.g. udp://:14540")
    parser.add_argument("--port", type=int, default=14540, help="UDP listen port used when --system-address is omitted")
    parser.add_argument("--alt", type=float, default=10.0, help="relative mission altitude in meters")
    parser.add_argument("--wps", default="", help='absolute GPS targets: "lat,lon;lat,lon,alt"')
    parser.add_argument("--mission-file", default="", help="GPS mission JSON exported by the mission editor")
    parser.add_argument("--forward-m", type=float, default=1000.0, help="default northbound smoke distance")
    parser.add_argument("--interp-m", type=float, default=200.0, help="mission item spacing")
    parser.add_argument("--bearing", type=float, default=0.0, help="default route bearing in degrees; 0=north")
    parser.add_argument("--speed", type=float, default=5.0, help="mission speed in m/s")
    parser.add_argument("--acceptance-radius", type=float, default=5.0, help="waypoint acceptance radius in meters")
    parser.add_argument("--connect-timeout", type=float, default=20.0)
    parser.add_argument("--health-timeout", type=float, default=90.0)
    parser.add_argument("--mission-timeout", type=float, default=420.0)
    parser.add_argument("--verify-tolerance-m", type=float, default=0.5)
    parser.add_argument("--no-rtl", action="store_true", help="do not return to launch after mission completion")
    return parser.parse_args()


async def async_main(args: argparse.Namespace) -> int:
    address = args.system_address or args.url or f"udp://:{args.port}"
    print(f"[S0] connecting MAVSDK to {address}")
    drone = System()
    await drone.connect(system_address=address)
    await wait_connected(drone, args.connect_timeout)
    home = await wait_home(drone, args.health_timeout)

    if args.mission_file:
        route = load_mission_file(args.mission_file, args.alt)
    else:
        targets = parse_waypoints(args.wps, args.alt) if args.wps else None
        route = build_route(home, targets, args.alt, args.forward_m, args.interp_m, args.bearing)
    print(
        f"[S3] built route: {len(route)} item(s), "
        f"first={route[0].latitude_deg:.7f},{route[0].longitude_deg:.7f}, "
        f"last={route[-1].latitude_deg:.7f},{route[-1].longitude_deg:.7f}"
    )
    items = [make_mission_item(point, args.speed, args.acceptance_radius) for point in route]
    set_rtl = getattr(drone.mission, "set_return_to_launch_after_mission", None)
    if set_rtl is not None:
        await set_rtl(not args.no_rtl)
    await upload_and_verify(drone, items, route, args.verify_tolerance_m)
    await start_and_watch(drone, args.mission_timeout, rtl=not args.no_rtl)
    print("PASS: mission upload, round-trip verification, execution, and completion succeeded")
    return 0


def main() -> int:
    args = parse_args()
    try:
        return asyncio.run(async_main(args))
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
