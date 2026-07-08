#!/usr/bin/env python3
"""Upload a recorded AERION GPS route as a MAVROS mission.

This is the FCU-backed counterpart of ``manual_mavros_control`` route replay:
instead of continuously publishing velocity setpoints, it converts the recorded
GPS JSON into MAVLink mission items through MAVROS.
"""

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from json import JSONDecodeError
from pathlib import Path

import rclpy
from rclpy.node import Node

try:
    from mavros_msgs.msg import HomePosition, Waypoint
    from mavros_msgs.srv import (
        CommandBool,
        CommandLong,
        SetMode,
        WaypointClear,
        WaypointPush,
        WaypointSetCurrent,
    )
except ImportError as exc:  # pragma: no cover - depends on ROS installation
    raise SystemExit('mavros_msgs is required: sudo apt install ros-humble-mavros-msgs') from exc


MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_RESULT_NAMES = {
    0: 'ACCEPTED',
    1: 'TEMPORARILY_REJECTED',
    2: 'DENIED',
    3: 'UNSUPPORTED',
    4: 'FAILED',
    5: 'IN_PROGRESS',
    6: 'CANCELLED',
}


@dataclass
class GpsRoutePoint:
    latitude: float
    longitude: float
    altitude: float


@dataclass
class HomeGeo:
    latitude: float
    longitude: float
    altitude: float


EARTH_RADIUS_M = 6378137.0


def _float_from(item: dict, names: tuple[str, ...], default: float | None = None) -> float:
    for name in names:
        if name in item:
            return float(item[name])
    if default is not None:
        return default
    raise KeyError(names[0])


def _component(value, names: tuple[str, ...], index: int, default: float | None = None) -> float:
    if isinstance(value, dict):
        return _float_from(value, names, default)
    if isinstance(value, (list, tuple)) and len(value) > index:
        return float(value[index])
    if default is not None:
        return default
    raise ValueError(f'missing coordinate component {names[0]}')


def local_to_global(home: HomeGeo, north_m: float, east_m: float, up_m: float) -> GpsRoutePoint:
    lat_rad = math.radians(home.latitude)
    latitude = home.latitude + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = home.longitude + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(lat_rad)))
    altitude = home.altitude + up_m
    return GpsRoutePoint(latitude=latitude, longitude=longitude, altitude=altitude)


def _global_point_from_item(item: dict) -> GpsRoutePoint:
    return GpsRoutePoint(
        latitude=_float_from(item, ('latitude', 'lat', 'x_lat')),
        longitude=_float_from(item, ('longitude', 'lon', 'lng', 'y_long')),
        altitude=_float_from(item, ('altitude', 'alt', 'z_alt'), 0.0),
    )


def _local_point_from_item(item, frame: str, home: HomeGeo) -> GpsRoutePoint:
    if isinstance(item, dict):
        if 'local_enu' in item:
            value = item['local_enu']
            east = _component(value, ('east', 'e', 'x'), 0)
            north = _component(value, ('north', 'n', 'y'), 1)
            up = _component(value, ('up', 'u', 'z', 'altitude', 'alt'), 2, 0.0)
            return local_to_global(home, north, east, up)
        if 'local_ned' in item or 'airsim_ned' in item:
            value = item.get('local_ned', item.get('airsim_ned'))
            north = _component(value, ('north', 'n', 'x'), 0)
            east = _component(value, ('east', 'e', 'y'), 1)
            down = _component(value, ('down', 'd', 'z'), 2, 0.0)
            return local_to_global(home, north, east, -down)
        if 'carla' in item or 'opendrive' in item:
            value = item.get('carla', item.get('opendrive'))
            east = _component(value, ('x', 'east', 'e'), 0)
            north = _component(value, ('y', 'north', 'n'), 1)
            up = _component(value, ('z', 'up', 'altitude', 'alt'), 2, 0.0)
            return local_to_global(home, north, east, up)

        item_frame = str(item.get('frame', item.get('coordinate_frame', frame))).lower()
        if item_frame in ('global', 'gps', 'wgs84'):
            return _global_point_from_item(item)
        if item_frame in ('local_enu', 'enu'):
            east = _float_from(item, ('east', 'e', 'x'), 0.0)
            north = _float_from(item, ('north', 'n', 'y'), 0.0)
            up = _float_from(item, ('up', 'u', 'z', 'altitude', 'alt'), 0.0)
            return local_to_global(home, north, east, up)
        if item_frame in ('local_ned', 'airsim_ned', 'ned'):
            north = _float_from(item, ('north', 'n', 'x'), 0.0)
            east = _float_from(item, ('east', 'e', 'y'), 0.0)
            down = _float_from(item, ('down', 'd', 'z'), 0.0)
            return local_to_global(home, north, east, -down)
        if item_frame in ('carla', 'opendrive', 'carla_opendrive'):
            east = _float_from(item, ('x', 'east', 'e'), 0.0)
            north = _float_from(item, ('y', 'north', 'n'), 0.0)
            up = _float_from(item, ('z', 'up', 'altitude', 'alt'), 0.0)
            return local_to_global(home, north, east, up)

    if frame in ('local_enu', 'enu'):
        east = _component(item, ('east',), 0)
        north = _component(item, ('north',), 1)
        up = _component(item, ('up',), 2, 0.0)
        return local_to_global(home, north, east, up)
    if frame in ('local_ned', 'airsim_ned', 'ned'):
        north = _component(item, ('north',), 0)
        east = _component(item, ('east',), 1)
        down = _component(item, ('down',), 2, 0.0)
        return local_to_global(home, north, east, -down)
    if frame in ('carla', 'opendrive', 'carla_opendrive'):
        east = _component(item, ('x',), 0)
        north = _component(item, ('y',), 1)
        up = _component(item, ('z',), 2, 0.0)
        return local_to_global(home, north, east, up)
    raise ValueError(f'unsupported waypoint/frame: {frame}')


def _load_route_json(path: Path) -> dict:
    text = path.read_text(encoding='utf-8')
    try:
        data = json.loads(text)
    except JSONDecodeError as exc:
        first_line = text.splitlines()[0].strip().lower() if text.splitlines() else ''
        if first_line in ('east,north,up', 'north,east,down', 'x,y,z'):
            raise ValueError(
                f'{path} looks like CSV, not JSON. Use the editor Mission JSON button, '
                'or convert the CSV with aerion_mission_builder before upload.'
            ) from exc
        raise
    if not isinstance(data, dict):
        raise ValueError(f'{path} must contain a JSON object mission/route')
    return data


def route_requires_home(path: Path | str) -> bool:
    path = Path(path)
    data = _load_route_json(path)
    if 'waypoints_ned_rel' in data:
        return True
    frame = str(data.get('frame', data.get('coordinate_frame', 'global'))).lower()
    if frame not in ('global', 'gps', 'wgs84'):
        return True
    for item in data.get('waypoints', []):
        if isinstance(item, dict):
            item_frame = str(item.get('frame', item.get('coordinate_frame', frame))).lower()
            if item_frame not in ('global', 'gps', 'wgs84'):
                return True
            has_latlon = (
                {'latitude', 'longitude'}.issubset(item.keys())
                or {'lat', 'lon'}.issubset(item.keys())
                or {'x_lat', 'y_long'}.issubset(item.keys())
            )
            if not has_latlon:
                return True
    return False


def load_route(path: Path | str, home: HomeGeo | None = None) -> list[GpsRoutePoint]:
    path = Path(path)
    data = _load_route_json(path)
    frame = str(data.get('frame', data.get('coordinate_frame', 'global'))).lower()
    points = []

    if 'waypoints_ned_rel' in data:
        if home is None:
            raise ValueError('waypoints_ned_rel requires current home position')
        for item in data.get('waypoints_ned_rel', []):
            points.append(_local_point_from_item(item, 'local_ned', home))
    else:
        for item in data.get('waypoints', []):
            if isinstance(item, dict):
                item_frame = str(item.get('frame', item.get('coordinate_frame', frame))).lower()
                if item_frame in ('global', 'gps', 'wgs84') and (
                    'latitude' in item or 'lat' in item or 'x_lat' in item
                ):
                    points.append(_global_point_from_item(item))
                else:
                    if home is None:
                        raise ValueError(f'{item_frame} waypoint requires current home position')
                    points.append(_local_point_from_item(item, item_frame, home))
            else:
                if home is None:
                    raise ValueError(f'{frame} waypoint requires current home position')
                points.append(_local_point_from_item(item, frame, home))

    if not points:
        raise ValueError(f'no waypoints found in {path}')
    return points


def altitude_for(
    point: GpsRoutePoint,
    points: list[GpsRoutePoint],
    mode: str,
    reference_altitude: float | None,
) -> float:
    if mode == 'absolute':
        return point.altitude
    if mode == 'relative':
        return point.altitude
    if mode == 'relative_from_first':
        return point.altitude - points[0].altitude
    if mode == 'relative_to_home':
        if reference_altitude is None:
            raise ValueError('relative_to_home requires a resolved home altitude')
        return point.altitude - reference_altitude
    raise ValueError(f'unsupported altitude mode: {mode}')


def mission_frame(altitude_mode: str) -> int:
    if altitude_mode == 'absolute':
        return Waypoint.FRAME_GLOBAL
    return Waypoint.FRAME_GLOBAL_REL_ALT


def build_waypoint(
    point: GpsRoutePoint,
    points: list[GpsRoutePoint],
    altitude_mode: str,
    command: int,
    is_current: bool,
    acceptance_radius: float,
    hold_time: float,
    yaw: float,
    reference_altitude: float | None,
) -> Waypoint:
    wp = Waypoint()
    wp.frame = mission_frame(altitude_mode)
    wp.command = command
    wp.is_current = is_current
    wp.autocontinue = True
    wp.param1 = float(hold_time)
    wp.param2 = float(acceptance_radius)
    wp.param3 = 0.0
    wp.param4 = float(yaw) if math.isfinite(yaw) else float('nan')
    wp.x_lat = point.latitude
    wp.y_long = point.longitude
    wp.z_alt = altitude_for(point, points, altitude_mode, reference_altitude)
    return wp


def build_mission(
    points: list[GpsRoutePoint],
    altitude_mode: str,
    include_takeoff: bool,
    takeoff_altitude: float,
    takeoff_altitude_source: str,
    takeoff_position_source: str,
    acceptance_radius: float,
    hold_time: float,
    yaw: float,
    reference_altitude: float | None = None,
    home_point: GpsRoutePoint | None = None,
) -> list[Waypoint]:
    mission = []
    first = points[0]
    if include_takeoff:
        route_takeoff_altitude = altitude_for(first, points, altitude_mode, reference_altitude)
        if takeoff_altitude_source == 'first_waypoint':
            takeoff_altitude = route_takeoff_altitude
        elif takeoff_altitude_source == 'max_fixed_or_first':
            takeoff_altitude = max(takeoff_altitude, route_takeoff_altitude)
        elif takeoff_altitude_source != 'fixed':
            raise ValueError(f'unsupported takeoff altitude source: {takeoff_altitude_source}')

        if not math.isfinite(takeoff_altitude) or takeoff_altitude <= 0.0:
            raise ValueError(f'takeoff altitude must be positive: {takeoff_altitude}')

        if takeoff_position_source == 'current':
            if home_point is None:
                raise ValueError('takeoff-position-source=current requires current home position')
            takeoff_point = GpsRoutePoint(home_point.latitude, home_point.longitude, takeoff_altitude)
        elif takeoff_position_source == 'first_waypoint':
            takeoff_point = GpsRoutePoint(first.latitude, first.longitude, takeoff_altitude)
        else:
            raise ValueError(f'unsupported takeoff position source: {takeoff_position_source}')
        mission.append(
            build_waypoint(
                takeoff_point,
                [takeoff_point],
                'relative',
                MAV_CMD_NAV_TAKEOFF,
                True,
                acceptance_radius,
                0.0,
                yaw,
                None,
            )
        )

    for index, point in enumerate(points):
        mission.append(
            build_waypoint(
                point,
                points,
                altitude_mode,
                MAV_CMD_NAV_WAYPOINT,
                is_current=(index == 0 and not include_takeoff),
                acceptance_radius=acceptance_radius,
                hold_time=hold_time,
                yaw=yaw,
                reference_altitude=reference_altitude,
            )
        )
    return mission


class MissionUploader(Node):
    def __init__(self, namespace: str, timeout: float):
        super().__init__('aerion_gps_route_mission')
        self.namespace = '/' + namespace.strip('/')
        self.timeout = timeout
        self.last_home = None

    def wait_for(self, client, service_name: str) -> bool:
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.2):
                return True
        self.get_logger().error(f'service unavailable: {service_name}')
        return False

    def call(self, service_type, suffix: str, request):
        service_name = f'{self.namespace}{suffix}'
        client = self.create_client(service_type, service_name)
        if not self.wait_for(client, service_name):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                return future.result()
        self.get_logger().error(f'service timeout: {service_name}')
        return None

    def wait_home_altitude(self) -> float | None:
        topic_name = f'{self.namespace}/home_position/home'
        latest = {'altitude': None}

        def on_home(message: HomePosition) -> None:
            self.last_home = message
            latest['altitude'] = float(message.geo.altitude)

        subscription = self.create_subscription(HomePosition, topic_name, on_home, 10)
        deadline = time.monotonic() + self.timeout
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if latest['altitude'] is not None and math.isfinite(latest['altitude']):
                    altitude = latest['altitude']
                    self.get_logger().info(
                        f'home altitude from {topic_name}: {altitude:.3f}m'
                    )
                    return altitude
        finally:
            self.destroy_subscription(subscription)

        self.get_logger().error(f'home altitude unavailable: {topic_name}')
        return None

    def clear(self) -> bool:
        result = self.call(WaypointClear, '/mission/clear', WaypointClear.Request())
        ok = bool(result and result.success)
        self.get_logger().info(f'mission clear: success={ok}')
        return ok

    def push(self, mission: list[Waypoint]) -> bool:
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = mission
        result = self.call(WaypointPush, '/mission/push', request)
        ok = bool(result and result.success and result.wp_transfered == len(mission))
        transferred = getattr(result, 'wp_transfered', 0) if result else 0
        self.get_logger().info(
            f'mission push: success={ok} transferred={transferred}/{len(mission)}'
        )
        return ok

    def set_current(self, seq: int) -> bool:
        request = WaypointSetCurrent.Request()
        request.wp_seq = int(seq)
        result = self.call(WaypointSetCurrent, '/mission/set_current', request)
        ok = bool(result and result.success)
        self.get_logger().info(f'mission set_current({seq}): success={ok}')
        return ok

    def arm(self) -> bool:
        request = CommandBool.Request()
        request.value = True
        result = self.call(CommandBool, '/cmd/arming', request)
        ok = bool(result and result.success)
        self.get_logger().info(f'arm: success={ok}')
        return ok

    def force_arm(self) -> bool:
        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_COMPONENT_ARM_DISARM
        request.confirmation = 0
        request.param1 = 1.0
        request.param2 = 21196.0
        result = self.call(CommandLong, '/cmd/command', request)
        ok = bool(result and result.success)
        ack_result = getattr(result, 'result', None) if result else None
        ack_name = MAV_RESULT_NAMES.get(ack_result, 'UNKNOWN')
        self.get_logger().info(f'force_arm: success={ok} result={ack_result}({ack_name})')
        return ok

    def set_mode(self, custom_mode: str) -> bool:
        request = SetMode.Request()
        request.custom_mode = custom_mode
        result = self.call(SetMode, '/set_mode', request)
        ok = bool(result and result.mode_sent)
        self.get_logger().info(f'set_mode({custom_mode}): sent={ok}')
        return ok


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Upload recorded GPS route as MAVROS mission')
    parser.add_argument('--route-file', required=True, help='recordings/gps/gps_route_*.json')
    parser.add_argument('--namespace', default='drone1/mavros', help='MAVROS namespace')
    parser.add_argument(
        '--firmware',
        choices=('px4', 'ardupilot'),
        default='px4',
        help='selects default AUTO mode string',
    )
    parser.add_argument(
        '--auto-mode',
        default='',
        help='override AUTO mission mode, e.g. AUTO.MISSION for PX4 or AUTO for ArduPilot',
    )
    parser.add_argument(
        '--altitude-mode',
        choices=('absolute', 'relative', 'relative_from_first', 'relative_to_home'),
        default='relative',
        help='how route waypoint altitude is written into mission items',
    )
    parser.add_argument('--include-takeoff', action='store_true', help='prepend MAV_CMD_NAV_TAKEOFF')
    parser.add_argument('--takeoff-altitude', type=float, default=5.0, help='relative takeoff altitude in meters')
    parser.add_argument(
        '--takeoff-altitude-source',
        choices=('fixed', 'first_waypoint', 'max_fixed_or_first'),
        default='fixed',
        help='choose the takeoff altitude from --takeoff-altitude or converted route altitude',
    )
    parser.add_argument(
        '--takeoff-position-source',
        choices=('current', 'first_waypoint'),
        default='current',
        help='where the prepended takeoff command is located',
    )
    parser.add_argument('--acceptance-radius', type=float, default=2.0, help='waypoint acceptance radius in meters')
    parser.add_argument('--hold-time', type=float, default=0.0, help='hold time at each waypoint in seconds')
    parser.add_argument('--yaw', type=float, default=float('nan'), help='mission yaw angle; NaN lets FCU decide')
    parser.add_argument('--skip-clear', action='store_true', help='do not clear existing mission before push')
    parser.add_argument('--skip-set-current', action='store_true', help='do not set current mission index to 0')
    parser.add_argument('--arm', action='store_true', help='arm after upload')
    parser.add_argument('--force-arm', action='store_true', help='use PX4 force-arm magic if normal arm fails')
    parser.add_argument('--start', action='store_true', help='switch to AUTO mission mode after upload')
    parser.add_argument('--timeout', type=float, default=8.0, help='service wait/call timeout in seconds')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    route_path = Path(args.route_file).expanduser()
    try:
        needs_home = (
            route_requires_home(route_path)
            or args.altitude_mode == 'relative_to_home'
            or (args.include_takeoff and args.takeoff_position_source == 'current')
        )
        if route_requires_home(route_path) and args.altitude_mode == 'relative':
            raise ValueError(
                'local-coordinate mission files must use --altitude-mode relative_to_home '
                'or absolute; relative would treat home+up as a relative altitude'
            )
    except Exception as exc:
        print(f'[gps_route_mission] route inspection failed: {exc}', file=sys.stderr)
        return 2

    auto_mode = args.auto_mode or ('AUTO.MISSION' if args.firmware == 'px4' else 'AUTO')
    rclpy.init()
    node = MissionUploader(args.namespace, args.timeout)
    try:
        home = None
        reference_altitude = None
        if needs_home:
            home_altitude = node.wait_home_altitude()
            if home_altitude is None:
                return 2
            home_message = node.last_home
            if home_message is None:
                return 2
            home = HomeGeo(
                latitude=float(home_message.geo.latitude),
                longitude=float(home_message.geo.longitude),
                altitude=home_altitude,
            )
            reference_altitude = home_altitude if args.altitude_mode == 'relative_to_home' else None

        points = load_route(route_path, home=home)

        mission = build_mission(
            points=points,
            altitude_mode=args.altitude_mode,
            include_takeoff=args.include_takeoff,
            takeoff_altitude=args.takeoff_altitude,
            takeoff_altitude_source=args.takeoff_altitude_source,
            takeoff_position_source=args.takeoff_position_source,
            acceptance_radius=args.acceptance_radius,
            hold_time=args.hold_time,
            yaw=args.yaw,
            reference_altitude=reference_altitude,
            home_point=GpsRoutePoint(home.latitude, home.longitude, args.takeoff_altitude) if home else None,
        )
        node.get_logger().info(
            f'loaded {len(points)} route waypoint(s); uploading {len(mission)} mission item(s) '
            f'to /{args.namespace.strip("/")}'
        )
        if not args.skip_clear and not node.clear():
            return 3
        if not node.push(mission):
            return 4
        if not args.skip_set_current and not node.set_current(0):
            return 5
        if args.arm:
            arm_ok = node.arm()
            if not arm_ok and args.force_arm:
                arm_ok = node.force_arm()
            if not arm_ok:
                return 6
        if args.start and not node.set_mode(auto_mode):
            return 7
        node.get_logger().info('mission upload complete')
        return 0
    except ValueError as exc:
        print(f'[gps_route_mission] route conversion failed: {exc}', file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
