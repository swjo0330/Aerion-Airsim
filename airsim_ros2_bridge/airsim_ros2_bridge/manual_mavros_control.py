#!/usr/bin/env python3
"""Keyboard velocity teleop for AERION MAVROS/PX4 validation.

Publishes geometry_msgs/Twist to a MAVROS setpoint_velocity/cmd_vel_unstamped
topic at a fixed rate. This is intentionally small and dependency-light so it
can be used during CARLA map lag checks without bringing up the formation node.
"""

import argparse
import csv
import json
import math
import os
import select
import sys
import termios
import time
import tty
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

try:
    from mavros_msgs.srv import CommandBool, CommandLong, SetMode
except ImportError:
    CommandBool = None
    CommandLong = None
    SetMode = None


KEY_BINDINGS = {
    'w': (1.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0, 0.0),
    'd': (0.0, -1.0, 0.0, 0.0),
    'r': (0.0, 0.0, 1.0, 0.0),
    'f': (0.0, 0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 0.0, 1.0),
    'e': (0.0, 0.0, 0.0, -1.0),
    ' ': (0.0, 0.0, 0.0, 0.0),
}


@dataclass
class GpsWaypoint:
    latitude: float
    longitude: float
    altitude: float
    stamp_sec: int
    stamp_nanosec: int
    recorded_at: str


class GpsRouteRecorder:
    def __init__(self, node, topic, output_dir, route_file):
        self._node = node
        self._topic = topic
        self._output_dir = Path(output_dir).expanduser()
        self._route_file = Path(route_file).expanduser() if route_file else None
        self._latest_fix = None
        self._waypoints = []
        self._loaded_waypoints = []
        self._node.create_subscription(NavSatFix, topic, self._on_fix, qos_profile_sensor_data)

    @property
    def latest_fix(self):
        return self._latest_fix

    @property
    def active_waypoints(self):
        return self._loaded_waypoints or self._waypoints

    def _on_fix(self, msg):
        self._latest_fix = msg

    def record_current(self):
        if self._latest_fix is None:
            self._node.get_logger().warn(f'GPS record skipped; no fix received from {self._topic}')
            return False
        stamp = self._latest_fix.header.stamp
        waypoint = GpsWaypoint(
            latitude=float(self._latest_fix.latitude),
            longitude=float(self._latest_fix.longitude),
            altitude=float(self._latest_fix.altitude),
            stamp_sec=int(stamp.sec),
            stamp_nanosec=int(stamp.nanosec),
            recorded_at=datetime.now().isoformat(timespec='seconds'),
        )
        self._waypoints.append(waypoint)
        self._node.get_logger().info(
            f'GPS waypoint #{len(self._waypoints)} recorded: '
            f'lat={waypoint.latitude:.7f}, lon={waypoint.longitude:.7f}, alt={waypoint.altitude:.2f}m'
        )
        return True

    def export(self):
        if not self._waypoints:
            self._node.get_logger().warn('GPS export skipped; no recorded waypoints')
            return None
        self._output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        json_path = self._output_dir / f'gps_route_{timestamp}.json'
        csv_path = self._output_dir / f'gps_route_{timestamp}.csv'
        payload = {
            'created_at': datetime.now().isoformat(timespec='seconds'),
            'source_topic': self._topic,
            'waypoint_count': len(self._waypoints),
            'waypoints': [asdict(waypoint) for waypoint in self._waypoints],
        }
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        with csv_path.open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(
                f,
                fieldnames=['latitude', 'longitude', 'altitude', 'stamp_sec', 'stamp_nanosec', 'recorded_at'],
            )
            writer.writeheader()
            for waypoint in self._waypoints:
                writer.writerow(asdict(waypoint))
        self._route_file = json_path
        self._node.get_logger().info(f'GPS route exported: {json_path} ({csv_path.name})')
        return json_path

    def load(self, route_file=None):
        path = Path(route_file).expanduser() if route_file else self._route_file
        if path is None:
            self._node.get_logger().warn('GPS route load skipped; --route-file is not set and no export exists')
            return False
        if not path.exists():
            self._node.get_logger().warn(f'GPS route load skipped; file not found: {path}')
            return False
        data = json.loads(path.read_text(encoding='utf-8'))
        self._loaded_waypoints = [GpsWaypoint(**item) for item in data.get('waypoints', [])]
        self._node.get_logger().info(f'GPS route loaded: {path} ({len(self._loaded_waypoints)} waypoint(s))')
        return bool(self._loaded_waypoints)


class GpsRouteFollower:
    def __init__(self, node, recorder, speed, vertical_speed, waypoint_radius, altitude_tolerance):
        self._node = node
        self._recorder = recorder
        self._speed = speed
        self._vertical_speed = vertical_speed
        self._waypoint_radius = waypoint_radius
        self._altitude_tolerance = altitude_tolerance
        self._active = False
        self._index = 0
        self._setpoint_pub = node.create_publisher(Twist, '/aerion/manual/gps_route_setpoint', 10)

    @property
    def active(self):
        return self._active

    def toggle(self):
        if self._active:
            self._active = False
            self._node.get_logger().info('GPS route follow stopped')
            return
        if not self._recorder.active_waypoints:
            self._node.get_logger().warn('GPS route follow skipped; no recorded or loaded waypoints')
            return
        if self._recorder.latest_fix is None:
            self._node.get_logger().warn('GPS route follow skipped; no current GPS fix')
            return
        self._index = 0
        self._active = True
        self._node.get_logger().info(
            f'GPS route follow started ({len(self._recorder.active_waypoints)} waypoint(s))'
        )

    def make_command(self):
        msg = Twist()
        if not self._active:
            return msg
        fix = self._recorder.latest_fix
        waypoints = self._recorder.active_waypoints
        if fix is None or self._index >= len(waypoints):
            self._active = False
            self._node.get_logger().info('GPS route follow complete')
            return msg

        target = waypoints[self._index]
        north_m, east_m = gps_delta_meters(
            float(fix.latitude),
            float(fix.longitude),
            target.latitude,
            target.longitude,
        )
        alt_error = target.altitude - float(fix.altitude)
        horizontal_distance = math.hypot(north_m, east_m)
        if horizontal_distance <= self._waypoint_radius and abs(alt_error) <= self._altitude_tolerance:
            self._node.get_logger().info(
                f'GPS waypoint #{self._index + 1} reached '
                f'(dist={horizontal_distance:.2f}m, alt_error={alt_error:.2f}m)'
            )
            self._index += 1
            if self._index >= len(waypoints):
                self._active = False
                self._node.get_logger().info('GPS route follow complete')
            return msg

        if horizontal_distance > 0.05:
            scale = min(self._speed, horizontal_distance) / horizontal_distance
            # MAVROS local velocity setpoints are ROS ENU: x=east, y=north, z=up.
            msg.linear.x = east_m * scale
            msg.linear.y = north_m * scale
        if abs(alt_error) > self._altitude_tolerance:
            msg.linear.z = max(-self._vertical_speed, min(self._vertical_speed, alt_error))
        self._setpoint_pub.publish(msg)
        return msg


class CameraRecorder:
    def __init__(self, node, topic, output_dir, fps, codec):
        self._node = node
        self._topic = topic
        self._output_dir = Path(output_dir).expanduser()
        self._fps = fps
        self._codec = codec
        self._writer = None
        self._recording = False
        self._started_at = None
        self._frame_count = 0
        self._size = None
        self._temp_path = None
        self._node.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)

    @property
    def recording(self):
        return self._recording

    def toggle(self):
        if self._recording:
            self.stop()
        else:
            self.start()

    def start(self):
        if cv2 is None or np is None:
            self._node.get_logger().error('camera recording requires python3-opencv and numpy')
            return False
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._started_at = datetime.now()
        self._temp_path = self._output_dir / f'camera_recording_{self._started_at.strftime("%Y%m%d_%H%M%S")}.avi'
        self._writer = None
        self._size = None
        self._frame_count = 0
        self._recording = True
        self._node.get_logger().info(f'camera recording started from {self._topic}')
        return True

    def stop(self):
        if not self._recording:
            return
        self._recording = False
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        ended_at = datetime.now()
        final_path = self._output_dir / f'camera_{ended_at.strftime("%Y%m%d_%H%M%S")}.avi'
        if self._temp_path and self._temp_path.exists():
            os.replace(self._temp_path, final_path)
            self._node.get_logger().info(
                f'camera recording saved: {final_path} ({self._frame_count} frame(s))'
            )
        else:
            self._node.get_logger().warn('camera recording stopped; no frames were written')

    def _on_image(self, msg):
        if not self._recording:
            return
        frame = image_msg_to_bgr(msg)
        if frame is None:
            return
        height, width = frame.shape[:2]
        if self._writer is None:
            self._writer = self._open_writer(width, height)
            self._size = (width, height)
            if self._writer is None:
                self._recording = False
                self._node.get_logger().error(f'failed to open camera recording file: {self._temp_path}')
                return
        elif self._size != (width, height):
            frame = cv2.resize(frame, self._size)
        self._writer.write(frame)
        self._frame_count += 1

    def _open_writer(self, width, height):
        codecs = [self._codec]
        if self._codec != 'MJPG':
            codecs.append('MJPG')
        for codec in codecs:
            fourcc = cv2.VideoWriter_fourcc(*codec[:4])
            writer = cv2.VideoWriter(str(self._temp_path), fourcc, self._fps, (width, height))
            if writer.isOpened():
                if codec != self._codec:
                    self._node.get_logger().warn(
                        f'camera codec {self._codec} unavailable; falling back to {codec}'
                    )
                else:
                    self._node.get_logger().info(f'camera recording codec: {codec}')
                return writer
            writer.release()
        return None


def gps_delta_meters(lat1, lon1, lat2, lon2):
    mean_lat = math.radians((lat1 + lat2) * 0.5)
    north_m = (lat2 - lat1) * 111_320.0
    east_m = (lon2 - lon1) * 111_320.0 * math.cos(mean_lat)
    return north_m, east_m


def image_msg_to_bgr(msg):
    if cv2 is None or np is None:
        return None
    channels_by_encoding = {
        'rgb8': 3,
        'bgr8': 3,
        'rgba8': 4,
        'bgra8': 4,
        'mono8': 1,
    }
    encoding = msg.encoding.lower()
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        return None
    array = np.frombuffer(msg.data, dtype=np.uint8)
    expected = int(msg.height) * int(msg.width) * channels
    if array.size < expected:
        return None
    frame = array[:expected].reshape((int(msg.height), int(msg.width), channels))
    if encoding == 'rgb8':
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if encoding == 'rgba8':
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    if encoding == 'bgra8':
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if encoding == 'mono8':
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame.copy()


def parse_args():
    parser = argparse.ArgumentParser(description='Manual MAVROS velocity controller')
    parser.add_argument('--namespace', default='drone1/mavros',
                        help='MAVROS namespace, e.g. drone1/mavros or mavros0')
    parser.add_argument('--namespaces', default='',
                        help='Comma-separated MAVROS namespaces to control together')
    parser.add_argument('--vehicles', default='',
                        help='Comma-separated vehicle names; expands to <vehicle>/mavros namespaces')
    parser.add_argument('--topic', default='',
                        help='Absolute Twist topic override; comma-separated values are allowed')
    parser.add_argument('--rate', type=float, default=20.0,
                        help='Publish rate in Hz')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Horizontal speed in m/s')
    parser.add_argument('--vertical-speed', type=float, default=0.6,
                        help='Vertical speed in m/s, ENU positive is up')
    parser.add_argument('--yaw-rate', type=float, default=0.7,
                        help='Yaw rate in rad/s')
    parser.add_argument('--hold-timeout', type=float, default=0.25,
                        help='Seconds after last key before command decays to hover')
    parser.add_argument('--arm', action='store_true',
                        help='Call cmd/arming and set OFFBOARD before teleop')
    parser.add_argument('--force-arm', action='store_true',
                        help='Use MAV_CMD_COMPONENT_ARM_DISARM force code when normal arming is rejected')
    parser.add_argument('--gps-topic', default='',
                        help='NavSatFix topic for route recording/following')
    parser.add_argument('--recording-dir', default='recordings',
                        help='Base output directory for GPS routes and camera recordings')
    parser.add_argument('--route-file', default='',
                        help='JSON GPS route file to load with the l key')
    parser.add_argument('--follow-speed', type=float, default=1.5,
                        help='GPS route follower horizontal speed in m/s')
    parser.add_argument('--follow-vertical-speed', type=float, default=0.8,
                        help='GPS route follower vertical speed in m/s')
    parser.add_argument('--waypoint-radius', type=float, default=1.5,
                        help='GPS route follower waypoint acceptance radius in meters')
    parser.add_argument('--altitude-tolerance', type=float, default=1.0,
                        help='GPS route follower altitude acceptance tolerance in meters')
    parser.add_argument('--camera-topic', default='/drone1/camera/image',
                        help='sensor_msgs/Image topic to record')
    parser.add_argument('--camera-record-fps', type=float, default=10.0,
                        help='Video file frame rate')
    parser.add_argument('--camera-record-codec', default='FFV1',
                        help='FourCC video codec. FFV1 preserves color better; falls back to MJPG if unavailable')
    return parser.parse_args()


def parse_csv(value):
    return [item.strip() for item in value.split(',') if item.strip()]


def resolve_topics(args):
    explicit_topics = parse_csv(args.topic)
    if explicit_topics:
        return [topic if topic.startswith('/') else f'/{topic}' for topic in explicit_topics]

    vehicles = parse_csv(args.vehicles)
    if vehicles:
        namespaces = [f'{vehicle.strip("/")}/mavros' for vehicle in vehicles]
    else:
        namespaces = parse_csv(args.namespaces) or [args.namespace]

    topics = []
    for namespace in namespaces:
        namespace = namespace.strip('/')
        topics.append(f'/{namespace}/setpoint_velocity/cmd_vel_unstamped')
    return topics


def namespace_from_setpoint_topic(topic):
    suffix = '/setpoint_velocity/cmd_vel_unstamped'
    if not topic.endswith(suffix):
        return ''
    return topic[:-len(suffix)].strip('/')


def make_twist(command, speed, vertical_speed, yaw_rate):
    msg = Twist()
    msg.linear.x = command[0] * speed
    msg.linear.y = command[1] * speed
    msg.linear.z = command[2] * vertical_speed
    msg.angular.z = command[3] * yaw_rate
    return msg


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return ''
    return sys.stdin.read(1)


def wait_for_service(node, client, name, timeout_sec=4.0):
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.2):
            return True
    node.get_logger().warn(f'service unavailable: {name}')
    return False


def spin_service_call(node, future, timeout_sec=3.0):
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            return future.result()
    return None


def call_startup_services(node, namespace, force_arm=False):
    if CommandBool is None or SetMode is None:
        node.get_logger().warn('mavros_msgs is not available; skipping --arm startup services')
        return

    arm_client = node.create_client(CommandBool, f'/{namespace}/cmd/arming')
    force_arm_client = (
        node.create_client(CommandLong, f'/{namespace}/cmd/command')
        if CommandLong is not None
        else None
    )
    mode_client = node.create_client(SetMode, f'/{namespace}/set_mode')

    if wait_for_service(node, mode_client, f'/{namespace}/set_mode'):
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        spin_service_call(node, mode_client.call_async(req))
        node.get_logger().info(f'OFFBOARD requested on /{namespace}/set_mode')

    normal_arm_ok = False
    if wait_for_service(node, arm_client, f'/{namespace}/cmd/arming'):
        req = CommandBool.Request()
        req.value = True
        res = spin_service_call(node, arm_client.call_async(req))
        normal_arm_ok = bool(getattr(res, 'success', False))
        node.get_logger().info(f'arming requested on /{namespace}/cmd/arming')

    if force_arm and not normal_arm_ok and force_arm_client is not None:
        if wait_for_service(node, force_arm_client, f'/{namespace}/cmd/command'):
            req = CommandLong.Request()
            req.command = 400
            req.param1 = 1.0
            req.param2 = 21196.0
            res = spin_service_call(node, force_arm_client.call_async(req))
            success = bool(getattr(res, 'success', False))
            node.get_logger().info(
                f'force arming requested on /{namespace}/cmd/command: success={success}'
            )


def main():
    args = parse_args()
    topics = resolve_topics(args)

    rclpy.init()
    node = rclpy.create_node('aerion_manual_mavros_control')
    publishers = [node.create_publisher(Twist, topic, 10) for topic in topics]

    mavros_namespaces = [namespace_from_setpoint_topic(topic) for topic in topics]
    mavros_namespaces = [namespace for namespace in mavros_namespaces if namespace]
    gps_topic = args.gps_topic or (
        f'/{mavros_namespaces[0]}/global_position/global'
        if mavros_namespaces
        else '/drone1/mavros/global_position/global'
    )
    recording_dir = Path(args.recording_dir).expanduser()
    gps_recorder = GpsRouteRecorder(
        node=node,
        topic=gps_topic,
        output_dir=recording_dir / 'gps',
        route_file=args.route_file,
    )
    gps_follower = GpsRouteFollower(
        node=node,
        recorder=gps_recorder,
        speed=args.follow_speed,
        vertical_speed=args.follow_vertical_speed,
        waypoint_radius=args.waypoint_radius,
        altitude_tolerance=args.altitude_tolerance,
    )
    camera_recorder = CameraRecorder(
        node=node,
        topic=args.camera_topic,
        output_dir=recording_dir / 'camera',
        fps=args.camera_record_fps,
        codec=args.camera_record_codec,
    )

    if args.arm and mavros_namespaces:
        node.get_logger().info('streaming neutral setpoints before OFFBOARD/arming')
        for _ in range(int(max(args.rate, 1.0))):
            for publisher in publishers:
                publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / max(args.rate, 1.0))
        for namespace in mavros_namespaces:
            call_startup_services(node, namespace, force_arm=args.force_arm)
    elif args.arm:
        node.get_logger().info('--arm skipped because no MAVROS setpoint topic was selected')

    old_settings = termios.tcgetattr(sys.stdin)
    command = (0.0, 0.0, 0.0, 0.0)
    last_key_time = 0.0

    node.get_logger().info(f'publishing manual setpoints on {", ".join(topics)}')
    node.get_logger().info(f'GPS route topic: {gps_topic}')
    node.get_logger().info(f'camera record topic: {args.camera_topic}')
    node.get_logger().info(
        'keys: w/s forward/back, a/d left/right, r/f up/down, q/e yaw, space hover, '
        'g save GPS, o export GPS, l load GPS, v follow GPS, c camera rec, x exit'
    )

    try:
        tty.setcbreak(sys.stdin.fileno())
        period = 1.0 / max(args.rate, 1.0)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(period)
            now = time.monotonic()
            if key:
                if key in ('x', '\x03'):
                    break
                if key == 'g':
                    gps_recorder.record_current()
                    continue
                if key == 'o':
                    gps_recorder.export()
                    continue
                if key == 'l':
                    gps_recorder.load(args.route_file)
                    continue
                if key == 'v':
                    gps_follower.toggle()
                    continue
                if key == 'c':
                    camera_recorder.toggle()
                    continue
                if key in KEY_BINDINGS:
                    command = KEY_BINDINGS[key]
                    last_key_time = now
            elif now - last_key_time > args.hold_timeout:
                command = (0.0, 0.0, 0.0, 0.0)

            if gps_follower.active:
                msg = gps_follower.make_command()
            else:
                msg = make_twist(command, args.speed, args.vertical_speed, args.yaw_rate)
            for publisher in publishers:
                publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        camera_recorder.stop()
        for publisher in publishers:
            publisher.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
