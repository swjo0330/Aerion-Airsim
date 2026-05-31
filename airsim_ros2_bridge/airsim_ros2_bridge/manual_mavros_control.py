#!/usr/bin/env python3
"""Keyboard velocity teleop for AERION MAVROS/PX4 validation.

Publishes geometry_msgs/Twist to a MAVROS setpoint_velocity/cmd_vel_unstamped
topic at a fixed rate. This is intentionally small and dependency-light so it
can be used during CARLA map lag checks without bringing up the formation node.
"""

import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist

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
    node.get_logger().info('keys: w/s x, a/d y, r/f z, q/e yaw, space hover, x exit')

    try:
        tty.setcbreak(sys.stdin.fileno())
        period = 1.0 / max(args.rate, 1.0)
        while rclpy.ok():
            key = read_key(period)
            now = time.monotonic()
            if key:
                if key in ('x', '\x03'):
                    break
                if key in KEY_BINDINGS:
                    command = KEY_BINDINGS[key]
                    last_key_time = now
            elif now - last_key_time > args.hold_timeout:
                command = (0.0, 0.0, 0.0, 0.0)

            msg = make_twist(command, args.speed, args.vertical_speed, args.yaw_rate)
            for publisher in publishers:
                publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        for publisher in publishers:
            publisher.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
