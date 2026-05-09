import math
import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


class ApPoseCmdVelProbe(Node):
    def __init__(self):
        super().__init__('ap_pose_cmdvel_probe')

        self.declare_parameter('pose_topic', '/ap/pose/filtered')
        self.declare_parameter('cmd_vel_topic', '/ap/cmd_vel')
        self.declare_parameter('target_dx', 1.0)
        self.declare_parameter('target_dy', 0.0)
        self.declare_parameter('target_dz', 0.0)
        self.declare_parameter('control_z', False)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('tolerance', 0.15)
        self.declare_parameter('max_duration_sec', 60.0)

        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.target_offset = (
            self.get_parameter('target_dx').get_parameter_value().double_value,
            self.get_parameter('target_dy').get_parameter_value().double_value,
            self.get_parameter('target_dz').get_parameter_value().double_value,
        )
        self.control_z = self.get_parameter('control_z').get_parameter_value().bool_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.max_duration_sec = self.get_parameter(
            'max_duration_sec'
        ).get_parameter_value().double_value

        self.latest_pose: Optional[PoseStamped] = None
        self.pose_seq = 0
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_callback,
            10,
        )

    def _pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg
        self.pose_seq += 1

    def stop(self):
        self.cmd_pub.publish(Twist())


def _xyz_from_pose(msg: PoseStamped) -> tuple[float, float, float]:
    return (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


def _format_xyz(value: tuple[float, float, float]) -> str:
    return f'({value[0]:.3f}, {value[1]:.3f}, {value[2]:.3f})'


def _distance(error: tuple[float, float, float], control_z: bool) -> float:
    components = error if control_z else error[:2]
    return math.sqrt(sum(component * component for component in components))


def _build_command(
    error: tuple[float, float, float],
    distance: float,
    kp: float,
    max_speed: float,
    control_z: bool,
) -> Twist:
    scale = min(kp, max_speed / distance) if distance > 0.0 else 0.0
    cmd = Twist()
    cmd.linear.x = error[0] * scale
    cmd.linear.y = error[1] * scale
    cmd.linear.z = error[2] * scale if control_z else 0.0
    return cmd


def main(args=None):
    rclpy.init(args=args)
    node = ApPoseCmdVelProbe()
    started_at = time.monotonic()
    last_command_pose_seq = -1
    start_xyz = None
    target_xyz = None

    print(
        f'Listening on {node.pose_topic}; publishing velocity commands to {node.cmd_vel_topic}',
        flush=True,
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()

            if now - started_at > node.max_duration_sec:
                print(
                    f'ERROR: timed out after {node.max_duration_sec:.1f}s before reaching target',
                    flush=True,
                )
                node.stop()
                return 2

            if node.latest_pose is None:
                continue

            if node.pose_seq == last_command_pose_seq:
                continue

            if start_xyz is None:
                start_xyz = _xyz_from_pose(node.latest_pose)
                target_xyz = tuple(
                    start + offset for start, offset in zip(start_xyz, node.target_offset)
                )
                print(
                    f'Start ENU position {_format_xyz(start_xyz)}; '
                    f'target {_format_xyz(target_xyz)}',
                    flush=True,
                )

            current_xyz = _xyz_from_pose(node.latest_pose)
            error = tuple(target - value for target, value in zip(target_xyz, current_xyz))
            distance = _distance(error, node.control_z)

            if distance <= node.tolerance:
                print(
                    f'Reached target within {distance:.3f} m; '
                    f'final ENU position {_format_xyz(current_xyz)}',
                    flush=True,
                )
                node.stop()
                return 0

            node.cmd_pub.publish(
                _build_command(error, distance, node.kp, node.max_speed, node.control_z)
            )
            last_command_pose_seq = node.pose_seq
            print(
                f'Commanded from ENU {_format_xyz(current_xyz)}; remaining {distance:.3f} m',
                flush=True,
            )
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 1


if __name__ == '__main__':
    sys.exit(main())
