"""AERION Phase 4 — 자동 leader_pose 발행 노드.

목적:
  formation_node는 `/aerion/formation/leader_pose`를 외부에서 받아야 동작.
  실제 mission planner가 없을 때 Phase 4 시연 / 검증 용도로 단순한 leader_pose를 자동 생성.

동작 모드:
  - `static`  : (x, y, z, yaw) 고정 leader_pose를 주기적으로 publish.
  - `circle`  : 반지름 R, 각속도 ω의 원궤도 (Phase 4 동적 시연용 — 5대 포메이션이 leader 따라 회전).
  - `line`    : 전진 (x 방향 1m/s)으로 leader가 일정 속도 이동.

토픽:
  Publish: /aerion/formation/leader_pose  (geometry_msgs/PoseStamped, ENU)

향후 (mission_planner 통합 후):
  본 노드는 mission_planner에 흡수될 가능성. 현재는 독립 단일 책임.

변경 이력:
  2026-05-18 v1: 최초 작성 (Phase 4 시연 자동화)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class LeaderPublisher(Node):
    def __init__(self):
        super().__init__('aerion_leader')

        self.declare_parameter('mode', 'static')            # static / circle / line
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_z', 5.0)               # ENU 양수 = 위
        self.declare_parameter('init_yaw', 0.0)             # rad
        self.declare_parameter('circle_radius', 5.0)        # m
        self.declare_parameter('circle_angular_vel', 0.1)   # rad/s
        self.declare_parameter('line_speed', 1.0)           # m/s (전진)

        self._mode = str(self.get_parameter('mode').value)
        self._rate = float(self.get_parameter('publish_rate').value)
        self._x0 = float(self.get_parameter('init_x').value)
        self._y0 = float(self.get_parameter('init_y').value)
        self._z0 = float(self.get_parameter('init_z').value)
        self._yaw0 = float(self.get_parameter('init_yaw').value)
        self._R = float(self.get_parameter('circle_radius').value)
        self._omega = float(self.get_parameter('circle_angular_vel').value)
        self._speed = float(self.get_parameter('line_speed').value)

        self._pub = self.create_publisher(PoseStamped, '/aerion/formation/leader_pose', 10)
        self._t0 = self.get_clock().now()
        self._timer = self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f'AERION leader publisher: mode={self._mode}, rate={self._rate}Hz, '
            f'init=({self._x0:.1f}, {self._y0:.1f}, {self._z0:.1f}, yaw={self._yaw0:.2f})'
        )

    def _tick(self):
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9

        if self._mode == 'circle':
            theta = self._omega * t
            x = self._x0 + self._R * math.cos(theta)
            y = self._y0 + self._R * math.sin(theta)
            z = self._z0
            yaw = self._yaw0 + theta + math.pi / 2.0   # 진행 방향과 일치 (접선)
        elif self._mode == 'line':
            x = self._x0 + self._speed * t
            y = self._y0
            z = self._z0
            yaw = self._yaw0
        else:  # static
            x, y, z, yaw = self._x0, self._y0, self._z0, self._yaw0

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
