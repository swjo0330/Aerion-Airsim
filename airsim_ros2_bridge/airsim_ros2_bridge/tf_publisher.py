"""AERION Phase 3+ — TF tree publisher (Round 4).

목적:
  RViz 등에서 시각화 가능하도록 `map → drone{N}/odom → drone{N}/base_link → ...` TF tree 발행.
  RViz "Fixed Frame [map] does not exist" 에러 해결.

동작:
  - Static (init 시 1회):
      map → drone{N}/odom               (settings.json spawn 위치 기반)
      drone{N}/base_link → drone{N}/camera_link
      drone{N}/camera_link → drone{N}/camera_optical_frame  (REP-103 회전)
      drone{N}/base_link → drone{N}/range_{front,left,right}_link
  - Dynamic (mavros pose 구독, 발행 주기 = pose 토픽 주기):
      drone{N}/odom → drone{N}/base_link

좌표:
  REP-105 ENU 기준 (x=East, y=North, z=Up). drone_controller가 mavros local_position/pose를
  ENU로 발행한다는 가정. AirSim NED→ENU 변환은 bridge 내부 책임.

향후 (Phase 1 CARLA 통합 시):
  CARLA 측 차량/보행자 TF는 carla-ros-bridge가 별도 발행. 본 모듈은 드론만 담당.

변경 이력:
  2026-05-18 v1: 최초 작성 (Round 4, "Frame [map] does not exist" 해결)
"""

from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


# settings.json의 Vehicles[drone{N}].X/Y/Z (AirSim 좌표 = NED, 본 모듈은 ENU로 변환해 사용).
# sf_5drones_phase3.json 기준 5m 간격 X축 배치 (AirSim NED X = ROS ENU X 일치, Y/Z만 부호 다름).
SPAWN_POSITIONS_ENU: Dict[int, Tuple[float, float, float]] = {
    1: (0.0,  0.0, 0.0),
    2: (5.0,  0.0, 0.0),
    3: (10.0, 0.0, 0.0),
    4: (15.0, 0.0, 0.0),
    5: (20.0, 0.0, 0.0),
}

# 카메라 마운트 위치 (FLU 기준, settings.json: X=0.25, Y=0, Z=-0.18). ENU z = -NED z 이므로 +0.18.
CAMERA_OFFSET_FLU: Tuple[float, float, float] = (0.25, 0.0, 0.18)

# 거리센서 위치 (FLU). settings.json: front=(0.30,0,0), left=(0,-0.30,0) NED → ENU y left=+0.30, right=-0.30.
RANGE_OFFSETS_FLU: Dict[str, Tuple[float, float, float]] = {
    'range_front_link': (0.30,  0.0, 0.0),
    'range_left_link':  (0.0,   0.30, 0.0),
    'range_right_link': (0.0,  -0.30, 0.0),
}

# REP-103 카메라 optical frame (FLU base → optical): x_o = -y_b, y_o = -z_b, z_o = x_b.
# 이 회전의 quaternion (xyzw): (-0.5, 0.5, -0.5, 0.5).
CAMERA_OPTICAL_QUAT_XYZW: Tuple[float, float, float, float] = (-0.5, 0.5, -0.5, 0.5)


class AerionTFPublisher(Node):
    """드론 N대의 TF tree를 발행하는 단일 노드. drone_count 파라미터로 1~5 지원."""

    def __init__(self):
        super().__init__('aerion_tf_publisher')

        self.declare_parameter('drone_count', 5)
        self._n = int(self.get_parameter('drone_count').value)
        if not (1 <= self._n <= 5):
            raise ValueError(f'drone_count 1~5만 지원: {self._n}')

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)

        # ----- 정적 TF: map → odom + base_link → sensor frames -----
        static_msgs = []
        stamp = self.get_clock().now().to_msg()

        for i in range(1, self._n + 1):
            sx, sy, sz = SPAWN_POSITIONS_ENU.get(i, (0.0, 0.0, 0.0))

            # map → drone{i}/odom (spawn 위치, 초기 yaw=0)
            static_msgs.append(self._mk_tf(stamp, 'map', f'drone{i}/odom',
                                            sx, sy, sz, 0.0, 0.0, 0.0, 1.0))

            # drone{i}/base_link → drone{i}/camera_link
            cx, cy, cz = CAMERA_OFFSET_FLU
            static_msgs.append(self._mk_tf(stamp, f'drone{i}/base_link', f'drone{i}/camera_link',
                                            cx, cy, cz, 0.0, 0.0, 0.0, 1.0))

            # drone{i}/camera_link → drone{i}/camera_optical_frame (REP-103 회전)
            qx, qy, qz, qw = CAMERA_OPTICAL_QUAT_XYZW
            static_msgs.append(self._mk_tf(stamp,
                                            f'drone{i}/camera_link',
                                            f'drone{i}/camera_optical_frame',
                                            0.0, 0.0, 0.0, qx, qy, qz, qw))

            # drone{i}/base_link → drone{i}/range_*_link
            for child, off in RANGE_OFFSETS_FLU.items():
                static_msgs.append(self._mk_tf(stamp, f'drone{i}/base_link', f'drone{i}/{child}',
                                                off[0], off[1], off[2], 0.0, 0.0, 0.0, 1.0))

        self._static_broadcaster.sendTransform(static_msgs)

        # ----- 동적 TF: odom → base_link (mavros pose 구독) -----
        for i in range(1, self._n + 1):
            self.create_subscription(
                PoseStamped, f'/drone{i}/mavros/local_position/pose',
                lambda msg, idx=i: self._on_pose(idx, msg), 10
            )

        self.get_logger().info(
            f'AERION TF publisher ready: {self._n} drones, '
            f'static frames published, dynamic via mavros local_position/pose'
        )

    @staticmethod
    def _mk_tf(stamp, parent, child, x, y, z, qx, qy, qz, qw) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        return t

    def _on_pose(self, idx: int, msg: PoseStamped):
        """mavros local_position/pose → drone{idx}/odom → drone{idx}/base_link dynamic TF."""
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = f'drone{idx}/odom'
        t.child_frame_id = f'drone{idx}/base_link'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = AerionTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
