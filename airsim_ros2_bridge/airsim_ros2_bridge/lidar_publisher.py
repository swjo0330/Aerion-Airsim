"""AirSim LiDAR publisher for AERION single-drone PX4 validation."""

import math
import struct

import airsim
from geometry_msgs.msg import Point
from mavros_msgs.msg import ObstacleDistance3D
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from airsim_ros2_bridge.topic_naming import resolve_namespace


MAV_DISTANCE_SENSOR_LASER = 0
MAV_FRAME_BODY_FRD = 12


class LidarPublisher:
    """Publish AirSim LiDAR as PointCloud2 and optional MAVROS obstacle input.

    AirSim LiDAR points in SensorLocalFrame are treated as body FRD
    (x forward, y right, z down). PointCloud2 is exposed as ROS FLU
    (x forward, y left, z up). ObstacleDistance3D is sent in MAV_FRAME_BODY_FRD
    for PX4/MAVLink consumers.
    """

    def __init__(
        self,
        node: Node,
        client: airsim.MultirotorClient,
        vehicle_name: str,
        lidar_name: str = 'Lidar_Front',
        publish_rate: float = 10.0,
        enable_obstacle: bool = True,
        obstacle_topic: str | None = None,
        min_distance: float = 0.3,
        max_distance: float = 25.0,
        obstacle_forward_fov_deg: float = 90.0,
        obstacle_vertical_fov_deg: float = 45.0,
        topic_namespace: str | None = None,
    ):
        self._node = node
        self._client = client
        self._vehicle_name = vehicle_name
        self._lidar_name = lidar_name
        self._enable_obstacle = enable_obstacle
        self._min_distance = float(min_distance)
        self._max_distance = float(max_distance)
        self._forward_half_tan = math.tan(math.radians(obstacle_forward_fov_deg) * 0.5)
        self._vertical_half_tan = math.tan(math.radians(obstacle_vertical_fov_deg) * 0.5)
        self._last_error_ns = 0
        self._callback_group = ReentrantCallbackGroup()

        self._frame_id = f'{vehicle_name}_lidar_link'
        _ns = resolve_namespace(vehicle_name, topic_namespace)
        ns_prefix = f'/{_ns}' if _ns else ''
        self._cloud_pub = node.create_publisher(PointCloud2, f'{ns_prefix}/lidar/points', 10)
        self._obstacle_pub = (
            node.create_publisher(
                ObstacleDistance3D,
                obstacle_topic or f'{ns_prefix}/mavros/obstacle_distance_3d/send',
                10,
            )
            if enable_obstacle
            else None
        )

        node.get_logger().info(
            f'[{vehicle_name}] LiDAR publisher: {ns_prefix}/lidar/points '
            f'({lidar_name}) @ {publish_rate:.1f}Hz'
        )
        if self._obstacle_pub is not None:
            node.get_logger().info(
                f'[{vehicle_name}] LiDAR obstacle output: '
                f'{obstacle_topic or f"{ns_prefix}/mavros/obstacle_distance_3d/send"}'
            )

        self._probe_once()
        self._timer = node.create_timer(
            1.0 / publish_rate,
            self._publish_callback,
            callback_group=self._callback_group,
        )

    def _probe_once(self):
        try:
            self._client.getLidarData(self._lidar_name, self._vehicle_name)
        except Exception as e:
            self._node.get_logger().warn(
                f'[{self._vehicle_name}] LiDAR {self._lidar_name} probe failed '
                f'(settings.json Sensors에 정의되어 있는지 확인): {e}'
            )

    def _publish_callback(self):
        try:
            data = self._client.getLidarData(self._lidar_name, self._vehicle_name)
            points = list(getattr(data, 'point_cloud', []) or [])
            stamp = self._node.get_clock().now().to_msg()
            self._cloud_pub.publish(self._build_cloud(points, stamp))

            if self._obstacle_pub is not None:
                obstacle = self._nearest_forward_obstacle(points)
                if obstacle is not None:
                    self._obstacle_pub.publish(self._build_obstacle(obstacle, stamp))
        except Exception as e:
            now_ns = self._node.get_clock().now().nanoseconds
            if now_ns - self._last_error_ns > 5_000_000_000:
                self._node.get_logger().warn(
                    f'[{self._vehicle_name}] LiDAR {self._lidar_name} error: {e}'
                )
                self._last_error_ns = now_ns

    def _build_cloud(self, points: list[float], stamp) -> PointCloud2:
        xyz = []
        for idx in range(0, len(points) - 2, 3):
            # AirSim sensor-local FRD -> ROS FLU.
            x_fwd = float(points[idx])
            y_left = -float(points[idx + 1])
            z_up = -float(points[idx + 2])
            xyz.append((x_fwd, y_left, z_up))

        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=self._frame_id)
        msg.height = 1
        msg.width = len(xyz)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = b''.join(struct.pack('<fff', *point) for point in xyz)
        msg.is_dense = True
        return msg

    def _nearest_forward_obstacle(self, points: list[float]) -> tuple[float, float, float] | None:
        best = None
        best_dist = self._max_distance

        for idx in range(0, len(points) - 2, 3):
            x = float(points[idx])
            y = float(points[idx + 1])
            z = float(points[idx + 2])
            if x <= self._min_distance:
                continue
            dist = math.sqrt(x * x + y * y + z * z)
            if dist < self._min_distance or dist > self._max_distance:
                continue
            if abs(y) > max(x, 1e-3) * self._forward_half_tan:
                continue
            if abs(z) > max(x, 1e-3) * self._vertical_half_tan:
                continue
            if dist < best_dist:
                best = (x, y, z)
                best_dist = dist

        return best

    def _build_obstacle(self, point_frd: tuple[float, float, float], stamp) -> ObstacleDistance3D:
        msg = ObstacleDistance3D()
        msg.header = Header(stamp=stamp, frame_id=f'{self._vehicle_name}_base_link')
        msg.sensor_type = MAV_DISTANCE_SENSOR_LASER
        msg.frame = MAV_FRAME_BODY_FRD
        msg.obstacle_id = 1
        msg.position = Point(x=point_frd[0], y=point_frd[1], z=point_frd[2])
        msg.min_distance = self._min_distance
        msg.max_distance = self._max_distance
        return msg
