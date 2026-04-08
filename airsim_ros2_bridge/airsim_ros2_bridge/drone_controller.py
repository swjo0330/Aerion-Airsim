import airsim
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


class DroneController:
    """Subscribes to ROS2 control topics and forwards to AirSim API."""

    def __init__(
        self,
        node: Node,
        client: airsim.MultirotorClient,
        vehicle_name: str,
    ):
        self._node = node
        self._client = client
        self._vehicle_name = vehicle_name

        topic_prefix = f'/{vehicle_name}'

        self._vel_sub = node.create_subscription(
            Twist,
            f'{topic_prefix}/cmd_vel',
            self._cmd_vel_callback,
            10,
        )

        self._pos_sub = node.create_subscription(
            PoseStamped,
            f'{topic_prefix}/cmd_pos',
            self._cmd_pos_callback,
            10,
        )

        node.get_logger().info(
            f'[{vehicle_name}] Controller listening on {topic_prefix}/cmd_vel and {topic_prefix}/cmd_pos'
        )

    def _cmd_vel_callback(self, msg: Twist):
        """Forward velocity command to AirSim.

        Twist.linear.x/y/z -> vx, vy, vz in NED frame (m/s).
        Duration is fixed at 0.1s for continuous streaming.
        """
        try:
            self._client.moveByVelocityAsync(
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                duration=0.1,
                vehicle_name=self._vehicle_name,
            )
        except Exception as e:
            self._node.get_logger().warn(f'[{self._vehicle_name}] cmd_vel error: {e}')

    def _cmd_pos_callback(self, msg: PoseStamped):
        """Forward position command to AirSim.

        PoseStamped.pose.position.x/y/z -> target NED position.
        Velocity is fixed at 5 m/s.
        """
        try:
            self._client.moveToPositionAsync(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                velocity=5.0,
                vehicle_name=self._vehicle_name,
            )
        except Exception as e:
            self._node.get_logger().warn(f'[{self._vehicle_name}] cmd_pos error: {e}')
