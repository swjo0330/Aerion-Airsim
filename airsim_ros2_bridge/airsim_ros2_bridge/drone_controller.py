import airsim
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from std_msgs.msg import String


class DroneController:
    """Subscribes to control topics and forwards commands to AirSim.

    Legacy /DroneN/cmd_* topics use AirSim/NED coordinates.
    MAVROS-compatible topics use ROS ENU coordinates and are converted to NED.
    """

    def __init__(
        self,
        node: Node,
        client: airsim.MultirotorClient,
        vehicle_name: str,
        enable_ardu_compat: bool = False,
        velocity_control_mode: str = 'kinematic',
        velocity_command_duration: float = 0.2,
        kinematic_z_ned: float = -1.0,
    ):
        self._node = node
        self._client = client
        self._vehicle_name = vehicle_name
        self._enable_ardu_compat = enable_ardu_compat
        self._velocity_control_mode = velocity_control_mode
        self._velocity_command_duration = velocity_command_duration
        self._kinematic_z_ned = kinematic_z_ned
        self._velocity_command_count = 0
        self._last_velocity_enu = (0.0, 0.0, 0.0)

        topic_prefix = f'/{vehicle_name}'
        mavros_prefix = f'{topic_prefix}/mavros'

        self._enable_vehicle_control()

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

        self._mavros_vel_sub = node.create_subscription(
            TwistStamped,
            f'{mavros_prefix}/setpoint_velocity/cmd_vel',
            self._mavros_cmd_vel_stamped_callback,
            10,
        )

        self._mavros_vel_unstamped_sub = node.create_subscription(
            Twist,
            f'{mavros_prefix}/setpoint_velocity/cmd_vel_unstamped',
            self._mavros_cmd_vel_unstamped_callback,
            10,
        )

        self._mavros_pos_sub = node.create_subscription(
            PoseStamped,
            f'{mavros_prefix}/setpoint_position/local',
            self._mavros_cmd_pos_callback,
            10,
        )

        self._local_pose_pub = node.create_publisher(
            PoseStamped,
            f'{mavros_prefix}/local_position/pose',
            10,
        )

        if enable_ardu_compat:
            self._ap_cmd_vel_sub = node.create_subscription(
                Twist,
                '/ap/cmd_vel',
                self._mavros_cmd_vel_unstamped_callback,
                10,
            )
            self._mavros_alias_vel_sub = node.create_subscription(
                Twist,
                '/mavros/setpoint_velocity/cmd_vel_unstamped',
                self._mavros_cmd_vel_unstamped_callback,
                10,
            )
            self._mavros_alias_pose_pub = node.create_publisher(
                PoseStamped,
                '/mavros/local_position/pose',
                10,
            )
            self._ap_pose_pub = node.create_publisher(
                PoseStamped,
                '/ap/pose/filtered',
                10,
            )
            self._ap_twist_pub = node.create_publisher(
                TwistStamped,
                '/ap/twist/filtered',
                10,
            )
            self._ap_status_pub = node.create_publisher(
                String,
                '/ap/status',
                10,
            )
        else:
            self._ap_cmd_vel_sub = None
            self._mavros_alias_vel_sub = None
            self._mavros_alias_pose_pub = None
            self._ap_pose_pub = None
            self._ap_twist_pub = None
            self._ap_status_pub = None

        self._pose_warn_count = 0
        self._pose_timer = node.create_timer(0.1, self._publish_local_pose)

        node.get_logger().info(
            f'[{vehicle_name}] Controller listening on {topic_prefix}/cmd_* and {mavros_prefix}/setpoint_*'
        )
        node.get_logger().info(
            f'[{vehicle_name}] Velocity control mode={velocity_control_mode}, duration={velocity_command_duration:.2f}s'
        )
        if velocity_control_mode == 'kinematic':
            node.get_logger().info(f'[{vehicle_name}] Kinematic altitude locked at NED z={kinematic_z_ned:.2f}')
        if enable_ardu_compat:
            node.get_logger().info(
                f'[{vehicle_name}] ArduPilot compatibility enabled on /ap/* and /mavros/* aliases'
            )

    def _enable_vehicle_control(self):
        try:
            self._client.enableApiControl(True, vehicle_name=self._vehicle_name)
            self._client.armDisarm(True, vehicle_name=self._vehicle_name)
            self._node.get_logger().info(f'[{self._vehicle_name}] API control enabled and vehicle armed')
        except Exception as e:
            self._node.get_logger().warn(f'[{self._vehicle_name}] API control/arm error: {e}')

    def _cmd_vel_callback(self, msg: Twist):
        """Forward velocity command to AirSim.

        Twist.linear.x/y/z -> vx, vy, vz in NED frame (m/s).
        Duration is fixed at 0.1s for continuous streaming.
        """
        self._send_velocity_ned(msg.linear.x, msg.linear.y, msg.linear.z, source='cmd_vel')

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

    def _mavros_cmd_vel_stamped_callback(self, msg: TwistStamped):
        """Forward MAVROS velocity setpoint from ROS ENU to AirSim NED."""
        self._send_velocity_enu(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)

    def _mavros_cmd_vel_unstamped_callback(self, msg: Twist):
        """Forward MAVROS unstamped velocity setpoint from ROS ENU to AirSim NED."""
        self._send_velocity_enu(msg.linear.x, msg.linear.y, msg.linear.z)

    def _mavros_cmd_pos_callback(self, msg: PoseStamped):
        """Forward MAVROS local position setpoint from ROS ENU to AirSim NED."""
        ned_x, ned_y, ned_z = self._enu_to_ned(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        try:
            self._client.moveToPositionAsync(
                ned_x,
                ned_y,
                ned_z,
                velocity=5.0,
                vehicle_name=self._vehicle_name,
            )
        except Exception as e:
            self._node.get_logger().warn(f'[{self._vehicle_name}] mavros setpoint_position error: {e}')

    def _send_velocity_enu(self, enu_x: float, enu_y: float, enu_z: float):
        self._last_velocity_enu = (enu_x, enu_y, enu_z)
        ned_x, ned_y, ned_z = self._enu_to_ned(enu_x, enu_y, enu_z)
        self._send_velocity_ned(ned_x, ned_y, ned_z, source='setpoint_velocity', enu=(enu_x, enu_y, enu_z))

    def _send_velocity_ned(
        self,
        ned_x: float,
        ned_y: float,
        ned_z: float,
        source: str,
        enu: tuple[float, float, float] | None = None,
    ):
        try:
            if enu is None:
                self._last_velocity_enu = self._ned_to_enu(ned_x, ned_y, ned_z)
            self._velocity_command_count += 1
            if self._velocity_command_count <= 5:
                if enu is None:
                    self._node.get_logger().info(
                        f'[{self._vehicle_name}] {source} NED=({ned_x:.2f}, {ned_y:.2f}, {ned_z:.2f})'
                    )
                else:
                    self._node.get_logger().info(
                        f'[{self._vehicle_name}] {source} ENU=({enu[0]:.2f}, {enu[1]:.2f}, {enu[2]:.2f}) '
                        f'NED=({ned_x:.2f}, {ned_y:.2f}, {ned_z:.2f})'
                    )

            if self._velocity_control_mode == 'kinematic':
                self._apply_kinematic_velocity(ned_x, ned_y, ned_z)
            else:
                self._client.moveByVelocityAsync(
                    ned_x,
                    ned_y,
                    ned_z,
                    duration=self._velocity_command_duration,
                    vehicle_name=self._vehicle_name,
                )
        except Exception as e:
            self._node.get_logger().warn(f'[{self._vehicle_name}] mavros setpoint_velocity error: {e}')

    def _apply_kinematic_velocity(self, ned_x: float, ned_y: float, ned_z: float):
        raw_state = self._client.client.call('getMultirotorState', self._vehicle_name)
        kinematics = raw_state[1]
        position = kinematics[0]
        orientation_wxyz = kinematics[1]
        duration = self._velocity_command_duration
        next_position = [
            position[0] + ned_x * duration,
            position[1] + ned_y * duration,
            self._kinematic_z_ned if ned_z == 0.0 else position[2] + ned_z * duration,
        ]
        self._client.client.call(
            'simSetVehiclePose',
            [next_position, orientation_wxyz],
            True,
            self._vehicle_name,
        )

    def _publish_local_pose(self):
        try:
            position, orientation, _linear_velocity = self._get_vehicle_kinematics()

            msg = PoseStamped()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'

            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self._ned_to_enu(
                position[0],
                position[1],
                position[2],
            )

            msg.pose.orientation.x = orientation[1]
            msg.pose.orientation.y = orientation[0]
            msg.pose.orientation.z = -orientation[2]
            msg.pose.orientation.w = orientation[3]

            self._local_pose_pub.publish(msg)
            if self._enable_ardu_compat:
                self._mavros_alias_pose_pub.publish(msg)
                self._ap_pose_pub.publish(msg)

                twist_msg = TwistStamped()
                twist_msg.header = msg.header
                twist_msg.twist.linear.x = self._last_velocity_enu[0]
                twist_msg.twist.linear.y = self._last_velocity_enu[1]
                twist_msg.twist.linear.z = self._last_velocity_enu[2]
                self._ap_twist_pub.publish(twist_msg)

                status_msg = String()
                status_msg.data = (
                    f'vehicle={self._vehicle_name};'
                    f'api_control=true;armed=true;'
                    f'velocity_control_mode={self._velocity_control_mode}'
                )
                self._ap_status_pub.publish(status_msg)
        except Exception as e:
            self._pose_warn_count += 1
            if self._pose_warn_count <= 5 or self._pose_warn_count % 100 == 0:
                self._node.get_logger().warn(f'[{self._vehicle_name}] local_position error: {e}')

    def _get_vehicle_kinematics(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float], tuple[float, float, float]]:
        """Return AirSim NED position, quaternion, and linear velocity."""
        try:
            raw_state = self._client.client.call('getMultirotorState', self._vehicle_name)
            if isinstance(raw_state, list):
                kinematics = raw_state[1]
                position = kinematics[0]
                orientation_wxyz = kinematics[1]
                linear_velocity = kinematics[2]
                return (
                    (position[0], position[1], position[2]),
                    (
                        orientation_wxyz[1],
                        orientation_wxyz[2],
                        orientation_wxyz[3],
                        orientation_wxyz[0],
                    ),
                    (linear_velocity[0], linear_velocity[1], linear_velocity[2]),
                )

            state = self._client.getMultirotorState(vehicle_name=self._vehicle_name)
            kinematics = state.kinematics_estimated
            position = kinematics.position
            orientation = kinematics.orientation
            linear_velocity = kinematics.linear_velocity
            return (
                (position.x_val, position.y_val, position.z_val),
                (orientation.x_val, orientation.y_val, orientation.z_val, orientation.w_val),
                (linear_velocity.x_val, linear_velocity.y_val, linear_velocity.z_val),
            )
        except AttributeError:
            state = self._client.getMultirotorState(vehicle_name=self._vehicle_name)
            kinematics = state.kinematics_estimated
            position = kinematics.position
            orientation = kinematics.orientation
            linear_velocity = kinematics.linear_velocity
            return (
                (position.x_val, position.y_val, position.z_val),
                (orientation.x_val, orientation.y_val, orientation.z_val, orientation.w_val),
                (linear_velocity.x_val, linear_velocity.y_val, linear_velocity.z_val),
            )

    @staticmethod
    def _enu_to_ned(x: float, y: float, z: float) -> tuple[float, float, float]:
        return y, x, -z

    @staticmethod
    def _ned_to_enu(x: float, y: float, z: float) -> tuple[float, float, float]:
        return y, x, -z
