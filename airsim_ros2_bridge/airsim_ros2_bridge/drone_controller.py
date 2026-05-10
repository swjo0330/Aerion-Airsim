import math

import airsim
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, Header, String
from rosgraph_msgs.msg import Clock

try:
    from mavros_msgs.msg import State
    from mavros_msgs.msg import ExtendedState
    from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
except ImportError:
    State = None
    ExtendedState = None
    CommandBool = None
    CommandTOL = None
    SetMode = None


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
        home_latitude: float = 37.5665,
        home_longitude: float = 126.9780,
        home_altitude: float = 0.0,
        mavros_instance_namespace: str | None = None,
    ):
        self._node = node
        self._client = client
        self._vehicle_name = vehicle_name
        self._enable_ardu_compat = enable_ardu_compat
        self._velocity_control_mode = velocity_control_mode
        self._velocity_command_duration = velocity_command_duration
        self._kinematic_z_ned = kinematic_z_ned
        self._home_latitude = home_latitude
        self._home_longitude = home_longitude
        self._home_altitude = home_altitude
        self._mavros_instance_namespace = mavros_instance_namespace
        self._velocity_command_count = 0
        self._last_velocity_enu = (0.0, 0.0, 0.0)
        self._armed = True
        self._guided = True
        self._mode = 'GUIDED'
        self._mavros_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

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
        self._local_odom_pub = node.create_publisher(
            Odometry,
            f'{mavros_prefix}/local_position/odom',
            10,
        )
        self._local_pose_cov_pub = node.create_publisher(
            PoseWithCovarianceStamped,
            f'{mavros_prefix}/local_position/pose_cov',
            10,
        )
        self._velocity_local_pub = node.create_publisher(
            TwistStamped,
            f'{mavros_prefix}/local_position/velocity_local',
            10,
        )
        self._velocity_local_cov_pub = node.create_publisher(
            TwistWithCovarianceStamped,
            f'{mavros_prefix}/local_position/velocity_local_cov',
            10,
        )
        self._velocity_body_pub = node.create_publisher(
            TwistStamped,
            f'{mavros_prefix}/local_position/velocity_body',
            10,
        )
        self._velocity_body_cov_pub = node.create_publisher(
            TwistWithCovarianceStamped,
            f'{mavros_prefix}/local_position/velocity_body_cov',
            10,
        )
        self._global_fix_pub = node.create_publisher(
            NavSatFix,
            f'{mavros_prefix}/global_position/global',
            10,
        )
        self._global_local_pub = node.create_publisher(
            Odometry,
            f'{mavros_prefix}/global_position/local',
            10,
        )
        self._global_raw_fix_pub = node.create_publisher(
            NavSatFix,
            f'{mavros_prefix}/global_position/raw/fix',
            10,
        )
        self._global_rel_alt_pub = node.create_publisher(
            Float64,
            f'{mavros_prefix}/global_position/rel_alt',
            10,
        )
        self._global_compass_hdg_pub = node.create_publisher(
            Float64,
            f'{mavros_prefix}/global_position/compass_hdg',
            10,
        )
        self._imu_pub = node.create_publisher(
            Imu,
            f'{mavros_prefix}/imu/data',
            10,
        )
        self._imu_raw_pub = node.create_publisher(
            Imu,
            f'{mavros_prefix}/imu/data_raw',
            10,
        )
        self._battery_pub = node.create_publisher(
            BatteryState,
            f'{mavros_prefix}/battery',
            10,
        )
        self._state_pub = (
            node.create_publisher(State, f'{mavros_prefix}/state', 10)
            if State is not None
            else None
        )
        self._extended_state_pub = (
            node.create_publisher(ExtendedState, f'{mavros_prefix}/extended_state', 10)
            if ExtendedState is not None
            else None
        )
        self._mavros_services = self._create_mavros_services(mavros_prefix)

        if mavros_instance_namespace:
            mavros_instance_prefix = f'/{mavros_instance_namespace}'
            self._mavros_instance_vel_sub = node.create_subscription(
                Twist,
                f'{mavros_instance_prefix}/setpoint_velocity/cmd_vel_unstamped',
                self._mavros_cmd_vel_unstamped_callback,
                10,
            )
            self._mavros_instance_vel_stamped_sub = node.create_subscription(
                TwistStamped,
                f'{mavros_instance_prefix}/setpoint_velocity/cmd_vel',
                self._mavros_cmd_vel_stamped_callback,
                10,
            )
            self._mavros_instance_pos_sub = node.create_subscription(
                PoseStamped,
                f'{mavros_instance_prefix}/setpoint_position/local',
                self._mavros_cmd_pos_callback,
                10,
            )
            self._mavros_instance_pubs = self._create_mavros_namespace_publishers(mavros_instance_prefix)
            self._mavros_instance_services = self._create_mavros_services(mavros_instance_prefix)
        else:
            self._mavros_instance_vel_sub = None
            self._mavros_instance_vel_stamped_sub = None
            self._mavros_instance_pos_sub = None
            self._mavros_instance_pubs = None
            self._mavros_instance_services = None

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
            self._mavros_alias_vel_stamped_sub = node.create_subscription(
                TwistStamped,
                '/mavros/setpoint_velocity/cmd_vel',
                self._mavros_cmd_vel_stamped_callback,
                10,
            )
            self._mavros_alias_pos_sub = node.create_subscription(
                PoseStamped,
                '/mavros/setpoint_position/local',
                self._mavros_cmd_pos_callback,
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
            self._ap_navsat_pub = node.create_publisher(
                NavSatFix,
                '/ap/navsat',
                10,
            )
            self._ap_imu_pub = node.create_publisher(
                Imu,
                '/ap/imu/experimental/data',
                10,
            )
            self._ap_battery_pub = node.create_publisher(
                BatteryState,
                '/ap/battery',
                10,
            )
            self._ap_clock_pub = node.create_publisher(
                Clock,
                '/ap/clock',
                10,
            )
            self._root_odom_pub = node.create_publisher(
                Odometry,
                '/odometry',
                10,
            )
            self._root_imu_pub = node.create_publisher(
                Imu,
                '/imu',
                10,
            )
            self._root_navsat_pub = node.create_publisher(
                NavSatFix,
                '/navsat',
                10,
            )
            self._root_battery_pub = node.create_publisher(
                BatteryState,
                '/battery',
                10,
            )
            self._root_clock_pub = node.create_publisher(
                Clock,
                '/clock',
                10,
            )
            self._mavros_alias_odom_pub = node.create_publisher(
                Odometry,
                '/mavros/local_position/odom',
                10,
            )
            self._mavros_alias_pose_cov_pub = node.create_publisher(
                PoseWithCovarianceStamped,
                '/mavros/local_position/pose_cov',
                10,
            )
            self._mavros_alias_velocity_local_pub = node.create_publisher(
                TwistStamped,
                '/mavros/local_position/velocity_local',
                10,
            )
            self._mavros_alias_velocity_local_cov_pub = node.create_publisher(
                TwistWithCovarianceStamped,
                '/mavros/local_position/velocity_local_cov',
                10,
            )
            self._mavros_alias_velocity_body_pub = node.create_publisher(
                TwistStamped,
                '/mavros/local_position/velocity_body',
                10,
            )
            self._mavros_alias_velocity_body_cov_pub = node.create_publisher(
                TwistWithCovarianceStamped,
                '/mavros/local_position/velocity_body_cov',
                10,
            )
            self._mavros_alias_global_pub = node.create_publisher(
                NavSatFix,
                '/mavros/global_position/global',
                10,
            )
            self._mavros_alias_global_local_pub = node.create_publisher(
                Odometry,
                '/mavros/global_position/local',
                10,
            )
            self._mavros_alias_raw_fix_pub = node.create_publisher(
                NavSatFix,
                '/mavros/global_position/raw/fix',
                10,
            )
            self._mavros_alias_rel_alt_pub = node.create_publisher(
                Float64,
                '/mavros/global_position/rel_alt',
                10,
            )
            self._mavros_alias_compass_hdg_pub = node.create_publisher(
                Float64,
                '/mavros/global_position/compass_hdg',
                10,
            )
            self._mavros_alias_imu_pub = node.create_publisher(
                Imu,
                '/mavros/imu/data',
                10,
            )
            self._mavros_alias_imu_raw_pub = node.create_publisher(
                Imu,
                '/mavros/imu/data_raw',
                10,
            )
            self._mavros_alias_battery_pub = node.create_publisher(
                BatteryState,
                '/mavros/battery',
                10,
            )
            self._mavros_alias_state_pub = (
                node.create_publisher(State, '/mavros/state', 10)
                if State is not None
                else None
            )
            self._mavros_alias_extended_state_pub = (
                node.create_publisher(ExtendedState, '/mavros/extended_state', 10)
                if ExtendedState is not None
                else None
            )
            self._mavros_alias_services = self._create_mavros_services('/mavros')
        else:
            self._ap_cmd_vel_sub = None
            self._mavros_alias_vel_sub = None
            self._mavros_alias_vel_stamped_sub = None
            self._mavros_alias_pos_sub = None
            self._mavros_alias_pose_pub = None
            self._ap_pose_pub = None
            self._ap_twist_pub = None
            self._ap_status_pub = None
            self._ap_navsat_pub = None
            self._ap_imu_pub = None
            self._ap_battery_pub = None
            self._ap_clock_pub = None
            self._root_odom_pub = None
            self._root_imu_pub = None
            self._root_navsat_pub = None
            self._root_battery_pub = None
            self._root_clock_pub = None
            self._mavros_alias_odom_pub = None
            self._mavros_alias_pose_cov_pub = None
            self._mavros_alias_velocity_local_pub = None
            self._mavros_alias_velocity_local_cov_pub = None
            self._mavros_alias_velocity_body_pub = None
            self._mavros_alias_velocity_body_cov_pub = None
            self._mavros_alias_global_pub = None
            self._mavros_alias_global_local_pub = None
            self._mavros_alias_raw_fix_pub = None
            self._mavros_alias_rel_alt_pub = None
            self._mavros_alias_compass_hdg_pub = None
            self._mavros_alias_imu_pub = None
            self._mavros_alias_imu_raw_pub = None
            self._mavros_alias_battery_pub = None
            self._mavros_alias_state_pub = None
            self._mavros_alias_extended_state_pub = None
            self._mavros_alias_services = None

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
        if mavros_instance_namespace:
            node.get_logger().info(
                f'[{vehicle_name}] MAVROS instance aliases enabled on /{mavros_instance_namespace}/*'
            )

    def _create_mavros_namespace_publishers(self, prefix: str) -> dict:
        return {
            'pose': self._node.create_publisher(
                PoseStamped,
                f'{prefix}/local_position/pose',
                self._mavros_sensor_qos,
            ),
            'odom': self._node.create_publisher(
                Odometry,
                f'{prefix}/local_position/odom',
                self._mavros_sensor_qos,
            ),
            'pose_cov': self._node.create_publisher(
                PoseWithCovarianceStamped,
                f'{prefix}/local_position/pose_cov',
                self._mavros_sensor_qos,
            ),
            'velocity_local': self._node.create_publisher(
                TwistStamped,
                f'{prefix}/local_position/velocity_local',
                self._mavros_sensor_qos,
            ),
            'velocity_local_cov': self._node.create_publisher(
                TwistWithCovarianceStamped,
                f'{prefix}/local_position/velocity_local_cov',
                self._mavros_sensor_qos,
            ),
            'velocity_body': self._node.create_publisher(
                TwistStamped,
                f'{prefix}/local_position/velocity_body',
                self._mavros_sensor_qos,
            ),
            'velocity_body_cov': self._node.create_publisher(
                TwistWithCovarianceStamped,
                f'{prefix}/local_position/velocity_body_cov',
                self._mavros_sensor_qos,
            ),
            'global': self._node.create_publisher(
                NavSatFix,
                f'{prefix}/global_position/global',
                self._mavros_sensor_qos,
            ),
            'global_local': self._node.create_publisher(
                Odometry,
                f'{prefix}/global_position/local',
                self._mavros_sensor_qos,
            ),
            'raw_fix': self._node.create_publisher(
                NavSatFix,
                f'{prefix}/global_position/raw/fix',
                self._mavros_sensor_qos,
            ),
            'rel_alt': self._node.create_publisher(
                Float64,
                f'{prefix}/global_position/rel_alt',
                self._mavros_sensor_qos,
            ),
            'compass_hdg': self._node.create_publisher(
                Float64,
                f'{prefix}/global_position/compass_hdg',
                self._mavros_sensor_qos,
            ),
            'imu': self._node.create_publisher(
                Imu,
                f'{prefix}/imu/data',
                self._mavros_sensor_qos,
            ),
            'imu_raw': self._node.create_publisher(
                Imu,
                f'{prefix}/imu/data_raw',
                self._mavros_sensor_qos,
            ),
            'battery': self._node.create_publisher(
                BatteryState,
                f'{prefix}/battery',
                self._mavros_sensor_qos,
            ),
            'state': (
                self._node.create_publisher(State, f'{prefix}/state', 10)
                if State is not None
                else None
            ),
            'extended_state': (
                self._node.create_publisher(ExtendedState, f'{prefix}/extended_state', 10)
                if ExtendedState is not None
                else None
            ),
        }

    def _create_mavros_services(self, prefix: str) -> dict | None:
        if CommandBool is None or SetMode is None:
            return None
        services = {
            'arming': self._node.create_service(
                CommandBool,
                f'{prefix}/cmd/arming',
                self._mavros_cmd_arming_callback,
            ),
            'set_mode': self._node.create_service(
                SetMode,
                f'{prefix}/set_mode',
                self._mavros_set_mode_callback,
            ),
        }
        if CommandTOL is not None:
            services['takeoff'] = self._node.create_service(
                CommandTOL,
                f'{prefix}/cmd/takeoff',
                self._mavros_cmd_takeoff_callback,
            )
            services['land'] = self._node.create_service(
                CommandTOL,
                f'{prefix}/cmd/land',
                self._mavros_cmd_land_callback,
            )
        return services

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

    def _mavros_cmd_arming_callback(self, request, response):
        desired_arm = bool(getattr(request, 'value', False))
        ok = True
        try:
            self._client.enableApiControl(True, vehicle_name=self._vehicle_name)
            self._client.armDisarm(desired_arm, vehicle_name=self._vehicle_name)
            self._armed = desired_arm
            self._guided = True
            self._mode = 'GUIDED' if desired_arm else 'STANDBY'
        except Exception as e:
            ok = False
            self._node.get_logger().warn(f'[{self._vehicle_name}] mavros cmd/arming error: {e}')

        if hasattr(response, 'success'):
            response.success = ok
        if hasattr(response, 'result'):
            response.result = 0 if ok else 1
        return response

    def _mavros_set_mode_callback(self, request, response):
        custom_mode = getattr(request, 'custom_mode', '') or ''
        base_mode = getattr(request, 'base_mode', 0)
        requested_mode = custom_mode if custom_mode else f'BASE_{base_mode}'

        ok = True
        try:
            self._client.enableApiControl(True, vehicle_name=self._vehicle_name)
            self._guided = True
            self._mode = requested_mode.upper()
        except Exception as e:
            ok = False
            self._node.get_logger().warn(f'[{self._vehicle_name}] mavros set_mode error: {e}')

        if hasattr(response, 'mode_sent'):
            response.mode_sent = ok
        return response

    def _mavros_cmd_takeoff_callback(self, request, response):
        del request
        ok = True
        try:
            self._client.enableApiControl(True, vehicle_name=self._vehicle_name)
            self._client.armDisarm(True, vehicle_name=self._vehicle_name)
            self._client.takeoffAsync(vehicle_name=self._vehicle_name).join()
            self._armed = True
            self._guided = True
            self._mode = 'GUIDED'
        except Exception as e:
            ok = False
            self._node.get_logger().warn(f'[{self._vehicle_name}] mavros cmd/takeoff error: {e}')

        if hasattr(response, 'success'):
            response.success = ok
        if hasattr(response, 'result'):
            response.result = 0 if ok else 1
        return response

    def _mavros_cmd_land_callback(self, request, response):
        del request
        ok = True
        try:
            self._client.enableApiControl(True, vehicle_name=self._vehicle_name)
            self._client.landAsync(vehicle_name=self._vehicle_name).join()
            self._armed = False
            self._guided = True
            self._mode = 'LAND'
        except Exception as e:
            ok = False
            self._node.get_logger().warn(f'[{self._vehicle_name}] mavros cmd/land error: {e}')

        if hasattr(response, 'success'):
            response.success = ok
        if hasattr(response, 'result'):
            response.result = 0 if ok else 1
        return response

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
            linear_velocity_enu = self._ned_to_enu(
                _linear_velocity[0],
                _linear_velocity[1],
                _linear_velocity[2],
            )

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
            odom_msg = self._build_odometry_msg(msg, linear_velocity_enu)
            pose_cov_msg = self._build_pose_cov_msg(msg)
            velocity_local_msg = self._build_twist_msg(msg, linear_velocity_enu, 'map')
            velocity_local_cov_msg = self._build_twist_cov_msg(msg, linear_velocity_enu, 'map')
            velocity_body_msg = self._build_twist_msg(msg, linear_velocity_enu, 'base_link')
            velocity_body_cov_msg = self._build_twist_cov_msg(msg, linear_velocity_enu, 'base_link')
            navsat_msg = self._build_navsat_msg(msg)
            imu_msg = self._build_imu_msg(msg)
            battery_msg = self._build_battery_msg(msg)
            state_msg = self._build_state_msg()
            extended_state_msg = self._build_extended_state_msg(msg)
            clock_msg = Clock()
            clock_msg.clock = msg.header.stamp
            rel_alt_msg = Float64()
            rel_alt_msg.data = msg.pose.position.z
            compass_hdg_msg = Float64()
            compass_hdg_msg.data = self._yaw_degrees_from_quaternion(msg.pose.orientation)

            self._local_odom_pub.publish(odom_msg)
            self._local_pose_cov_pub.publish(pose_cov_msg)
            self._velocity_local_pub.publish(velocity_local_msg)
            self._velocity_local_cov_pub.publish(velocity_local_cov_msg)
            self._velocity_body_pub.publish(velocity_body_msg)
            self._velocity_body_cov_pub.publish(velocity_body_cov_msg)
            self._global_fix_pub.publish(navsat_msg)
            self._global_local_pub.publish(odom_msg)
            self._global_raw_fix_pub.publish(navsat_msg)
            self._global_rel_alt_pub.publish(rel_alt_msg)
            self._global_compass_hdg_pub.publish(compass_hdg_msg)
            self._imu_pub.publish(imu_msg)
            self._imu_raw_pub.publish(imu_msg)
            self._battery_pub.publish(battery_msg)
            if self._state_pub is not None and state_msg is not None:
                self._state_pub.publish(state_msg)
            if self._extended_state_pub is not None and extended_state_msg is not None:
                self._extended_state_pub.publish(extended_state_msg)

            if self._mavros_instance_pubs is not None:
                self._publish_mavros_namespace(
                    self._mavros_instance_pubs,
                    msg,
                    odom_msg,
                    pose_cov_msg,
                    velocity_local_msg,
                    velocity_local_cov_msg,
                    velocity_body_msg,
                    velocity_body_cov_msg,
                    navsat_msg,
                    rel_alt_msg,
                    compass_hdg_msg,
                    imu_msg,
                    battery_msg,
                    state_msg,
                    extended_state_msg,
                )

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
                    f'api_control=true;armed={str(self._armed).lower()};'
                    f'velocity_control_mode={self._velocity_control_mode}'
                )
                self._ap_status_pub.publish(status_msg)
                self._ap_navsat_pub.publish(navsat_msg)
                self._ap_imu_pub.publish(imu_msg)
                self._ap_battery_pub.publish(battery_msg)
                self._ap_clock_pub.publish(clock_msg)
                self._root_odom_pub.publish(odom_msg)
                self._root_imu_pub.publish(imu_msg)
                self._root_navsat_pub.publish(navsat_msg)
                self._root_battery_pub.publish(battery_msg)
                self._root_clock_pub.publish(clock_msg)
                self._mavros_alias_odom_pub.publish(odom_msg)
                self._mavros_alias_pose_cov_pub.publish(pose_cov_msg)
                self._mavros_alias_velocity_local_pub.publish(velocity_local_msg)
                self._mavros_alias_velocity_local_cov_pub.publish(velocity_local_cov_msg)
                self._mavros_alias_velocity_body_pub.publish(velocity_body_msg)
                self._mavros_alias_velocity_body_cov_pub.publish(velocity_body_cov_msg)
                self._mavros_alias_global_pub.publish(navsat_msg)
                self._mavros_alias_global_local_pub.publish(odom_msg)
                self._mavros_alias_raw_fix_pub.publish(navsat_msg)
                self._mavros_alias_rel_alt_pub.publish(rel_alt_msg)
                self._mavros_alias_compass_hdg_pub.publish(compass_hdg_msg)
                self._mavros_alias_imu_pub.publish(imu_msg)
                self._mavros_alias_imu_raw_pub.publish(imu_msg)
                self._mavros_alias_battery_pub.publish(battery_msg)
                if self._mavros_alias_state_pub is not None and state_msg is not None:
                    self._mavros_alias_state_pub.publish(state_msg)
                if self._mavros_alias_extended_state_pub is not None and extended_state_msg is not None:
                    self._mavros_alias_extended_state_pub.publish(extended_state_msg)
        except Exception as e:
            self._pose_warn_count += 1
            if self._pose_warn_count <= 5 or self._pose_warn_count % 100 == 0:
                self._node.get_logger().warn(f'[{self._vehicle_name}] local_position error: {e}')

    @staticmethod
    def _publish_mavros_namespace(
        publishers: dict,
        pose_msg: PoseStamped,
        odom_msg: Odometry,
        pose_cov_msg: PoseWithCovarianceStamped,
        velocity_local_msg: TwistStamped,
        velocity_local_cov_msg: TwistWithCovarianceStamped,
        velocity_body_msg: TwistStamped,
        velocity_body_cov_msg: TwistWithCovarianceStamped,
        navsat_msg: NavSatFix,
        rel_alt_msg: Float64,
        compass_hdg_msg: Float64,
        imu_msg: Imu,
        battery_msg: BatteryState,
        state_msg,
        extended_state_msg,
    ):
        publishers['pose'].publish(pose_msg)
        publishers['odom'].publish(odom_msg)
        publishers['pose_cov'].publish(pose_cov_msg)
        publishers['velocity_local'].publish(velocity_local_msg)
        publishers['velocity_local_cov'].publish(velocity_local_cov_msg)
        publishers['velocity_body'].publish(velocity_body_msg)
        publishers['velocity_body_cov'].publish(velocity_body_cov_msg)
        publishers['global'].publish(navsat_msg)
        publishers['global_local'].publish(odom_msg)
        publishers['raw_fix'].publish(navsat_msg)
        publishers['rel_alt'].publish(rel_alt_msg)
        publishers['compass_hdg'].publish(compass_hdg_msg)
        publishers['imu'].publish(imu_msg)
        publishers['imu_raw'].publish(imu_msg)
        publishers['battery'].publish(battery_msg)
        if publishers['state'] is not None and state_msg is not None:
            publishers['state'].publish(state_msg)
        if publishers['extended_state'] is not None and extended_state_msg is not None:
            publishers['extended_state'].publish(extended_state_msg)

    def _build_odometry_msg(
        self,
        pose_msg: PoseStamped,
        linear_velocity_enu: tuple[float, float, float],
    ) -> Odometry:
        msg = Odometry()
        msg.header = pose_msg.header
        msg.child_frame_id = 'base_link'
        msg.pose.pose = pose_msg.pose
        msg.twist.twist.linear.x = linear_velocity_enu[0]
        msg.twist.twist.linear.y = linear_velocity_enu[1]
        msg.twist.twist.linear.z = linear_velocity_enu[2]
        return msg

    def _build_pose_cov_msg(self, pose_msg: PoseStamped) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header = pose_msg.header
        msg.pose.pose = pose_msg.pose
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[14] = 0.1
        msg.pose.covariance[21] = 0.01
        msg.pose.covariance[28] = 0.01
        msg.pose.covariance[35] = 0.01
        return msg

    def _build_twist_msg(
        self,
        pose_msg: PoseStamped,
        linear_velocity_enu: tuple[float, float, float],
        frame_id: str,
    ) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = pose_msg.header.stamp
        msg.header.frame_id = frame_id
        msg.twist.linear.x = linear_velocity_enu[0]
        msg.twist.linear.y = linear_velocity_enu[1]
        msg.twist.linear.z = linear_velocity_enu[2]
        return msg

    def _build_twist_cov_msg(
        self,
        pose_msg: PoseStamped,
        linear_velocity_enu: tuple[float, float, float],
        frame_id: str,
    ) -> TwistWithCovarianceStamped:
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = pose_msg.header.stamp
        msg.header.frame_id = frame_id
        msg.twist.twist.linear.x = linear_velocity_enu[0]
        msg.twist.twist.linear.y = linear_velocity_enu[1]
        msg.twist.twist.linear.z = linear_velocity_enu[2]
        msg.twist.covariance[0] = 0.1
        msg.twist.covariance[7] = 0.1
        msg.twist.covariance[14] = 0.1
        msg.twist.covariance[21] = 0.01
        msg.twist.covariance[28] = 0.01
        msg.twist.covariance[35] = 0.01
        return msg

    def _build_navsat_msg(self, pose_msg: PoseStamped) -> NavSatFix:
        msg = NavSatFix()
        msg.header = self._make_header(pose_msg, 'gps')
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        east_m = pose_msg.pose.position.x
        north_m = pose_msg.pose.position.y
        up_m = pose_msg.pose.position.z
        lat_rad = math.radians(self._home_latitude)
        meters_per_deg_lat = 111_320.0
        meters_per_deg_lon = max(1.0, 111_320.0 * math.cos(lat_rad))

        msg.latitude = self._home_latitude + north_m / meters_per_deg_lat
        msg.longitude = self._home_longitude + east_m / meters_per_deg_lon
        msg.altitude = self._home_altitude + up_m
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 2.0,
        ]
        return msg

    def _build_imu_msg(self, pose_msg: PoseStamped) -> Imu:
        msg = Imu()
        msg.header = self._make_header(pose_msg, 'base_link')
        msg.orientation = pose_msg.pose.orientation
        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        return msg

    def _build_battery_msg(self, pose_msg: PoseStamped) -> BatteryState:
        msg = BatteryState()
        msg.header = pose_msg.header
        msg.voltage = 16.0
        msg.current = float('nan')
        msg.percentage = 1.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        return msg

    def _build_state_msg(self):
        if State is None:
            return None
        msg = State()
        for field_name, value in (
            ('connected', True),
            ('armed', self._armed),
            ('guided', self._guided),
            ('manual_input', False),
            ('mode', self._mode),
            ('system_status', 4),
        ):
            if hasattr(msg, field_name):
                setattr(msg, field_name, value)
        return msg

    def _build_extended_state_msg(self, pose_msg: PoseStamped):
        if ExtendedState is None:
            return None

        msg = ExtendedState()
        msg.header = pose_msg.header
        up_m = pose_msg.pose.position.z
        landed_state = (
            getattr(ExtendedState, 'LANDED_STATE_ON_GROUND', 1)
            if up_m <= 0.15
            else getattr(ExtendedState, 'LANDED_STATE_IN_AIR', 2)
        )

        if hasattr(msg, 'landed_state'):
            msg.landed_state = landed_state
        if hasattr(msg, 'vtol_state'):
            msg.vtol_state = getattr(ExtendedState, 'VTOL_STATE_UNDEFINED', 0)
        return msg

    @staticmethod
    def _make_header(pose_msg: PoseStamped, frame_id: str) -> Header:
        header = Header()
        header.stamp = pose_msg.header.stamp
        header.frame_id = frame_id
        return header

    @staticmethod
    def _yaw_degrees_from_quaternion(quaternion) -> float:
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        return yaw % 360.0

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
