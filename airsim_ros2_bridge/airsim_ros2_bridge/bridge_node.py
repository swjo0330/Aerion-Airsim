import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import airsim
import threading

from airsim_ros2_bridge.camera_publisher import CameraPublisher
from airsim_ros2_bridge.drone_controller import DroneController


class ThreadSafeAirSimClient:
    """Serialize AirSim RPC calls across threads to avoid Tornado IOLoop re-entry."""

    def __init__(self, client):
        self._client = client
        self._lock = threading.RLock()

    def __getattr__(self, name):
        attr = getattr(self._client, name)
        if not callable(attr):
            return attr

        def locked_call(*args, **kwargs):
            with self._lock:
                return attr(*args, **kwargs)

        return locked_call


class AirSimBridgeNode(Node):
    def __init__(self):
        super().__init__('airsim_bridge')

        # Parameters
        self.declare_parameter('vehicle_names', ['Drone0', 'Drone1'])
        self.declare_parameter('camera_name', 'front_center')
        self.declare_parameter('camera_fps', 30.0)
        self.declare_parameter('enable_camera', False)
        self.declare_parameter('enable_ardu_compat', True)
        self.declare_parameter('ardu_compat_vehicle', 'Drone0')
        self.declare_parameter('velocity_control_mode', 'kinematic')
        self.declare_parameter('control_backend', 'px4_mavros')
        self.declare_parameter('velocity_command_duration', 0.2)
        self.declare_parameter('kinematic_z_ned', -1.0)
        self.declare_parameter('airsim_ip', '127.0.0.1')
        self.declare_parameter('airsim_port', 41451)
        self.declare_parameter('airsim_timeout_sec', 2.0)
        self.declare_parameter('home_latitude', 37.5665)
        self.declare_parameter('home_longitude', 126.9780)
        self.declare_parameter('home_altitude', 0.0)

        vehicle_names = self.get_parameter('vehicle_names').get_parameter_value().string_array_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        camera_fps = self.get_parameter('camera_fps').get_parameter_value().double_value
        enable_camera = self.get_parameter('enable_camera').get_parameter_value().bool_value
        enable_ardu_compat = self.get_parameter('enable_ardu_compat').get_parameter_value().bool_value
        ardu_compat_vehicle = self.get_parameter('ardu_compat_vehicle').get_parameter_value().string_value
        velocity_control_mode = self.get_parameter('velocity_control_mode').get_parameter_value().string_value
        control_backend = self.get_parameter('control_backend').get_parameter_value().string_value
        velocity_command_duration = (
            self.get_parameter('velocity_command_duration').get_parameter_value().double_value
        )
        kinematic_z_ned = self.get_parameter('kinematic_z_ned').get_parameter_value().double_value
        airsim_ip = self.get_parameter('airsim_ip').get_parameter_value().string_value
        airsim_port = self.get_parameter('airsim_port').get_parameter_value().integer_value
        airsim_timeout_sec = self.get_parameter('airsim_timeout_sec').get_parameter_value().double_value
        home_latitude = self.get_parameter('home_latitude').get_parameter_value().double_value
        home_longitude = self.get_parameter('home_longitude').get_parameter_value().double_value
        home_altitude = self.get_parameter('home_altitude').get_parameter_value().double_value

        # Connect to AirSim
        self.get_logger().info(f'Connecting to AirSim at {airsim_ip}:{airsim_port}...')
        raw_client = airsim.MultirotorClient(
            ip=airsim_ip,
            port=airsim_port,
            timeout_value=airsim_timeout_sec,
        )
        self._client = ThreadSafeAirSimClient(raw_client)
        self._client.confirmConnection()
        self.get_logger().info('Connected to AirSim!')

        # Create camera publishers and controllers for each vehicle
        self._camera_publishers = []
        self._drone_controllers = []

        for index, vehicle_name in enumerate(vehicle_names):
            self.get_logger().info(f'Setting up {vehicle_name}...')

            if enable_camera:
                try:
                    cam_pub = CameraPublisher(
                        node=self,
                        client=self._client,
                        vehicle_name=vehicle_name,
                        camera_name=camera_name,
                        publish_rate=camera_fps,
                    )
                    self._camera_publishers.append(cam_pub)
                except Exception as e:
                    self.get_logger().warn(
                        f'[{vehicle_name}] Camera publisher disabled after initialization error: {e}'
                    )

            controller = DroneController(
                node=self,
                client=self._client,
                vehicle_name=vehicle_name,
                enable_ardu_compat=enable_ardu_compat and vehicle_name == ardu_compat_vehicle,
                velocity_control_mode=velocity_control_mode,
                control_backend=control_backend,
                velocity_command_duration=velocity_command_duration,
                kinematic_z_ned=kinematic_z_ned,
                home_latitude=home_latitude,
                home_longitude=home_longitude,
                home_altitude=home_altitude,
                mavros_instance_namespace=f'mavros{index}',
            )
            self._drone_controllers.append(controller)

        self.get_logger().info(f'Bridge running for {len(vehicle_names)} vehicles')


def main(args=None):
    rclpy.init(args=args)
    node = AirSimBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
