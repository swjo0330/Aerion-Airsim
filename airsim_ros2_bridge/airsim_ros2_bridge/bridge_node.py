import rclpy
from rclpy.node import Node
import airsim

from airsim_ros2_bridge.camera_publisher import CameraPublisher
from airsim_ros2_bridge.drone_controller import DroneController


class AirSimBridgeNode(Node):
    def __init__(self):
        super().__init__('airsim_bridge')

        # Parameters
        self.declare_parameter('vehicle_names', ['Drone0', 'Drone1'])
        self.declare_parameter('camera_name', 'front_center')
        self.declare_parameter('camera_fps', 30.0)
        self.declare_parameter('airsim_ip', '127.0.0.1')
        self.declare_parameter('airsim_port', 41451)

        vehicle_names = self.get_parameter('vehicle_names').get_parameter_value().string_array_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        camera_fps = self.get_parameter('camera_fps').get_parameter_value().double_value
        airsim_ip = self.get_parameter('airsim_ip').get_parameter_value().string_value
        airsim_port = self.get_parameter('airsim_port').get_parameter_value().integer_value

        # Connect to AirSim
        self.get_logger().info(f'Connecting to AirSim at {airsim_ip}:{airsim_port}...')
        self._client = airsim.MultirotorClient(ip=airsim_ip, port=airsim_port)
        self._client.confirmConnection()
        self.get_logger().info('Connected to AirSim!')

        # Create camera publishers and controllers for each vehicle
        self._camera_publishers = []
        self._drone_controllers = []

        for vehicle_name in vehicle_names:
            self.get_logger().info(f'Setting up {vehicle_name}...')

            cam_pub = CameraPublisher(
                node=self,
                client=self._client,
                vehicle_name=vehicle_name,
                camera_name=camera_name,
                publish_rate=camera_fps,
            )
            self._camera_publishers.append(cam_pub)

            controller = DroneController(
                node=self,
                client=self._client,
                vehicle_name=vehicle_name,
            )
            self._drone_controllers.append(controller)

        self.get_logger().info(f'Bridge running for {len(vehicle_names)} vehicles')


def main(args=None):
    rclpy.init(args=args)
    node = AirSimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
