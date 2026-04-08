import airsim
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from airsim_ros2_bridge.utils import build_camera_info, airsim_rgb_to_image_msg


class CameraPublisher:
    """Publishes camera image and info for a single drone."""

    def __init__(
        self,
        node: Node,
        client: airsim.MultirotorClient,
        vehicle_name: str,
        camera_name: str = 'front_center',
        publish_rate: float = 30.0,
    ):
        self._node = node
        self._client = client
        self._vehicle_name = vehicle_name
        self._camera_name = camera_name

        topic_prefix = f'/{vehicle_name}/camera'
        self._image_pub = node.create_publisher(Image, f'{topic_prefix}/image', 10)
        self._info_pub = node.create_publisher(CameraInfo, f'{topic_prefix}/camera_info', 10)

        self._frame_id = f'{vehicle_name}_{camera_name}_optical'

        # Get camera info once (FOV from settings)
        camera_info = client.simGetCameraInfo(camera_name, vehicle_name=vehicle_name)
        self._fov = camera_info.fov

        # Get image dimensions from a test capture
        test_response = client.simGetImages([
            airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)
        ], vehicle_name=vehicle_name)
        self._width = test_response[0].width
        self._height = test_response[0].height

        node.get_logger().info(
            f'[{vehicle_name}] Camera: {self._width}x{self._height}, FOV={self._fov:.1f}'
        )

        self._timer = node.create_timer(1.0 / publish_rate, self._publish_callback)

    def _publish_callback(self):
        try:
            responses = self._client.simGetImages([
                airsim.ImageRequest(self._camera_name, airsim.ImageType.Scene, False, False)
            ], vehicle_name=self._vehicle_name)

            if not responses or responses[0].width == 0:
                return

            r = responses[0]
            stamp = self._node.get_clock().now().to_msg()

            image_msg = airsim_rgb_to_image_msg(
                r.image_data_uint8, r.width, r.height, self._frame_id, stamp
            )
            self._image_pub.publish(image_msg)

            info_msg = build_camera_info(
                self._fov, r.width, r.height, self._frame_id, stamp
            )
            self._info_pub.publish(info_msg)

        except Exception as e:
            self._node.get_logger().warn(f'[{self._vehicle_name}] Camera error: {e}')
