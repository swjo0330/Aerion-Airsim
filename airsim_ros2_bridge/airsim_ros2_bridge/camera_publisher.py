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

        # Get camera info once (FOV from settings).
        self._fov = self._safe_get_camera_fov(client, camera_name, vehicle_name)

        # Get image dimensions from a test capture.
        self._width, self._height = self._safe_get_image_size(client, camera_name, vehicle_name)

        node.get_logger().info(
            f'[{vehicle_name}] Camera: {self._width}x{self._height}, FOV={self._fov:.1f}'
        )

        self._timer = node.create_timer(1.0 / publish_rate, self._publish_callback)

    def _publish_callback(self):
        try:
            responses = self._client.simGetImages([
                airsim.ImageRequest(self._camera_name, airsim.ImageType.Scene, False, False)
            ], vehicle_name=self._vehicle_name)

            if not responses:
                return

            r = self._normalize_response(responses[0])
            if r is None or r.width == 0:
                return

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

    def _safe_get_camera_fov(self, client, camera_name: str, vehicle_name: str) -> float:
        try:
            camera_info = client.simGetCameraInfo(camera_name, vehicle_name=vehicle_name)
            if hasattr(camera_info, 'fov'):
                return float(camera_info.fov)
            if isinstance(camera_info, dict) and 'fov' in camera_info:
                return float(camera_info['fov'])
        except Exception as e:
            self._node.get_logger().warn(
                f'[{vehicle_name}] CameraInfo fetch failed for {camera_name}; using fallback FOV: {e}'
            )
        return 90.0

    def _safe_get_image_size(self, client, camera_name: str, vehicle_name: str) -> tuple[int, int]:
        try:
            test_response = client.simGetImages([
                airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)
            ], vehicle_name=vehicle_name)
            if test_response:
                r = self._normalize_response(test_response[0])
                if r is not None and r.width > 0 and r.height > 0:
                    return int(r.width), int(r.height)
        except Exception as e:
            self._node.get_logger().warn(
                f'[{vehicle_name}] Initial image size fetch failed for {camera_name}; using fallback size: {e}'
            )
        return 640, 480

    @staticmethod
    def _normalize_response(resp):
        if resp is None:
            return None
        if hasattr(resp, 'width') and hasattr(resp, 'height') and hasattr(resp, 'image_data_uint8'):
            return resp
        if isinstance(resp, dict):
            width = int(resp.get('width', 0))
            height = int(resp.get('height', 0))
            image_data = resp.get('image_data_uint8', b'')

            class _Response:
                pass

            wrapped = _Response()
            wrapped.width = width
            wrapped.height = height
            wrapped.image_data_uint8 = image_data
            return wrapped
        return None
