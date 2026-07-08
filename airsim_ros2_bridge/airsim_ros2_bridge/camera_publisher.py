"""AERION Phase 2~: 단일 드론 RGB 카메라 → ROS2 토픽 발행 모듈.

설계 의도:
  - 한 vehicle = 한 인스턴스 (1 드론 = 1 프로세스 패턴, AirSim RPC 격리).
  - 카메라 폴링은 timer 기반 (ReentrantCallbackGroup → MultiThreadedExecutor에서 병렬 콜백 허용).
  - AirSim RPC 50Hz 상한 안에서 카메라 + Range + DroneController가 공존하므로 카메라 권장 ~10Hz.
  - 카메라 이름은 settings.json에 정의된 것 그대로 (`front_center` 기본). 일부 fork에서 `0`, `1` 같은
    숫자 이름을 쓰는 경우 _resolve_camera_name 폴백.

토픽 / 프레임:
  Topic:  /{vehicle_name}/camera/{image,camera_info}
  Frame:  {vehicle_name}_{camera_name}_optical  (현재. 향후 표준 `drone{N}/camera_optical_frame` 통일 검토)

알려진 함정:
  - AirSim RPC 동시 호출 시 'IOLoop is already running' (microsoft/AirSim#2607). ThreadSafeAirSimClient
    (bridge_node.py 안)이 RLock으로 직렬화. 본 모듈은 단일 client 사용해 외부 격리 의존.
  - simGetCameraInfo / simGetImages 의 vehicle_name 인자가 일부 fork에서 거부됨 → try/except 폴백.
"""

import airsim
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from airsim_ros2_bridge.utils import build_camera_info, airsim_rgb_to_image_msg
from airsim_ros2_bridge.topic_naming import sensor_topic_prefix


class CameraPublisher:
    """단일 드론의 RGB 카메라(`front_center` 기본) → sensor_msgs/Image + CameraInfo 발행."""

    def __init__(
        self,
        node: Node,
        client: airsim.MultirotorClient,
        vehicle_name: str,
        camera_name: str = 'front_center',
        publish_rate: float = 30.0,
        image_channel_order: str = 'bgr',
        publish_compressed: bool = False,
        jpeg_quality: int = 70,
        topic_namespace: str | None = None,
    ):
        self._node = node
        self._client = client
        self._vehicle_name = vehicle_name
        self._camera_name = camera_name
        self._image_channel_order = image_channel_order
        self._publish_compressed = bool(publish_compressed)
        self._jpeg_quality = int(jpeg_quality)
        self._camera_candidates = self._build_camera_candidates(camera_name)
        self._last_camera_error_ns = 0
        self._callback_group = ReentrantCallbackGroup()

        topic_prefix = sensor_topic_prefix(vehicle_name, 'camera', topic_namespace)
        self._image_pub = node.create_publisher(Image, f'{topic_prefix}/image', 10)
        self._info_pub = node.create_publisher(CameraInfo, f'{topic_prefix}/camera_info', 10)
        # 압축 전송용: raw 2.76MB는 zenoh/Tailscale에서 단편 드롭되므로 JPEG로 발행.
        self._compressed_pub = (
            node.create_publisher(CompressedImage, f'{topic_prefix}/image/compressed', 10)
            if self._publish_compressed else None
        )

        self._frame_id = f'{vehicle_name}_{camera_name}_optical'

        # Resolve camera name first, then fetch FOV/size.
        self._camera_name = self._resolve_camera_name(vehicle_name)
        self._frame_id = f'{vehicle_name}_{self._camera_name}_optical'

        # Get camera info once (FOV from settings).
        self._fov = self._safe_get_camera_fov(client, self._camera_name, vehicle_name)

        # Get image dimensions from a test capture.
        self._width, self._height = self._safe_get_image_size(client, self._camera_name, vehicle_name)

        node.get_logger().info(
            f'[{vehicle_name}] Camera: {self._width}x{self._height}, FOV={self._fov:.1f}'
        )

        self._timer = node.create_timer(
            1.0 / publish_rate,
            self._publish_callback,
            callback_group=self._callback_group,
        )

    def _publish_callback(self):
        try:
            responses = self._get_images(self._camera_name, self._vehicle_name)

            if not responses:
                return

            r = self._normalize_response(responses[0])
            if r is None or r.width == 0:
                return

            stamp = self._node.get_clock().now().to_msg()

            if self._compressed_pub is not None:
                self._compressed_pub.publish(self._encode_jpeg(r, stamp))
            else:
                image_msg = airsim_rgb_to_image_msg(
                    r.image_data_uint8,
                    r.width,
                    r.height,
                    self._frame_id,
                    stamp,
                    self._image_channel_order,
                )
                self._image_pub.publish(image_msg)

            info_msg = build_camera_info(
                self._fov, r.width, r.height, self._frame_id, stamp
            )
            self._info_pub.publish(info_msg)

        except Exception as e:
            now_ns = self._node.get_clock().now().nanoseconds
            if now_ns - self._last_camera_error_ns > 5_000_000_000:
                self._node.get_logger().warn(f'[{self._vehicle_name}] Camera error: {e}')
                self._last_camera_error_ns = now_ns

    def _encode_jpeg(self, r, stamp) -> CompressedImage:
        """AirSim raw(BGR) → JPEG CompressedImage. cv2가 BGR을 기대하므로 채널 스왑 없이 인코딩."""
        import numpy as np
        import cv2

        px = r.width * r.height
        buf = np.frombuffer(r.image_data_uint8, dtype=np.uint8)
        ch = (buf.size // px) if px else 3
        if ch not in (3, 4):
            ch = 3
        img = buf[: px * ch].reshape(r.height, r.width, ch)
        if ch == 4:
            img = img[:, :, :3]
        ok, enc = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.format = 'jpeg'
        msg.data = enc.tobytes() if ok else b''
        return msg

    def _safe_get_camera_fov(self, client, camera_name: str, vehicle_name: str) -> float:
        try:
            camera_info = self._get_camera_info(camera_name, vehicle_name)
            if hasattr(camera_info, 'fov'):
                return float(camera_info.fov)
            if isinstance(camera_info, dict) and 'fov' in camera_info:
                return float(camera_info['fov'])
            if isinstance(camera_info, list):
                for item in camera_info:
                    if isinstance(item, dict) and 'fov' in item:
                        return float(item['fov'])
                    if hasattr(item, 'fov'):
                        return float(item.fov)
        except Exception as e:
            self._node.get_logger().warn(
                f'[{vehicle_name}] CameraInfo fetch failed for {camera_name}; using fallback FOV: {e}'
            )
        return 90.0

    def _safe_get_image_size(self, client, camera_name: str, vehicle_name: str) -> tuple[int, int]:
        try:
            test_response = self._get_images(camera_name, vehicle_name)
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

    def _get_camera_info(self, camera_name: str, vehicle_name: str):
        try:
            return self._client.simGetCameraInfo(camera_name, vehicle_name=vehicle_name)
        except Exception:
            # Some AirSim forks reject vehicle_name for camera RPC.
            return self._client.simGetCameraInfo(camera_name)

    def _get_images(self, camera_name: str, vehicle_name: str):
        req = [airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)]
        try:
            return self._client.simGetImages(req, vehicle_name=vehicle_name)
        except Exception as first_error:
            # Some AirSim forks reject vehicle_name for camera RPC.
            try:
                return self._client.simGetImages(req)
            except Exception:
                raise first_error

    def _resolve_camera_name(self, vehicle_name: str) -> str:
        for candidate in self._camera_candidates:
            try:
                responses = self._get_images(candidate, vehicle_name)
                if responses:
                    r = self._normalize_response(responses[0])
                    if r is not None and r.width > 0 and r.height > 0:
                        if candidate != self._camera_name:
                            self._node.get_logger().info(
                                f'[{vehicle_name}] Camera name fallback: {self._camera_name} -> {candidate}'
                            )
                        return candidate
            except Exception:
                continue
        return self._camera_name

    @staticmethod
    def _build_camera_candidates(camera_name: str):
        ordered = [camera_name, 'front_center', '0', '1']
        dedup = []
        for name in ordered:
            if name not in dedup:
                dedup.append(name)
        return dedup
