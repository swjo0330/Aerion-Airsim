"""AERION 공통 유틸: 카메라 intrinsics 계산 + AirSim 이미지 → ROS sensor_msgs/Image 변환.

본 모듈은 외부 ROS 의존성 외 모두 표준 라이브러리(numpy)만 사용. 단위 함수 모음이라 단독 import 가능.
"""

import math
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


def fov_to_intrinsics(fov_degrees: float, width: int, height: int) -> tuple[float, float, float, float]:
    """수평 FOV(도) + 해상도 → 카메라 intrinsic 파라미터 (fx, fy, cx, cy).

    AirSim 카메라는 pinhole 모델 + 왜곡 없음 가정. fx = width / (2*tan(FOV/2)).
    fy = fx (square pixel). cx, cy = 영상 중심.
    """
    fov_rad = math.radians(fov_degrees)
    fx = width / (2.0 * math.tan(fov_rad / 2.0))
    fy = fx  # square pixels
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


def build_camera_info(
    fov_degrees: float,
    width: int,
    height: int,
    frame_id: str,
    stamp: Time,
) -> CameraInfo:
    """Build a sensor_msgs/CameraInfo from AirSim camera parameters."""
    fx, fy, cx, cy = fov_to_intrinsics(fov_degrees, width, height)

    msg = CameraInfo()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def airsim_rgb_to_image_msg(
    image_data: bytes,
    width: int,
    height: int,
    frame_id: str,
    stamp: Time,
    channel_order: str = 'rgb',
) -> Image:
    """Convert raw AirSim image bytes to sensor_msgs/Image.

    AirSim may return RGB/RGBA or BGR/BGRA depending on UE/AirSim fork.
    Always output rgb8 for the /camera/image topic.
    """
    msg = Image()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.encoding = 'rgb8'
    msg.is_bigendian = False
    msg.step = width * 3

    channels = len(image_data) // (height * width)
    raw = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, channels)
    rgb = raw[:, :, :3]
    if channel_order.lower() in ('bgr', 'bgra'):
        rgb = rgb[:, :, ::-1]
    msg.data = rgb.tobytes()
    return msg
