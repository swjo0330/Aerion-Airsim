import math
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


def fov_to_intrinsics(fov_degrees: float, width: int, height: int) -> tuple[float, float, float, float]:
    """Convert horizontal FOV + resolution to camera intrinsic parameters.

    Returns (fx, fy, cx, cy).
    AirSim uses a pinhole camera model with no distortion.
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
) -> Image:
    """Convert raw AirSim image bytes to sensor_msgs/Image.

    AirSim may return RGB(3ch) or RGBA(4ch) depending on UE version.
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
    msg.data = rgb.tobytes()
    return msg
