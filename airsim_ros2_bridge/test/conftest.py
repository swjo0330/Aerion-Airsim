"""conftest.py — stub ROS2 modules so pure-Python tests run without a ROS2 runtime."""
import sys
from unittest.mock import MagicMock

# Stub out ROS2 / geometry / sensor / std modules that formation_node.py imports
# at module level.  Only the dataclasses (DroneState, MorphState) need to be
# reachable — no ROS2 runtime is needed for those.
for _mod in (
    'rclpy',
    'rclpy.node',
    'geometry_msgs',
    'geometry_msgs.msg',
    'sensor_msgs',
    'sensor_msgs.msg',
    'std_msgs',
    'std_msgs.msg',
):
    sys.modules.setdefault(_mod, MagicMock())
