#!/usr/bin/env bash
# Smoke-test the verified AirSim ROS2 bridge path from WSL.
set -euo pipefail

ROS_WS="${ROS_WS:-$HOME/aerion_ros2_ws}"
AIRSIM_IP="${AIRSIM_IP:-172.23.80.1}"
AIRSIM_PORT="${AIRSIM_PORT:-41451}"
AIRSIM_TIMEOUT_SEC="${AIRSIM_TIMEOUT_SEC:-2.0}"
VEHICLE_NAME="${VEHICLE_NAME:-Drone0}"
ROS_TOPIC_TIMEOUT="${ROS_TOPIC_TIMEOUT:-20}"
MOVE_SPEED_X="${MOVE_SPEED_X:-0.5}"
MOVE_DURATION_SEC="${MOVE_DURATION_SEC:-1.0}"
MIN_MOVE_DELTA="${MIN_MOVE_DELTA:-0.05}"
RESET_ROS_DAEMON="${RESET_ROS_DAEMON:-false}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: ROS2 Humble setup not found at /opt/ros/humble/setup.bash" >&2
    exit 1
fi

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
    echo "ERROR: ROS workspace setup not found at $ROS_WS/install/setup.bash" >&2
    echo "Build first: cd $ROS_WS && colcon build --packages-select airsim_ros2_bridge" >&2
    exit 1
fi

cd "$ROS_WS"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

if [ "$RESET_ROS_DAEMON" = "true" ]; then
    ros2 daemon stop || true
    ros2 daemon start
fi

pose_xyz() {
    python3 - "$AIRSIM_IP" "$AIRSIM_PORT" "$AIRSIM_TIMEOUT_SEC" "$VEHICLE_NAME" <<'PY'
import sys
import contextlib
import airsim

ip, port, timeout, vehicle = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
c = airsim.MultirotorClient(ip=ip, port=port, timeout_value=timeout)
with open("/dev/null", "w") as devnull, contextlib.redirect_stdout(devnull):
    c.confirmConnection()
state = c.client.call("getMultirotorState", vehicle)
pos = state[1][0]
print(f"{pos[0]} {pos[1]} {pos[2]}")
PY
}

echo "Checking AirSim RPC at ${AIRSIM_IP}:${AIRSIM_PORT} for ${VEHICLE_NAME}..."
before_pose="$(pose_xyz)"
echo "Initial AirSim pose: $before_pose"

echo "Checking /ap/status..."
timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once /ap/status >/tmp/aerion_ap_status.txt
cat /tmp/aerion_ap_status.txt

echo "Checking /ap/pose/filtered..."
timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once /ap/pose/filtered >/tmp/aerion_ap_pose.txt
head -n 20 /tmp/aerion_ap_pose.txt

echo "Checking /ap/twist/filtered..."
timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once /ap/twist/filtered >/tmp/aerion_ap_twist.txt
head -n 20 /tmp/aerion_ap_twist.txt

echo "Publishing /ap/cmd_vel..."
publish_count="$(python3 - "$MOVE_DURATION_SEC" <<'PY'
import math
import sys

duration = float(sys.argv[1])
print(max(1, math.ceil(duration * 5.0)))
PY
)"
ros2 topic pub --times "$publish_count" -r 5 /ap/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: ${MOVE_SPEED_X}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

after_pose="$(pose_xyz)"
echo "Final AirSim pose: $after_pose"

python3 - "$before_pose" "$after_pose" "$MIN_MOVE_DELTA" <<'PY'
import math
import sys

before = [float(v) for v in sys.argv[1].split()]
after = [float(v) for v in sys.argv[2].split()]
minimum = float(sys.argv[3])
delta = math.sqrt(sum((b - a) ** 2 for b, a in zip(before, after)))
print(f"Pose delta: {delta:.3f} m")
if delta < minimum:
    raise SystemExit(f"ERROR: pose delta {delta:.3f} m is below MIN_MOVE_DELTA={minimum}")
PY

echo "Smoke test passed."
