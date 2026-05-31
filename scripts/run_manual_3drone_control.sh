#!/usr/bin/env bash
# Keyboard teleop for the 3-drone CARLA/AirSim validation setup.
set -euo pipefail

ROS_WS="${ROS_WS:-$HOME/workspace/projects/aerion-airsim}"
VEHICLES="${VEHICLES:-drone1,drone2,drone3}"
RATE="${RATE:-20}"
SPEED="${SPEED:-1.0}"
VERTICAL_SPEED="${VERTICAL_SPEED:-0.6}"
YAW_RATE="${YAW_RATE:-0.7}"
HOLD_TIMEOUT="${HOLD_TIMEOUT:-0.25}"
ARM_FLAG="${ARM_FLAG:-true}"
FORCE_ARM_FLAG="${FORCE_ARM_FLAG:-true}"

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

args=(
    --vehicles "$VEHICLES"
    --rate "$RATE"
    --speed "$SPEED"
    --vertical-speed "$VERTICAL_SPEED"
    --yaw-rate "$YAW_RATE"
    --hold-timeout "$HOLD_TIMEOUT"
)

if [ "$ARM_FLAG" = "true" ]; then
    args+=(--arm)
fi

if [ "$FORCE_ARM_FLAG" = "true" ]; then
    args+=(--force-arm)
fi

exec ros2 run airsim_ros2_bridge aerion_manual_mavros_control "${args[@]}"
