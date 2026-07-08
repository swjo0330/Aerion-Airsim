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
RECORDING_DIR="${RECORDING_DIR:-$ROS_WS/recordings}"
ROUTE_FILE="${ROUTE_FILE:-}"
GPS_TOPIC="${GPS_TOPIC:-}"
FOLLOW_SPEED="${FOLLOW_SPEED:-1.5}"
FOLLOW_VERTICAL_SPEED="${FOLLOW_VERTICAL_SPEED:-0.8}"
WAYPOINT_RADIUS="${WAYPOINT_RADIUS:-1.5}"
ALTITUDE_TOLERANCE="${ALTITUDE_TOLERANCE:-1.0}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/drone1/camera/image}"
CAMERA_RECORD_FPS="${CAMERA_RECORD_FPS:-10.0}"
CAMERA_RECORD_CODEC="${CAMERA_RECORD_CODEC:-FFV1}"

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
    --recording-dir "$RECORDING_DIR"
    --follow-speed "$FOLLOW_SPEED"
    --follow-vertical-speed "$FOLLOW_VERTICAL_SPEED"
    --waypoint-radius "$WAYPOINT_RADIUS"
    --altitude-tolerance "$ALTITUDE_TOLERANCE"
    --camera-topic "$CAMERA_TOPIC"
    --camera-record-fps "$CAMERA_RECORD_FPS"
    --camera-record-codec "$CAMERA_RECORD_CODEC"
)

if [ -n "$GPS_TOPIC" ]; then
    args+=(--gps-topic "$GPS_TOPIC")
fi

if [ -n "$ROUTE_FILE" ]; then
    args+=(--route-file "$ROUTE_FILE")
fi

if [ "$ARM_FLAG" = "true" ]; then
    args+=(--arm)
fi

if [ "$FORCE_ARM_FLAG" = "true" ]; then
    args+=(--force-arm)
fi

exec ros2 run airsim_ros2_bridge aerion_manual_mavros_control "${args[@]}"
