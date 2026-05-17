#!/usr/bin/env bash
# Run the verified AirSim ROS2 bridge configuration from WSL.
set -euo pipefail

ROS_WS="${ROS_WS:-$HOME/aerion_ros2_ws}"
AIRSIM_IP="${AIRSIM_IP:-172.23.80.1}"
AIRSIM_PORT="${AIRSIM_PORT:-41451}"
AIRSIM_TIMEOUT_SEC="${AIRSIM_TIMEOUT_SEC:-2.0}"
VEHICLE_NAME="${VEHICLE_NAME:-Drone0}"
VEHICLE_INDEX="${VEHICLE_INDEX:-${VEHICLE_NAME##*[!0-9]}}"
VEHICLE_INDEX="${VEHICLE_INDEX:-0}"
MAVROS_NAMESPACE="${MAVROS_NAMESPACE:-mavros${VEHICLE_INDEX}}"
NODE_NAME="${NODE_NAME:-airsim_bridge_${VEHICLE_NAME}}"
ARDU_COMPAT_VEHICLE="${ARDU_COMPAT_VEHICLE:-Drone0}"
ENABLE_CAMERA="${ENABLE_CAMERA:-false}"
if [ -z "${ENABLE_ARDU_COMPAT+x}" ]; then
    if [ "$VEHICLE_NAME" = "$ARDU_COMPAT_VEHICLE" ]; then
        ENABLE_ARDU_COMPAT=true
    else
        ENABLE_ARDU_COMPAT=false
    fi
fi
VELOCITY_CONTROL_MODE="${VELOCITY_CONTROL_MODE:-kinematic}"
CONTROL_BACKEND="${CONTROL_BACKEND:-px4_mavros}"
VELOCITY_COMMAND_DURATION="${VELOCITY_COMMAND_DURATION:-0.2}"
KINEMATIC_Z_NED="${KINEMATIC_Z_NED:--1.0}"

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

exec ros2 run airsim_ros2_bridge bridge_node --ros-args \
    -r __node:="$NODE_NAME" \
    -p vehicle_name:="$VEHICLE_NAME" \
    -p vehicle_index:="$VEHICLE_INDEX" \
    -p mavros_instance_namespace:="$MAVROS_NAMESPACE" \
    -p airsim_ip:="$AIRSIM_IP" \
    -p airsim_port:="$AIRSIM_PORT" \
    -p enable_camera:="$ENABLE_CAMERA" \
    -p enable_ardu_compat:="$ENABLE_ARDU_COMPAT" \
    -p ardu_compat_vehicle:="$ARDU_COMPAT_VEHICLE" \
    -p velocity_control_mode:="$VELOCITY_CONTROL_MODE" \
    -p control_backend:="$CONTROL_BACKEND" \
    -p velocity_command_duration:="$VELOCITY_COMMAND_DURATION" \
    -p kinematic_z_ned:="$KINEMATIC_Z_NED" \
    -p airsim_timeout_sec:="$AIRSIM_TIMEOUT_SEC"
