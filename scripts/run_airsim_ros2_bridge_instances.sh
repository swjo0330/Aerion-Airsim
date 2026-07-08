#!/usr/bin/env bash
# Run one AirSim ROS2 bridge process per vehicle from WSL.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRONE_COUNT="${DRONE_COUNT:-3}"
VEHICLES="${VEHICLES:-}"
MASTER_VEHICLE="${MASTER_VEHICLE:-drone1}"
PIDS=""

if [ -z "$VEHICLES" ]; then
    for ((i = 1; i <= DRONE_COUNT; i++)); do
        VEHICLES="${VEHICLES} drone${i}"
    done
    VEHICLES="${VEHICLES# }"
fi

cleanup() {
    for pid in $PIDS; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}

trap cleanup INT TERM EXIT

index=0
for vehicle_name in $VEHICLES; do
    vehicle_index="${vehicle_name##*[!0-9]}"
    vehicle_index="${vehicle_index:-$index}"
    if [[ "$vehicle_name" =~ ^drone([0-9]+)$ ]]; then
        vehicle_index=$((${BASH_REMATCH[1]} - 1))
    fi
    mavros_namespace="${MAVROS_NAMESPACE:-$vehicle_name/mavros}"
    enable_camera=false
    if [ "$vehicle_name" = "$MASTER_VEHICLE" ]; then
        enable_camera=true
    fi
    echo "Starting bridge instance for ${vehicle_name} on /${mavros_namespace} (camera=${enable_camera})..."
    VEHICLE_NAME="$vehicle_name" \
    VEHICLE_INDEX="$vehicle_index" \
    MAVROS_NAMESPACE="$mavros_namespace" \
    NODE_NAME="airsim_bridge_${vehicle_name}" \
    ENABLE_CAMERA="$enable_camera" \
    ENABLE_RANGE="${ENABLE_RANGE:-true}" \
    AIRSIM_IMAGE_CHANNEL_ORDER="${AIRSIM_IMAGE_CHANNEL_ORDER:-bgr}" \
    ENABLE_LIDAR="${ENABLE_LIDAR:-false}" \
    LIDAR_NAME="${LIDAR_NAME:-Lidar_Front}" \
    LIDAR_PUBLISH_RATE="${LIDAR_PUBLISH_RATE:-10.0}" \
    ENABLE_LIDAR_OBSTACLE="${ENABLE_LIDAR_OBSTACLE:-true}" \
    ENABLE_GPS_DEBUG_LOG="${ENABLE_GPS_DEBUG_LOG:-false}" \
    GPS_DEBUG_LOG_RATE="${GPS_DEBUG_LOG_RATE:-2.0}" \
    GPS_DEBUG_SENSOR_NAME="${GPS_DEBUG_SENSOR_NAME:-Gps}" \
    "$SCRIPT_DIR/run_airsim_ros2_bridge.sh" &
    PIDS="$PIDS $!"
    index=$((index + 1))
done

wait
