#!/usr/bin/env bash
# Run one AirSim ROS2 bridge process per vehicle from WSL.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRONE_COUNT="${DRONE_COUNT:-2}"
VEHICLES="${VEHICLES:-}"
PIDS=""

if [ -z "$VEHICLES" ]; then
    for ((i = 0; i < DRONE_COUNT; i++)); do
        VEHICLES="${VEHICLES} Drone${i}"
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
    echo "Starting bridge instance for ${vehicle_name} on /mavros${vehicle_index}..."
    VEHICLE_NAME="$vehicle_name" \
    VEHICLE_INDEX="$vehicle_index" \
    MAVROS_NAMESPACE="mavros${vehicle_index}" \
    NODE_NAME="airsim_bridge_${vehicle_name}" \
    "$SCRIPT_DIR/run_airsim_ros2_bridge.sh" &
    PIDS="$PIDS $!"
    index=$((index + 1))
done

wait
