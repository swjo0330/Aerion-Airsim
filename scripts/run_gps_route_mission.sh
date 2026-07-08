#!/usr/bin/env bash
# Upload a recorded GPS route JSON to the real MAVROS mission interface.
set -euo pipefail

ROS_WS="${ROS_WS:-$HOME/workspace/projects/aerion-airsim}"
ROUTE_FILE="${ROUTE_FILE:?set ROUTE_FILE to recordings/gps/gps_route_*.json}"
MAVROS_NAMESPACE="${MAVROS_NAMESPACE:-drone1/mavros}"
FIRMWARE="${FIRMWARE:-px4}"
TAKEOFF_ALTITUDE="${TAKEOFF_ALTITUDE:-5.0}"
TAKEOFF_ALTITUDE_SOURCE="${TAKEOFF_ALTITUDE_SOURCE:-fixed}"
ACCEPTANCE_RADIUS="${ACCEPTANCE_RADIUS:-2.0}"
HOLD_TIME="${HOLD_TIME:-0.0}"
ARM_FLAG="${ARM_FLAG:-false}"
START_FLAG="${START_FLAG:-false}"
DRY_RUN="${DRY_RUN:-false}"

ROUTE_FRAME="$(
ROUTE_FILE="$ROUTE_FILE" python3 - <<'PY'
import json
import os
from pathlib import Path

path = Path(os.environ["ROUTE_FILE"]).expanduser()
data = json.loads(path.read_text(encoding="utf-8"))
frame = str(data.get("coordinate_frame", data.get("frame", "global"))).lower()
if frame in ("global", "gps", "wgs84"):
    print("gps")
else:
    print("local")
PY
)"

if [ "$ROUTE_FRAME" = "gps" ]; then
    ALTITUDE_MODE="${ALTITUDE_MODE:-relative}"
    TAKEOFF_FLAG="${TAKEOFF_FLAG:-false}"
else
    ALTITUDE_MODE="${ALTITUDE_MODE:-relative_to_home}"
    TAKEOFF_FLAG="${TAKEOFF_FLAG:-true}"
fi

args=(
    --route-file "$ROUTE_FILE"
    --namespace "$MAVROS_NAMESPACE"
    --firmware "$FIRMWARE"
    --altitude-mode "$ALTITUDE_MODE"
    --takeoff-altitude "$TAKEOFF_ALTITUDE"
    --takeoff-altitude-source "$TAKEOFF_ALTITUDE_SOURCE"
    --acceptance-radius "$ACCEPTANCE_RADIUS"
    --hold-time "$HOLD_TIME"
)

if [ "$TAKEOFF_FLAG" = "true" ]; then
    args+=(--include-takeoff)
fi

if [ "$ARM_FLAG" = "true" ]; then
    args+=(--arm)
fi

if [ "$START_FLAG" = "true" ]; then
    args+=(--start)
fi

if [ "$DRY_RUN" = "true" ]; then
    cat <<EOF
GPS route mission upload
  file:       $ROUTE_FILE
  frame:      $ROUTE_FRAME
  namespace:  /${MAVROS_NAMESPACE#/}
  firmware:   $FIRMWARE
  altitude:   $ALTITUDE_MODE
  takeoff:    $TAKEOFF_FLAG source=$TAKEOFF_ALTITUDE_SOURCE
  arm/start:  arm=$ARM_FLAG start=$START_FLAG

EOF
    printf 'DRY_RUN command: ros2 run airsim_ros2_bridge aerion_gps_route_mission'
    printf ' %q' "${args[@]}"
    printf '\n'
    exit 0
fi

cd "$ROS_WS"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

exec ros2 run airsim_ros2_bridge aerion_gps_route_mission "${args[@]}"
