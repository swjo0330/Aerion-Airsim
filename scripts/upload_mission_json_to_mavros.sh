#!/usr/bin/env bash
# Upload an AERION mission JSON to the active MAVROS mission interface.
#
# This is the narrow upload-only entrypoint for local or remote MAVROS:
#   gps mission JSON -> MAVROS mission/push
#   local_enu legacy JSON -> current MAVROS home -> GPS mission items -> mission/push
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROS_WS="${ROS_WS:-$WORKSPACE}"
DEFAULT_MISSION_FILE="$WORKSPACE/recordings/missions/clicked_mission_gps.json"
if [ ! -f "$DEFAULT_MISSION_FILE" ]; then
    DEFAULT_MISSION_FILE="$WORKSPACE/recordings/missions/clicked_mission_current_home_fcu_axes.json"
fi
MISSION_FILE="${MISSION_FILE:-${ROUTE_FILE:-$DEFAULT_MISSION_FILE}}"
MAVROS_NAMESPACE="${MAVROS_NAMESPACE:-drone1/mavros}"
FIRMWARE="${FIRMWARE:-px4}"
TAKEOFF_ALTITUDE="${TAKEOFF_ALTITUDE:-5.0}"
TAKEOFF_ALTITUDE_SOURCE="${TAKEOFF_ALTITUDE_SOURCE:-first_waypoint}"
TAKEOFF_POSITION_SOURCE="${TAKEOFF_POSITION_SOURCE:-current}"
ACCEPTANCE_RADIUS="${ACCEPTANCE_RADIUS:-2.5}"
HOLD_TIME="${HOLD_TIME:-0.0}"
YAW="${YAW:-nan}"
MISSION_ARM="${MISSION_ARM:-false}"
FORCE_ARM="${FORCE_ARM:-true}"
START_MISSION="${START_MISSION:-false}"
TIMEOUT="${TIMEOUT:-8.0}"
DRY_RUN="${DRY_RUN:-false}"

if [ ! -f "$MISSION_FILE" ]; then
    echo "ERROR: mission file not found: $MISSION_FILE" >&2
    exit 2
fi

MISSION_FRAME="$(
MISSION_FILE="$MISSION_FILE" python3 - <<'PY'
import json
import os
from pathlib import Path

path = Path(os.environ["MISSION_FILE"]).expanduser()
data = json.loads(path.read_text(encoding="utf-8"))
frame = str(data.get("coordinate_frame", data.get("frame", "global"))).lower()
if frame in ("global", "gps", "wgs84"):
    print("gps")
else:
    print("local")
PY
)"

if [ "$MISSION_FRAME" = "gps" ]; then
    ALTITUDE_MODE="${ALTITUDE_MODE:-relative}"
    INCLUDE_TAKEOFF="${INCLUDE_TAKEOFF:-false}"
else
    ALTITUDE_MODE="${ALTITUDE_MODE:-relative_to_home}"
    INCLUDE_TAKEOFF="${INCLUDE_TAKEOFF:-true}"
fi

args=(
    --route-file "$MISSION_FILE"
    --namespace "$MAVROS_NAMESPACE"
    --firmware "$FIRMWARE"
    --altitude-mode "$ALTITUDE_MODE"
    --takeoff-altitude "$TAKEOFF_ALTITUDE"
    --takeoff-altitude-source "$TAKEOFF_ALTITUDE_SOURCE"
    --takeoff-position-source "$TAKEOFF_POSITION_SOURCE"
    --acceptance-radius "$ACCEPTANCE_RADIUS"
    --hold-time "$HOLD_TIME"
    --yaw "$YAW"
    --timeout "$TIMEOUT"
)

if [ "$INCLUDE_TAKEOFF" = "true" ]; then
    args+=(--include-takeoff)
fi
if [ "$MISSION_ARM" = "true" ]; then
    args+=(--arm)
    if [ "$FORCE_ARM" = "true" ]; then
        args+=(--force-arm)
    fi
fi
if [ "$START_MISSION" = "true" ]; then
    args+=(--start)
fi

cat <<EOF
Mission upload
  file:       $MISSION_FILE
  frame:      $MISSION_FRAME
  namespace:  /${MAVROS_NAMESPACE#/}
  firmware:   $FIRMWARE
  altitude:   $ALTITUDE_MODE
  takeoff:    $INCLUDE_TAKEOFF source=$TAKEOFF_ALTITUDE_SOURCE position=$TAKEOFF_POSITION_SOURCE
  arm/start:  arm=$MISSION_ARM force_arm=$FORCE_ARM start=$START_MISSION

EOF

if [ "$DRY_RUN" = "true" ]; then
    printf 'DRY_RUN command: ros2 run airsim_ros2_bridge aerion_gps_route_mission'
    printf ' %q' "${args[@]}"
    printf '\n'
    exit 0
fi

cd "$ROS_WS"
set +u
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
set -u

exec ros2 run airsim_ros2_bridge aerion_gps_route_mission "${args[@]}"
