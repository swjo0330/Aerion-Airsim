#!/usr/bin/env bash
# CARLA + AirSim + PX4/MAVROS 3-drone manual-control validation runner.
#
# Flow:
#   1. Deploy PX4 3-drone AirSim settings.
#   2. Wait for UE/CARLA Play.
#   3. Optionally launch CARLA stock dynamic traffic.
#   4. Launch PX4 SITL x3, MAVROS x3, bridge x3.
#   5. Print PX4 manual-control preparation and 3-drone teleop commands.
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/workspace/projects/aerion-airsim}"
ROS_WS="${ROS_WS:-$WORKSPACE}"
PX4_DIR="${PX4_DIR:-$HOME/airsim/PX4-Autopilot}"
if [ ! -d "$PX4_DIR" ] && [ -d "$HOME/PX4-Autopilot" ]; then
    PX4_DIR="$HOME/PX4-Autopilot"
fi
SETTINGS_SRC="${SETTINGS_SRC:-$WORKSPACE/settings/px4_3drones_phase4_delta.json}"
SETTINGS_DST="${SETTINGS_DST:-$HOME/Documents/AirSim/settings.json}"
AIRSIM_IP="${AIRSIM_IP:-127.0.0.1}"
AIRSIM_PORT="${AIRSIM_PORT:-41451}"
DRONE_COUNT="${DRONE_COUNT:-3}"
LOG_DIR="${LOG_DIR:-/tmp/aerion_carla_px4_manual}"
ENABLE_CARLA_TRAFFIC="${ENABLE_CARLA_TRAFFIC:-true}"
CARLA_TRAFFIC_VEHICLES="${CARLA_TRAFFIC_VEHICLES:-12}"
CARLA_TRAFFIC_WALKERS="${CARLA_TRAFFIC_WALKERS:-8}"
CARLA_TRAFFIC_SEED="${CARLA_TRAFFIC_SEED:-42}"
SPAWN_OFFSET_X="${SPAWN_OFFSET_X:-0.0}"
SPAWN_OFFSET_Y="${SPAWN_OFFSET_Y:-0.0}"
SPAWN_OFFSET_Z="${SPAWN_OFFSET_Z:-0.0}"
DRONE1_X="${DRONE1_X:-}"
DRONE1_Y="${DRONE1_Y:-}"
DRONE1_Z="${DRONE1_Z:-}"
DRONE2_X="${DRONE2_X:-}"
DRONE2_Y="${DRONE2_Y:-}"
DRONE2_Z="${DRONE2_Z:-}"
DRONE3_X="${DRONE3_X:-}"
DRONE3_Y="${DRONE3_Y:-}"
DRONE3_Z="${DRONE3_Z:-}"
PIDS=""

log() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '\n[%s] ERROR: %s\n' "$(date +%H:%M:%S)" "$*" >&2; exit 1; }

cleanup() {
    for pid in $PIDS; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

[ "$DRONE_COUNT" = "3" ] || die "This validation runner is pinned to DRONE_COUNT=3."
[ -f "$SETTINGS_SRC" ] || die "settings not found: $SETTINGS_SRC"
[ -f "$WORKSPACE/scripts/launch_px4_instances.sh" ] || die "missing PX4 launcher"
[ -f "$WORKSPACE/scripts/launch_mavros_px4_instances.sh" ] || die "missing MAVROS launcher"
[ -f "$WORKSPACE/scripts/run_airsim_ros2_bridge_instances.sh" ] || die "missing bridge launcher"
[ -f "$WORKSPACE/scripts/run_carla_example_traffic.sh" ] || die "missing CARLA traffic launcher"

mkdir -p "$LOG_DIR" "$(dirname "$SETTINGS_DST")"

log "[1/5] Deploy AirSim settings"
cp "$SETTINGS_SRC" "$SETTINGS_DST"
SETTINGS_DST="$SETTINGS_DST" \
SPAWN_OFFSET_X="$SPAWN_OFFSET_X" SPAWN_OFFSET_Y="$SPAWN_OFFSET_Y" SPAWN_OFFSET_Z="$SPAWN_OFFSET_Z" \
DRONE1_X="$DRONE1_X" DRONE1_Y="$DRONE1_Y" DRONE1_Z="$DRONE1_Z" \
DRONE2_X="$DRONE2_X" DRONE2_Y="$DRONE2_Y" DRONE2_Z="$DRONE2_Z" \
DRONE3_X="$DRONE3_X" DRONE3_Y="$DRONE3_Y" DRONE3_Z="$DRONE3_Z" \
python3 - <<'PY'
import json
import os

path = os.path.expanduser(os.environ["SETTINGS_DST"])
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)

vehicles = data.get("Vehicles", {})
offx = float(os.environ.get("SPAWN_OFFSET_X", "0.0"))
offy = float(os.environ.get("SPAWN_OFFSET_Y", "0.0"))
offz = float(os.environ.get("SPAWN_OFFSET_Z", "0.0"))

for vehicle in vehicles.values():
    if isinstance(vehicle, dict):
        vehicle["X"] = float(vehicle.get("X", 0.0)) + offx
        vehicle["Y"] = float(vehicle.get("Y", 0.0)) + offy
        vehicle["Z"] = float(vehicle.get("Z", 0.0)) + offz

for name in ("drone1", "drone2", "drone3"):
    vehicle = vehicles.get(name)
    if not isinstance(vehicle, dict):
        continue
    prefix = name.upper()
    for axis in ("X", "Y", "Z"):
        value = os.environ.get(f"{prefix}_{axis}", "")
        if value != "":
            vehicle[axis] = float(value)

with open(path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2)
PY
echo "  source: $SETTINGS_SRC"
echo "  target: $SETTINGS_DST"
echo "  spawn offset xyz=($SPAWN_OFFSET_X, $SPAWN_OFFSET_Y, $SPAWN_OFFSET_Z)"
echo "  md5: $(md5sum "$SETTINGS_DST" | awk '{print $1}')"

log "[2/5] Start CARLA+AirSim UE Play"
echo "  Confirm drone1/drone2/drone3 are spawned, then press Enter."
read -r -p "  ready: " _ || true

if [[ "$ENABLE_CARLA_TRAFFIC" == "1" || "$ENABLE_CARLA_TRAFFIC" == "true" ]]; then
    log "[3/6] Launch CARLA stock dynamic traffic"
    CARLA_TRAFFIC_VEHICLES="$CARLA_TRAFFIC_VEHICLES" \
    CARLA_TRAFFIC_WALKERS="$CARLA_TRAFFIC_WALKERS" \
    CARLA_TRAFFIC_SEED="$CARLA_TRAFFIC_SEED" \
        "$WORKSPACE/scripts/run_carla_example_traffic.sh" >"$LOG_DIR/carla_traffic.log" 2>&1 &
    PIDS="$PIDS $!"
    sleep 4
else
    log "[3/6] Skip CARLA traffic"
fi

log "[4/6] Launch PX4 SITL x3"
PX4_DIR="$PX4_DIR" DRONE_COUNT=3 BACKGROUND_MODE=false \
    "$WORKSPACE/scripts/launch_px4_instances.sh" >"$LOG_DIR/px4.log" 2>&1 &
PIDS="$PIDS $!"
sleep 8

log "[5/6] Launch MAVROS x3"
DRONE_COUNT=3 MAVROS_NAMESPACE_STYLE=drone \
    "$WORKSPACE/scripts/launch_mavros_px4_instances.sh" >"$LOG_DIR/mavros.log" 2>&1 &
PIDS="$PIDS $!"
sleep 8

log "[6/6] Launch AirSim ROS2 bridge x3"
ROS_WS="$ROS_WS" AIRSIM_IP="$AIRSIM_IP" AIRSIM_PORT="$AIRSIM_PORT" \
DRONE_COUNT=3 MASTER_VEHICLE=drone1 ENABLE_RANGE=true \
    "$WORKSPACE/scripts/run_airsim_ros2_bridge_instances.sh" >"$LOG_DIR/bridge.log" 2>&1 &
PIDS="$PIDS $!"

cat <<EOF

Logs:
  CARLA traffic: $LOG_DIR/carla_traffic.log
  PX4:    $LOG_DIR/px4.log
  MAVROS: $LOG_DIR/mavros.log
  bridge: $LOG_DIR/bridge.log

Prepare PX4 manual control:
  source /opt/ros/humble/setup.bash
  source "$ROS_WS/install/setup.bash"
  DRONE_COUNT=3 bash "$WORKSPACE/scripts/prepare_px4_manual_control.sh"

Manual control, all 3 drones:
  source /opt/ros/humble/setup.bash
  source "$ROS_WS/install/setup.bash"
  VEHICLES=drone1,drone2,drone3 ARM_FLAG=false FORCE_ARM_FLAG=false SPEED=3.0 VERTICAL_SPEED=1.5 YAW_RATE=1.2 HOLD_TIMEOUT=0.35 \
    bash "$WORKSPACE/scripts/run_manual_3drone_control.sh"

Expected sensor topics:
  /drone1/camera/image
  /drone{1,2,3}/range/{front,left,right}
  /drone{1,2,3}/mavros/state
  /drone{1,2,3}/mavros/setpoint_velocity/cmd_vel_unstamped

Press Ctrl+C here to stop PX4/MAVROS/bridge.
EOF

wait
