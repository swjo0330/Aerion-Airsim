#!/usr/bin/env bash
# GPS-first integrated runner for CARLA/AirSim + PX4 + MAVROS mission checks.
#
# Intended flow:
#   mission editor exports GPS JSON
#   -> this runner deploys settings, starts UE/PX4/MAVROS
#   -> dry-runs upload args
#   -> optionally uploads, arms, and starts AUTO.MISSION.
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROS_WS="${ROS_WS:-$WORKSPACE}"
PX4_DIR="${PX4_DIR:-$HOME/airsim/PX4-Autopilot}"
UE_BIN="${UE_BIN:-$HOME/workspace/engines/UE5.5_carla/UnrealEngine/Engine/Binaries/Linux/UnrealEditor}"
UE_PROJECT="${UE_PROJECT:-$HOME/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject}"
UE_QUALITY_LEVEL="${UE_QUALITY_LEVEL:-High}"
UE_RENDER_PROFILE="${UE_RENDER_PROFILE:-town10_stable}"
MISSION_FILE="${MISSION_FILE:-${ROUTE_FILE:-$WORKSPACE/recordings/missions/clicked_mission_gps.json}}"
LEGACY_PREPARED_MISSION="${LEGACY_PREPARED_MISSION:-$WORKSPACE/recordings/missions/clicked_mission_current_home_fcu_axes.json}"
MISSION_EDITOR_HTML="${MISSION_EDITOR_HTML:-$WORKSPACE/recordings/maps/mission_editor.html}"
RUNNER_MODE="${RUNNER_MODE:-full}"       # full | upload-only | dry-run | editor
BUILD_BRIDGE="${BUILD_BRIDGE:-true}"
REBUILD_EDITOR="${REBUILD_EDITOR:-false}"
OPEN_EDITOR="${OPEN_EDITOR:-false}"
WAIT_FOR_EDITOR_EXPORT="${WAIT_FOR_EDITOR_EXPORT:-false}"
DEPLOY_SETTINGS="${DEPLOY_SETTINGS:-true}"
START_UE="${START_UE:-true}"
START_PX4="${START_PX4:-true}"
START_MAVROS="${START_MAVROS:-true}"
START_BRIDGE="${START_BRIDGE:-false}"
UPLOAD_MISSION="${UPLOAD_MISSION:-true}"
UPLOAD_DRY_RUN_FIRST="${UPLOAD_DRY_RUN_FIRST:-true}"
START_MISSION="${START_MISSION:-false}"
MISSION_ARM="${MISSION_ARM:-false}"
FORCE_ARM="${FORCE_ARM:-true}"
PREPARE_PX4_PARAMS="${PREPARE_PX4_PARAMS:-false}"
PREARM_OFFBOARD="${PREARM_OFFBOARD:-false}"
STOP_ON_EXIT="${STOP_ON_EXIT:-prompt}"   # prompt | true | false
CLEAN_START_STOP_UE="${CLEAN_START_STOP_UE:-true}"
SKIP_START_CONFIRM="${SKIP_START_CONFIRM:-false}"
CONFIRM_UPLOAD="${CONFIRM_UPLOAD:-true}"
ALLOW_NONINTERACTIVE_PAUSE="${ALLOW_NONINTERACTIVE_PAUSE:-false}"
CHECK_AIRSIM_SPAWN="${CHECK_AIRSIM_SPAWN:-true}"
AIRSIM_VEHICLE="${AIRSIM_VEHICLE:-drone1}"
AIRSIM_IP="${AIRSIM_IP:-127.0.0.1}"
AIRSIM_PORT="${AIRSIM_PORT:-41451}"
AIRSIM_TIMEOUT="${AIRSIM_TIMEOUT:-5}"
AIRSIM_SPAWN_MAX_ABS_Z="${AIRSIM_SPAWN_MAX_ABS_Z:-50}"
MAVROS_NAMESPACE="${MAVROS_NAMESPACE:-drone1/mavros}"
FIRMWARE="${FIRMWARE:-px4}"
ALTITUDE_MODE="${ALTITUDE_MODE:-}"
TAKEOFF_ALTITUDE="${TAKEOFF_ALTITUDE:-5.0}"
TAKEOFF_ALTITUDE_SOURCE="${TAKEOFF_ALTITUDE_SOURCE:-first_waypoint}"
TAKEOFF_POSITION_SOURCE="${TAKEOFF_POSITION_SOURCE:-current}"
ACCEPTANCE_RADIUS="${ACCEPTANCE_RADIUS:-2.5}"
FCU_BIND_BASE_PORT="${FCU_BIND_BASE_PORT:-14550}"
PX4_REMOTE_BASE_PORT="${PX4_REMOTE_BASE_PORT:-18570}"
DRONE_COUNT="${DRONE_COUNT:-1}"
PX4_INSTANCE_IDS="${PX4_INSTANCE_IDS:-0}"
DRONE1_X="${DRONE1_X:-}"
DRONE1_Y="${DRONE1_Y:-}"
DRONE1_Z="${DRONE1_Z:-}"
SPAWN_OFFSET_X="${SPAWN_OFFSET_X:-0.0}"
SPAWN_OFFSET_Y="${SPAWN_OFFSET_Y:-0.0}"
SPAWN_OFFSET_Z="${SPAWN_OFFSET_Z:-0.0}"
LOG_DIR="${LOG_DIR:-/tmp/aerion_gps_mission_runner}"
DDS_MODE="${DDS_MODE:-fastrtps}"         # fastrtps | cyclone | existing
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

PIDS=""

log() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '\n[%s] ERROR: %s\n' "$(date +%H:%M:%S)" "$*" >&2; exit 1; }

pause() {
    local message="${1:-Press Enter to continue}"
    if [ "$ALLOW_NONINTERACTIVE_PAUSE" = "true" ]; then
        echo "$message"
        return
    fi
    if ! read -r -p "$message " _; then
        die "interactive confirmation is required; rerun from a terminal or set ALLOW_NONINTERACTIVE_PAUSE=true"
    fi
}

configure_ros_env() {
    export ROS_DOMAIN_ID
    case "$DDS_MODE" in
        fastrtps)
            unset CYCLONEDDS_URI
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            ;;
        cyclone)
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            ;;
        existing)
            ;;
        *)
            die "unsupported DDS_MODE=$DDS_MODE (use fastrtps, cyclone, existing)"
            ;;
    esac
}

source_ros() {
    configure_ros_env
    set +u
    source /opt/ros/humble/setup.bash
    [ -f "$ROS_WS/install/setup.bash" ] && source "$ROS_WS/install/setup.bash"
    set -u
}

start_bg() {
    local name="$1"
    shift
    mkdir -p "$LOG_DIR"
    nohup "$@" >"$LOG_DIR/$name.log" 2>&1 &
    local pid=$!
    echo "$pid" >"$LOG_DIR/$name.pid"
    PIDS="$PIDS $pid"
    echo "  $name PID=$pid log=$LOG_DIR/$name.log"
}

cleanup() {
    log "cleanup"
    for pid in $PIDS; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup INT TERM

mission_frame() {
    python3 - "$MISSION_FILE" <<'PY'
import json
import sys
from pathlib import Path

data = json.loads(Path(sys.argv[1]).expanduser().read_text(encoding="utf-8"))
frame = str(data.get("coordinate_frame", data.get("frame", "global"))).lower()
print("gps" if frame in ("global", "gps", "wgs84") else "local")
PY
}

mission_count() {
    python3 - "$MISSION_FILE" <<'PY'
import json
import sys
from pathlib import Path

data = json.loads(Path(sys.argv[1]).expanduser().read_text(encoding="utf-8"))
print(len(data.get("waypoints", [])))
PY
}

check_files() {
    [ -d "$WORKSPACE" ] || die "workspace not found: $WORKSPACE"
    [ -f "$MISSION_FILE" ] || die "mission file not found: $MISSION_FILE"
    [ -f "$WORKSPACE/scripts/upload_mission_json_to_mavros.sh" ] || die "missing upload script"
    [ -f "$WORKSPACE/scripts/deploy_px4_1drone_lidar_settings.sh" ] || die "missing settings deploy script"
    [ -f "$WORKSPACE/scripts/launch_px4_instances.sh" ] || die "missing PX4 launcher"
    [ -f "$WORKSPACE/scripts/launch_mavros_px4_instances.sh" ] || die "missing MAVROS launcher"
    if [ "$START_UE" = "true" ]; then
        [ -x "$UE_BIN" ] || die "UE binary not executable: $UE_BIN"
        [ -f "$UE_PROJECT" ] || die "UE project not found: $UE_PROJECT"
    fi
    if [ "$START_PX4" = "true" ]; then
        [ -d "$PX4_DIR" ] || die "PX4_DIR not found: $PX4_DIR"
    fi
}

show_config() {
    local frame
    frame="$(mission_frame)"
    cat <<EOF

Integrated mission runner
  mode:             $RUNNER_MODE
  workspace:        $WORKSPACE
  mission:          $MISSION_FILE
  mission_frame:    $frame
  mission_count:    $(mission_count)
  editor_html:      $MISSION_EDITOR_HTML
  dds:              $DDS_MODE domain=$ROS_DOMAIN_ID
  ue:               start=$START_UE profile=$UE_RENDER_PROFILE quality=$UE_QUALITY_LEVEL
  px4:              start=$START_PX4 dir=$PX4_DIR
  mavros:           start=$START_MAVROS ns=/$MAVROS_NAMESPACE bind=$FCU_BIND_BASE_PORT remote=$PX4_REMOTE_BASE_PORT
  upload:           enabled=$UPLOAD_MISSION dry_run_first=$UPLOAD_DRY_RUN_FIRST start=$START_MISSION arm=$MISSION_ARM
  settings:         deploy=$DEPLOY_SETTINGS spawn=($DRONE1_X,$DRONE1_Y,$DRONE1_Z) offset=($SPAWN_OFFSET_X,$SPAWN_OFFSET_Y,$SPAWN_OFFSET_Z)
  logs:             $LOG_DIR
EOF
}

stop_existing() {
    log "Stop old experiment processes"
    local patterns=(
        'aerion_gps_route_mission'
        'bridge_node'
        'run_airsim_ros2_bridge_instances.sh'
        'ros2 launch mavros'
        'mavros_node'
        'mavproxy.py'
        'px4_sitl_default/bin/px4'
        'px4 -i '
    )
    if [ "$CLEAN_START_STOP_UE" = "true" ]; then
        patterns+=('UnrealEditor .*CarlaUnreal\.uproject')
        patterns+=('UnrealEditor .*BlocksV2\.uproject')
    fi

    terminate_pattern() {
        local signal="$1"
        local pattern="$2"
        local pid
        while read -r pid; do
            [ -n "$pid" ] || continue
            [ "$pid" = "$$" ] && continue
            [ "$pid" = "${BASHPID:-}" ] && continue
            kill "-$signal" "$pid" 2>/dev/null || true
        done < <(pgrep -f "$pattern" 2>/dev/null || true)
    }

    for pattern in "${patterns[@]}"; do terminate_pattern TERM "$pattern"; done
    sleep 2
    for pattern in "${patterns[@]}"; do terminate_pattern KILL "$pattern"; done
    rm -f "$LOG_DIR"/*.pid 2>/dev/null || true

    echo "  remaining relevant processes:"
    pgrep -af 'UnrealEditor|CarlaUnreal|px4_sitl_default/bin/px4|mavros_node|mavproxy.py|bridge_node|aerion_gps_route_mission' || echo "  none"
    echo "  relevant UDP listeners:"
    ss -lunp 2>/dev/null | grep -E ':(14540|14550|14580|14601|18570)\b' || echo "  none"
}

build_editor() {
    if [ "$REBUILD_EDITOR" != "true" ]; then
        return
    fi
    log "Rebuild mission editor"
    cd "$WORKSPACE"
    SCAN_HEIGHTMAP="${SCAN_HEIGHTMAP:-false}" \
    QUERY_HOME_ORIGIN="${QUERY_HOME_ORIGIN:-false}" \
        bash scripts/build_mission_planning_assets.sh
}

open_editor() {
    if [ "$OPEN_EDITOR" != "true" ]; then
        return
    fi
    log "Open mission editor"
    [ -f "$MISSION_EDITOR_HTML" ] || die "mission editor HTML not found: $MISSION_EDITOR_HTML"
    if command -v xdg-open >/dev/null 2>&1; then
        xdg-open "$MISSION_EDITOR_HTML" >/dev/null 2>&1 &
    else
        echo "Open manually: $MISSION_EDITOR_HTML"
    fi
    if [ "$WAIT_FOR_EDITOR_EXPORT" = "true" ]; then
        pause "Export GPS Mission JSON from the editor, set MISSION_FILE if needed, then press Enter:"
        [ -f "$MISSION_FILE" ] || die "mission file not found after editor export: $MISSION_FILE"
    fi
}

build_bridge() {
    if [ "$BUILD_BRIDGE" != "true" ]; then
        return
    fi
    log "Build airsim_ros2_bridge"
    cd "$WORKSPACE"
    source_ros
    colcon build --packages-select airsim_ros2_bridge
    source_ros
}

deploy_settings() {
    if [ "$DEPLOY_SETTINGS" != "true" ]; then
        return
    fi
    log "Deploy AirSim PX4 one-drone settings"
    cd "$WORKSPACE"
    DRONE1_X="$DRONE1_X" \
    DRONE1_Y="$DRONE1_Y" \
    DRONE1_Z="$DRONE1_Z" \
    SPAWN_OFFSET_X="$SPAWN_OFFSET_X" \
    SPAWN_OFFSET_Y="$SPAWN_OFFSET_Y" \
    SPAWN_OFFSET_Z="$SPAWN_OFFSET_Z" \
        bash scripts/deploy_px4_1drone_lidar_settings.sh
    echo "  If UE is already in Play, press Stop -> Play after this step."
}

launch_ue() {
    if [ "$START_UE" != "true" ]; then
        return
    fi
    log "Launch CARLA + AirSim UE project"
    cd "$WORKSPACE"
    AERION_AUTOPILOT_ENABLED=0 \
    UE_BIN="$UE_BIN" \
    UE_PROJECT="$UE_PROJECT" \
    UE_QUALITY_LEVEL="$UE_QUALITY_LEVEL" \
    UE_RENDER_PROFILE="$UE_RENDER_PROFILE" \
        bash scripts/run_ue_blocksv2.sh
    sleep 3
    if ! pgrep -af 'UnrealEditor' | grep -F "$UE_PROJECT" >/dev/null; then
        local latest_ue_log
        latest_ue_log="$(ls -t "$HOME"/workspace/logs/ue/ue_*.log 2>/dev/null | head -1 || true)"
        [ -n "$latest_ue_log" ] && tail -n 120 "$latest_ue_log" >&2
        die "UE launch failed or exited early: $UE_PROJECT"
    fi
    pause "Open the CARLA map, press UE Play, wait for drone spawn, then press Enter:"
}

check_airsim_spawn() {
    if [ "$CHECK_AIRSIM_SPAWN" != "true" ] || [ "$START_UE" != "true" ]; then
        return
    fi
    log "Check AirSim RPC and spawn sanity"
    python3 - "$AIRSIM_IP" "$AIRSIM_PORT" "$AIRSIM_TIMEOUT" "$AIRSIM_VEHICLE" "$AIRSIM_SPAWN_MAX_ABS_Z" <<'PY'
import contextlib
import math
import sys
import time

import airsim

ip = sys.argv[1]
port = int(sys.argv[2])
timeout = float(sys.argv[3])
vehicle = sys.argv[4]
max_abs_z = float(sys.argv[5])
deadline = time.monotonic() + max(10.0, timeout)
last_error = None

while time.monotonic() < deadline:
    try:
        client = airsim.MultirotorClient(ip=ip, port=port, timeout_value=timeout)
        with contextlib.redirect_stdout(sys.stderr):
            client.confirmConnection()
        pose = client.simGetVehiclePose(vehicle_name=vehicle)
        pos = pose.position
        print(f"  AirSim pose {vehicle}: x={pos.x_val:.3f} y={pos.y_val:.3f} z={pos.z_val:.3f}")
        if not all(math.isfinite(v) for v in (pos.x_val, pos.y_val, pos.z_val)):
            raise RuntimeError("non-finite AirSim pose")
        if abs(pos.z_val) > max_abs_z:
            raise RuntimeError(f"abs(z)={abs(pos.z_val):.3f} > {max_abs_z:.3f}; spawn likely failed")
        raise SystemExit(0)
    except SystemExit:
        raise
    except Exception as exc:
        last_error = exc
        time.sleep(1.0)

print(f"AirSim RPC unavailable at {ip}:{port}: {last_error}", file=sys.stderr)
raise SystemExit(2)
PY
}

launch_px4() {
    if [ "$START_PX4" != "true" ]; then
        return
    fi
    log "Launch PX4 SITL"
    cd "$WORKSPACE"
    PX4_DIR="$PX4_DIR" \
    DRONE_COUNT="$DRONE_COUNT" \
    PX4_INSTANCE_IDS="$PX4_INSTANCE_IDS" \
    BACKGROUND_MODE=true \
        bash scripts/launch_px4_instances.sh
    echo "  waiting 5s for PX4/AirSim lockstep..."
    sleep 5
    tail -n 30 /tmp/px4_sitl_0.log 2>/dev/null || true
    if grep -q 'Startup script returned with return value' /tmp/px4_sitl_0.log 2>/dev/null; then
        tail -n 100 /tmp/px4_sitl_0.log >&2 || true
        die "PX4 startup failed. Confirm UE is running and Play was pressed."
    fi
    pause "If PX4 shows simulator connected, press Enter:"
}

launch_mavros() {
    if [ "$START_MAVROS" != "true" ]; then
        return
    fi
    log "Launch MAVROS"
    cd "$WORKSPACE"
    source_ros
    start_bg mavros env \
        ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-}" \
        CYCLONEDDS_URI="${CYCLONEDDS_URI:-}" \
        DRONE_COUNT="$DRONE_COUNT" \
        PX4_INSTANCE_IDS="$PX4_INSTANCE_IDS" \
        MAVROS_NAMESPACE_STYLE=drone \
        FCU_BIND_BASE_PORT="$FCU_BIND_BASE_PORT" \
        PX4_REMOTE_BASE_PORT="$PX4_REMOTE_BASE_PORT" \
        bash "$WORKSPACE/scripts/launch_mavros_px4_instances.sh"
    echo "  waiting 8s for MAVROS heartbeat..."
    sleep 8
    tail -n 60 "$LOG_DIR/mavros.log" 2>/dev/null || true
    source_ros
    timeout 6 ros2 topic echo --once "/${MAVROS_NAMESPACE#/}/state" || true
    pause "If connected:true is visible, press Enter:"
}

prepare_px4_params() {
    if [ "$PREPARE_PX4_PARAMS" != "true" ]; then
        return
    fi
    log "Apply PX4 validation params"
    cd "$WORKSPACE"
    source_ros
    DRONE_COUNT=1 ARM_AFTER_PREP="$PREARM_OFFBOARD" bash scripts/prepare_px4_manual_control.sh || true
}

launch_bridge_optional() {
    if [ "$START_BRIDGE" != "true" ]; then
        return
    fi
    log "Launch AirSim ROS2 bridge"
    cd "$WORKSPACE"
    source_ros
    start_bg bridge env \
        ROS_WS="$ROS_WS" \
        AIRSIM_IP="$AIRSIM_IP" \
        AIRSIM_PORT="$AIRSIM_PORT" \
        DRONE_COUNT=1 \
        VEHICLES=drone1 \
        MASTER_VEHICLE=drone1 \
        ENABLE_RANGE=true \
        ENABLE_LIDAR=true \
        ENABLE_CAMERA=true \
        bash "$WORKSPACE/scripts/run_airsim_ros2_bridge_instances.sh"
    sleep 5
    tail -n 40 "$LOG_DIR/bridge.log" 2>/dev/null || true
}

upload_mission() {
    if [ "$UPLOAD_MISSION" != "true" ]; then
        return
    fi
    log "Mission upload"
    cd "$WORKSPACE"

    if [ "$UPLOAD_DRY_RUN_FIRST" = "true" ]; then
        DRY_RUN=true \
        MISSION_FILE="$MISSION_FILE" \
        MAVROS_NAMESPACE="$MAVROS_NAMESPACE" \
        FIRMWARE="$FIRMWARE" \
        ALTITUDE_MODE="$ALTITUDE_MODE" \
        TAKEOFF_ALTITUDE="$TAKEOFF_ALTITUDE" \
        TAKEOFF_ALTITUDE_SOURCE="$TAKEOFF_ALTITUDE_SOURCE" \
        TAKEOFF_POSITION_SOURCE="$TAKEOFF_POSITION_SOURCE" \
        ACCEPTANCE_RADIUS="$ACCEPTANCE_RADIUS" \
        MISSION_ARM="$MISSION_ARM" \
        FORCE_ARM="$FORCE_ARM" \
        START_MISSION="$START_MISSION" \
            bash scripts/upload_mission_json_to_mavros.sh
    fi

    if [ "$RUNNER_MODE" = "dry-run" ]; then
        return
    fi
    if [ "$CONFIRM_UPLOAD" = "true" ]; then
        pause "Press Enter to execute mission upload with START_MISSION=$START_MISSION MISSION_ARM=$MISSION_ARM:"
    fi

    configure_ros_env
    MISSION_FILE="$MISSION_FILE" \
    MAVROS_NAMESPACE="$MAVROS_NAMESPACE" \
    FIRMWARE="$FIRMWARE" \
    ALTITUDE_MODE="$ALTITUDE_MODE" \
    TAKEOFF_ALTITUDE="$TAKEOFF_ALTITUDE" \
    TAKEOFF_ALTITUDE_SOURCE="$TAKEOFF_ALTITUDE_SOURCE" \
    TAKEOFF_POSITION_SOURCE="$TAKEOFF_POSITION_SOURCE" \
    ACCEPTANCE_RADIUS="$ACCEPTANCE_RADIUS" \
    MISSION_ARM="$MISSION_ARM" \
    FORCE_ARM="$FORCE_ARM" \
    START_MISSION="$START_MISSION" \
        bash scripts/upload_mission_json_to_mavros.sh
}

status_tail() {
    log "Final status"
    source_ros
    timeout 5 ros2 topic echo --once "/${MAVROS_NAMESPACE#/}/state" || true
    echo ""
    echo "Logs:"
    echo "  PX4:    /tmp/px4_sitl_0.log"
    echo "  MAVROS: $LOG_DIR/mavros.log"
    echo "  Bridge: $LOG_DIR/bridge.log"
    echo ""
    echo "Useful commands:"
    echo "  DRY_RUN=true bash scripts/upload_mission_json_to_mavros.sh"
    echo "  env -u CYCLONEDDS_URI RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic echo --once /${MAVROS_NAMESPACE#/}/state"
    echo "  tail -f /tmp/px4_sitl_0.log"
    echo "  tail -f $LOG_DIR/mavros.log"
}

finish() {
    case "$STOP_ON_EXIT" in
        true)
            cleanup
            ;;
        false)
            echo "Leaving background processes running."
            ;;
        prompt)
            pause "Experiment finished. Press Enter to stop MAVROS/bridge started by this runner, or Ctrl+C to leave them running:"
            cleanup
            ;;
        *)
            die "unsupported STOP_ON_EXIT=$STOP_ON_EXIT"
            ;;
    esac
}

main() {
    mkdir -p "$LOG_DIR"
    configure_ros_env

    case "$RUNNER_MODE" in
        full)
            ;;
        upload-only)
            START_UE=false
            START_PX4=false
            START_MAVROS=false
            DEPLOY_SETTINGS=false
            BUILD_BRIDGE=false
            ;;
        dry-run)
            START_UE=false
            START_PX4=false
            START_MAVROS=false
            DEPLOY_SETTINGS=false
            BUILD_BRIDGE=false
            CONFIRM_UPLOAD=false
            STOP_ON_EXIT=false
            ;;
        editor)
            START_UE=false
            START_PX4=false
            START_MAVROS=false
            DEPLOY_SETTINGS=false
            BUILD_BRIDGE=false
            UPLOAD_MISSION=false
            OPEN_EDITOR=true
            ;;
        *)
            die "unsupported RUNNER_MODE=$RUNNER_MODE (use full, upload-only, dry-run, editor)"
            ;;
    esac

    check_files
    show_config
    if [ "$SKIP_START_CONFIRM" != "true" ]; then
        pause "Press Enter to start runner mode=$RUNNER_MODE:"
    fi

    build_editor
    open_editor
    [ "$RUNNER_MODE" = "editor" ] && return 0

    if [ "$RUNNER_MODE" = "full" ]; then
        stop_existing
    fi
    build_bridge
    deploy_settings
    launch_ue
    check_airsim_spawn
    launch_px4
    launch_mavros
    prepare_px4_params
    launch_bridge_optional
    upload_mission
    if [ "$RUNNER_MODE" = "dry-run" ]; then
        return 0
    fi
    status_tail
    finish
}

main "$@"
