#!/usr/bin/env bash
# Guided full-stack runner for the mission exported from mission_editor.html.
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DEFAULT_SOURCE_MISSION="$WORKSPACE/recordings/missions/clicked_mission_gps.json"
if [ ! -f "$DEFAULT_SOURCE_MISSION" ]; then
  DEFAULT_SOURCE_MISSION="$WORKSPACE/recordings/missions/clicked_mission.json"
fi
SOURCE_MISSION="${SOURCE_MISSION:-$DEFAULT_SOURCE_MISSION}"
PREPARED_MISSION="${PREPARED_MISSION:-$WORKSPACE/recordings/missions/clicked_mission_current_home_fcu_axes.json}"
SPAWN_MODE="${SPAWN_MODE:-previous_home}"
SPAWN_AT_MISSION_FIRST="${SPAWN_AT_MISSION_FIRST:-false}"
AIRSIM_NED_ORIGIN="${AIRSIM_NED_ORIGIN:-$WORKSPACE/recordings/maps/airsim_ned_origin.json}"
# This is the known-good AirSim global-NED spawn offset from the successful
# Town10HD_Opt mission run. It is not a map/OpenDRIVE coordinate.
PREVIOUS_HOME_AIRSIM_X="${PREVIOUS_HOME_AIRSIM_X:-${HOME_MAP_X:-240}}"
PREVIOUS_HOME_AIRSIM_Y="${PREVIOUS_HOME_AIRSIM_Y:-${HOME_MAP_Y:--170}}"
PREVIOUS_HOME_AIRSIM_Z="${PREVIOUS_HOME_AIRSIM_Z:-0.2}"

if [ "$SPAWN_AT_MISSION_FIRST" = "true" ]; then
  SPAWN_MODE="mission_first"
fi

case "$SPAWN_MODE" in
  previous_home)
    if [ ! -f "$AIRSIM_NED_ORIGIN" ]; then
      cat >&2 <<EOF
Missing AirSim NED origin calibration:
  $AIRSIM_NED_ORIGIN

The previous-home spawn is an AirSim NED offset, not a map coordinate.
Run calibration once, or extract it from a UE log that contains AERION_NED_ORIGIN_UE:
  cd $WORKSPACE
  bash scripts/run_clicked_mission_calibration.sh

Then rerun this mission runner.
EOF
      exit 2
    fi
    export DRONE1_X="$PREVIOUS_HOME_AIRSIM_X"
    export DRONE1_Y="$PREVIOUS_HOME_AIRSIM_Y"
    export DRONE1_Z="$PREVIOUS_HOME_AIRSIM_Z"
    export SPAWN_OFFSET_X="${SPAWN_OFFSET_X:-0.0}"
    export SPAWN_OFFSET_Y="${SPAWN_OFFSET_Y:-0.0}"
    export SPAWN_OFFSET_Z="${SPAWN_OFFSET_Z:-0.0}"
    eval "$(
      python3 "$WORKSPACE/scripts/compute_map_home_for_airsim_spawn.py" \
        --origin "$AIRSIM_NED_ORIGIN" \
        --airsim-x "$DRONE1_X" \
        --airsim-y "$DRONE1_Y" \
        --airsim-z "$DRONE1_Z" \
        --print-shell
    )"
    export MISSION_HOME_MODE="${MISSION_HOME_MODE:-explicit}"
    export MISSION_HOME_EAST MISSION_HOME_NORTH
    echo "Spawning drone1 at previous known home -> AirSim X=$DRONE1_X Y=$DRONE1_Y Z=$DRONE1_Z"
    echo "Legacy local mission home reference -> E=$MISSION_HOME_EAST N=$MISSION_HOME_NORTH"
    ;;
  mission_first)
    if [ ! -f "$AIRSIM_NED_ORIGIN" ]; then
      cat >&2 <<EOF
Missing AirSim NED origin calibration:
  $AIRSIM_NED_ORIGIN

Run calibration once after rebuilding/restarting UE:
  cd $WORKSPACE
  bash scripts/run_clicked_mission_calibration.sh

Then rerun this mission runner.
EOF
      exit 2
    fi
    eval "$(
      python3 "$WORKSPACE/scripts/compute_airsim_spawn_for_mission_node.py" \
        --mission "$SOURCE_MISSION" \
        --origin "$AIRSIM_NED_ORIGIN" \
        --node-index 1 \
        --z 0.2 \
        --print-shell
    )"
    export DRONE1_X DRONE1_Y DRONE1_Z
    export SPAWN_OFFSET_X="${SPAWN_OFFSET_X:-0.0}"
    export SPAWN_OFFSET_Y="${SPAWN_OFFSET_Y:-0.0}"
    export SPAWN_OFFSET_Z="${SPAWN_OFFSET_Z:-0.0}"
    export MISSION_HOME_MODE="${MISSION_HOME_MODE:-first_waypoint}"
    echo "Spawning drone1 at mission node #1 using $AIRSIM_NED_ORIGIN -> AirSim X=$DRONE1_X Y=$DRONE1_Y Z=$DRONE1_Z"
    ;;
  manual)
    : "${DRONE1_X:?SPAWN_MODE=manual requires DRONE1_X}"
    : "${DRONE1_Y:?SPAWN_MODE=manual requires DRONE1_Y}"
    export DRONE1_X DRONE1_Y
    export DRONE1_Z="${DRONE1_Z:-0.2}"
    export SPAWN_OFFSET_X="${SPAWN_OFFSET_X:-0.0}"
    export SPAWN_OFFSET_Y="${SPAWN_OFFSET_Y:-0.0}"
    export SPAWN_OFFSET_Z="${SPAWN_OFFSET_Z:-0.0}"
    echo "Spawning drone1 at manual AirSim coordinates -> X=$DRONE1_X Y=$DRONE1_Y Z=$DRONE1_Z"
    ;;
  none)
    echo "Using settings/default spawn without clicked-runner override"
    ;;
  *)
    echo "ERROR: unsupported SPAWN_MODE=$SPAWN_MODE (use previous_home, mission_first, manual, none)" >&2
    exit 2
    ;;
esac

export WORKSPACE
export MISSION_FILE="${MISSION_FILE:-$SOURCE_MISSION}"
export SOURCE_ROUTE_FILE="${SOURCE_ROUTE_FILE:-$SOURCE_MISSION}"
export PREPARED_ROUTE_FILE="${PREPARED_ROUTE_FILE:-$PREPARED_MISSION}"
export ROUTE_FILE="${ROUTE_FILE:-$SOURCE_MISSION}"
export PREPARE_ROUTE_FROM_CURRENT_AIRSIM_HOME="${PREPARE_ROUTE_FROM_CURRENT_AIRSIM_HOME:-auto}"
export MISSION_HOME_MODE="${MISSION_HOME_MODE:-first_waypoint}"
export ROUTE_EAST_FROM="${ROUTE_EAST_FROM:-north}"
export ROUTE_NORTH_FROM="${ROUTE_NORTH_FROM:-east}"
export ALTITUDE_MODE="${ALTITUDE_MODE:-}"
export TAKEOFF_ALTITUDE_SOURCE="${TAKEOFF_ALTITUDE_SOURCE:-first_waypoint}"
export TAKEOFF_POSITION_SOURCE="${TAKEOFF_POSITION_SOURCE:-current}"
export TAKEOFF_ALTITUDE="${TAKEOFF_ALTITUDE:-5.0}"
export ACCEPTANCE_RADIUS="${ACCEPTANCE_RADIUS:-2.5}"
export PREPARE_PX4_PARAMS="${PREPARE_PX4_PARAMS:-true}"
export PREARM_OFFBOARD="${PREARM_OFFBOARD:-false}"
export MISSION_ARM="${MISSION_ARM:-false}"
export FORCE_ARM="${FORCE_ARM:-true}"
export START_MISSION="${START_MISSION:-false}"
export START_BRIDGE="${START_BRIDGE:-false}"
export CLEAN_START_STOP_UE="${CLEAN_START_STOP_UE:-true}"
export SKIP_START_CONFIRM="${SKIP_START_CONFIRM:-true}"
export UE_RENDER_PROFILE="${UE_RENDER_PROFILE:-town10_stable}"
export UE_QUALITY_LEVEL="${UE_QUALITY_LEVEL:-High}"

exec "$WORKSPACE/scripts/run_gps_route_mission_experiment.sh"
