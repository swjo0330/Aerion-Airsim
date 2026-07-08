#!/usr/bin/env bash
# One-command workflow for GPS mission editor/export/upload/flight validation.
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
MISSION_FILE="${MISSION_FILE:-$WORKSPACE/recordings/missions/clicked_mission_gps.json}"
RUNNER="$WORKSPACE/scripts/run_gps_route_mission_experiment.sh"

usage() {
  cat <<EOF
Usage:
  bash scripts/aerion_mission_workflow.sh <command>

Commands:
  editor      Open the GPS mission editor only.
  dry-run     Validate the current GPS mission upload command without ROS/MAVROS.
  full        Guided full stack: deploy settings, UE, PX4, MAVROS, upload only.
  start       Guided full stack and intentionally arm + start AUTO.MISSION.
  upload      Upload to already-running MAVROS only; does not arm/start.
  upload-run  Upload to already-running MAVROS and arm + start AUTO.MISSION.

Default mission:
  $MISSION_FILE

Useful overrides:
  MISSION_FILE=/path/to/mission.json
  DDS_MODE=fastrtps|cyclone|existing
  MAVROS_NAMESPACE=drone1/mavros
  START_MISSION=true
  MISSION_ARM=true
EOF
}

need_runner() {
  [ -x "$RUNNER" ] || {
    echo "ERROR: runner is not executable: $RUNNER" >&2
    echo "Run: chmod +x $RUNNER" >&2
    exit 2
  }
}

command="${1:-}"
case "$command" in
  editor)
    need_runner
    MISSION_FILE="$MISSION_FILE" \
    RUNNER_MODE=editor \
    OPEN_EDITOR=true \
    SKIP_START_CONFIRM=true \
      "$RUNNER"
    ;;
  dry-run)
    need_runner
    MISSION_FILE="$MISSION_FILE" \
    RUNNER_MODE=dry-run \
    SKIP_START_CONFIRM=true \
      "$RUNNER"
    ;;
  full)
    need_runner
    MISSION_FILE="$MISSION_FILE" \
    START_MISSION=false \
    MISSION_ARM=false \
      "$RUNNER"
    ;;
  start)
    need_runner
    MISSION_FILE="$MISSION_FILE" \
    START_MISSION=true \
    MISSION_ARM=true \
      "$RUNNER"
    ;;
  upload)
    MISSION_FILE="$MISSION_FILE" \
    RUNNER_MODE=upload-only \
    START_MISSION=false \
    MISSION_ARM=false \
      "$RUNNER"
    ;;
  upload-run)
    MISSION_FILE="$MISSION_FILE" \
    RUNNER_MODE=upload-only \
    START_MISSION=true \
    MISSION_ARM=true \
      "$RUNNER"
    ;;
  ""|-h|--help|help)
    usage
    ;;
  *)
    echo "ERROR: unknown command: $command" >&2
    echo >&2
    usage >&2
    exit 2
    ;;
esac
