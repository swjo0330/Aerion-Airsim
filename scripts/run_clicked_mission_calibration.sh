#!/usr/bin/env bash
# One-time calibration for AirSim global NED origin in Town10HD_Opt.
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
UE_BIN="${UE_BIN:-$HOME/workspace/engines/UE5.5_carla/UnrealEngine/Engine/Binaries/Linux/UnrealEditor}"
UE_PROJECT="${UE_PROJECT:-$HOME/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject}"
UE_RENDER_PROFILE="${UE_RENDER_PROFILE:-town10_stable}"
UE_QUALITY_LEVEL="${UE_QUALITY_LEVEL:-High}"
OUTPUT="${AIRSIM_NED_ORIGIN:-$WORKSPACE/recordings/maps/airsim_ned_origin.json}"
LOG_DIR="${LOG_DIR:-$HOME/workspace/logs/ue}"
CLEAN_START_STOP_UE="${CLEAN_START_STOP_UE:-true}"

die() { echo "ERROR: $*" >&2; exit 1; }

cd "$WORKSPACE"

if [ "$CLEAN_START_STOP_UE" = "true" ]; then
  pkill -TERM -f 'UnrealEditor .*CarlaUnreal\.uproject' 2>/dev/null || true
  sleep 2
  pkill -KILL -f 'UnrealEditor .*CarlaUnreal\.uproject' 2>/dev/null || true
fi

echo "[calibration] deploy default AirSim settings at NED origin"
DRONE1_X=0.0 DRONE1_Y=0.0 DRONE1_Z=0.2 \
SPAWN_OFFSET_X=0.0 SPAWN_OFFSET_Y=0.0 SPAWN_OFFSET_Z=0.0 \
  bash scripts/deploy_px4_1drone_lidar_settings.sh

echo
echo "[calibration] launch UE"
launch_epoch="$(date +%s)"
AERION_AUTOPILOT_ENABLED=0 \
UE_BIN="$UE_BIN" \
UE_PROJECT="$UE_PROJECT" \
UE_QUALITY_LEVEL="$UE_QUALITY_LEVEL" \
UE_RENDER_PROFILE="$UE_RENDER_PROFILE" \
  bash scripts/run_ue_blocksv2.sh

latest_log=""
for _ in $(seq 1 20); do
  latest_log="$(
    find "$LOG_DIR" -maxdepth 1 -type f -name 'ue_*.log' -newermt "@$launch_epoch" \
      -printf '%T@ %p\n' 2>/dev/null \
      | sort -nr \
      | awk 'NR==1 {print $2}'
  )"
  [ -n "$latest_log" ] && break
  sleep 0.5
done
[ -n "$latest_log" ] || die "UE log not found in $LOG_DIR"

cat <<EOF

Open Town10HD_Opt if needed, press UE Play, wait until the drone appears,
then press Enter here.

Watching log:
  $latest_log
EOF
read -r -p "Press Enter after UE Play spawned AirSim drone: " _

echo
echo "[calibration] extract AirSim NED origin"
python3 scripts/extract_airsim_ned_origin_from_ue_log.py \
  --log "$latest_log" \
  --output "$OUTPUT"

echo
echo "[calibration] computed node #1 spawn preview"
python3 scripts/compute_airsim_spawn_for_mission_node.py \
  --mission "${MISSION_FILE:-$WORKSPACE/recordings/missions/clicked_mission_gps.json}" \
  --origin "$OUTPUT" \
  --node-index 1

echo
echo "Calibration complete:"
echo "  $OUTPUT"
