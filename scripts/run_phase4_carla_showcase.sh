#!/usr/bin/env bash
# CARLA + AirSim showcase runner (3 drones, smooth formation demo).
#
# 사용법:
#   1) CARLA 통합 UE 프로젝트를 열고 Play
#   2) bash scripts/run_phase4_carla_showcase.sh

set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/workspace/projects/aerion-airsim}"
SETTINGS_SRC="${SETTINGS_SRC:-$WORKSPACE/settings/carla_3drones_showcase.json}"
SETTINGS_DST="${SETTINGS_DST:-$HOME/Documents/AirSim/settings.json}"

# 안정/시연 기본값
DRONE_COUNT="${DRONE_COUNT:-3}"
PATTERNS="${PATTERNS:-TRIANGLE,LINE_H,ECHELON_R,DIAMOND3,ECHELON_L}"
CONTROL_MODE="${CONTROL_MODE:-showcase}"
VELOCITY="${VELOCITY:-3.0}"
SEGMENT_SEC="${SEGMENT_SEC:-8.0}"
TICK_HZ="${TICK_HZ:-4.5}"
COMMAND_STAGGER_MS="${COMMAND_STAGGER_MS:-35}"
CAPTURE_SEC="${CAPTURE_SEC:-4.0}"
FORMATION_HOLD_SEC="${FORMATION_HOLD_SEC:-10.0}"
TOL_M="${TOL_M:-0.60}"
TOL_XY_M="${TOL_XY_M:-1.40}"
TOL_Z_M="${TOL_Z_M:-0.90}"
MIN_SEPARATION_M="${MIN_SEPARATION_M:-4.0}"
TRANSITION_Z_LIFT="${TRANSITION_Z_LIFT:-0.9}"
SAFETY_FLOOR_Z="${SAFETY_FLOOR_Z:--0.20}"
SAFETY_RECOVERY_Z="${SAFETY_RECOVERY_Z:--3.50}"
THIRD_PERSON_FOLLOW="${THIRD_PERSON_FOLLOW:-1}"
THIRD_PERSON_CAMERA_NAME="${THIRD_PERSON_CAMERA_NAME:-front_center}"
THIRD_PERSON_FOLLOW_VEHICLE="${THIRD_PERSON_FOLLOW_VEHICLE:-drone1}"
THIRD_PERSON_DISTANCE_M="${THIRD_PERSON_DISTANCE_M:-14.0}"
THIRD_PERSON_HEIGHT_M="${THIRD_PERSON_HEIGHT_M:-6.0}"
THIRD_PERSON_LOOKAHEAD_M="${THIRD_PERSON_LOOKAHEAD_M:-2.0}"
THIRD_PERSON_UPDATE_HZ="${THIRD_PERSON_UPDATE_HZ:-8.0}"
FAIL_ON_MISS="${FAIL_ON_MISS:-0}"
CYCLES="${CYCLES:-0}"
AIRSIM_IP="${AIRSIM_IP:-127.0.0.1}"
AIRSIM_PORT="${AIRSIM_PORT:-41451}"
LEADER_X="${LEADER_X:-0.0}"
LEADER_Y="${LEADER_Y:-0.0}"
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

log() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '\n[%s] ERROR: %s\n' "$(date +%H:%M:%S)" "$*" >&2; exit 1; }

[ -f "$WORKSPACE/scripts/phase4_delta_simple.py" ] || die "phase4_delta_simple.py 없음"
[ -f "$SETTINGS_SRC" ] || die "settings 없음: $SETTINGS_SRC"

log "[Step 1] CARLA showcase settings deploy"
mkdir -p "$(dirname "$SETTINGS_DST")"
cp "$SETTINGS_SRC" "$SETTINGS_DST"
SETTINGS_DST="$SETTINGS_DST" \
SPAWN_OFFSET_X="$SPAWN_OFFSET_X" SPAWN_OFFSET_Y="$SPAWN_OFFSET_Y" SPAWN_OFFSET_Z="$SPAWN_OFFSET_Z" \
DRONE1_X="$DRONE1_X" DRONE1_Y="$DRONE1_Y" DRONE1_Z="$DRONE1_Z" \
DRONE2_X="$DRONE2_X" DRONE2_Y="$DRONE2_Y" DRONE2_Z="$DRONE2_Z" \
DRONE3_X="$DRONE3_X" DRONE3_Y="$DRONE3_Y" DRONE3_Z="$DRONE3_Z" \
python3 - <<'PY'
import json, os
path = os.path.expanduser(os.environ["SETTINGS_DST"])
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)

vehicles = data.get("Vehicles", {})
offx = float(os.environ.get("SPAWN_OFFSET_X", "0.0"))
offy = float(os.environ.get("SPAWN_OFFSET_Y", "0.0"))
offz = float(os.environ.get("SPAWN_OFFSET_Z", "0.0"))

for name, v in vehicles.items():
    if isinstance(v, dict):
        v["X"] = float(v.get("X", 0.0)) + offx
        v["Y"] = float(v.get("Y", 0.0)) + offy
        v["Z"] = float(v.get("Z", 0.0)) + offz

def override_xyz(drone):
    vx = os.environ.get(f"{drone}_X", "")
    vy = os.environ.get(f"{drone}_Y", "")
    vz = os.environ.get(f"{drone}_Z", "")
    if drone.lower() in vehicles and isinstance(vehicles[drone.lower()], dict):
        obj = vehicles[drone.lower()]
        if vx != "":
            obj["X"] = float(vx)
        if vy != "":
            obj["Y"] = float(vy)
        if vz != "":
            obj["Z"] = float(vz)

override_xyz("DRONE1")
override_xyz("DRONE2")
override_xyz("DRONE3")

with open(path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2)
PY
echo "  md5: $(md5sum "$SETTINGS_DST" | awk '{print $1}')"
echo "  spawn offset xyz=($SPAWN_OFFSET_X, $SPAWN_OFFSET_Y, $SPAWN_OFFSET_Z)"
echo "  leader start xy=($LEADER_X, $LEADER_Y)"

log "[Step 2] CARLA 통합 UE 프로젝트에서 Stop -> Play"
echo "  - AirSim settings는 Play 시점에만 재로딩됩니다."
echo "  - CARLA 맵 로딩 후 drone1/2/3 spawn 확인"
read -r -p "  준비되면 Enter: " _ || true

log "[Step 3] Showcase 실행"
echo "  ip=$AIRSIM_IP:$AIRSIM_PORT  patterns=$PATTERNS  vel=$VELOCITY"
echo "  segment=$SEGMENT_SEC  hold=$FORMATION_HOLD_SEC  tick=$TICK_HZ"
echo "  min_separation_m=$MIN_SEPARATION_M  transition_z_lift=$TRANSITION_Z_LIFT"
echo "  safety_floor_z=$SAFETY_FLOOR_Z  safety_recovery_z=$SAFETY_RECOVERY_Z"
echo "  third_person_follow=$THIRD_PERSON_FOLLOW ($THIRD_PERSON_FOLLOW_VEHICLE/$THIRD_PERSON_CAMERA_NAME)"
echo "  leader_xy=($LEADER_X, $LEADER_Y)"
echo ""

CMD=(python3 "$WORKSPACE/scripts/phase4_delta_simple.py" \
  --ip "$AIRSIM_IP" \
  --port "$AIRSIM_PORT" \
  --drones "$DRONE_COUNT" \
  --patterns "$PATTERNS" \
  --control-mode "$CONTROL_MODE" \
  --leader-x "$LEADER_X" \
  --leader-y "$LEADER_Y" \
  --velocity "$VELOCITY" \
  --segment-sec "$SEGMENT_SEC" \
  --tick-hz "$TICK_HZ" \
  --command-stagger-ms "$COMMAND_STAGGER_MS" \
  --capture-sec "$CAPTURE_SEC" \
  --formation-hold-sec "$FORMATION_HOLD_SEC" \
  --min-separation-m "$MIN_SEPARATION_M" \
  --transition-z-lift "$TRANSITION_Z_LIFT" \
  --tol-m "$TOL_M" \
  --tol-xy-m "$TOL_XY_M" \
  --tol-z-m "$TOL_Z_M" \
  --safety-floor-z "$SAFETY_FLOOR_Z" \
  --safety-recovery-z "$SAFETY_RECOVERY_Z" \
  --third-person-camera-name "$THIRD_PERSON_CAMERA_NAME" \
  --third-person-follow-vehicle "$THIRD_PERSON_FOLLOW_VEHICLE" \
  --third-person-distance-m "$THIRD_PERSON_DISTANCE_M" \
  --third-person-height-m "$THIRD_PERSON_HEIGHT_M" \
  --third-person-lookahead-m "$THIRD_PERSON_LOOKAHEAD_M" \
  --third-person-update-hz "$THIRD_PERSON_UPDATE_HZ" \
  --cycles "$CYCLES")

if [ "$FAIL_ON_MISS" = "1" ]; then
  CMD+=(--fail-on-miss)
fi
if [ "$THIRD_PERSON_FOLLOW" = "1" ]; then
  CMD+=(--third-person-follow)
fi

exec "${CMD[@]}"
