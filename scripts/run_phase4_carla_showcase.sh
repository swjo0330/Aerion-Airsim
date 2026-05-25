#!/usr/bin/env bash
# CARLA + AirSim showcase runner (3 drones, smooth formation demo).
#
# 사용법:
#   1) CARLA 통합 UE 프로젝트를 열고 Play
#   2) bash scripts/run_phase4_carla_showcase.sh

set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/workspace/projects/aerion-airsim}"
SETTINGS_DST="${SETTINGS_DST:-$HOME/Documents/AirSim/settings.json}"
AIRSIM_VIEW_MODE="${AIRSIM_VIEW_MODE:-Manual}"

# 안정/시연 기본값
DRONE_COUNT="${DRONE_COUNT:-3}"
PATTERNS="${PATTERNS:-TRIANGLE,LINE_H,ECHELON_R,DIAMOND3,ECHELON_L}"
CONTROL_MODE="${CONTROL_MODE:-showcase}"
VELOCITY="${VELOCITY:-3.0}"
SEGMENT_SEC="${SEGMENT_SEC:-8.0}"
TICK_HZ="${TICK_HZ:-4.5}"
COMMAND_STAGGER_MS="${COMMAND_STAGGER_MS:-35}"
CAPTURE_SEC="${CAPTURE_SEC:-4.0}"
PRE_SETTLE="${PRE_SETTLE:-1}"
PRE_SETTLE_RETRIES="${PRE_SETTLE_RETRIES:-3}"
FORMATION_HOLD_SEC="${FORMATION_HOLD_SEC:-10.0}"
TOL_M="${TOL_M:-0.60}"
TOL_XY_M="${TOL_XY_M:-1.40}"
TOL_Z_M="${TOL_Z_M:-0.90}"
MIN_SEPARATION_M="${MIN_SEPARATION_M:-4.0}"
TRANSITION_Z_LIFT="${TRANSITION_Z_LIFT:-0.9}"
COLLISION_HARD_MIN_M="${COLLISION_HARD_MIN_M:-2.6}"
COLLISION_MITIGATION_ROUNDS="${COLLISION_MITIGATION_ROUNDS:-3}"
CA_CROSSING_PENALTY="${CA_CROSSING_PENALTY:-20.0}"
CA_CONTINUITY_PENALTY="${CA_CONTINUITY_PENALTY:-6.0}"
ROLE_ASSIGNMENT="${ROLE_ASSIGNMENT:-fixed}"
CA_NEAR_DIST_M="${CA_NEAR_DIST_M:-6.0}"
CA_SLOWDOWN_FACTOR="${CA_SLOWDOWN_FACTOR:-0.72}"
REANCHOR_GAIN="${REANCHOR_GAIN:-0.85}"
SAFETY_FLOOR_Z="${SAFETY_FLOOR_Z:--0.20}"
SAFETY_RECOVERY_Z="${SAFETY_RECOVERY_Z:--3.50}"
THIRD_PERSON_FOLLOW="${THIRD_PERSON_FOLLOW:-1}"
THIRD_PERSON_MODE="${THIRD_PERSON_MODE:-external}"
THIRD_PERSON_CAMERA_NAME="${THIRD_PERSON_CAMERA_NAME:-0}"
THIRD_PERSON_FOLLOW_VEHICLE="${THIRD_PERSON_FOLLOW_VEHICLE:-drone1}"
THIRD_PERSON_DISTANCE_M="${THIRD_PERSON_DISTANCE_M:-14.0}"
THIRD_PERSON_HEIGHT_M="${THIRD_PERSON_HEIGHT_M:-6.0}"
THIRD_PERSON_LOOKAHEAD_M="${THIRD_PERSON_LOOKAHEAD_M:-2.0}"
THIRD_PERSON_UPDATE_HZ="${THIRD_PERSON_UPDATE_HZ:-8.0}"
THIRD_PERSON_VIEW_AZIMUTH_DEG="${THIRD_PERSON_VIEW_AZIMUTH_DEG:-135.0}"
THIRD_PERSON_VIEW_ELEVATION_BIAS_M="${THIRD_PERSON_VIEW_ELEVATION_BIAS_M:-0.0}"
OBSERVER_X="${OBSERVER_X:-0.0}"
OBSERVER_Y="${OBSERVER_Y:-0.0}"
OBSERVER_Z="${OBSERVER_Z:--20.0}"
FAIL_ON_MISS="${FAIL_ON_MISS:-0}"
CYCLES="${CYCLES:-0}"
AIRSIM_IP="${AIRSIM_IP:-127.0.0.1}"
AIRSIM_PORT="${AIRSIM_PORT:-41451}"
LEADER_X="${LEADER_X:-}"
LEADER_Y="${LEADER_Y:-}"
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

if [ -z "${SETTINGS_SRC:-}" ]; then
  if [ "$DRONE_COUNT" = "5" ]; then
    SETTINGS_SRC="$WORKSPACE/settings/carla_5drones_showcase.json"
  else
    SETTINGS_SRC="$WORKSPACE/settings/carla_3drones_showcase.json"
  fi
fi

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
AIRSIM_VIEW_MODE="$AIRSIM_VIEW_MODE" \
python3 - <<'PY'
import json, os
path = os.path.expanduser(os.environ["SETTINGS_DST"])
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)

data["ViewMode"] = os.environ.get("AIRSIM_VIEW_MODE", "Manual")
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
echo "  view mode: $AIRSIM_VIEW_MODE"
echo "  spawn offset xyz=($SPAWN_OFFSET_X, $SPAWN_OFFSET_Y, $SPAWN_OFFSET_Z)"
if [ -n "$LEADER_X" ] && [ -n "$LEADER_Y" ]; then
  echo "  leader start xy=($LEADER_X, $LEADER_Y)"
else
  echo "  leader start xy=(auto from drone1 spawn)"
fi

log "[Step 2] CARLA 통합 UE 프로젝트에서 Stop -> Play"
echo "  - AirSim settings는 Play 시점에만 재로딩됩니다."
echo "  - CARLA 맵 로딩 후 drone1/2/3 spawn 확인"
read -r -p "  준비되면 Enter: " _ || true

log "[Step 3] Showcase 실행"
echo "  ip=$AIRSIM_IP:$AIRSIM_PORT  patterns=$PATTERNS  vel=$VELOCITY"
echo "  segment=$SEGMENT_SEC  hold=$FORMATION_HOLD_SEC  tick=$TICK_HZ"
echo "  pre_settle=$PRE_SETTLE retries=$PRE_SETTLE_RETRIES"
echo "  min_separation_m=$MIN_SEPARATION_M  transition_z_lift=$TRANSITION_Z_LIFT"
echo "  collision_hard_min_m=$COLLISION_HARD_MIN_M collision_mitigation_rounds=$COLLISION_MITIGATION_ROUNDS"
echo "  ca_crossing_penalty=$CA_CROSSING_PENALTY ca_continuity_penalty=$CA_CONTINUITY_PENALTY"
echo "  role_assignment=$ROLE_ASSIGNMENT"
echo "  ca_near_dist_m=$CA_NEAR_DIST_M ca_slowdown_factor=$CA_SLOWDOWN_FACTOR"
echo "  reanchor_gain=$REANCHOR_GAIN"
echo "  safety_floor_z=$SAFETY_FLOOR_Z  safety_recovery_z=$SAFETY_RECOVERY_Z"
echo "  third_person_follow=$THIRD_PERSON_FOLLOW mode=$THIRD_PERSON_MODE camera=$THIRD_PERSON_CAMERA_NAME"
echo "  view_azimuth_deg=$THIRD_PERSON_VIEW_AZIMUTH_DEG view_elevation_bias_m=$THIRD_PERSON_VIEW_ELEVATION_BIAS_M"
echo "  observer_xyz=($OBSERVER_X, $OBSERVER_Y, $OBSERVER_Z)"
if [ -n "$LEADER_X" ] && [ -n "$LEADER_Y" ]; then
  echo "  leader_xy=($LEADER_X, $LEADER_Y)"
else
  echo "  leader_xy=(auto from drone1 spawn)"
fi
echo ""

CMD=(python3 "$WORKSPACE/scripts/phase4_delta_simple.py" \
  --ip "$AIRSIM_IP" \
  --port "$AIRSIM_PORT" \
  --settings-json "$SETTINGS_DST" \
  --drones "$DRONE_COUNT" \
  --patterns "$PATTERNS" \
  --control-mode "$CONTROL_MODE" \
  --velocity "$VELOCITY" \
  --segment-sec "$SEGMENT_SEC" \
  --tick-hz "$TICK_HZ" \
  --command-stagger-ms "$COMMAND_STAGGER_MS" \
  --capture-sec "$CAPTURE_SEC" \
  --pre-settle-retries "$PRE_SETTLE_RETRIES" \
  --formation-hold-sec "$FORMATION_HOLD_SEC" \
  --min-separation-m "$MIN_SEPARATION_M" \
  --transition-z-lift "$TRANSITION_Z_LIFT" \
  --collision-hard-min-m "$COLLISION_HARD_MIN_M" \
  --collision-mitigation-rounds "$COLLISION_MITIGATION_ROUNDS" \
  --ca-crossing-penalty "$CA_CROSSING_PENALTY" \
  --ca-continuity-penalty "$CA_CONTINUITY_PENALTY" \
  --role-assignment "$ROLE_ASSIGNMENT" \
  --ca-near-dist-m "$CA_NEAR_DIST_M" \
  --ca-slowdown-factor "$CA_SLOWDOWN_FACTOR" \
  --reanchor-gain "$REANCHOR_GAIN" \
  --tol-m "$TOL_M" \
  --tol-xy-m "$TOL_XY_M" \
  --tol-z-m "$TOL_Z_M" \
  --safety-floor-z "$SAFETY_FLOOR_Z" \
  --safety-recovery-z "$SAFETY_RECOVERY_Z" \
  --third-person-camera-name "$THIRD_PERSON_CAMERA_NAME" \
  --third-person-mode "$THIRD_PERSON_MODE" \
  --third-person-follow-vehicle "$THIRD_PERSON_FOLLOW_VEHICLE" \
  --third-person-distance-m "$THIRD_PERSON_DISTANCE_M" \
  --third-person-height-m "$THIRD_PERSON_HEIGHT_M" \
  --third-person-lookahead-m "$THIRD_PERSON_LOOKAHEAD_M" \
  --third-person-update-hz "$THIRD_PERSON_UPDATE_HZ" \
  --third-person-view-azimuth-deg "$THIRD_PERSON_VIEW_AZIMUTH_DEG" \
  --third-person-view-elevation-bias-m "$THIRD_PERSON_VIEW_ELEVATION_BIAS_M" \
  --observer-x "$OBSERVER_X" \
  --observer-y "$OBSERVER_Y" \
  --observer-z "$OBSERVER_Z" \
  --cycles "$CYCLES")

if [ "$FAIL_ON_MISS" = "1" ]; then
  CMD+=(--fail-on-miss)
fi
if [ "$PRE_SETTLE" = "1" ]; then
  CMD+=(--pre-settle)
fi
if [ -n "$LEADER_X" ]; then
  CMD+=(--leader-x "$LEADER_X")
fi
if [ -n "$LEADER_Y" ]; then
  CMD+=(--leader-y "$LEADER_Y")
fi
if [ "$THIRD_PERSON_FOLLOW" = "1" ]; then
  CMD+=(--third-person-follow)
fi

exec "${CMD[@]}"
