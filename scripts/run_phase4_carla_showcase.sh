#!/usr/bin/env bash
# CARLA + AirSim showcase runner (3 drones, smooth formation demo).
#
# 사용법:
#   1) CARLA 통합 UE 프로젝트를 열고 Play
#   2) bash scripts/run_phase4_carla_showcase.sh

set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/workspace/projects/aerion-airsim}"
SETTINGS_DST="${SETTINGS_DST:-$HOME/Documents/AirSim/settings.json}"
ENV_FILE="${ENV_FILE:-$WORKSPACE/configs/carla_6drones_showcase.env}"

if [ -f "$ENV_FILE" ]; then
  # 고정 쇼케이스 파라미터는 env 파일에서 관리한다. 외부에서 넘긴 값은 유지.
  # shellcheck disable=SC1090
  source "$ENV_FILE"
fi

AIRSIM_VIEW_MODE="${AIRSIM_VIEW_MODE:-Manual}"

# 안정/시연 기본값
DRONE_COUNT="${DRONE_COUNT:-3}"
PATTERNS="${PATTERNS:-TRIANGLE,LINE_H,ECHELON_R,CIRCLE,DIAMOND3,ECHELON_L}"
CONTROL_MODE="${CONTROL_MODE:-showcase}"
VELOCITY="${VELOCITY:-4.8}"
SEGMENT_SEC="${SEGMENT_SEC:-5.6}"
TICK_HZ="${TICK_HZ:-7.0}"
COMMAND_STAGGER_MS="${COMMAND_STAGGER_MS:-12}"
CAPTURE_SEC="${CAPTURE_SEC:-1.6}"
PRE_SETTLE="${PRE_SETTLE:-1}"
PRE_SETTLE_RETRIES="${PRE_SETTLE_RETRIES:-3}"
FORMATION_HOLD_SEC="${FORMATION_HOLD_SEC:-3.0}"
CIRCLE_HOLD_SEC="${CIRCLE_HOLD_SEC:-3.0}"
TOL_M="${TOL_M:-0.60}"
TOL_XY_M="${TOL_XY_M:-1.40}"
TOL_Z_M="${TOL_Z_M:-0.90}"
MIN_SEPARATION_M="${MIN_SEPARATION_M:-6.2}"
TRANSITION_Z_LIFT="${TRANSITION_Z_LIFT:-0.8}"
Z_LIFT_GAIN="${Z_LIFT_GAIN:-0.28}"
Z_ALIGN_SEC="${Z_ALIGN_SEC:-0.8}"
Z_COMMAND_DEADBAND_M="${Z_COMMAND_DEADBAND_M:-0.26}"
Z_COMMAND_MAX_STEP_M="${Z_COMMAND_MAX_STEP_M:-0.22}"
Z_COMMAND_HOLD_BAND_M="${Z_COMMAND_HOLD_BAND_M:-0.12}"
Z_COMMAND_COOLDOWN_SEC="${Z_COMMAND_COOLDOWN_SEC:-0.55}"
Z_COLLISION_XY_THRESH_M="${Z_COLLISION_XY_THRESH_M:-4.8}"
Z_COLLISION_MIN_DZ_M="${Z_COLLISION_MIN_DZ_M:-1.35}"
Z_COLLISION_DZ_GAIN_PER_XY_M="${Z_COLLISION_DZ_GAIN_PER_XY_M:-0.45}"
COLLISION_HARD_MIN_M="${COLLISION_HARD_MIN_M:-3.6}"
COLLISION_MITIGATION_ROUNDS="${COLLISION_MITIGATION_ROUNDS:-3}"
CA_CROSSING_PENALTY="${CA_CROSSING_PENALTY:-20.0}"
CA_CONTINUITY_PENALTY="${CA_CONTINUITY_PENALTY:-6.0}"
ROLE_ASSIGNMENT="${ROLE_ASSIGNMENT:-fixed}"
CA_NEAR_DIST_M="${CA_NEAR_DIST_M:-8.0}"
CA_SLOWDOWN_FACTOR="${CA_SLOWDOWN_FACTOR:-0.70}"
REANCHOR_GAIN="${REANCHOR_GAIN:-0.85}"
SAFETY_FLOOR_Z="${SAFETY_FLOOR_Z:--0.20}"
SAFETY_RECOVERY_Z="${SAFETY_RECOVERY_Z:--3.50}"
POST_ACTION_ENABLED="${POST_ACTION_ENABLED:-1}"
POST_ACTION_STYLE="${POST_ACTION_STYLE:-mixed}"
POST_ACTION_SEGMENT_SEC="${POST_ACTION_SEGMENT_SEC:-4.8}"
POST_LINE_TRANSLATE_M="${POST_LINE_TRANSLATE_M:-18.0}"
POST_ROTATE_DEG_LIST="${POST_ROTATE_DEG_LIST:-90}"
POST_ACTION_VELOCITY_SCALE="${POST_ACTION_VELOCITY_SCALE:-1.22}"
POST_ACTION_PAUSE_SEC="${POST_ACTION_PAUSE_SEC:-1.2}"
TRANSITION_WHILE_MOVING="${TRANSITION_WHILE_MOVING:-0}"
THIRD_PERSON_FOLLOW="${THIRD_PERSON_FOLLOW:-1}"
THIRD_PERSON_MODE="${THIRD_PERSON_MODE:-external}"
THIRD_PERSON_CAMERA_NAME="${THIRD_PERSON_CAMERA_NAME:-0}"
SUBWINDOW_CAMERA_NAME="${SUBWINDOW_CAMERA_NAME:-back_center}"
SUBWINDOW_VEHICLE="${SUBWINDOW_VEHICLE:-drone1}"
AUTO_CAMERA_PROBE="${AUTO_CAMERA_PROBE:-1}"
THIRD_PERSON_FOLLOW_VEHICLE="${THIRD_PERSON_FOLLOW_VEHICLE:-drone1}"
THIRD_PERSON_DISTANCE_M="${THIRD_PERSON_DISTANCE_M:-14.0}"
THIRD_PERSON_HEIGHT_M="${THIRD_PERSON_HEIGHT_M:-6.0}"
THIRD_PERSON_LOOKAHEAD_M="${THIRD_PERSON_LOOKAHEAD_M:-2.0}"
THIRD_PERSON_UPDATE_HZ="${THIRD_PERSON_UPDATE_HZ:-8.0}"
THIRD_PERSON_VIEW_AZIMUTH_DEG="${THIRD_PERSON_VIEW_AZIMUTH_DEG:-135.0}"
THIRD_PERSON_VIEW_ELEVATION_BIAS_M="${THIRD_PERSON_VIEW_ELEVATION_BIAS_M:-0.0}"
CAM_CENTER_SMOOTH_ALPHA="${CAM_CENTER_SMOOTH_ALPHA:-0.22}"
CAM_DIR_SMOOTH_ALPHA="${CAM_DIR_SMOOTH_ALPHA:-0.35}"
CAM_YAW_RATE_LIMIT_DEG_S="${CAM_YAW_RATE_LIMIT_DEG_S:-40.0}"
CAM_MOTION_DEADBAND_MPS="${CAM_MOTION_DEADBAND_MPS:-0.35}"
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
  if [ "$DRONE_COUNT" = "6" ]; then
    SETTINGS_SRC="$WORKSPACE/settings/carla_6drones_showcase.json"
  elif [ "$DRONE_COUNT" = "5" ]; then
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
SUBWINDOW_CAMERA_NAME="$SUBWINDOW_CAMERA_NAME" \
SUBWINDOW_VEHICLE="$SUBWINDOW_VEHICLE" \
python3 - <<'PY'
import json, os
path = os.path.expanduser(os.environ["SETTINGS_DST"])
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)

data["ViewMode"] = os.environ.get("AIRSIM_VIEW_MODE", "Manual")
data["SubWindows"] = [{
    "WindowID": 0,
    "CameraName": os.environ.get("SUBWINDOW_CAMERA_NAME", "0"),
    "ImageType": 0,
    "VehicleName": os.environ.get("SUBWINDOW_VEHICLE", "drone1"),
    "Visible": True
}]
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
echo "  env file: $ENV_FILE"
echo "  view mode: $AIRSIM_VIEW_MODE"
echo "  subwindow: camera=$SUBWINDOW_CAMERA_NAME vehicle=$SUBWINDOW_VEHICLE"
echo "  spawn offset xyz=($SPAWN_OFFSET_X, $SPAWN_OFFSET_Y, $SPAWN_OFFSET_Z)"
if [ -n "$LEADER_X" ] && [ -n "$LEADER_Y" ]; then
  echo "  leader start xy=($LEADER_X, $LEADER_Y)"
else
  echo "  leader start xy=(auto from drone1 spawn)"
fi

log "[Step 2] CARLA 통합 UE 프로젝트에서 Stop -> Play"
echo "  - AirSim settings는 Play 시점에만 재로딩됩니다."
echo "  - CARLA 맵 로딩 후 drone1..drone${DRONE_COUNT} spawn 확인"
read -r -p "  준비되면 Enter: " _ || true

if [ "$AUTO_CAMERA_PROBE" = "1" ]; then
  log "[Step 2.1] 내장 카메라/센서 생존 진단"
  AIRSIM_IP="$AIRSIM_IP" AIRSIM_PORT="$AIRSIM_PORT" SUBWINDOW_VEHICLE="$SUBWINDOW_VEHICLE" \
  python3 - <<'PY'
import os, sys
import airsim
ip = os.environ.get("AIRSIM_IP", "127.0.0.1")
port = int(os.environ.get("AIRSIM_PORT", "41451"))
vehicle = os.environ.get("SUBWINDOW_VEHICLE", "drone1")
cams = ["0", "front_center", "fpv", "1", "2", "back_center", "bottom_center"]

print(f"[camera-probe] airsim module: {getattr(airsim, '__file__', 'unknown')}")
ClientCls = getattr(airsim, "MultirotorClient", None)
if ClientCls is None:
    try:
        from airsim.client import MultirotorClient as ClientCls
    except Exception as e:
        print(f"[camera-probe] MultirotorClient import failed: {e}")
        sys.exit(0)

ImageRequestCls = getattr(airsim, "ImageRequest", None)
ImageTypeScene = None
if hasattr(airsim, "ImageType"):
    try:
        ImageTypeScene = airsim.ImageType.Scene
    except Exception:
        ImageTypeScene = None
if ImageRequestCls is None or ImageTypeScene is None:
    try:
        from airsim.types import ImageRequest as ImageRequestCls, ImageType
        ImageTypeScene = ImageType.Scene
    except Exception as e:
        print(f"[camera-probe] ImageRequest/ImageType import failed: {e}")
        sys.exit(0)

try:
    c = ClientCls(ip=ip, port=port)
    c.confirmConnection()
except Exception as e:
    print(f"[camera-probe] connect failed: {e}")
    sys.exit(0)

def score(resp):
    if resp is None or resp.width <= 0 or resp.height <= 0:
        return -1.0
    data = resp.image_data_uint8
    if data is None or len(data) == 0:
        return -1.0
    # RGB bytes mean intensity
    step = max(1, len(data)//2000)
    sample = data[0::step]
    if not sample:
        return -1.0
    return float(sum(sample))/len(sample)

best = None
for cam in cams:
    try:
        resp = c.simGetImages([ImageRequestCls(cam, ImageTypeScene, False, False)], vehicle_name=vehicle)[0]
        s = score(resp)
        print(f"[camera-probe] {vehicle}:{cam} size={resp.width}x{resp.height} score={s:.2f}")
        if best is None or s > best[1]:
            best = (cam, s, resp.width, resp.height)
    except Exception as e:
        print(f"[camera-probe] {vehicle}:{cam} error={e}")

if best is None:
    print("[camera-probe] no camera response")
else:
    cam, s, w, h = best
    if s < 0.5:
        print(f"[camera-probe] all feeds look invalid/black. best={cam} score={s:.2f}")
    else:
        print(f"[camera-probe] 추천 camera={cam} (score={s:.2f}, {w}x{h})")
PY
fi

log "[Step 3] Showcase 실행"
echo "  ip=$AIRSIM_IP:$AIRSIM_PORT  patterns=$PATTERNS  vel=$VELOCITY"
echo "  segment=$SEGMENT_SEC  hold=$FORMATION_HOLD_SEC (circle=$CIRCLE_HOLD_SEC)  tick=$TICK_HZ"
echo "  pre_settle=$PRE_SETTLE retries=$PRE_SETTLE_RETRIES"
echo "  min_separation_m=$MIN_SEPARATION_M  transition_z_lift=$TRANSITION_Z_LIFT z_lift_gain=$Z_LIFT_GAIN z_align_sec=$Z_ALIGN_SEC"
echo "  z_command_deadband_m=$Z_COMMAND_DEADBAND_M z_command_max_step_m=$Z_COMMAND_MAX_STEP_M"
echo "  z_command_hold_band_m=$Z_COMMAND_HOLD_BAND_M z_command_cooldown_sec=$Z_COMMAND_COOLDOWN_SEC"
echo "  z_collision_xy_thresh_m=$Z_COLLISION_XY_THRESH_M z_collision_min_dz_m=$Z_COLLISION_MIN_DZ_M"
echo "  z_collision_dz_gain_per_xy_m=$Z_COLLISION_DZ_GAIN_PER_XY_M"
echo "  collision_hard_min_m=$COLLISION_HARD_MIN_M collision_mitigation_rounds=$COLLISION_MITIGATION_ROUNDS"
echo "  ca_crossing_penalty=$CA_CROSSING_PENALTY ca_continuity_penalty=$CA_CONTINUITY_PENALTY"
echo "  role_assignment=$ROLE_ASSIGNMENT"
echo "  ca_near_dist_m=$CA_NEAR_DIST_M ca_slowdown_factor=$CA_SLOWDOWN_FACTOR"
echo "  reanchor_gain=$REANCHOR_GAIN"
echo "  safety_floor_z=$SAFETY_FLOOR_Z  safety_recovery_z=$SAFETY_RECOVERY_Z"
echo "  post_action_enabled=$POST_ACTION_ENABLED post_action_style=$POST_ACTION_STYLE post_action_segment_sec=$POST_ACTION_SEGMENT_SEC"
echo "  post_line_translate_m=$POST_LINE_TRANSLATE_M post_rotate_deg_list=$POST_ROTATE_DEG_LIST post_action_velocity_scale=$POST_ACTION_VELOCITY_SCALE post_action_pause_sec=$POST_ACTION_PAUSE_SEC"
echo "  transition_while_moving=$TRANSITION_WHILE_MOVING"
echo "  third_person_follow=$THIRD_PERSON_FOLLOW mode=$THIRD_PERSON_MODE camera=$THIRD_PERSON_CAMERA_NAME"
echo "  view_azimuth_deg=$THIRD_PERSON_VIEW_AZIMUTH_DEG view_elevation_bias_m=$THIRD_PERSON_VIEW_ELEVATION_BIAS_M"
echo "  cam_smooth(center=$CAM_CENTER_SMOOTH_ALPHA, dir=$CAM_DIR_SMOOTH_ALPHA) yaw_rate_limit=$CAM_YAW_RATE_LIMIT_DEG_S deadband=$CAM_MOTION_DEADBAND_MPS"
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
  --circle-hold-sec "$CIRCLE_HOLD_SEC" \
  --min-separation-m "$MIN_SEPARATION_M" \
  --transition-z-lift "$TRANSITION_Z_LIFT" \
  --z-lift-gain "$Z_LIFT_GAIN" \
  --z-align-sec "$Z_ALIGN_SEC" \
  --z-command-deadband-m "$Z_COMMAND_DEADBAND_M" \
  --z-command-max-step-m "$Z_COMMAND_MAX_STEP_M" \
  --z-command-hold-band-m "$Z_COMMAND_HOLD_BAND_M" \
  --z-command-cooldown-sec "$Z_COMMAND_COOLDOWN_SEC" \
  --z-collision-xy-thresh-m "$Z_COLLISION_XY_THRESH_M" \
  --z-collision-min-dz-m "$Z_COLLISION_MIN_DZ_M" \
  --z-collision-dz-gain-per-xy-m "$Z_COLLISION_DZ_GAIN_PER_XY_M" \
  --collision-hard-min-m "$COLLISION_HARD_MIN_M" \
  --collision-mitigation-rounds "$COLLISION_MITIGATION_ROUNDS" \
  --ca-crossing-penalty "$CA_CROSSING_PENALTY" \
  --ca-continuity-penalty "$CA_CONTINUITY_PENALTY" \
  --role-assignment "$ROLE_ASSIGNMENT" \
  --ca-near-dist-m "$CA_NEAR_DIST_M" \
  --ca-slowdown-factor "$CA_SLOWDOWN_FACTOR" \
  --reanchor-gain "$REANCHOR_GAIN" \
  --post-action-segment-sec "$POST_ACTION_SEGMENT_SEC" \
  --post-action-style "$POST_ACTION_STYLE" \
  --post-line-translate-m "$POST_LINE_TRANSLATE_M" \
  --post-rotate-deg-list "$POST_ROTATE_DEG_LIST" \
  --post-action-velocity-scale "$POST_ACTION_VELOCITY_SCALE" \
  --post-action-pause-sec "$POST_ACTION_PAUSE_SEC" \
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
  --cam-center-smooth-alpha "$CAM_CENTER_SMOOTH_ALPHA" \
  --cam-dir-smooth-alpha "$CAM_DIR_SMOOTH_ALPHA" \
  --cam-yaw-rate-limit-deg-s "$CAM_YAW_RATE_LIMIT_DEG_S" \
  --cam-motion-deadband-mps "$CAM_MOTION_DEADBAND_MPS" \
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
if [ "$POST_ACTION_ENABLED" = "1" ]; then
  CMD+=(--post-action-enabled)
fi
if [ "$TRANSITION_WHILE_MOVING" = "1" ]; then
  CMD+=(--transition-while-moving)
fi

exec "${CMD[@]}"
