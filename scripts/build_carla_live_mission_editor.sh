#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
XODR="${XODR:-$HOME/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr}"
CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2000}"
CARLA_TIMEOUT="${CARLA_TIMEOUT:-20.0}"
BOUNDS="${BOUNDS:-}"
CROP_TO_KNOWN_HEIGHTMAP="${CROP_TO_KNOWN_HEIGHTMAP:-$WORKSPACE/recordings/maps/heightmap.json}"
CROP_PAD_M="${CROP_PAD_M:-30}"
TOPDOWN_IMAGE="${TOPDOWN_IMAGE:-$WORKSPACE/recordings/maps/carla_topdown_mosaic.png}"
TOPDOWN_META="${TOPDOWN_META:-$WORKSPACE/recordings/maps/carla_topdown_mosaic.json}"
HEIGHTMAP="${HEIGHTMAP:-$WORKSPACE/recordings/maps/carla_live_heightmap.json}"
HEIGHTMAP_IMAGE="${HEIGHTMAP_IMAGE:-$WORKSPACE/recordings/maps/carla_live_heightmap_map.png}"
EDITOR_HTML="${EDITOR_HTML:-$WORKSPACE/recordings/maps/mission_editor.html}"
CAPTURE_TOPDOWN="${CAPTURE_TOPDOWN:-true}"
SCAN_HEIGHTMAP="${SCAN_HEIGHTMAP:-true}"
RENDER_HEIGHTMAP="${RENDER_HEIGHTMAP:-true}"
TILE_M="${TILE_M:-120}"
PIXELS_PER_METER="${PIXELS_PER_METER:-4}"
CAMERA_ALTITUDE_M="${CAMERA_ALTITUDE_M:-450}"
CAMERA_TYPE="${CAMERA_TYPE:-rgb}"
SAFETY_CAMERA_TYPE="${SAFETY_CAMERA_TYPE:-semantic}"
SAFETY_TOPDOWN_IMAGE="${SAFETY_TOPDOWN_IMAGE:-$WORKSPACE/recordings/maps/carla_topdown_safety_semantic.png}"
SAFETY_TOPDOWN_META="${SAFETY_TOPDOWN_META:-$WORKSPACE/recordings/maps/carla_topdown_safety_semantic.json}"
CAMERA_YAW_DEG="${CAMERA_YAW_DEG:-0}"
CAMERA_FOV_DEG="${CAMERA_FOV_DEG:-}"
CAMERA_WARMUP_FRAMES="${CAMERA_WARMUP_FRAMES:-3}"
CAMERA_MIN_LUMA="${CAMERA_MIN_LUMA:-3.0}"
CAMERA_DARK_RETRIES="${CAMERA_DARK_RETRIES:-1}"
CAMERA_ALLOW_DARK="${CAMERA_ALLOW_DARK:-true}"
RAW_ROTATE="${RAW_ROTATE:-0}"
RAW_FLIP_X="${RAW_FLIP_X:-false}"
RAW_FLIP_Y="${RAW_FLIP_Y:-false}"
HEIGHTMAP_RESOLUTION="${HEIGHTMAP_RESOLUTION:-1.0}"
RAYCAST_Z_TOP="${RAYCAST_Z_TOP:-300}"
RAYCAST_Z_BOTTOM="${RAYCAST_Z_BOTTOM:--80}"
RAYCAST_WORKERS="${RAYCAST_WORKERS:-2}"
QUERY_HOME_ORIGIN="${QUERY_HOME_ORIGIN:-false}"
DISPLAY_FLIP_Y="${DISPLAY_FLIP_Y:-false}"

cd "$WORKSPACE"
mkdir -p "$WORKSPACE/recordings/maps"

capture_bounds_args=()
scan_bounds_args=()
if [[ -n "$BOUNDS" ]]; then
  read -r west east south north <<<"$BOUNDS"
  capture_bounds_args=(--bounds "$west" "$east" "$south" "$north")
  scan_bounds_args=(--bounds "$west" "$east" "$south" "$north")
elif [[ -f "$CROP_TO_KNOWN_HEIGHTMAP" ]]; then
  capture_bounds_args=(--crop-to-known-heightmap "$CROP_TO_KNOWN_HEIGHTMAP" --crop-pad-m "$CROP_PAD_M")
fi

camera_extra=()
if [[ -n "$CAMERA_FOV_DEG" ]]; then
  camera_extra+=(--fov-deg "$CAMERA_FOV_DEG")
fi
if [[ "$CAMERA_ALLOW_DARK" == "true" ]]; then
  camera_extra+=(--allow-dark)
fi
if [[ "$RAW_FLIP_X" == "true" ]]; then
  camera_extra+=(--raw-flip-x)
fi
if [[ "$RAW_FLIP_Y" == "true" ]]; then
  camera_extra+=(--raw-flip-y)
fi

if [[ "$CAPTURE_TOPDOWN" == "true" ]]; then
  echo "[capture] CARLA live topdown mosaic"
  python3 scripts/capture_carla_topdown_mosaic.py \
    --host "$CARLA_HOST" \
    --port "$CARLA_PORT" \
    --timeout "$CARLA_TIMEOUT" \
    --xodr "$XODR" \
    "${capture_bounds_args[@]}" \
    --output "$TOPDOWN_IMAGE" \
    --metadata "$TOPDOWN_META" \
    --camera-type "$CAMERA_TYPE" \
    --tile-m "$TILE_M" \
    --pixels-per-meter "$PIXELS_PER_METER" \
    --altitude-m "$CAMERA_ALTITUDE_M" \
    --yaw-deg "$CAMERA_YAW_DEG" \
    --warmup-frames "$CAMERA_WARMUP_FRAMES" \
    --min-luma "$CAMERA_MIN_LUMA" \
    --dark-retries "$CAMERA_DARK_RETRIES" \
    --dark-pixel-threshold 8 \
    --raw-rotate "$RAW_ROTATE" \
    "${camera_extra[@]}"

  if [[ "$SAFETY_CAMERA_TYPE" != "$CAMERA_TYPE" ]]; then
    echo
    echo "[capture] CARLA safety topdown mosaic ($SAFETY_CAMERA_TYPE)"
    python3 scripts/capture_carla_topdown_mosaic.py \
      --host "$CARLA_HOST" \
      --port "$CARLA_PORT" \
      --timeout "$CARLA_TIMEOUT" \
      --xodr "$XODR" \
      "${capture_bounds_args[@]}" \
      --output "$SAFETY_TOPDOWN_IMAGE" \
      --metadata "$SAFETY_TOPDOWN_META" \
      --camera-type "$SAFETY_CAMERA_TYPE" \
      --tile-m "$TILE_M" \
      --pixels-per-meter "$PIXELS_PER_METER" \
      --altitude-m "$CAMERA_ALTITUDE_M" \
      --yaw-deg "$CAMERA_YAW_DEG" \
      --warmup-frames "$CAMERA_WARMUP_FRAMES" \
      --min-luma "$CAMERA_MIN_LUMA" \
      --dark-retries "$CAMERA_DARK_RETRIES" \
      --dark-pixel-threshold 8 \
      --raw-rotate "$RAW_ROTATE" \
      "${camera_extra[@]}"
  fi
fi

if [[ ! -f "$TOPDOWN_META" ]]; then
  echo "missing topdown metadata: $TOPDOWN_META" >&2
  exit 2
fi

read -r bg_west bg_east bg_south bg_north <<<"$(python3 - <<'PY' "$TOPDOWN_META"
import json, sys
d=json.load(open(sys.argv[1]))
b=d["bounds"]
print(b["west"], b["east"], b["south"], b["north"])
PY
)"

if [[ ${#scan_bounds_args[@]} -eq 0 ]]; then
  scan_bounds_args=(--bounds "$bg_west" "$bg_east" "$bg_south" "$bg_north")
fi

if [[ "$SCAN_HEIGHTMAP" == "true" ]]; then
  echo
  echo "[scan] CARLA raycast heightmap on topdown bounds"
  python3 scripts/scan_carla_raycast_heightmap.py \
    --host "$CARLA_HOST" \
    --port "$CARLA_PORT" \
    --timeout "$CARLA_TIMEOUT" \
    --xodr "$XODR" \
    "${scan_bounds_args[@]}" \
    --output "$HEIGHTMAP" \
    --resolution "$HEIGHTMAP_RESOLUTION" \
    --z-top "$RAYCAST_Z_TOP" \
    --z-bottom "$RAYCAST_Z_BOTTOM" \
    --workers "$RAYCAST_WORKERS" \
    --checkpoint-every 10 \
    --resume
fi

if [[ "$RENDER_HEIGHTMAP" == "true" && -f "$HEIGHTMAP" ]]; then
  echo
  echo "[render] heightmap visual"
  python3 scripts/render_heightmap_map.py \
    --heightmap "$HEIGHTMAP" \
    --output "$HEIGHTMAP_IMAGE" \
    --scale 2 \
    --mode semantic
fi

echo
echo "[editor] build live-map mission editor"
MAP_BACKGROUND_IMAGE="$TOPDOWN_IMAGE" \
MAP_BACKGROUND_BOUNDS="$bg_west $bg_east $bg_south $bg_north" \
TOPVIEW_ALIGNMENT="" \
AUTO_TOPVIEW_ALIGNMENT=false \
DISPLAY_FLIP_Y="$DISPLAY_FLIP_Y" \
SCAN_HEIGHTMAP=false \
HEIGHTMAP="$HEIGHTMAP" \
HEIGHTMAP_IMAGE="$HEIGHTMAP_IMAGE" \
QUERY_HOME_ORIGIN="$QUERY_HOME_ORIGIN" \
EDITOR_HTML="$EDITOR_HTML" \
bash scripts/build_mission_planning_assets.sh

cat <<EOF

Open:
  google-chrome $EDITOR_HTML

Topdown:
  $TOPDOWN_IMAGE
  bounds: $bg_west $bg_east $bg_south $bg_north

Safety topdown:
  $SAFETY_TOPDOWN_IMAGE
EOF
