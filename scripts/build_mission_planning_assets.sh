#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
XODR="${XODR:-$HOME/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr}"
HEIGHTMAP="${HEIGHTMAP:-$WORKSPACE/recordings/maps/heightmap.json}"
HEIGHTMAP_ALIGNMENT="${HEIGHTMAP_ALIGNMENT:-}"
HEIGHTMAP_IMAGE="${HEIGHTMAP_IMAGE:-$WORKSPACE/recordings/maps/heightmap_map.png}"
EDITOR_HTML="${EDITOR_HTML:-$WORKSPACE/recordings/maps/mission_editor.html}"
MAP_BACKGROUND_IMAGE="${MAP_BACKGROUND_IMAGE:-}"
MAP_BACKGROUND_BOUNDS="${MAP_BACKGROUND_BOUNDS:-}"
TOPVIEW_ALIGNMENT="${TOPVIEW_ALIGNMENT:-}"
AUTO_TOPVIEW_ALIGNMENT="${AUTO_TOPVIEW_ALIGNMENT:-true}"
DISPLAY_FLIP_X="${DISPLAY_FLIP_X:-false}"
DISPLAY_FLIP_Y="${DISPLAY_FLIP_Y:-false}"
MAP_LAYER_FLIP_X="${MAP_LAYER_FLIP_X:-false}"
HEIGHTMAP_FLIP_X="${HEIGHTMAP_FLIP_X:-false}"
HEIGHTMAP_FLIP_Y="${HEIGHTMAP_FLIP_Y:-false}"
HEIGHTMAP_ROTATE_180="${HEIGHTMAP_ROTATE_180:-false}"
MAP_ROTATE_180="${MAP_ROTATE_180:-false}"
SCAN_HEIGHTMAP="${SCAN_HEIGHTMAP:-true}"
REQUIRE_HEIGHTMAP="${REQUIRE_HEIGHTMAP:-false}"
HEIGHTMAP_SCAN_METHOD="${HEIGHTMAP_SCAN_METHOD:-carla_raycast}"
ALLOW_UNSTABLE_AIRSIM_MESH_SCAN="${ALLOW_UNSTABLE_AIRSIM_MESH_SCAN:-false}"
SCAN_TIMEOUT_SEC="${SCAN_TIMEOUT_SEC:-120}"
AIRSIM_SCAN_TIMEOUT="${AIRSIM_SCAN_TIMEOUT:-$SCAN_TIMEOUT_SEC}"
HEIGHTMAP_RESOLUTION="${HEIGHTMAP_RESOLUTION:-2.0}"
HEIGHTMAP_BOUNDS="${HEIGHTMAP_BOUNDS:-}"
HEIGHTMAP_COORDINATE_FRAME="${HEIGHTMAP_COORDINATE_FRAME:-airsim_ned}"
HEIGHTMAP_VERTICES_ONLY="${HEIGHTMAP_VERTICES_ONLY:-false}"
HEIGHTMAP_MESH_REGEX="${HEIGHTMAP_MESH_REGEX:-.*}"
HEIGHTMAP_IGNORE_REGEX="${HEIGHTMAP_IGNORE_REGEX:-(?i)(sky|cloud|weather|sun|fog)}"
MAP_X_OFFSET="${MAP_X_OFFSET:-0.0}"
MAP_Y_OFFSET="${MAP_Y_OFFSET:-0.0}"
UP_OFFSET="${UP_OFFSET:-0.0}"
QUERY_HOME_ORIGIN="${QUERY_HOME_ORIGIN:-true}"
VEHICLE_NAME="${VEHICLE_NAME:-drone1}"
AIRSIM_IP="${AIRSIM_IP:-127.0.0.1}"
AIRSIM_PORT="${AIRSIM_PORT:-41451}"
AIRSIM_TIMEOUT="${AIRSIM_TIMEOUT:-5.0}"
CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2000}"
CARLA_TIMEOUT="${CARLA_TIMEOUT:-10.0}"
CARLA_RAYCAST_Z_TOP="${CARLA_RAYCAST_Z_TOP:-250.0}"
CARLA_RAYCAST_Z_BOTTOM="${CARLA_RAYCAST_Z_BOTTOM:--50.0}"
CARLA_RAYCAST_WORKERS="${CARLA_RAYCAST_WORKERS:-1}"
CARLA_RAYCAST_PROGRESS_EVERY="${CARLA_RAYCAST_PROGRESS_EVERY:-10}"
HOME_MAP_X="${HOME_MAP_X:-}"
HOME_MAP_Y="${HOME_MAP_Y:-}"
GPS_ORIGIN_LAT="${GPS_ORIGIN_LAT:-37.5665}"
GPS_ORIGIN_LON="${GPS_ORIGIN_LON:-126.9780}"
GPS_ORIGIN_ALT="${GPS_ORIGIN_ALT:-38.0}"

cd "$WORKSPACE"
mkdir -p "$(dirname "$HEIGHTMAP")" "$(dirname "$EDITOR_HTML")"

if [[ "$AUTO_TOPVIEW_ALIGNMENT" == "true" && -z "$TOPVIEW_ALIGNMENT" && -n "$MAP_BACKGROUND_IMAGE" && -f "$WORKSPACE/recordings/maps/topview_alignment.json" ]]; then
  TOPVIEW_ALIGNMENT="$WORKSPACE/recordings/maps/topview_alignment.json"
fi

echo "Mission planning assets"
echo "  WORKSPACE:    $WORKSPACE"
echo "  XODR:         $XODR"
echo "  HEIGHTMAP:    $HEIGHTMAP"
echo "  HEIGHTMAP_ALN:${HEIGHTMAP_ALIGNMENT:-<none>}"
echo "  MAP_IMAGE:    $HEIGHTMAP_IMAGE"
echo "  USER_BG:      ${MAP_BACKGROUND_IMAGE:-<none>}"
echo "  ALIGNMENT:    ${TOPVIEW_ALIGNMENT:-<none>}"
echo "  DISPLAY_FLIP_X:$DISPLAY_FLIP_X"
echo "  DISPLAY_FLIP_Y:$DISPLAY_FLIP_Y"
echo "  MAP_LAYER_FLIP_X:$MAP_LAYER_FLIP_X"
echo "  HEIGHTMAP_FLIP_X:$HEIGHTMAP_FLIP_X"
echo "  HEIGHTMAP_FLIP_Y:$HEIGHTMAP_FLIP_Y"
echo "  HEIGHTMAP_ROTATE_180:$HEIGHTMAP_ROTATE_180"
echo "  MAP_ROTATE_180:$MAP_ROTATE_180"
echo "  EDITOR_HTML:  $EDITOR_HTML"
echo "  SCAN:         $SCAN_HEIGHTMAP"
echo "  REQUIRE_MAP:  $REQUIRE_HEIGHTMAP"
echo "  METHOD:       $HEIGHTMAP_SCAN_METHOD"
echo "  SCAN_TIMEOUT: $SCAN_TIMEOUT_SEC"
echo "  RPC_TIMEOUT:  $AIRSIM_SCAN_TIMEOUT"
echo "  VERTICES_ONLY:$HEIGHTMAP_VERTICES_ONLY"
echo "  QUERY_HOME:   $QUERY_HOME_ORIGIN"
echo "  GPS_ORIGIN:   $GPS_ORIGIN_LAT,$GPS_ORIGIN_LON,$GPS_ORIGIN_ALT"

if [[ "$QUERY_HOME_ORIGIN" == "true" && ( -z "$HOME_MAP_X" || -z "$HOME_MAP_Y" ) ]]; then
  echo
  echo "[origin] AirSim vehicle pose -> editor home map origin"
  origin_json="$(
    timeout "$AIRSIM_TIMEOUT" python3 scripts/query_airsim_home_origin.py \
      --vehicle "$VEHICLE_NAME" \
      --coordinate-frame "$HEIGHTMAP_COORDINATE_FRAME" \
      --map-x-offset "$MAP_X_OFFSET" \
      --map-y-offset "$MAP_Y_OFFSET" \
      --up-offset "$UP_OFFSET" \
      --airsim-ip "$AIRSIM_IP" \
      --airsim-port "$AIRSIM_PORT" \
      --airsim-timeout "$AIRSIM_TIMEOUT" 2>/tmp/aerion_home_origin.err || true
  )"
  if [[ -n "$origin_json" ]]; then
    HOME_MAP_X="$(python3 -c 'import json,sys; print(json.load(sys.stdin)["map_x"])' <<<"$origin_json")"
    HOME_MAP_Y="$(python3 -c 'import json,sys; print(json.load(sys.stdin)["map_y"])' <<<"$origin_json")"
    echo "$origin_json" >"$WORKSPACE/recordings/maps/home_origin.json"
    echo "  home map origin: x=$HOME_MAP_X y=$HOME_MAP_Y"
  else
    echo "[origin] query failed; using HOME_MAP_X/Y defaults (${HOME_MAP_X:-0}/${HOME_MAP_Y:-0})" >&2
    sed -n '1,40p' /tmp/aerion_home_origin.err >&2 || true
  fi
fi

HOME_MAP_X="${HOME_MAP_X:-0.0}"
HOME_MAP_Y="${HOME_MAP_Y:-0.0}"

heightmap_arg=()
heightmap_alignment_arg=()
background_arg=()
alignment_arg=()
if [[ "$SCAN_HEIGHTMAP" == "true" ]]; then
  echo
  echo "[scan] $HEIGHTMAP_SCAN_METHOD -> heightmap"
  scan_ok=false
  if [[ "$HEIGHTMAP_SCAN_METHOD" == "carla_raycast" ]]; then
    carla_scan_args=(
        --xodr "$XODR"
        --output "$HEIGHTMAP"
        --resolution "$HEIGHTMAP_RESOLUTION"
        --host "$CARLA_HOST"
        --port "$CARLA_PORT"
        --timeout "$CARLA_TIMEOUT"
        --workers "$CARLA_RAYCAST_WORKERS"
        --progress-every "$CARLA_RAYCAST_PROGRESS_EVERY"
        --z-top "$CARLA_RAYCAST_Z_TOP"
        --z-bottom "$CARLA_RAYCAST_Z_BOTTOM"
    )
    if [[ -n "$HEIGHTMAP_BOUNDS" ]]; then
      read -r west east south north <<<"$HEIGHTMAP_BOUNDS"
      carla_scan_args+=(--bounds "$west" "$east" "$south" "$north")
    fi
    if timeout "$SCAN_TIMEOUT_SEC" python3 scripts/scan_carla_raycast_heightmap.py \
        "${carla_scan_args[@]}"; then
      scan_ok=true
    fi
  elif [[ "$HEIGHTMAP_SCAN_METHOD" == "airsim_mesh" ]]; then
    if [[ "$ALLOW_UNSTABLE_AIRSIM_MESH_SCAN" != "true" ]]; then
      echo "[scan] refusing AirSim mesh scan because it can crash Unreal/Vulkan on large CARLA maps." >&2
      echo "[scan] set ALLOW_UNSTABLE_AIRSIM_MESH_SCAN=true only if you explicitly accept that risk." >&2
    else
      scan_args=(
      --xodr "$XODR"
      --output "$HEIGHTMAP"
      --resolution "$HEIGHTMAP_RESOLUTION"
      --coordinate-frame "$HEIGHTMAP_COORDINATE_FRAME"
      --map-x-offset "$MAP_X_OFFSET"
      --map-y-offset "$MAP_Y_OFFSET"
      --up-offset "$UP_OFFSET"
      --mesh-regex "$HEIGHTMAP_MESH_REGEX"
      --ignore-regex "$HEIGHTMAP_IGNORE_REGEX"
      --airsim-ip "$AIRSIM_IP"
      --airsim-port "$AIRSIM_PORT"
      --airsim-timeout "$AIRSIM_SCAN_TIMEOUT"
      )
      if [[ "$HEIGHTMAP_VERTICES_ONLY" == "true" ]]; then
        scan_args+=(--vertices-only)
      fi
      if timeout "$SCAN_TIMEOUT_SEC" python3 scripts/scan_airsim_heightmap.py \
          "${scan_args[@]}"; then
        scan_ok=true
      fi
    fi
  else
    echo "[scan] unknown HEIGHTMAP_SCAN_METHOD=$HEIGHTMAP_SCAN_METHOD" >&2
  fi

  if [[ "$scan_ok" == "true" ]]; then
    heightmap_arg=(--heightmap "$HEIGHTMAP")
    echo
    echo "[render] heightmap -> visual map"
    python3 scripts/render_heightmap_map.py \
      --heightmap "$HEIGHTMAP" \
      --output "$HEIGHTMAP_IMAGE" \
      --scale 3
    background_arg=(--background-image "$HEIGHTMAP_IMAGE")
  else
    echo "[scan] heightmap scan failed or timed out; building editor without scanned heights" >&2
    if [[ "$REQUIRE_HEIGHTMAP" == "true" ]]; then
      echo "[scan] REQUIRE_HEIGHTMAP=true, aborting instead of writing a no-scan editor." >&2
      exit 2
    fi
  fi
elif [[ -f "$HEIGHTMAP" ]]; then
  heightmap_arg=(--heightmap "$HEIGHTMAP")
  if [[ -n "$HEIGHTMAP_ALIGNMENT" && -f "$HEIGHTMAP_ALIGNMENT" ]]; then
    heightmap_alignment_arg=(--heightmap-alignment "$HEIGHTMAP_ALIGNMENT")
  fi
  if [[ ! -f "$HEIGHTMAP_IMAGE" ]]; then
    echo
    echo "[render] existing heightmap -> visual map"
    python3 scripts/render_heightmap_map.py \
      --heightmap "$HEIGHTMAP" \
      --output "$HEIGHTMAP_IMAGE" \
      --scale 3
  fi
  background_arg=(--background-image "$HEIGHTMAP_IMAGE")
fi

if [[ -n "$MAP_BACKGROUND_IMAGE" ]]; then
  background_arg=(--background-image "$MAP_BACKGROUND_IMAGE")
  if [[ -n "$MAP_BACKGROUND_BOUNDS" ]]; then
    read -r bg_west bg_east bg_south bg_north <<<"$MAP_BACKGROUND_BOUNDS"
    background_arg+=(--background-bounds "$bg_west" "$bg_east" "$bg_south" "$bg_north")
  fi
fi

if [[ -n "$TOPVIEW_ALIGNMENT" ]]; then
  alignment_arg=(--alignment "$TOPVIEW_ALIGNMENT")
fi

display_arg=()
if [[ "$DISPLAY_FLIP_X" == "true" ]]; then
  display_arg+=(--display-flip-x)
fi
if [[ "$DISPLAY_FLIP_Y" == "true" ]]; then
  display_arg+=(--display-flip-y)
fi
if [[ "$MAP_LAYER_FLIP_X" == "true" ]]; then
  display_arg+=(--map-layer-flip-x)
fi
if [[ "$HEIGHTMAP_FLIP_X" == "true" ]]; then
  display_arg+=(--heightmap-flip-x)
fi
if [[ "$HEIGHTMAP_FLIP_Y" == "true" ]]; then
  display_arg+=(--heightmap-flip-y)
fi
if [[ "$HEIGHTMAP_ROTATE_180" == "true" ]]; then
  display_arg+=(--heightmap-rotate-180)
fi
if [[ "$MAP_ROTATE_180" == "true" ]]; then
  display_arg+=(--map-rotate-180)
fi

echo
echo "[editor] build standalone mission editor"
python3 scripts/build_mission_editor.py \
  --xodr "$XODR" \
  --output "$EDITOR_HTML" \
  --home-map-x "$HOME_MAP_X" \
  --home-map-y "$HOME_MAP_Y" \
  --gps-origin-lat "$GPS_ORIGIN_LAT" \
  --gps-origin-lon "$GPS_ORIGIN_LON" \
  --gps-origin-alt "$GPS_ORIGIN_ALT" \
  "${heightmap_arg[@]}" \
  "${heightmap_alignment_arg[@]}" \
  "${alignment_arg[@]}" \
  "${display_arg[@]}" \
  "${background_arg[@]}"

echo
echo "Open:"
echo "  google-chrome $EDITOR_HTML"
