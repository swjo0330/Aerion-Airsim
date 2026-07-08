#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2000}"
CARLA_TIMEOUT="${CARLA_TIMEOUT:-30}"
WAIT_SECONDS="${WAIT_SECONDS:-180}"
XODR="${XODR:-$HOME/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr}"

AIRSIM_IP="${AIRSIM_IP:-127.0.0.1}"
AIRSIM_PORT="${AIRSIM_PORT:-41451}"
AIRSIM_TIMEOUT="${AIRSIM_TIMEOUT:-10}"
VEHICLE_NAME="${VEHICLE_NAME:-drone1}"
AIRSIM_CAMERA="${AIRSIM_CAMERA:-front_center}"

TOPDOWN_IMAGE="${TOPDOWN_IMAGE:-$WORKSPACE/recordings/maps/town10_live_topdown_airsim_rgb.png}"
TOPDOWN_META="${TOPDOWN_META:-$WORKSPACE/recordings/maps/town10_live_topdown_airsim_rgb.json}"
SAFETY_TOPDOWN_IMAGE="${SAFETY_TOPDOWN_IMAGE:-$WORKSPACE/recordings/maps/town10_live_topdown_semantic.png}"
SAFETY_TOPDOWN_META="${SAFETY_TOPDOWN_META:-$WORKSPACE/recordings/maps/town10_live_topdown_semantic.json}"
SAFETY_RAW_IMAGE="${SAFETY_RAW_IMAGE:-$WORKSPACE/recordings/maps/town10_live_topdown_semantic_raw.png}"
SAFETY_RAW_META="${SAFETY_RAW_META:-$WORKSPACE/recordings/maps/town10_live_topdown_semantic_raw.json}"
HEIGHTMAP="${HEIGHTMAP:-$WORKSPACE/recordings/maps/town10_live_heightmap_1m.json}"
HEIGHTMAP_ALIGNMENT="${HEIGHTMAP_ALIGNMENT:-$WORKSPACE/recordings/maps/topview_heightmap_alignment.json}"
HEIGHTMAP_IMAGE="${HEIGHTMAP_IMAGE:-$WORKSPACE/recordings/maps/town10_live_heightmap_semantic.png}"
EDITOR_HTML="${EDITOR_HTML:-$WORKSPACE/recordings/maps/mission_editor.html}"

TILE_M="${TILE_M:-40}"
PIXELS_PER_METER="${PIXELS_PER_METER:-8}"
HEIGHTMAP_RESOLUTION="${HEIGHTMAP_RESOLUTION:-1.0}"
RAYCAST_WORKERS="${RAYCAST_WORKERS:-2}"
DISPLAY_FLIP_X="${DISPLAY_FLIP_X:-false}"
DISPLAY_FLIP_Y="${DISPLAY_FLIP_Y:-false}"
MAP_LAYER_FLIP_X="${MAP_LAYER_FLIP_X:-false}"
HEIGHTMAP_FLIP_X="${HEIGHTMAP_FLIP_X:-false}"
HEIGHTMAP_FLIP_Y="${HEIGHTMAP_FLIP_Y:-false}"
HEIGHTMAP_ROTATE_180="${HEIGHTMAP_ROTATE_180:-false}"
MAP_ROTATE_180="${MAP_ROTATE_180:-true}"
CAPTURE_SAFETY="${CAPTURE_SAFETY:-true}"
SCAN_HEIGHTMAP="${SCAN_HEIGHTMAP:-false}"
AIRSIM_ALTITUDE_M="${AIRSIM_ALTITUDE_M:-300}"
AIRSIM_OVERSCAN="${AIRSIM_OVERSCAN:-2.0}"
RGB_MAP_MARGIN_M="${RGB_MAP_MARGIN_M:-40}"
TOPVIEW_ALIGNMENT="${TOPVIEW_ALIGNMENT:-$WORKSPACE/recordings/maps/topview_alignment.json}"
USE_TOPVIEW_ALIGNMENT="${USE_TOPVIEW_ALIGNMENT:-true}"

cd "$WORKSPACE"

echo "Town10 live topview capture"
echo "  CARLA:             $CARLA_HOST:$CARLA_PORT"
echo "  AirSim:            $AIRSIM_IP:$AIRSIM_PORT vehicle=$VEHICLE_NAME camera=$AIRSIM_CAMERA"
echo "  wait seconds:      $WAIT_SECONDS"
echo "  RGB topview:       $TOPDOWN_IMAGE"
echo "  semantic topview:  $SAFETY_TOPDOWN_IMAGE"
echo "  alignment:         $TOPVIEW_ALIGNMENT"
echo "  AirSim altitude:   $AIRSIM_ALTITUDE_M overscan=$AIRSIM_OVERSCAN"
echo "  RGB map margin:    $RGB_MAP_MARGIN_M m"
echo "  heightmap:         $HEIGHTMAP"
echo "  heightmap align:   $HEIGHTMAP_ALIGNMENT"
echo "  editor:            $EDITOR_HTML"
echo
echo "Waiting for CARLA RPC. In UE Editor, open Town10HD_Opt and press Play if needed."

python3 - <<'PY' "$CARLA_HOST" "$CARLA_PORT" "$WAIT_SECONDS"
import sys
import time

import carla

host = sys.argv[1]
port = int(sys.argv[2])
deadline = time.time() + float(sys.argv[3])
last = None
while time.time() < deadline:
    try:
        client = carla.Client(host, port)
        client.set_timeout(1.0)
        world = client.get_world()
        print(f"Connected to CARLA world: {world.get_map().name}", flush=True)
        raise SystemExit(0)
    except Exception as exc:
        message = str(exc)
        if message != last:
            print(f"  waiting: {message}", flush=True)
            last = message
        time.sleep(1.0)
print("Timed out waiting for CARLA RPC.", file=sys.stderr)
raise SystemExit(1)
PY

echo
echo "[capture] AirSim RGB topdown mosaic"
airsim_bounds_args=()
if [[ -n "${BOUNDS:-}" ]]; then
  read -r west east south north <<<"$BOUNDS"
  airsim_bounds_args=(--bounds "$west" "$east" "$south" "$north")
elif [[ "$RGB_MAP_MARGIN_M" != "0" && "$RGB_MAP_MARGIN_M" != "0.0" ]]; then
  read -r west east south north <<<"$(python3 - <<'PY' "$XODR" "$RGB_MAP_MARGIN_M"
import sys
import xml.etree.ElementTree as ET

xodr = sys.argv[1]
margin = float(sys.argv[2])
header = ET.parse(xodr).getroot().find("header")
if header is None:
    raise SystemExit(f"OpenDRIVE header not found: {xodr}")
west = float(header.attrib["west"]) - margin
east = float(header.attrib["east"]) + margin
south = float(header.attrib["south"]) - margin
north = float(header.attrib["north"]) + margin
print(west, east, south, north)
PY
)"
  airsim_bounds_args=(--bounds "$west" "$east" "$south" "$north")
fi
python3 scripts/capture_airsim_topdown_mosaic.py \
  --airsim-ip "$AIRSIM_IP" \
  --airsim-port "$AIRSIM_PORT" \
  --airsim-timeout "$AIRSIM_TIMEOUT" \
  --vehicle "$VEHICLE_NAME" \
  --camera "$AIRSIM_CAMERA" \
  --xodr "$XODR" \
  "${airsim_bounds_args[@]}" \
  --output "$TOPDOWN_IMAGE" \
  --metadata "$TOPDOWN_META" \
  --tile-m "$TILE_M" \
  --pixels-per-meter "$PIXELS_PER_METER" \
  --altitude-m "$AIRSIM_ALTITUDE_M" \
  --overscan "$AIRSIM_OVERSCAN"

read -r bg_west bg_east bg_south bg_north <<<"$(python3 - <<'PY' "$TOPDOWN_META"
import json, sys
d=json.load(open(sys.argv[1]))
b=d["bounds"]
print(b["west"], b["east"], b["south"], b["north"])
PY
)"

if [[ "$CAPTURE_SAFETY" == "true" ]]; then
  echo
  echo "[capture] CARLA semantic safety topdown mosaic"
  python3 scripts/capture_carla_topdown_mosaic.py \
    --host "$CARLA_HOST" \
    --port "$CARLA_PORT" \
    --timeout "$CARLA_TIMEOUT" \
    --xodr "$XODR" \
    --bounds "$bg_west" "$bg_east" "$bg_south" "$bg_north" \
    --output "$SAFETY_RAW_IMAGE" \
    --metadata "$SAFETY_RAW_META" \
    --camera-type semantic \
    --tile-m "$TILE_M" \
    --pixels-per-meter "$PIXELS_PER_METER" \
    --altitude-m 450 \
    --warmup-frames 3 \
    --allow-dark

  if [[ "$USE_TOPVIEW_ALIGNMENT" != "true" || ! -f "$TOPVIEW_ALIGNMENT" ]]; then
    echo "missing photo alignment for safety warp: $TOPVIEW_ALIGNMENT" >&2
    echo "Run calibration first or set USE_TOPVIEW_ALIGNMENT=false CAPTURE_SAFETY=false." >&2
    exit 2
  fi
  python3 scripts/warp_map_image_to_reference.py \
    --source-image "$SAFETY_RAW_IMAGE" \
    --source-metadata "$SAFETY_RAW_META" \
    --reference-image "$TOPDOWN_IMAGE" \
    --reference-alignment "$TOPVIEW_ALIGNMENT" \
    --output "$SAFETY_TOPDOWN_IMAGE" \
    --metadata-output "$SAFETY_TOPDOWN_META" \
    --fill 0 0 0
fi

if [[ "$SCAN_HEIGHTMAP" == "true" ]]; then
  echo
  echo "[scan] CARLA raycast heightmap"
  python3 scripts/scan_carla_raycast_heightmap.py \
    --host "$CARLA_HOST" \
    --port "$CARLA_PORT" \
    --timeout "$CARLA_TIMEOUT" \
    --xodr "$XODR" \
    --bounds "$bg_west" "$bg_east" "$bg_south" "$bg_north" \
    --output "$HEIGHTMAP" \
    --resolution "$HEIGHTMAP_RESOLUTION" \
    --z-top 300 \
    --z-bottom -80 \
    --workers "$RAYCAST_WORKERS" \
    --checkpoint-every 10 \
    --resume

  python3 scripts/render_heightmap_map.py \
    --heightmap "$HEIGHTMAP" \
    --output "$HEIGHTMAP_IMAGE" \
    --scale 2 \
    --mode semantic
fi

echo
echo "[editor] build mission editor from actual AirSim topview"
alignment_value=""
if [[ "$USE_TOPVIEW_ALIGNMENT" == "true" && -f "$TOPVIEW_ALIGNMENT" ]]; then
  alignment_value="$TOPVIEW_ALIGNMENT"
fi
MAP_BACKGROUND_IMAGE="$TOPDOWN_IMAGE" \
MAP_BACKGROUND_BOUNDS="$bg_west $bg_east $bg_south $bg_north" \
TOPVIEW_ALIGNMENT="$alignment_value" \
AUTO_TOPVIEW_ALIGNMENT=false \
DISPLAY_FLIP_Y="$DISPLAY_FLIP_Y" \
DISPLAY_FLIP_X="$DISPLAY_FLIP_X" \
MAP_LAYER_FLIP_X="$MAP_LAYER_FLIP_X" \
HEIGHTMAP_FLIP_X="$HEIGHTMAP_FLIP_X" \
HEIGHTMAP_FLIP_Y="$HEIGHTMAP_FLIP_Y" \
HEIGHTMAP_ROTATE_180="$HEIGHTMAP_ROTATE_180" \
MAP_ROTATE_180="$MAP_ROTATE_180" \
SCAN_HEIGHTMAP=false \
QUERY_HOME_ORIGIN=false \
HEIGHTMAP_ALIGNMENT="$HEIGHTMAP_ALIGNMENT" \
EDITOR_HTML="$EDITOR_HTML" \
bash scripts/build_mission_planning_assets.sh

echo
echo "Done."
echo "Open:"
echo "  google-chrome $EDITOR_HTML"
