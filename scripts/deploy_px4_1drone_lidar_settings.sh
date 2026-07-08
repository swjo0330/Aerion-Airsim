#!/usr/bin/env bash
# Deploy AirSim settings for the 1-drone PX4 + LiDAR CARLA validation setup.
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/workspace/projects/aerion-airsim}"
SETTINGS_SRC="${SETTINGS_SRC:-$WORKSPACE/settings/px4_1drone_lidar.json}"
SETTINGS_DST="${SETTINGS_DST:-$HOME/Documents/AirSim/settings.json}"
SPAWN_OFFSET_X="${SPAWN_OFFSET_X:-0.0}"
SPAWN_OFFSET_Y="${SPAWN_OFFSET_Y:-0.0}"
SPAWN_OFFSET_Z="${SPAWN_OFFSET_Z:-0.0}"
DRONE1_X="${DRONE1_X:-}"
DRONE1_Y="${DRONE1_Y:-}"
DRONE1_Z="${DRONE1_Z:-}"

[ -f "$SETTINGS_SRC" ] || {
    echo "ERROR: settings not found: $SETTINGS_SRC" >&2
    exit 1
}

mkdir -p "$(dirname "$SETTINGS_DST")"
cp "$SETTINGS_SRC" "$SETTINGS_DST"

SETTINGS_DST="$SETTINGS_DST" \
SPAWN_OFFSET_X="$SPAWN_OFFSET_X" SPAWN_OFFSET_Y="$SPAWN_OFFSET_Y" SPAWN_OFFSET_Z="$SPAWN_OFFSET_Z" \
DRONE1_X="$DRONE1_X" DRONE1_Y="$DRONE1_Y" DRONE1_Z="$DRONE1_Z" \
python3 - <<'PY'
import json
import os

path = os.path.expanduser(os.environ["SETTINGS_DST"])
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)

data["EnableRpc"] = True
vehicle = data.get("Vehicles", {}).get("drone1", {})
if isinstance(vehicle, dict):
    vehicle["X"] = float(vehicle.get("X", 0.0)) + float(os.environ.get("SPAWN_OFFSET_X", "0.0"))
    vehicle["Y"] = float(vehicle.get("Y", 0.0)) + float(os.environ.get("SPAWN_OFFSET_Y", "0.0"))
    vehicle["Z"] = float(vehicle.get("Z", 0.0)) + float(os.environ.get("SPAWN_OFFSET_Z", "0.0"))
    for axis in ("X", "Y", "Z"):
        value = os.environ.get(f"DRONE1_{axis}", "")
        if value != "":
            vehicle[axis] = float(value)

with open(path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2)

print("deployed:", path)
print("vehicle:", list(data.get("Vehicles", {}).keys()))
print("view:", data.get("ViewMode"))
print("sensors:", list(vehicle.get("Sensors", {}).keys()) if isinstance(vehicle, dict) else [])
PY
