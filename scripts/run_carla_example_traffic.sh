#!/usr/bin/env bash
# Run CARLA's stock generate_traffic.py example against the current UE Play session.
set -euo pipefail

CARLA_ROOT="${CARLA_ROOT:-$HOME/workspace/engines/CarlaUE5}"
CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2000}"
CARLA_TM_PORT="${CARLA_TM_PORT:-8000}"
CARLA_TRAFFIC_VEHICLES="${CARLA_TRAFFIC_VEHICLES:-12}"
CARLA_TRAFFIC_WALKERS="${CARLA_TRAFFIC_WALKERS:-8}"
CARLA_TRAFFIC_SEED="${CARLA_TRAFFIC_SEED:-42}"

EXAMPLE="$CARLA_ROOT/PythonAPI/examples/generate_traffic.py"
[ -f "$EXAMPLE" ] || {
  echo "CARLA generate_traffic.py not found: $EXAMPLE" >&2
  exit 1
}

python3 "$EXAMPLE" \
  --host "$CARLA_HOST" \
  --port "$CARLA_PORT" \
  --tm-port "$CARLA_TM_PORT" \
  --number-of-vehicles "$CARLA_TRAFFIC_VEHICLES" \
  --number-of-walkers "$CARLA_TRAFFIC_WALKERS" \
  --seed "$CARLA_TRAFFIC_SEED" \
  --seedw "$CARLA_TRAFFIC_SEED" \
  --safe \
  --asynch
