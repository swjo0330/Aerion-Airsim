#!/usr/bin/env bash
# Start CARLA Traffic Manager actors for the current Play session.
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/workspace/projects/aerion-airsim}"
CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2000}"
CARLA_TM_PORT="${CARLA_TM_PORT:-8000}"
CARLA_TRAFFIC_VEHICLES="${CARLA_TRAFFIC_VEHICLES:-20}"
CARLA_TRAFFIC_WALKERS="${CARLA_TRAFFIC_WALKERS:-20}"
CARLA_TRAFFIC_SPEED_DIFF="${CARLA_TRAFFIC_SPEED_DIFF:--15}"
CARLA_TRAFFIC_SEED="${CARLA_TRAFFIC_SEED:-42}"
CARLA_TRAFFIC_KEEP_EXISTING_ONLY="${CARLA_TRAFFIC_KEEP_EXISTING_ONLY:-0}"
CARLA_TRAFFIC_NO_CLEANUP="${CARLA_TRAFFIC_NO_CLEANUP:-0}"

args=(
  --host "$CARLA_HOST"
  --port "$CARLA_PORT"
  --tm-port "$CARLA_TM_PORT"
  --vehicles "$CARLA_TRAFFIC_VEHICLES"
  --walkers "$CARLA_TRAFFIC_WALKERS"
  --speed-diff "$CARLA_TRAFFIC_SPEED_DIFF"
  --seed "$CARLA_TRAFFIC_SEED"
)

if [[ "$CARLA_TRAFFIC_KEEP_EXISTING_ONLY" == "1" || "$CARLA_TRAFFIC_KEEP_EXISTING_ONLY" == "true" ]]; then
  args+=(--keep-existing-only)
fi

if [[ "$CARLA_TRAFFIC_NO_CLEANUP" == "1" || "$CARLA_TRAFFIC_NO_CLEANUP" == "true" ]]; then
  args+=(--no-cleanup)
fi

python3 "$WORKSPACE/scripts/run_carla_dynamic_traffic.py" "${args[@]}"
