#!/usr/bin/env bash
# Run the MAVSDK-only GPS mission smoke from the team-lead procedure.
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
MAVSDK_PORT="${MAVSDK_PORT:-14540}"
MAVSDK_URL="${MAVSDK_URL:-}"
MISSION_ALT="${MISSION_ALT:-10}"
MISSION_FILE="${MISSION_FILE:-}"
MISSION_WPS="${MISSION_WPS:-}"
MISSION_FORWARD_M="${MISSION_FORWARD_M:-1000}"
MISSION_INTERP_M="${MISSION_INTERP_M:-200}"
MISSION_BEARING="${MISSION_BEARING:-0}"
MISSION_SPEED="${MISSION_SPEED:-5}"

cd "$WORKSPACE"

args=(
  --port "$MAVSDK_PORT"
  --alt "$MISSION_ALT"
  --forward-m "$MISSION_FORWARD_M"
  --interp-m "$MISSION_INTERP_M"
  --bearing "$MISSION_BEARING"
  --speed "$MISSION_SPEED"
)

if [ -n "$MAVSDK_URL" ]; then
  args=(--url "$MAVSDK_URL" "${args[@]:2}")
fi

if [ -n "$MISSION_FILE" ]; then
  args+=(--mission-file "$MISSION_FILE")
fi

if [ -n "$MISSION_WPS" ]; then
  args+=(--wps "$MISSION_WPS")
fi

exec python3 scripts/mission_smoke.py "${args[@]}" "$@"
