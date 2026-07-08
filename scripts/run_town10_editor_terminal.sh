#!/usr/bin/env bash
# File-manager friendly wrapper for the locked Town10HD_Opt UE editor launch.
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE"

if [ -z "${AERION_TERMINAL_LAUNCHED:-}" ] && [ ! -t 0 ]; then
  if command -v gnome-terminal >/dev/null 2>&1; then
    exec gnome-terminal -- bash -lc "cd '$WORKSPACE'; AERION_TERMINAL_LAUNCHED=1 bash scripts/run_town10_editor_terminal.sh"
  elif command -v xterm >/dev/null 2>&1; then
    exec xterm -e "cd '$WORKSPACE'; AERION_TERMINAL_LAUNCHED=1 bash scripts/run_town10_editor_terminal.sh"
  fi
fi

echo "AERION Town10HD_Opt UE editor"
echo "Workspace: $WORKSPACE"
echo

export AERION_AUTOPILOT_ENABLED="${AERION_AUTOPILOT_ENABLED:-0}"
export UE_BIN="${UE_BIN:-$HOME/workspace/engines/UE5.5_carla/UnrealEngine/Engine/Binaries/Linux/UnrealEditor}"
export UE_PROJECT="${UE_PROJECT:-$HOME/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject}"
export UE_RENDER_PROFILE="${UE_RENDER_PROFILE:-town10_stable}"
export UE_QUALITY_LEVEL="${UE_QUALITY_LEVEL:-High}"

set +e
bash scripts/run_ue_blocksv2.sh
status=$?
set -e

echo
if [ "$status" -eq 0 ]; then
  echo "UE editor launch command completed."
  echo "Open Town10HD_Opt in the editor if needed, then press Play."
else
  echo "UE editor launch command failed with status $status."
fi
echo
read -r -p "Press Enter to close this terminal: " _ || true
exit "$status"
