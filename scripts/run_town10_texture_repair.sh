#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
UE_BIN="${UE_BIN:-$HOME/workspace/engines/UE5.5_carla/UnrealEngine/Engine/Binaries/Linux/UnrealEditor}"
UE_PROJECT="${UE_PROJECT:-$HOME/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject}"
REPAIR_SCRIPT="${REPAIR_SCRIPT:-$WORKSPACE/scripts/repair_town10hdopt_assets.py}"
LOG_DIR="${LOG_DIR:-$HOME/workspace/logs/ue}"
LOG="$LOG_DIR/town10_texture_repair_$(date +%Y%m%d_%H%M%S).log"

[ -x "$UE_BIN" ] || { echo "ERROR: UE binary not executable: $UE_BIN" >&2; exit 1; }
[ -f "$UE_PROJECT" ] || { echo "ERROR: UE project not found: $UE_PROJECT" >&2; exit 1; }
[ -f "$REPAIR_SCRIPT" ] || { echo "ERROR: repair script not found: $REPAIR_SCRIPT" >&2; exit 1; }
mkdir -p "$LOG_DIR"

if pgrep -af 'UnrealEditor' | grep -F "$UE_PROJECT" >/dev/null; then
  echo "ERROR: UnrealEditor is already running for this project." >&2
  echo "Save and close UE first, then rerun:" >&2
  echo "  bash $WORKSPACE/scripts/run_town10_texture_repair.sh" >&2
  exit 2
fi

echo "Town10HD_Opt texture/material repair"
echo "  UE_BIN:        $UE_BIN"
echo "  UE_PROJECT:    $UE_PROJECT"
echo "  REPAIR_SCRIPT: $REPAIR_SCRIPT"
echo "  LOG:           $LOG"

"$UE_BIN" "$UE_PROJECT" \
  -run=pythonscript \
  -script="$REPAIR_SCRIPT" \
  -unattended \
  -nop4 \
  -NoSound \
  -log \
  >"$LOG" 2>&1

tail -n 120 "$LOG"

if grep -En "\[town10-repair\] completed with errors|missing source asset|duplicate failed" "$LOG" >/dev/null; then
  echo "ERROR: repair completed with errors. See log: $LOG" >&2
  exit 3
fi

echo "Repair complete."
echo "Log: $LOG"
