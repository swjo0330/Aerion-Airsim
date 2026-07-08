#!/usr/bin/env bash
# File-manager friendly wrapper for the successful Town10 clicked mission runner.
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE"

if [ -z "${AERION_TERMINAL_LAUNCHED:-}" ] && [ ! -t 0 ]; then
  if command -v gnome-terminal >/dev/null 2>&1; then
    exec gnome-terminal -- bash -lc "cd '$WORKSPACE'; AERION_TERMINAL_LAUNCHED=1 bash scripts/run_clicked_mission_experiment_terminal.sh"
  elif command -v xterm >/dev/null 2>&1; then
    exec xterm -e "cd '$WORKSPACE'; AERION_TERMINAL_LAUNCHED=1 bash scripts/run_clicked_mission_experiment_terminal.sh"
  fi
fi

echo "AERION Town10 clicked mission experiment"
echo "Workspace: $WORKSPACE"
echo

set +e
bash scripts/run_clicked_mission_experiment.sh
status=$?
set -e

echo
if [ "$status" -eq 0 ]; then
  echo "Mission runner exited successfully."
else
  echo "Mission runner exited with status $status."
fi
echo
read -r -p "Press Enter to close this terminal: " _ || true
exit "$status"
