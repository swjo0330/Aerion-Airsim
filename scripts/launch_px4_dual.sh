#!/usr/bin/env bash
# Launch 2 PX4 SITL instances for Colosseum/AirSim.
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_MODEL="${PX4_MODEL:-none_iris}"
PX4_SIM_HOSTNAME="${PX4_SIM_HOSTNAME:-localhost}"

if [ ! -f "$PX4_BIN" ]; then
    echo "ERROR: PX4 SITL not built. Run 'cd $PX4_DIR && make px4_sitl_default none_iris' first."
    exit 1
fi

echo "Starting PX4 SITL instance 0 (SysID=1, TCP:4560, sim host:$PX4_SIM_HOSTNAME)..."
cd "$PX4_BUILD_DIR"
PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME" PX4_SIM_MODEL="$PX4_MODEL" "$PX4_BIN" -i 0 -d "$PX4_BUILD_DIR/etc" >"/tmp/px4_sitl_0.log" 2>&1 &
PID0=$!
echo "  PID: $PID0, log: /tmp/px4_sitl_0.log"

sleep 2

echo "Starting PX4 SITL instance 1 (SysID=2, TCP:4561, sim host:$PX4_SIM_HOSTNAME)..."
PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME" PX4_SIM_MODEL="$PX4_MODEL" "$PX4_BIN" -i 1 -d "$PX4_BUILD_DIR/etc" >"/tmp/px4_sitl_1.log" 2>&1 &
PID1=$!
echo "  PID: $PID1, log: /tmp/px4_sitl_1.log"

echo ""
echo "Both PX4 SITL instances running."
echo "  Instance 0: PID=$PID0, TCP:4560, MAVROS FCU: udp://:14540@127.0.0.1:14580"
echo "  Instance 1: PID=$PID1, TCP:4561, MAVROS FCU: udp://:14541@127.0.0.1:14581"
echo ""
echo "Press Ctrl+C to stop both instances."

trap "kill $PID0 $PID1 2>/dev/null; echo 'Stopped.'; exit 0" SIGINT SIGTERM
wait
