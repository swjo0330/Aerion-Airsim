#!/usr/bin/env bash
# Launch one PX4 SITL instance per AirSim drone.
set -euo pipefail

DRONE_COUNT="${DRONE_COUNT:-2}"
PX4_INSTANCE_IDS="${PX4_INSTANCE_IDS:-}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_MODEL="${PX4_MODEL:-none_iris}"
PX4_SIM_HOSTNAME="${PX4_SIM_HOSTNAME:-localhost}"
PIDS=""

if [ ! -f "$PX4_BIN" ]; then
    echo "ERROR: PX4 SITL not built. Run 'cd $PX4_DIR && make px4_sitl_default none_iris' first." >&2
    exit 1
fi

cleanup() {
    for pid in $PIDS; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}

trap cleanup INT TERM EXIT

cd "$PX4_BUILD_DIR"

if [ -n "$PX4_INSTANCE_IDS" ]; then
    read -r -a instance_ids <<<"$PX4_INSTANCE_IDS"
else
    instance_ids=()
    for ((i = 0; i < DRONE_COUNT; i++)); do
        instance_ids+=("$i")
    done
fi

if [ "${#instance_ids[@]}" -ne "$DRONE_COUNT" ]; then
    echo "ERROR: PX4_INSTANCE_IDS must contain exactly DRONE_COUNT entries." >&2
    exit 1
fi

for ((i = 0; i < DRONE_COUNT; i++)); do
    px4_instance="${instance_ids[$i]}"
    tcp_port=$((4560 + px4_instance))
    mavros_bind_port=$((14540 + px4_instance))
    px4_remote_port=$((14580 + px4_instance))
    system_id=$((px4_instance + 1))
    log_path="/tmp/px4_sitl_${px4_instance}.log"

    echo "Starting Drone${i} with PX4 SITL instance ${px4_instance} (SysID=${system_id}, TCP:${tcp_port}, sim host:${PX4_SIM_HOSTNAME})..."
    PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME" PX4_SIM_MODEL="$PX4_MODEL" "$PX4_BIN" -i "$px4_instance" -d "$PX4_BUILD_DIR/etc" >"$log_path" 2>&1 &
    pid=$!
    PIDS="$PIDS $pid"
    echo "  PID: $pid, log: $log_path"
    echo "  MAVROS FCU: udp://:${mavros_bind_port}@127.0.0.1:${px4_remote_port}"
    sleep 2
done

echo ""
echo "PX4 SITL instances running for ${DRONE_COUNT} drone(s)."
echo "Press Ctrl+C to stop all instances."

wait
