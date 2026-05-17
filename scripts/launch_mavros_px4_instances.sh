#!/usr/bin/env bash
# Launch one MAVROS PX4 instance per drone using the AERION port convention.
set -euo pipefail

DRONE_COUNT="${DRONE_COUNT:-2}"
PX4_INSTANCE_IDS="${PX4_INSTANCE_IDS:-}"
MAVROS_NAMESPACE_PREFIX="${MAVROS_NAMESPACE_PREFIX:-mavros}"
FCU_BIND_BASE_PORT="${FCU_BIND_BASE_PORT:-14540}"
PX4_REMOTE_BASE_PORT="${PX4_REMOTE_BASE_PORT:-14580}"
FCU_REMOTE_HOST="${FCU_REMOTE_HOST:-127.0.0.1}"
PIDS=""

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

cleanup() {
    for pid in $PIDS; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}

trap cleanup INT TERM EXIT

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
    namespace="${MAVROS_NAMESPACE_PREFIX}${i}"
    bind_port=$((FCU_BIND_BASE_PORT + px4_instance))
    remote_port=$((PX4_REMOTE_BASE_PORT + px4_instance))
    system_id=$((px4_instance + 1))
    fcu_url="udp://:${bind_port}@${FCU_REMOTE_HOST}:${remote_port}"
    log_path="/tmp/mavros_drone${i}_px4.log"

    echo "Starting MAVROS for Drone${i} (${namespace}, PX4 instance ${px4_instance}, ${fcu_url}, system_id=${system_id})..."
    ros2 launch mavros px4.launch \
        fcu_url:="$fcu_url" \
        tgt_system:="$system_id" \
        namespace:="$namespace" \
        >"$log_path" 2>&1 &
    pid=$!
    PIDS="$PIDS $pid"
    echo "  PID: $pid, log: $log_path"
    sleep 2
done

echo ""
echo "MAVROS PX4 instances running for ${DRONE_COUNT} drone(s)."
echo "Press Ctrl+C to stop all instances."

wait
