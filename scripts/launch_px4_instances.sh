#!/usr/bin/env bash
# Launch one PX4 SITL instance per AirSim drone.
# v2 (2026-05-21): instance별 별도 cwd (/tmp/px4_inst_$i) — etc symlink 충돌 회피.
#                  BACKGROUND_MODE 옵션 추가 — wait/trap 비활성 시 detach.
set -euo pipefail

DRONE_COUNT="${DRONE_COUNT:-2}"
PX4_INSTANCE_IDS="${PX4_INSTANCE_IDS:-}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_MODEL="${PX4_MODEL:-none_iris}"
PX4_SIM_HOSTNAME="${PX4_SIM_HOSTNAME:-localhost}"
BACKGROUND_MODE="${BACKGROUND_MODE:-false}"  # true면 trap/wait 안 함 → detached
PIDS=""

if [ ! -f "$PX4_BIN" ]; then
    echo "ERROR: PX4 SITL not built. Run 'cd $PX4_DIR && make px4_sitl_default none_iris' first." >&2
    exit 1
fi

if [ "$BACKGROUND_MODE" != "true" ]; then
    cleanup() {
        for pid in $PIDS; do
            kill "$pid" 2>/dev/null || true
        done
        wait 2>/dev/null || true
    }
    trap cleanup INT TERM EXIT
fi

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
    mavros_bind_port=$((14550 + px4_instance))
    px4_remote_port=$((18570 + px4_instance))
    system_id=$((px4_instance + 1))
    log_path="/tmp/px4_sitl_${px4_instance}.log"

    # v2: instance별 별도 cwd. PX4가 자기 cwd에 ./etc symlink/log/dataman 등 생성하므로 충돌 회피 필수.
    instance_cwd="/tmp/px4_inst_${px4_instance}"
    rm -rf "$instance_cwd"
    mkdir -p "$instance_cwd"

    echo "Starting Drone${i} with PX4 SITL instance ${px4_instance} (SysID=${system_id}, TCP:${tcp_port}, sim host:${PX4_SIM_HOSTNAME}, cwd:${instance_cwd})..."
    if [ "$BACKGROUND_MODE" = "true" ]; then
        (
            cd "$instance_cwd"
            PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME" PX4_SIM_MODEL="$PX4_MODEL" \
                nohup "$PX4_BIN" -i "$px4_instance" -d "$PX4_BUILD_DIR/etc" \
                >"$log_path" 2>&1 &
            echo $! > "$instance_cwd/px4.pid"
        )
        pid=$(cat "$instance_cwd/px4.pid")
    else
        (
            cd "$instance_cwd"
            exec env PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME" PX4_SIM_MODEL="$PX4_MODEL" \
                "$PX4_BIN" -i "$px4_instance" -d "$PX4_BUILD_DIR/etc"
        ) >"$log_path" 2>&1 &
        pid=$!
        echo "$pid" > "$instance_cwd/px4.pid"
    fi
    PIDS="$PIDS $pid"
    echo "  PID: $pid, log: $log_path"
    echo "  MAVROS FCU: udp://:${mavros_bind_port}@127.0.0.1:${px4_remote_port}"
    sleep 2
done

echo ""
echo "PX4 SITL instances running for ${DRONE_COUNT} drone(s)."

if [ "$BACKGROUND_MODE" = "true" ]; then
    echo "BACKGROUND_MODE=true → detached. PIDs: $PIDS"
    echo "Stop: pkill -KILL -f '/px4 -i'"
    exit 0
fi

echo "Press Ctrl+C to stop all instances."
wait
