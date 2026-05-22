#!/usr/bin/env bash
# AERION Phase 4-Δ — 3대 포메이션 시뮬레이션 통합 실행 스크립트.
#
# 환경: AirSim Colosseum (UE5.6) + PX4 SITL × 3 + MAVROS × 3 + Phase 4-Δ formation_node.
#
# CARLA+AirSim 동적 통합은 aerion-carlaair Phase 1 빌드 후 가능. 본 스크립트는 AirSim-only
# baseline 에서 동작 — Phase 1 빌드 완료 시 Step 2 의 UE 실행을 CARLA fork UE5.5 + 통합
# 빌드 프로젝트로 교체하면 동일 시퀀스로 CARLA+AirSim 환경 시연 가능.
#
# 의존:
#   - ~/airsim/PX4-Autopilot (빌드 완료)
#   - ~/airsim/scripts/launch_px4_instances.sh
#   - settings/px4_3drones_phase4_delta.json (본 repo)
#   - ROS2 Humble + airsim_ros2_bridge package 빌드됨 (colcon)
#   - UE Editor 가 별 터미널에서 살아있고 Play 누른 상태
#
# Usage:
#   bash scripts/run_phase4_delta.sh
#
# Override 환경변수:
#   DRONE_COUNT          : 기본 3 (1~5)
#   DEFAULT_PATTERN      : 기본 TRIANGLE (TRIANGLE|V3|COLUMN|DIAMOND3)
#   LEADER_MODE          : 기본 circle (static|circle|line)
#   RUN_CYCLE_DEMO       : 기본 true (false 면 Step 6 skip)
#   PX4_DIR              : 기본 ~/airsim/PX4-Autopilot
#   WORKSPACE            : 기본 ~/workspace/projects/aerion-airsim
#   ROS_SETUP            : 기본 /opt/ros/humble/setup.bash
#
# Cleanup: Ctrl+C → trap 에서 PX4/MAVROS/bridge KILL.

set -euo pipefail

# ---------- 설정 ----------
DRONE_COUNT="${DRONE_COUNT:-3}"
DEFAULT_PATTERN="${DEFAULT_PATTERN:-TRIANGLE}"
LEADER_MODE="${LEADER_MODE:-circle}"
RUN_CYCLE_DEMO="${RUN_CYCLE_DEMO:-true}"
PX4_DIR="${PX4_DIR:-$HOME/airsim/PX4-Autopilot}"
WORKSPACE="${WORKSPACE:-$HOME/workspace/projects/aerion-airsim}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
LAUNCH_PX4_SH="${LAUNCH_PX4_SH:-$WORKSPACE/scripts/launch_px4_instances.sh}"
SETTINGS_SRC="${SETTINGS_SRC:-$WORKSPACE/settings/px4_3drones_phase4_delta.json}"
SETTINGS_DST="${SETTINGS_DST:-$HOME/Documents/AirSim/settings.json}"
LOG_DIR="${LOG_DIR:-/tmp/aerion_phase4_delta}"

# ---------- 헬퍼 ----------
log() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
warn() { printf '\n[%s] WARN: %s\n' "$(date +%H:%M:%S)" "$*" >&2; }
die() { printf '\n[%s] ERROR: %s\n' "$(date +%H:%M:%S)" "$*" >&2; exit 1; }

cleanup() {
    log "Cleanup: 모든 시뮬레이션 프로세스 정리..."
    [ -n "${LAUNCH_PID:-}" ] && kill "$LAUNCH_PID" 2>/dev/null || true
    sleep 2
    pkill -INT  -f 'mavros_node'  2>/dev/null || true
    pkill -INT  -f 'bridge_node'  2>/dev/null || true
    pkill -INT  -f 'aerion_'      2>/dev/null || true
    sleep 2
    pkill -KILL -f 'bin/px4'      2>/dev/null || true
    pkill -KILL -f 'mavros_node'  2>/dev/null || true
    pkill -KILL -f 'bridge_node'  2>/dev/null || true
    pkill -KILL -f 'aerion_'      2>/dev/null || true
    log "Done. UE Editor 는 Stop/Play 따로 정리 필요."
}
trap cleanup EXIT INT TERM

# ---------- 사전 점검 ----------
log "Phase 4-Δ 시뮬레이션 시작 (DRONE_COUNT=$DRONE_COUNT, PATTERN=$DEFAULT_PATTERN, LEADER=$LEADER_MODE)"

[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] || \
    die "PX4 SITL 빌드 없음 — $PX4_DIR 에서 'make px4_sitl_default none_iris' 필요"
[ -f "$LAUNCH_PX4_SH" ] || die "PX4 launcher 없음: $LAUNCH_PX4_SH"
[ -f "$SETTINGS_SRC" ] || die "settings 없음: $SETTINGS_SRC"
[ -f "$ROS_SETUP" ] || die "ROS2 setup 없음: $ROS_SETUP"
(( 1 <= DRONE_COUNT && DRONE_COUNT <= 5 )) || die "DRONE_COUNT 1~5 만 지원: $DRONE_COUNT"

mkdir -p "$LOG_DIR"

# ---------- Step 0: 사전 정리 ----------
log "[Step 0] 기존 PX4/MAVROS/bridge 잔여 프로세스 정리..."
pkill -KILL -f 'bin/px4'      2>/dev/null || true
pkill -KILL -f 'mavros_node'  2>/dev/null || true
pkill -KILL -f 'bridge_node'  2>/dev/null || true
sleep 2

# ---------- Step 1: settings.json deploy ----------
log "[Step 1] settings.json deploy"
mkdir -p "$(dirname "$SETTINGS_DST")"
cp "$SETTINGS_SRC" "$SETTINGS_DST"
echo "  source: $SETTINGS_SRC"
echo "  target: $SETTINGS_DST"
echo "  md5:    $(md5sum "$SETTINGS_DST" | awk '{print $1}')"

# ---------- Step 2: UE Editor Play 안내 ----------
log "[Step 2] UE Editor 가 살아있고 Play 누른 상태인지 확인하세요."
echo "  - AirSim Colosseum: ~/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject"
echo "  - CARLA 통합 환경 (Phase 1 빌드 후): 통합 프로젝트 + CARLA fork UE5.5"
echo "  TCP 4560~$((4560 + DRONE_COUNT - 1)) listen 상태 확인:"
echo "    ss -tlnp 2>/dev/null | grep -E ':456[0-$((DRONE_COUNT - 1))]'"
read -r -p "  Play 누르고 Enter (또는 5초 타임아웃 자동 진행): " -t 5 _ || true
echo ""

# ---------- Step 3: PX4 SITL × N 백그라운드 기동 ----------
log "[Step 3] PX4 SITL × $DRONE_COUNT 백그라운드 기동"
PX4_DIR="$PX4_DIR" DRONE_COUNT="$DRONE_COUNT" BACKGROUND_MODE=true \
    bash "$LAUNCH_PX4_SH"
sleep 8
PX4_PIDS=$(pgrep -af 'bin/px4' | head -"$DRONE_COUNT" | awk '{print $1}' | tr '\n' ' ')
[ -n "$PX4_PIDS" ] || warn "PX4 SITL 프로세스 안 보임. UE Play 안 했거나 TCP 4560 listen 안 됨."
echo "  PX4 PIDs: $PX4_PIDS"
echo "  PX4 logs: /tmp/px4_sitl_{0..$((DRONE_COUNT - 1))}.log"
# preflight 메시지 일부 미리 보기 (sensor 정상 동작 신호 — Phase 5 EKF2 fix 이후 안정)
sleep 5
for n in $(seq 0 $((DRONE_COUNT - 1))); do
    [ -f "/tmp/px4_sitl_${n}.log" ] && {
        echo "  --- /tmp/px4_sitl_${n}.log 끝 5줄 ---"
        tail -5 "/tmp/px4_sitl_${n}.log" | sed 's/^/    /'
    }
done

# ---------- Step 4: ROS2 launch (Phase 5 stack — MAVROS + bridge + formation + leader + tf) ----------
log "[Step 4] ROS2 launch — MAVROS×$DRONE_COUNT + bridge×$DRONE_COUNT + formation + leader + tf"
# ROS2 setup.bash 는 unbound vars (AMENT_TRACE_SETUP_FILES 등) 를 참조하므로
# 본 스크립트의 set -u 를 일시 해제하고 source 후 복원 (표준 ROS2 패턴).
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
[ -f "$WORKSPACE/install/setup.bash" ] && {
    # shellcheck disable=SC1090
    source "$WORKSPACE/install/setup.bash"
}
set -u

LAUNCH_LOG="$LOG_DIR/launch_$(date +%Y%m%d_%H%M%S).log"
# aerion_phase5_px4.launch.py 는 PX4 외부 가정 — 정확히 우리 시나리오.
# default_pattern=TRIANGLE → formation_node 가 FORMATIONS_3 자동 선택 (drone_count != 5).
nohup ros2 launch airsim_ros2_bridge aerion_phase5_px4.launch.py \
    drone_count:=$DRONE_COUNT \
    default_pattern:=$DEFAULT_PATTERN \
    enable_leader_publisher:=true \
    leader_mode:=$LEADER_MODE \
    > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
disown
echo "  ros2 launch PID=$LAUNCH_PID, log=$LAUNCH_LOG"
echo "  20초 대기 (mavros handshake + bridge ready)..."
sleep 20

# mavros connected 확인
echo "  mavros state per drone:"
for n in $(seq 1 "$DRONE_COUNT"); do
    state=$(timeout 3 ros2 topic echo "/drone${n}/mavros/state" --once 2>/dev/null \
            | grep -E 'connected|system_status' | head -2 || echo "    (topic timeout)")
    echo "    drone${n}:"
    echo "$state" | sed 's/^/      /'
done

# ---------- Step 5: arm + OFFBOARD + takeoff ----------
log "[Step 5] arm + OFFBOARD + takeoff"
ARM_SCRIPT="$WORKSPACE/airsim_ros2_bridge/scripts/mavros_arm_all.py"
if [ -f "$ARM_SCRIPT" ]; then
    python3 "$ARM_SCRIPT" --count "$DRONE_COUNT" --takeoff-alt 5.0 || \
        warn "mavros_arm_all 일부 실패 — log 확인"
else
    warn "mavros_arm_all.py 미존재. 수동 arm 명령:"
    for n in $(seq 1 "$DRONE_COUNT"); do
        echo "    ros2 service call /drone${n}/mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'"
        echo "    ros2 service call /drone${n}/mavros/set_mode    mavros_msgs/srv/SetMode    '{custom_mode: OFFBOARD}'"
    done
fi
sleep 5

# ---------- Step 6: 포메이션 morphing-cycle 데모 ----------
if [ "$RUN_CYCLE_DEMO" = "true" ]; then
    log "[Step 6] Morphing-cycle 데모 — TRIANGLE → V3 → COLUMN → DIAMOND3 (각 10초)"
    echo "  Ctrl+C → Step 7 cleanup 진행"
    ros2 run airsim_ros2_bridge aerion_formation_demo \
        --demo morphing-cycle \
        --patterns "TRIANGLE,V3,COLUMN,DIAMOND3" \
        --hold-sec 10.0 || true
else
    log "[Step 6 skipped: RUN_CYCLE_DEMO=false]"
    log "수동 패턴 변경 예시:"
    echo "    ros2 topic pub --once /aerion/formation/pattern std_msgs/String '{data: V3}'"
    echo "    ros2 topic echo /aerion/formation/morph_progress"
    echo "    (Ctrl+C 종료)"
    # 무한 대기 — 사용자가 다른 터미널에서 작업
    read -r -p "  Enter 누르면 종료/cleanup 진행: " _ || true
fi

# cleanup trap 이 자동 처리
