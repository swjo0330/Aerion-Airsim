#!/usr/bin/env bash
# AERION Phase 4-Δ — SimpleFlight backend runner (PX4 stack 우회).
#
# 배경: Linux native + PX4 + Colosseum 환경의 settings.json Parameters 가 PX4 PARAM_SET
# 채널로 전달되지 않아 EKF2 heading invalid 가 풀리지 않음 + force-arm 도 result=1 거부.
# Phase 5 EKF2 작업 (commit 85b8e96) 의 검증은 1대 isolated console 까지였고 multi-vehicle
# arm 까지 가본 적 없음. 빠른 시연 우선 시 SimpleFlight backend 로 우회 (메모리에 검증된
# Phase 3 1→5대 확장 path; 3대 한도 안에서 안정).
#
# CARLA+AirSim 통합 환경 (aerion-carlaair Phase 1 빌드 후) 에서도 SimpleFlight backend
# 그대로 사용 가능. UE 실행 target 만 통합 .uproject 로 교체.
#
# 의존:
#   - UE Editor 살아있고 Play 직전 상태 (Editor Preferences "Use Less CPU when in
#     Background" 해제 권장)
#   - settings/sf_3drones_phase4_delta.json
#   - colcon build 완료 (없으면 자동 빌드)
#
# Usage:
#   bash scripts/run_phase4_delta_sf.sh
#
# Env override: DRONE_COUNT, DEFAULT_PATTERN, LEADER_MODE, RUN_CYCLE_DEMO, WORKSPACE,
#               ROS_SETUP, SETTINGS_SRC, FORCE_BUILD.

set -euo pipefail

# ---------- 설정 ----------
DRONE_COUNT="${DRONE_COUNT:-3}"
DEFAULT_PATTERN="${DEFAULT_PATTERN:-TRIANGLE}"
LEADER_MODE="${LEADER_MODE:-circle}"
RUN_CYCLE_DEMO="${RUN_CYCLE_DEMO:-true}"
WORKSPACE="${WORKSPACE:-$HOME/workspace/projects/aerion-airsim}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
SETTINGS_SRC="${SETTINGS_SRC:-$WORKSPACE/settings/sf_3drones_phase4_delta.json}"
SETTINGS_DST="${SETTINGS_DST:-$HOME/Documents/AirSim/settings.json}"
LOG_DIR="${LOG_DIR:-/tmp/aerion_phase4_delta_sf}"
FORCE_BUILD="${FORCE_BUILD:-false}"

# ---------- 헬퍼 ----------
log() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
warn() { printf '\n[%s] WARN: %s\n' "$(date +%H:%M:%S)" "$*" >&2; }
die() { printf '\n[%s] ERROR: %s\n' "$(date +%H:%M:%S)" "$*" >&2; exit 1; }

cleanup() {
    log "Cleanup: 시뮬레이션 프로세스 정리..."
    [ -n "${LAUNCH_PID:-}" ] && kill "$LAUNCH_PID" 2>/dev/null || true
    sleep 2
    pkill -INT  -f 'bridge_node'  2>/dev/null || true
    pkill -INT  -f 'aerion_'      2>/dev/null || true
    sleep 2
    pkill -KILL -f 'bridge_node'  2>/dev/null || true
    pkill -KILL -f 'aerion_'      2>/dev/null || true
    log "Done. UE Editor 는 Stop/Play 따로 정리."
}
trap cleanup EXIT INT TERM

# ---------- 사전 점검 ----------
log "Phase 4-Δ SimpleFlight 시뮬레이션 시작 (DRONE_COUNT=$DRONE_COUNT, PATTERN=$DEFAULT_PATTERN, LEADER=$LEADER_MODE)"
log "  Backend: SimpleFlight + AirSim API (mavros / PX4 미사용)"

[ -f "$SETTINGS_SRC" ] || die "settings 없음: $SETTINGS_SRC"
[ -f "$ROS_SETUP" ]    || die "ROS2 setup 없음: $ROS_SETUP"
(( 1 <= DRONE_COUNT && DRONE_COUNT <= 5 )) || die "DRONE_COUNT 1~5 만 지원: $DRONE_COUNT"
mkdir -p "$LOG_DIR"

# ---------- Step 0: 정리 ----------
log "[Step 0] 기존 PX4/MAVROS/bridge 잔여 정리 (SF backend 라 PX4 도 불필요)..."
pkill -KILL -f 'bin/px4'      2>/dev/null || true
pkill -KILL -f 'mavros_node'  2>/dev/null || true
pkill -KILL -f 'bridge_node'  2>/dev/null || true
pkill -KILL -f 'aerion_'      2>/dev/null || true
sleep 2

# ---------- Step 0.5: colcon build ----------
if [ ! -f "$WORKSPACE/install/setup.bash" ] || [ "$FORCE_BUILD" = "true" ]; then
    log "[Step 0.5] colcon build airsim_ros2_bridge..."
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    set -u
    (cd "$WORKSPACE" && colcon build --packages-select airsim_ros2_bridge --symlink-install) || \
        die "colcon build 실패"
fi

# ---------- Step 1: settings deploy ----------
log "[Step 1] settings.json deploy (SimpleFlight 3-drone)"
mkdir -p "$(dirname "$SETTINGS_DST")"
cp "$SETTINGS_SRC" "$SETTINGS_DST"
echo "  source: $SETTINGS_SRC"
echo "  md5:    $(md5sum "$SETTINGS_DST" | awk '{print $1}')"

# ---------- Step 2: UE Play 안내 ----------
log "[Step 2] UE Editor 에서 Stop → Play 다시 누르세요."
echo "  중요: settings.json 은 Play 시점에만 읽힘. 이전 시뮬레이션이 Play 중이면"
echo "        새 SF settings 무시되고 이전 vehicle 유지 (enableApiControl 거부)."
echo ""
echo "  체크리스트:"
echo "    1. UE Editor 에서 Stop (이전 Play 가 진행 중이었으면)"
echo "    2. Editor Preferences > Use Less CPU when in Background: 해제 (한 번만)"
echo "    3. Play 다시 누름 → 3대 drone1/2/3 spawn 까지 대기 (몇 초)"
echo "    4. 아래 Enter"
echo ""
read -r -p "  준비됐으면 Enter (timeout 없음): " _ || true
echo ""

# (Step 3: PX4 SITL 단계 없음 — SimpleFlight 는 UE 내부에서 동작)

# ---------- Step 4: ROS2 launch (Phase 4 stack — bridge + formation + leader + tf, mavros 없음) ----------
log "[Step 4] ROS2 launch — bridge×$DRONE_COUNT (airsim_direct) + formation + leader + tf"
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WORKSPACE/install/setup.bash"
set -u

LAUNCH_LOG="$LOG_DIR/launch_$(date +%Y%m%d_%H%M%S).log"
# aerion_phase4_formation.launch.py 는 bridge(airsim_direct) + formation + leader + tf 통합.
# default_pattern + drone_count 인자로 Phase 4-Δ FORMATIONS_3 자동 선택.
nohup ros2 launch airsim_ros2_bridge aerion_phase4_formation.launch.py \
    drone_count:=$DRONE_COUNT \
    default_pattern:=$DEFAULT_PATTERN \
    enable_leader_publisher:=true \
    leader_mode:=$LEADER_MODE \
    > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
disown
echo "  ros2 launch PID=$LAUNCH_PID, log=$LAUNCH_LOG"
echo "  15초 대기 (bridge 가 AirSim RPC 연결 + topic publish 시작)..."
sleep 15

# ---------- Step 5: arm + takeoff (AirSim API 직접) ----------
log "[Step 5] arm + takeoff (airsim_arm_all.py — AirSim API 직접 호출, mavros 미사용)"
ARM_SCRIPT="$WORKSPACE/airsim_ros2_bridge/scripts/airsim_arm_all.py"
if [ -f "$ARM_SCRIPT" ]; then
    python3 "$ARM_SCRIPT" --drones "$DRONE_COUNT" --altitude 5.0 --prefix drone \
        2>&1 | tee "$LOG_DIR/arm_$(date +%Y%m%d_%H%M%S).log" || \
        warn "airsim_arm_all 일부 실패 — UE Play 상태 + AirSim RPC (port 41451) 확인"
else
    die "airsim_arm_all.py 미존재: $ARM_SCRIPT"
fi
sleep 3

# ---------- Step 6: morphing-cycle 데모 ----------
if [ "$RUN_CYCLE_DEMO" = "true" ]; then
    log "[Step 6] Morphing-cycle 데모 — TRIANGLE → V3 → COLUMN → DIAMOND3 (각 10초)"
    echo "  Ctrl+C → cleanup"
    ros2 run airsim_ros2_bridge aerion_formation_demo \
        --demo morphing-cycle \
        --patterns "TRIANGLE,V3,COLUMN,DIAMOND3" \
        --hold-sec 10.0 || true
else
    log "[Step 6 skipped: RUN_CYCLE_DEMO=false] — 수동 패턴 변경:"
    echo "    ros2 topic pub --once /aerion/formation/pattern std_msgs/String '{data: V3}'"
    read -r -p "  Enter 누르면 종료/cleanup: " _ || true
fi

# trap 이 cleanup 자동
