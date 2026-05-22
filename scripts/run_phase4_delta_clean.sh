#!/usr/bin/env bash
# AERION Phase 4-Δ — clean slate runner (ROS2 / mavros / formation_node 우회).
#
# 단일 Python 스크립트 (phase4_delta_simple.py) 가 AirSim API 로 직접 3대 제어.
# 매 패턴 전환에서 .join() 으로 도달 대기 → 명령 충돌 없이 안정.
#
# Usage:
#   bash scripts/run_phase4_delta_clean.sh
#
# Env override: DRONE_COUNT, PATTERNS, HOLD_SEC, VELOCITY, CYCLES, WORKSPACE.

set -euo pipefail

DRONE_COUNT="${DRONE_COUNT:-3}"
PATTERNS="${PATTERNS:-TRIANGLE,V3,COLUMN,DIAMOND3}"
HOLD_SEC="${HOLD_SEC:-8.0}"
VELOCITY="${VELOCITY:-2.0}"
CYCLES="${CYCLES:-0}"
WORKSPACE="${WORKSPACE:-$HOME/workspace/projects/aerion-airsim}"
SETTINGS_SRC="${SETTINGS_SRC:-$WORKSPACE/settings/sf_3drones_phase4_delta.json}"
SETTINGS_DST="${SETTINGS_DST:-$HOME/Documents/AirSim/settings.json}"

log() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '\n[%s] ERROR: %s\n' "$(date +%H:%M:%S)" "$*" >&2; exit 1; }

[ -f "$WORKSPACE/scripts/phase4_delta_simple.py" ] || die "phase4_delta_simple.py 없음"
[ -f "$SETTINGS_SRC" ] || die "settings 없음: $SETTINGS_SRC"

# Step 0: 기존 ROS2 stack 잔여 정리 (별 path 사용하므로 안전)
log "[Step 0] 기존 PX4/MAVROS/bridge/ros2 launch 잔여 정리"
pkill -KILL -f 'bin/px4'      2>/dev/null || true
pkill -KILL -f 'mavros_node'  2>/dev/null || true
pkill -KILL -f 'bridge_node'  2>/dev/null || true
pkill -KILL -f 'aerion_'      2>/dev/null || true
sleep 1

# Step 1: settings.json deploy
log "[Step 1] settings.json deploy (SimpleFlight 3-drone)"
mkdir -p "$(dirname "$SETTINGS_DST")"
cp "$SETTINGS_SRC" "$SETTINGS_DST"
echo "  md5: $(md5sum "$SETTINGS_DST" | awk '{print $1}')"

# Step 2: UE Play 안내
log "[Step 2] UE Editor 에서 Stop → Play 다시 누르세요 (settings 적용)"
echo "  - settings.json 은 Play 시점에만 읽힘. 이전 시뮬 진행 중이면 Stop 필수."
echo "  - Editor Preferences > Use Less CPU when in Background: 해제 권장 (한 번)"
echo "  - 3대 drone1/2/3 spawn 까지 대기 (몇 초)"
read -r -p "  준비됐으면 Enter (timeout 없음): " _ || true

# Step 3: clean Python runner
log "[Step 3] phase4_delta_simple.py 실행 (Ctrl+C 종료 시 land+disarm)"
echo "  drones=$DRONE_COUNT  patterns=$PATTERNS  hold=$HOLD_SEC  vel=$VELOCITY  cycles=$CYCLES"
echo ""
exec python3 "$WORKSPACE/scripts/phase4_delta_simple.py" \
    --drones "$DRONE_COUNT" \
    --patterns "$PATTERNS" \
    --hold-sec "$HOLD_SEC" \
    --velocity "$VELOCITY" \
    --cycles "$CYCLES"
