#!/usr/bin/env bash
# AERION — UE Editor (Colosseum UE5.6) BlocksV2 프로젝트 실행 스크립트.
#
# 기본 동작: ~/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject 를
# ~/airsim/unreal-engine/Engine/Binaries/Linux/UnrealEditor 로 백그라운드 기동.
# DDC 캐시는 ~/workspace/cache/ue_ddc 로 분리.
#
# CARLA+AirSim 통합 환경 (aerion-carlaair Phase 1 빌드 후): UE_PROJECT 환경변수 또는
# 1번째 인자로 통합 .uproject 경로를 넘기면 동일 스크립트로 통합 프로젝트 실행 가능.
#
# Usage:
#   bash scripts/run_ue_blocksv2.sh
#   bash scripts/run_ue_blocksv2.sh /path/to/integrated.uproject
#   UE_PROJECT=/path/to/other.uproject UE_BIN=... bash scripts/run_ue_blocksv2.sh
#
# Note: 본 스크립트는 UE Editor 만 띄움. ▶ Play 는 사용자가 GUI 에서 직접 누름.
#       (Colosseum AirSim plugin 은 Editor PIE 모드에서만 안정 동작 — standalone
#       game 모드는 별도 빌드 패키징 필요. PIE 시작은 GUI action 만 가능.)
#
# Play 누른 후 simulation orchestrator (scripts/run_phase4_delta.sh) 실행하면
# PX4 SITL × 3 + MAVROS × 3 + Phase 4-Δ formation 시퀀스가 이어서 자동 진행.

set -euo pipefail

UE_BIN="${UE_BIN:-$HOME/airsim/unreal-engine/Engine/Binaries/Linux/UnrealEditor}"
UE_PROJECT_DEFAULT="$HOME/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject"
UE_PROJECT="${1:-${UE_PROJECT:-$UE_PROJECT_DEFAULT}}"
DDC_PATH="${DDC_PATH:-$HOME/workspace/cache/ue_ddc}"
LOG_DIR="${LOG_DIR:-$HOME/workspace/logs/ue}"

# ---------- 사전 점검 ----------
[ -x "$UE_BIN" ]      || { echo "ERROR: UE Editor 바이너리 없음: $UE_BIN" >&2; exit 1; }
[ -f "$UE_PROJECT" ]  || { echo "ERROR: .uproject 없음: $UE_PROJECT" >&2; exit 1; }
mkdir -p "$DDC_PATH" "$LOG_DIR"

# ---------- 중복 실행 차단 ----------
EXISTING_PID=$(pgrep -af 'UnrealEditor' | grep -F "$UE_PROJECT" | awk '{print $1}' | head -1 || true)
if [ -n "$EXISTING_PID" ]; then
    echo "이미 동일 .uproject 로 UE Editor 가 실행 중입니다."
    echo "  PID:     $EXISTING_PID"
    echo "  Project: $UE_PROJECT"
    echo "  중복 실행 안 함. 강제 종료: pkill -KILL -f UnrealEditor"
    exit 0
fi

# ---------- 기동 ----------
LOG="$LOG_DIR/ue_$(date +%Y%m%d_%H%M%S).log"
echo "============================================"
echo "  UE Editor 실행"
echo "  Binary:    $UE_BIN"
echo "  Project:   $UE_PROJECT"
echo "  DDC cache: $DDC_PATH"
echo "  Log:       $LOG"
echo "============================================"
echo ""
echo "Editor 로딩 후 GUI 의 ▶ Play 를 직접 누르세요. 그 후 다른 터미널에서:"
echo "  bash $HOME/workspace/projects/aerion-airsim/scripts/run_phase4_delta.sh"
echo ""

# UE 가 child process 의 DDC 위치를 인식하도록 env 로 전달.
# -log: stdout 으로 로그 출력 → tee 로 파일 + 터미널.
# -nullrhi/-game 같은 headless 옵션은 PIE 와 호환 안 됨 → editor GUI 모드.
export UE_SHAREDDATACACHE_PATH="$DDC_PATH"

nohup "$UE_BIN" "$UE_PROJECT" -log \
    > "$LOG" 2>&1 &
UE_PID=$!
disown

echo "UE Editor PID=$UE_PID"
echo "로그 tail: tail -f $LOG"
echo "종료:      kill $UE_PID  (또는 GUI File→Exit)"
echo ""
echo "TCP 4560 listen 대기 (Play 누른 후 PX4 SITL 가 connect 할 포트):"
echo "  watch -n 1 'ss -tlnp 2>/dev/null | grep -E \":456[0-9]\"'"
