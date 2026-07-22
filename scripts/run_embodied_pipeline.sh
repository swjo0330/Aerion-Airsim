#!/usr/bin/env bash
# AERION — 언리얼부터 embodied 링크까지 전체 실행 파이프라인 오케스트레이터.
#
# 무엇을 하나 (순서):
#   1. (옵션) UE Editor 기동 — Town10(기본) 또는 BlocksV2. 이미 떠 있으면 skip.
#   2. ▶ Play 대기 — AirSim RPC 41451 + PX4 lockstep 4560 LISTEN 될 때까지 폴링(수동 Play 안내).
#   3. 관측 채널 — test_embodied_link.sh 의 px4 → zenoh(Router 재시도) → bridge.
#   4. 제어/비행데이터 채널 — mavproxy (★ MAVPROXY_IN_PORT=14550, PX4 Normal/GCS; 14540은 AirSim 전용).
#   5. 상태 요약 + Mac 측 실행/검증 명령 출력.
#
# ⚠️ UE ▶ Play 는 GUI 수동(AirSim PIE 전용). 이 스크립트는 Play 를 대신 누를 수 없어 "대기"만 한다.
# ⚠️ Mac zenoh Router(:7447)가 먼저 떠 있어야 zenoh 단계 성공. 안 떠 있으면 재시도 루프에서 대기.
#
# Usage:
#   bash scripts/run_embodied_pipeline.sh                 # Town10 + 전체
#   UE_ENV=blocksv2 bash scripts/run_embodied_pipeline.sh # BlocksV2 (플러그인 재빌드 필요할 수 있음)
#   UE_ENV=none     bash scripts/run_embodied_pipeline.sh # UE 이미 실행중 — 링크만
#   SKIP_MAVPROXY=1 bash scripts/run_embodied_pipeline.sh # 관측만(제어 채널 생략)
#
# 주요 env:
#   UE_ENV=town10|blocksv2|none  (기본 town10)
#   MAVPROXY_IN_PORT=14550       (기본 14550 — 절대 14540 쓰지 말 것)
#   PLAY_WAIT_TIMEOUT=300        (Play 폴링 최대 초)
#   ROUTER_WAIT_TIMEOUT=120      (Mac Router 7447 재시도 최대 초)
#   MAC_TAILSCALE_IP=100.67.87.116
#   PX4_DIR=/home/clrobur/airsim/PX4-Autopilot
#   그 외 test_embodied_link.sh 의 env(CAMERA_FPS, RANGE_MODE, TOPIC_NAMESPACE ...) 그대로 전달됨.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LINK="$ROOT/scripts/test_embodied_link.sh"

UE_ENV="${UE_ENV:-town10}"
export MAVPROXY_IN_PORT="${MAVPROXY_IN_PORT:-14550}"   # ★ PX4 Normal/GCS. 14540=AirSim onboard(충돌)
export PX4_DIR="${PX4_DIR:-/home/clrobur/airsim/PX4-Autopilot}"
export MAC_TAILSCALE_IP="${MAC_TAILSCALE_IP:-100.67.87.116}"
PLAY_WAIT_TIMEOUT="${PLAY_WAIT_TIMEOUT:-300}"
ROUTER_WAIT_TIMEOUT="${ROUTER_WAIT_TIMEOUT:-120}"
SKIP_MAVPROXY="${SKIP_MAVPROXY:-0}"

say(){ printf '\033[1;36m[pipeline]\033[0m %s\n' "$*"; }
warn(){ printf '\033[1;33m[pipeline] ⚠ %s\033[0m\n' "$*"; }
die(){ printf '\033[1;31m[pipeline] ✗ %s\033[0m\n' "$*" >&2; exit 1; }

[ -x "$LINK" ] || die "test_embodied_link.sh 없음: $LINK"

port_listen(){ ss -tln 2>/dev/null | grep -q ":$1\b"; }
router_up(){ timeout 3 bash -c "cat < /dev/null > /dev/tcp/${MAC_TAILSCALE_IP}/7447" 2>/dev/null; }

# ── 1. UE 기동 ────────────────────────────────────────────────
case "$UE_ENV" in
  town10)   UE_LAUNCHER="$ROOT/scripts/run_ue_carla_town10.sh"; UE_TAG="Town10(CarlaUnreal)" ;;
  blocksv2) UE_LAUNCHER="$ROOT/scripts/run_ue_blocksv2.sh";     UE_TAG="BlocksV2" ;;
  none)     UE_LAUNCHER=""; UE_TAG="(기존 실행중 가정)" ;;
  *) die "UE_ENV 값 오류: $UE_ENV (town10|blocksv2|none)";;
esac

if [ "$UE_ENV" != "none" ]; then
  if port_listen 41451; then
    say "AirSim RPC 41451 이미 LISTEN — UE 재기동 skip"
  else
    say "UE Editor 기동: $UE_TAG"
    bash "$UE_LAUNCHER" || die "UE 런처 실패"
  fi
fi

# ── 2. ▶ Play 대기 (41451 + 4560) ─────────────────────────────
say "▶ Play 대기 — UE GUI 에서 Play 누르세요 (AirSim PIE). 41451 + 4560 LISTEN 폴링 (최대 ${PLAY_WAIT_TIMEOUT}s)..."
waited=0
until port_listen 41451 && port_listen 4560; do
  sleep 3; waited=$((waited+3))
  [ "$waited" -ge "$PLAY_WAIT_TIMEOUT" ] && die "Play 타임아웃 — 41451/4560 미확인. UE Play 확인 후 재실행"
  [ $((waited % 15)) -eq 0 ] && say "  ...대기중 ${waited}s (41451=$(port_listen 41451 && echo up || echo -), 4560=$(port_listen 4560 && echo up || echo -))"
done
say "✓ AirSim RPC 41451 + PX4 lockstep 4560 LISTEN — drone spawn 확인"

# ── 3. 관측 채널: px4 → zenoh(Router 재시도) → bridge ──────────
say "── PX4 SITL 기동 ──"; "$LINK" px4 || warn "px4 단계 경고(이미 실행중일 수 있음)"

say "── Mac Router(7447) 도달 확인 ──"
rwaited=0
until router_up; do
  sleep 3; rwaited=$((rwaited+3))
  if [ "$rwaited" -ge "$ROUTER_WAIT_TIMEOUT" ]; then
    warn "Mac Router 7447 ${ROUTER_WAIT_TIMEOUT}s 내 미도달 — zenoh 생략하고 진행(관측 Mac 미전달). Mac Router 띄운 뒤: '$LINK zenoh && $LINK bridge'"
    ZENOH_OK=0; break
  fi
  [ $((rwaited % 15)) -eq 0 ] && say "  ...Mac Router 대기중 ${rwaited}s (Mac에서 zenohd -l tcp/0.0.0.0:7447)"
done
if router_up; then ZENOH_OK=1; say "✓ Mac Router 7447 도달"; fi

if [ "${ZENOH_OK:-1}" = "1" ]; then
  say "── zenoh client 기동 ──"; "$LINK" zenoh || warn "zenoh 단계 경고"
fi
say "── bridge_node 기동 (camera/range) ──"; "$LINK" bridge || warn "bridge 단계 경고"

# ── 4. 제어/비행데이터: mavproxy (14550) ──────────────────────
if [ "$SKIP_MAVPROXY" = "1" ]; then
  say "SKIP_MAVPROXY=1 — 제어 채널 생략"
else
  say "── mavproxy (PX4 Normal/GCS ${MAVPROXY_IN_PORT} → Mac ${MAC_TAILSCALE_IP}:14555) ──"
  "$LINK" mavproxy || warn "mavproxy 단계 경고 (MAVPROXY_IN_PORT=$MAVPROXY_IN_PORT 확인)"
fi

# ── 5. 요약 + Mac 명령 ───────────────────────────────────────
say "═══ 파이프라인 기동 완료 ═══"
"$LINK" status 2>/dev/null || true
cat <<EOF

[Mac 측 실행]
  # (Router는 이미 가동 가정) MAVROS:
  ROS_DOMAIN_ID=0 ros2 run mavros mavros_node --ros-args \\
    -p fcu_url:="udp://:14555@100.120.219.68:${MAVPROXY_IN_PORT}" -p tgt_system:=1 -p tgt_component:=1
[Mac 측 검증]
  ROS_DOMAIN_ID=0 ros2 topic hz   /camera/image/compressed        # ~5Hz
  ROS_DOMAIN_ID=0 ros2 topic echo --once /range/front/points  sensor_msgs/msg/PointCloud2
  ROS_DOMAIN_ID=0 ros2 topic echo --once /mavros/state            # connected: true
[종료] $LINK stop        (STOP_PX4=false 로 PX4/UE 보존)
EOF
