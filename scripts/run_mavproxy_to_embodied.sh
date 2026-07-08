#!/usr/bin/env bash
# AERION: PX4 SITL MAVLink → Mac(체화지능) MAVROS 로 UDP 포워드 (Tailscale).
#   모델: AirSim PC = MAVLink forwarder, Mac = MAVROS(/mavros/* 네이티브 생성).
#   ★ 제어/서비스는 이 MAVLink 경로로만. Zenoh로는 절대 제어 토픽 보내지 않음(one-writer).
#   ★ AirSim PC의 local MAVROS는 끄고 실행할 것 — 14540 포트 충돌(launch_mavros_px4_instances.sh가 점유).
#   forwarder 전용: HOME/명령 주입 안 함(순수 relay).
set -euo pipefail

MAC_TAILSCALE_IP="${MAC_TAILSCALE_IP:-100.67.87.116}"
MASTER="${MASTER:-udpin:0.0.0.0:14540}"      # PX4 SITL이 MAVLink 내보내는 곳(=기존 MAVROS bind 포트)
MAC_MAVROS_PORT="${MAC_MAVROS_PORT:-14555}"  # Mac MAVROS fcu_url 수신 포트
OUT="udpout:${MAC_TAILSCALE_IP}:${MAC_MAVROS_PORT}"
# EXTRA_OUT: 추가 MAVLink 출력(공백구분). 로컬 자율제어(pymavlink)용 — 예 "udpout:127.0.0.1:14601".
# mavproxy는 양방향 브릿지라 이 출력으로 들어온 명령도 PX4로 전달됨(one-writer: 단일 제어원만 송신).
EXTRA_OUT="${EXTRA_OUT:-}"

command -v mavproxy.py >/dev/null || { echo "ERROR: mavproxy.py 없음 (pip install MAVProxy)"; exit 1; }

# 14540 점유(=local MAVROS 살아있음) 검사
IN_PORT="$(printf '%s' "$MASTER" | sed -E 's#.*:([0-9]+)$#\1#')"
if ss -lun 2>/dev/null | grep -q ":${IN_PORT}\b"; then
  echo "WARN: UDP ${IN_PORT} 이미 점유됨 — local MAVROS(launch_mavros_px4_instances.sh)가 떠있으면 먼저 끄세요."
  echo "      (mavproxy가 PX4 MAVLink를 못 받습니다. one-writer 위반 방지.)"
fi

EXTRA_ARGS=()
for o in $EXTRA_OUT; do EXTRA_ARGS+=(--out="$o"); done

echo "[mavproxy] PX4($MASTER) → Mac MAVROS($OUT)${EXTRA_OUT:+ + extra[$EXTRA_OUT]}"
echo "  Mac 쪽: ros2 run mavros mavros_node --ros-args -p fcu_url:=\"udp://:${MAC_MAVROS_PORT}@<AIRSIM_TAILSCALE_IP>:${IN_PORT}\""
exec mavproxy.py --master="$MASTER" --out="$OUT" "${EXTRA_ARGS[@]}" --state-basedir=/tmp/mav_embodied --daemon
