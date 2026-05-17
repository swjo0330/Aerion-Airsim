#!/usr/bin/env python3
"""AERION Phase 4 — 단일 진입점 mission planner CLI.

목적:
  사용자가 한 줄로 takeoff_all → set_pattern → set_pattern → ... → land_all 시퀀스를 실행 가능.
  Phase 4 시연 + 검증 시 콘솔에서 인터랙티브하게 패턴 전환을 트리거.

사용:
  # 5대 일괄 이륙 후 LINE 포메이션 60초 유지 + DIAMOND 60초 + ARROW 60초 + 착륙
  python3 ~/airsim/airsim_ros2_bridge/scripts/aerion_mission.py \\
      --takeoff --drones 5 --altitude 5.0 \\
      --sequence "LINE:60,DIAMOND:60,ARROW:60" \\
      --land

  # 패턴 한 번만 set하고 종료 (별도 leader_publisher가 떠 있다고 가정)
  python3 ~/airsim/airsim_ros2_bridge/scripts/aerion_mission.py --set-pattern DIAMOND

  # 그냥 takeoff만
  python3 ~/airsim/airsim_ros2_bridge/scripts/aerion_mission.py --takeoff --drones 5

전제:
  - bridge 5개 + formation_node 가 떠 있음 (aerion_phase4_formation.launch.py)
  - airsim_arm_all.py가 takeoff까지 수행해 둠 (또는 --takeoff 인자로 본 스크립트가 수행)
  - leader_pose는 별도 leader_publisher 또는 외부 mission planner에서 publish

토픽 인터페이스:
  /aerion/formation/pattern  publish  (transient_local QoS 적용)
  AirSim RPC :41451          --takeoff/--land 시 직접 호출

변경 이력:
  2026-05-18 v1: 최초 작성 (Round 2A)
"""

import argparse
import subprocess
import sys
import time
from typing import List, Tuple


def _run_arm_all(drones: int, altitude: float) -> int:
    """airsim_arm_all.py 호출 (subprocess)."""
    cmd = [
        sys.executable,
        f'/home/clrobur/airsim/airsim_ros2_bridge/scripts/airsim_arm_all.py',
        '--drones', str(drones),
        '--altitude', str(altitude),
    ]
    print(f'[mission] subprocess: {" ".join(cmd)}')
    return subprocess.call(cmd)


def _run_land_all(drones: int) -> int:
    cmd = [
        sys.executable,
        f'/home/clrobur/airsim/airsim_ros2_bridge/scripts/airsim_land_all.py',
        '--drones', str(drones),
    ]
    print(f'[mission] subprocess: {" ".join(cmd)}')
    return subprocess.call(cmd)


def _publish_pattern(pattern: str):
    """std_msgs/String을 transient_local QoS로 한 번 publish.

    ros2 topic pub --once는 기본 RELIABLE이지만 transient_local은 별도 옵션.
    formation_node가 String을 그냥 RELIABLE로 받아도 동작하지만, 늦게 join한 구독자를 위해
    transient_local 권장.
    """
    cmd = [
        'ros2', 'topic', 'pub', '--once',
        '--qos-reliability', 'reliable',
        '--qos-durability', 'transient_local',
        '/aerion/formation/pattern', 'std_msgs/msg/String',
        f'{{data: "{pattern}"}}',
    ]
    print(f'[mission] pattern → {pattern}')
    return subprocess.call(cmd)


def _parse_sequence(s: str) -> List[Tuple[str, float]]:
    """LINE:60,DIAMOND:60,ARROW:60 → [('LINE',60),('DIAMOND',60),('ARROW',60)]"""
    items = []
    for token in s.split(','):
        token = token.strip()
        if not token:
            continue
        if ':' not in token:
            raise ValueError(f'잘못된 sequence 항목: {token!r}. 형식: PATTERN:SECONDS')
        name, dur = token.split(':', 1)
        items.append((name.strip().upper(), float(dur.strip())))
    return items


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--drones', type=int, default=5)
    p.add_argument('--altitude', type=float, default=5.0)
    p.add_argument('--takeoff', action='store_true', help='시작 시 airsim_arm_all 호출')
    p.add_argument('--land', action='store_true', help='끝에 airsim_land_all 호출')
    p.add_argument('--set-pattern', dest='set_pattern',
                   help='단일 패턴 publish 후 종료 (sequence 무시)')
    p.add_argument('--sequence',
                   help='쉼표로 구분된 PATTERN:SECONDS 목록. 예: LINE:60,DIAMOND:60,ARROW:60')
    p.add_argument('--initial-delay', type=float, default=2.0,
                   help='takeoff 후 첫 패턴 publish까지 대기 시간(s)')
    args = p.parse_args()

    # ----- takeoff -----
    if args.takeoff:
        rc = _run_arm_all(args.drones, args.altitude)
        if rc != 0:
            print(f'[mission] arm_all 실패 (rc={rc}). 종료.', file=sys.stderr)
            sys.exit(rc)

    # ----- 단일 패턴 -----
    if args.set_pattern:
        rc = _publish_pattern(args.set_pattern.upper())
        if rc != 0:
            print(f'[mission] pattern publish 실패 (rc={rc}).', file=sys.stderr)
        sys.exit(rc)

    # ----- 시퀀스 -----
    if args.sequence:
        seq = _parse_sequence(args.sequence)
        print(f'[mission] 초기 대기 {args.initial_delay:.1f}s...')
        time.sleep(args.initial_delay)
        for pattern, dur in seq:
            _publish_pattern(pattern)
            print(f'[mission] {pattern} 유지 {dur:.0f}s')
            time.sleep(dur)

    # ----- land -----
    if args.land:
        _run_land_all(args.drones)

    print('[mission] done.')


if __name__ == '__main__':
    main()
