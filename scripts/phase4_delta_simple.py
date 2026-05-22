#!/usr/bin/env python3
"""AERION Phase 4-Δ — 3-drone formation runner (AirSim 공식 패턴).

설계 원칙 (이전 누적 시도들의 실패에서 학습):
  - AirSim Colosseum PythonClient/multirotor/multi_agent_drone.py 와 동일한 호출 패턴
    그대로 사용. 추가 방어 (yaw_mode 명시, cancelLastTask, armDisarm 반복, future
    timeout polling) 는 모두 제거 — 공식 예제에 없는 호출은 부작용만 만들었음.
  - settings.json 의 ClockType: SteppableClock 은 PX4 lockstep 전용. SimpleFlight 는
    default ScalableClock 을 써야 시뮬 시간이 자동 흐름. (이전엔 SteppableClock 이라
    .moveToPositionAsync future 가 영원히 완료 안 되는 hang 가능성)
  - 매 단계 명시적 print + getMultirotorState() 로 위치 보고 → 정확한 진단.

흐름:
  1. confirmConnection
  2. 3대 enableApiControl + armDisarm + takeoffAsync 동시 → 모두 .join()
  3. 3대 자기 spawn 위 5m 로 안정 hover → .join() + sleep(3)
  4. 패턴 순환: 3대 동시 moveToPositionAsync → 모두 .join() → sleep(hold)
  5. Ctrl+C → land + disarm

Usage:
  python3 scripts/phase4_delta_simple.py
  python3 scripts/phase4_delta_simple.py --hold-sec 10 --velocity 1.5
"""

import argparse
import signal
import sys
import time

import airsim


# settings.json (sf_3drones_phase4_delta.json) 의 vehicle spawn 위치 (NED, world frame).
# AirSim multi-vehicle 의 moveToPositionAsync / getMultirotorState 가 **vehicle local
# frame** 을 사용하므로 world ↔ local 변환은 이 spawn 좌표를 빼고/더해서 수행한다.
SPAWN_NED = {
    'drone1': (0.0,  0.0, 0.0),
    'drone2': (5.0,  0.0, 0.0),
    'drone3': (10.0, 0.0, 0.0),
}

# leader 기준 상대 offset (NED, world frame). AirSim 의 world 가 NED → z 음수 = 위.
FORMATIONS_NED = {
    'TRIANGLE': [(0.0,  0.0, 0.0), (-1.7, -1.0, 0.0), (-1.7,  1.0, 0.0)],
    'V3':       [(0.0,  0.0, 0.0), (-1.5, -1.5, 0.0), (-1.5,  1.5, 0.0)],
    'COLUMN':   [(0.0,  0.0, 0.0), (-2.0,  0.0, 0.0), (-4.0,  0.0, 0.0)],
    # DIAMOND3: D1 이 1m 위 (NED z = -1)
    'DIAMOND3': [(0.0,  0.0, -1.0), (-1.5, -1.0, 0.0), (-1.5,  1.0, 0.0)],
}

ALTITUDE_M = 5.0   # 이륙 고도 (ENU). AirSim NED z = -ALTITUDE_M


def pos_str(p) -> str:
    return f'({p.x_val:+.2f}, {p.y_val:+.2f}, {p.z_val:+.2f})'


def world_pos(state, v_name: str) -> tuple[float, float, float]:
    """vehicle local frame → world NED 변환 (spawn 좌표 더하기)."""
    p = state.kinematics_estimated.position
    sx, sy, sz = SPAWN_NED.get(v_name, (0.0, 0.0, 0.0))
    return p.x_val + sx, p.y_val + sy, p.z_val + sz


# NOTE: moveToPositionAsync 는 검색으로 확인된 바 world NED frame 사용
# (multi_agent_drone.py 의 Drone1 spawn (4,0) → moveToPosition(-5,5,-10) 호출 시
# 실제 world (-5, 5, -10) 으로 이동, "opposite quadrant from start" 명시).
# 따라서 world target 그대로 호출. 추가 변환 함수 불필요.


def main() -> int:
    p = argparse.ArgumentParser(description='AERION Phase 4-Δ — clean AirSim formation runner.')
    p.add_argument('--drones', type=int, default=3)
    p.add_argument('--ip', default='127.0.0.1')
    p.add_argument('--port', type=int, default=41451)
    p.add_argument('--patterns', default='TRIANGLE,V3,COLUMN,DIAMOND3',
                   help='쉼표 구분 패턴 순환 시퀀스')
    p.add_argument('--hold-sec', type=float, default=8.0,
                   help='패턴 도달 후 hover hold 시간 (s)')
    p.add_argument('--leader-x', type=float, default=0.0,
                   help='leader x (NED, m). default 0 = drone1 spawn 근처')
    p.add_argument('--leader-y', type=float, default=0.0)
    p.add_argument('--velocity', type=float, default=2.0,
                   help='moveToPosition 속도 (m/s)')
    p.add_argument('--prefix', default='drone',
                   help='vehicle key prefix (settings.json 의 vehicle key 와 일치)')
    p.add_argument('--cycles', type=int, default=0,
                   help='패턴 순환 횟수. 0 = 무한 (Ctrl+C 종료)')
    args = p.parse_args()

    drone_names = [f'{args.prefix}{i}' for i in range(1, args.drones + 1)]
    pattern_seq = [s.strip().upper() for s in args.patterns.split(',') if s.strip()]
    pattern_seq = [s for s in pattern_seq if s in FORMATIONS_NED]
    if not pattern_seq:
        print('ERROR: --patterns 에 유효한 패턴 없음. 가능: TRIANGLE,V3,COLUMN,DIAMOND3',
              file=sys.stderr)
        return 1

    print('=' * 60)
    print('  AERION Phase 4-Δ Simple Runner (AirSim 공식 패턴)')
    print(f'    drones={drone_names}')
    print(f'    patterns={pattern_seq}')
    print(f'    leader NED=({args.leader_x}, {args.leader_y}, {-ALTITUDE_M})')
    print(f'    velocity={args.velocity} m/s, hold={args.hold_sec}s')
    print('=' * 60)

    # ---- AirSim connect ----
    c = airsim.MultirotorClient(ip=args.ip, port=args.port)
    c.confirmConnection()
    listed = c.listVehicles()
    print(f'\n[connect] ping OK. vehicles in scene = {listed}')
    if not all(v in listed for v in drone_names):
        missing = [v for v in drone_names if v not in listed]
        print(f'WARN: {missing} 가 scene 에 없음. UE Stop/Play 다시 시도하거나 --prefix 확인.')

    # ---- Cleanup ----
    def cleanup(_sig=None, _frame=None):
        print('\n[cleanup] land + disarm...')
        try:
            fs = [c.landAsync(vehicle_name=v) for v in drone_names]
            for f in fs:
                try: f.join()
                except: pass
        except Exception as e:
            print(f'  landAsync error: {e}')
        for v in drone_names:
            try:
                c.armDisarm(False, vehicle_name=v)
                c.enableApiControl(False, vehicle_name=v)
            except: pass
        print('[cleanup] 완료')
        sys.exit(0)
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ---- Step 1: enable + arm + takeoff (AirSim 공식 패턴) ----
    print(f'\n[Step 1] enableApiControl + armDisarm + takeoffAsync × {len(drone_names)}')
    for v in drone_names:
        c.enableApiControl(True, vehicle_name=v)
        c.armDisarm(True, vehicle_name=v)
        print(f'  {v} enableApi + arm OK')
    takeoff_fs = [(v, c.takeoffAsync(vehicle_name=v)) for v in drone_names]
    for v, f in takeoff_fs:
        f.join()
        state = c.getMultirotorState(vehicle_name=v)
        p = state.kinematics_estimated.position
        print(f'  {v} takeoff done. world NED = ({p.x_val:+.2f}, {p.y_val:+.2f}, {p.z_val:+.2f})')

    # ---- Step 2: 각 vehicle 의 world spawn 위 ALTITUDE_M 으로 안정 hover ----
    # moveToPositionAsync 는 world NED 이므로 vehicle 의 spawn (x_w, y_w) + 위 ALTITUDE_M
    # 으로 직접 호출.
    print(f'\n[Step 2] 각 vehicle 의 world spawn 위 {ALTITUDE_M}m 로 hover (속도 {args.velocity}m/s)')
    hover_fs = []
    for v in drone_names:
        sx, sy, _ = SPAWN_NED.get(v, (0.0, 0.0, 0.0))
        f = c.moveToPositionAsync(sx, sy, -ALTITUDE_M, args.velocity, vehicle_name=v)
        hover_fs.append((v, f))
    for v, f in hover_fs:
        f.join()
        state = c.getMultirotorState(vehicle_name=v)
        p = state.kinematics_estimated.position
        print(f'  {v} hover. world NED = ({p.x_val:+.2f}, {p.y_val:+.2f}, {p.z_val:+.2f})')
    time.sleep(3.0)
    print('  안정 hover 3초 완료')

    # ---- Step 3: 패턴 순환 ----
    print(f'\n[Step 3] 패턴 순환 시작 (Ctrl+C 종료)')
    leader_x = args.leader_x
    leader_y = args.leader_y
    leader_z = -ALTITUDE_M
    idx = 0
    cycle = 0
    while args.cycles == 0 or cycle < args.cycles:
        pat = pattern_seq[idx % len(pattern_seq)]
        offsets = FORMATIONS_NED[pat]
        print(f'\n  [pattern {idx+1} → {pat}]')

        targets = []
        for i, v in enumerate(drone_names):
            if i >= len(offsets):
                continue
            ox, oy, oz = offsets[i]
            # world NED target (leader 기준). moveToPositionAsync 가 world frame 이므로
            # 변환 없이 그대로 사용.
            tx = leader_x + ox
            ty = leader_y + oy
            tz = leader_z + oz
            targets.append((v, tx, ty, tz))
            print(f'    {v} world target ({tx:+.2f}, {ty:+.2f}, {tz:+.2f})')

        # AirSim Issue #991: SimpleFlight 의 hover() 가 버그 — 호출 후 drone 추락.
        # AirSim Issue #4421: moveToPositionAsync 완료 후 drone shaky behavior.
        # → 도달 후 명령 stream 끊기면 SimpleFlight 가 hover 못함 → 추락.
        # 또한 enableApiControl(True) 매번 호출이 vehicle controller 를 disrupt →
        # 그 vehicle 만 추락 ("하나씩 추락" 패턴).
        #
        # Fix: enableApiControl 재호출 제거 + hold 동안 1Hz 로 같은 target 으로
        # moveToPositionAsync 를 .join() 없이 재발행 → 명령 stream 유지 → drone 안정.
        #
        # 명령 stream 이 유지되면 API control 도 release 안 됨 → drone2/3 freeze 도 회피.
        send_fs = []
        for v, tx, ty, tz in targets:
            f = c.moveToPositionAsync(tx, ty, tz, args.velocity, vehicle_name=v)
            send_fs.append((v, f, (tx, ty, tz)))
        print(f'    [send] {len(send_fs)} 드론 명령 발행. .join() 대기...')

        for v, f, _ in send_fs:
            f.join()
            state = c.getMultirotorState(vehicle_name=v)
            p = state.kinematics_estimated.position
            print(f'    [arrived] {v} world = ({p.x_val:+.2f}, {p.y_val:+.2f}, {p.z_val:+.2f})')

        # Hold 동안 명령 stream 유지 — 1Hz 로 같은 target 재발행 (.join() 안 함).
        # SimpleFlight 의 buggy hover 우회.
        hold_start = time.time()
        hold_tick = 0
        while time.time() - hold_start < args.hold_sec:
            for v, _, (tx, ty, tz) in send_fs:
                # join 안 함 → 명령만 발행. 같은 target 이므로 drone 은 그 위치에서 유지.
                c.moveToPositionAsync(tx, ty, tz, args.velocity, vehicle_name=v)
            hold_tick += 1
            # 첫 tick 만 상세 출력 (이후 spam 방지)
            if hold_tick == 1:
                print(f'    [hold-stream] {args.hold_sec}s 동안 1Hz target 재발행...')
            time.sleep(1.0)

        idx += 1
        if idx % len(pattern_seq) == 0:
            cycle += 1
            if args.cycles > 0:
                print(f'  cycle {cycle}/{args.cycles} 완료')

    print('\n[Step 4] cycle 완료. land + disarm')
    cleanup()
    return 0


if __name__ == '__main__':
    sys.exit(main())
