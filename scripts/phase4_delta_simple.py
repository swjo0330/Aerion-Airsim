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


# leader 기준 상대 offset (AirSim NED). AirSim 의 world 가 NED 이므로 z 음수 = 위.
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
        print(f'  {v} takeoff done. pos NED = {pos_str(state.kinematics_estimated.position)}')

    # ---- Step 2: 자기 spawn 위 ALTITUDE_M 으로 안정 hover ----
    print(f'\n[Step 2] 자기 spawn 위 {ALTITUDE_M}m 로 hover (속도 {args.velocity}m/s)')
    hover_fs = []
    for i, v in enumerate(drone_names):
        # settings.json 의 spawn X=0/5/10, Y=0 가정. 다른 spawn 이면 --leader-x 와 패턴 조정.
        spawn_x_ned = i * 5.0
        spawn_y_ned = 0.0
        target_z_ned = -ALTITUDE_M
        f = c.moveToPositionAsync(spawn_x_ned, spawn_y_ned, target_z_ned,
                                   args.velocity, vehicle_name=v)
        hover_fs.append((v, f))
    for v, f in hover_fs:
        f.join()
        state = c.getMultirotorState(vehicle_name=v)
        print(f'  {v} hover. pos NED = {pos_str(state.kinematics_estimated.position)}')
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
            tx = leader_x + ox
            ty = leader_y + oy
            tz = leader_z + oz
            targets.append((v, tx, ty, tz))
            print(f'    {v} target NED ({tx:+.2f}, {ty:+.2f}, {tz:+.2f})')

        # 명령 send: 공식 패턴 그대로 (yaw_mode 명시 안 함, cancelLastTask 호출 안 함,
        # armDisarm/enableApiControl 반복 호출 안 함). 새 moveToPositionAsync 가
        # 이전 명령을 자동 override.
        send_fs = []
        for v, tx, ty, tz in targets:
            f = c.moveToPositionAsync(tx, ty, tz, args.velocity, vehicle_name=v)
            send_fs.append((v, f))
        print(f'    [send] {len(send_fs)} 드론 명령 발행. .join() 대기...')

        for v, f in send_fs:
            f.join()
            state = c.getMultirotorState(vehicle_name=v)
            print(f'    [arrived] {v} pos NED = {pos_str(state.kinematics_estimated.position)}')

        print(f'    [hold] {args.hold_sec}s hover')
        time.sleep(args.hold_sec)

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
