#!/usr/bin/env python3
"""AERION Phase 4-Δ — 3-drone formation, clean slate runner.

ROS2 / mavros / formation_node / bridge_node 의 layered 구조를 우회하고 AirSim Python API
하나로 직접 제어. 매 패턴 전환에서 `future.join()` 으로 도달 대기 → 명령 충돌 없이 안정.

전제:
  - UE Editor 살아있고 ▶ Play 상태 (이전 시뮬레이션 진행 중이면 Stop → Play 다시)
  - settings.json = ~/Documents/AirSim/settings.json (SimpleFlight 3-drone)
    bash scripts/run_phase4_delta_sf.sh 의 Step 1 이 deploy 하거나, 직접:
      cp settings/sf_3drones_phase4_delta.json ~/Documents/AirSim/settings.json
  - airsim Python 패키지 (Colosseum/PythonClient/airsim) 설치됨

Usage:
  python3 scripts/phase4_delta_simple.py
  python3 scripts/phase4_delta_simple.py --drones 3 --hold-sec 8 \\
      --patterns TRIANGLE,V3,COLUMN,DIAMOND3 --velocity 1.5

Patterns (ENU 기준, leader 상대 offset; AirSim 내부 NED 변환은 z 만 부호 반전):

  TRIANGLE: D1(0,0,0)   D2(-1.7,-1, 0)  D3(-1.7,+1, 0)    정삼각, 변 2m
  V3:       D1(0,0,0)   D2(-1.5,-1.5,0) D3(-1.5,+1.5,0)  V 자
  COLUMN:   D1(0,0,0)   D2(-2,0,0)      D3(-4,0,0)        종대
  DIAMOND3: D1(0,0,+1)  D2(-1.5,-1, 0)  D3(-1.5,+1, 0)   수직 (D1 1m 위)

Ctrl+C → land + disarm 후 종료.
"""

import argparse
import math
import signal
import sys
import time

import airsim


# ENU offset (leader 기준, m). AirSim NED 변환은 hover-target 계산 시 z 만 부호 반전.
FORMATIONS_ENU = {
    'TRIANGLE': [(0.0,  0.0, 0.0), (-1.7, -1.0, 0.0), (-1.7,  1.0, 0.0)],
    'V3':       [(0.0,  0.0, 0.0), (-1.5, -1.5, 0.0), (-1.5,  1.5, 0.0)],
    'COLUMN':   [(0.0,  0.0, 0.0), (-2.0,  0.0, 0.0), (-4.0,  0.0, 0.0)],
    'DIAMOND3': [(0.0,  0.0, 1.0), (-1.5, -1.0, 0.0), (-1.5,  1.0, 0.0)],
}

ALTITUDE_M = 5.0   # 이륙 고도 (ENU); AirSim NED z = -ALTITUDE_M


def enu_to_ned(x: float, y: float, z: float) -> tuple[float, float, float]:
    """ENU → AirSim NED. (x 는 그대로, y 는 그대로, z 는 부호 반전.)

    참고: 실제 ENU↔NED 는 (x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu) 인 경우가
    표준이지만 본 시연에서는 패턴 offset 이 leader 좌표계 기준이고 AirSim 의 world
    좌표가 NED 라 z 만 부호 반전하면 충분 (xy 평면 회전은 leader 좌표계가 NED 와 동일).
    """
    return x, y, -z


def main() -> int:
    p = argparse.ArgumentParser(description='AERION Phase 4-Δ — clean 3-drone formation runner.')
    p.add_argument('--drones', type=int, default=3, help='드론 수 (1~3 권장, AirSim RPC 4-thread 한계 안)')
    p.add_argument('--ip', default='127.0.0.1')
    p.add_argument('--port', type=int, default=41451)
    p.add_argument('--patterns', default='TRIANGLE,V3,COLUMN,DIAMOND3',
                   help='쉼표 구분 패턴 순환 시퀀스')
    p.add_argument('--hold-sec', type=float, default=8.0,
                   help='패턴 도달 후 hover hold 시간 (s)')
    p.add_argument('--leader-x-ned', type=float, default=0.0,
                   help='leader x (AirSim NED, m). default 0 = drone1 spawn 근처')
    p.add_argument('--leader-y-ned', type=float, default=0.0)
    p.add_argument('--velocity', type=float, default=1.5,
                   help='moveToPosition 속도 (m/s). 1.0~2.0 권장 (부드러운 추종)')
    p.add_argument('--lookahead', type=float, default=2.0,
                   help='moveToPosition lookahead (m). 0 보다 크면 곡선 추종.')
    p.add_argument('--prefix', default='drone',
                   help='vehicle key prefix (settings.json 의 vehicle key 와 일치)')
    p.add_argument('--cycles', type=int, default=0,
                   help='패턴 순환 횟수. 0 = 무한 (Ctrl+C 종료)')
    p.add_argument('--move-timeout', type=float, default=20.0,
                   help='패턴 당 각 드론 도달 timeout (s). 도달 못해도 cancel 후 다음 패턴 진행.')
    args = p.parse_args()

    drone_names = [f'{args.prefix}{i}' for i in range(1, args.drones + 1)]
    pattern_seq = [s.strip().upper() for s in args.patterns.split(',') if s.strip()]
    pattern_seq = [s for s in pattern_seq if s in FORMATIONS_ENU]
    if not pattern_seq:
        print('ERROR: --patterns 에 유효한 패턴 없음. 가능: TRIANGLE,V3,COLUMN,DIAMOND3', file=sys.stderr)
        return 1

    print('=' * 60)
    print('  AERION Phase 4-Δ Simple Runner')
    print(f'    drones={drone_names}')
    print(f'    patterns={pattern_seq}')
    print(f'    hold_sec={args.hold_sec}, velocity={args.velocity}, lookahead={args.lookahead}')
    print(f'    leader NED=({args.leader_x_ned}, {args.leader_y_ned}, {-ALTITUDE_M})')
    print('=' * 60)

    # AirSim connection
    c = airsim.MultirotorClient(ip=args.ip, port=args.port)
    c.confirmConnection()
    listed = c.listVehicles()
    print(f'[connect] AirSim ping OK. vehicles in scene = {listed}')
    missing = [v for v in drone_names if v not in listed]
    if missing:
        print(f'WARN: settings.json 에 정의됐지만 scene 에 spawn 안 됨: {missing}')
        print('  → UE Editor Stop → Play 다시 후 재시도. 또는 --prefix 옵션 확인.')

    # Cleanup hook
    def cleanup(_sig=None, _frame=None):
        print('\n[cleanup] land + disarm 진행...')
        try:
            fs = [c.landAsync(vehicle_name=v) for v in drone_names]
            for f in fs:
                try:
                    f.join()
                except Exception:
                    pass
        except Exception as e:
            print(f'  landAsync error: {e}')
        for v in drone_names:
            try:
                c.armDisarm(False, vehicle_name=v)
                c.enableApiControl(False, vehicle_name=v)
            except Exception:
                pass
        print('[cleanup] 완료')
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # Step 1: enable + arm (3대 모두 — RPC 직렬화 회피 위해 짧게 stagger)
    print(f'\n[Step 1] enableApiControl + armDisarm × {args.drones}')
    for v in drone_names:
        c.enableApiControl(True, vehicle_name=v)
        c.armDisarm(True, vehicle_name=v)
        print(f'  {v} arm OK')

    # Step 2: takeoff (동시 async)
    print(f'\n[Step 2] takeoffAsync × {args.drones} (동시)')
    take_fs = [(v, c.takeoffAsync(vehicle_name=v)) for v in drone_names]
    for v, f in take_fs:
        try:
            f.join()
            print(f'  {v} takeoff done')
        except Exception as e:
            print(f'  {v} takeoff error: {e}')

    # Step 3: 각 드론 자기 spawn 위치 위 ALTITUDE_M 으로 올라가서 안정 hover
    print(f'\n[Step 3] 각 드론 spawn 위치 위 {ALTITUDE_M}m 로 안정 hover (5초)')
    fs_hover = []
    for i, v in enumerate(drone_names):
        # spawn 위치는 settings.json 의 X=0/5/10 NED — 그 위 ALTITUDE_M 높이로 이동.
        target_x = i * 5.0
        target_y = 0.0
        target_z = -ALTITUDE_M
        fs_hover.append((v, c.moveToPositionAsync(
            target_x, target_y, target_z,
            args.velocity,
            lookahead=args.lookahead,
            adaptive_lookahead=1,
            vehicle_name=v,
        )))
    for v, f in fs_hover:
        try:
            f.join()
        except Exception as e:
            print(f'  {v} hover-up error: {e}')
    time.sleep(5.0)
    print('  안정 hover 완료')

    # Step 4: 패턴 순환
    print(f'\n[Step 4] 패턴 순환 시작 — Ctrl+C 종료')
    leader_x = args.leader_x_ned
    leader_y = args.leader_y_ned
    leader_z = -ALTITUDE_M
    # Fixed-yaw setting — drone 이 도달 판정 시 yaw 진동 없이 정지하도록 명시.
    yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=0.0)

    def reacquire(v: str) -> None:
        """매 패턴 명령 전 API control + arm 재확인 (AirSim 의 idle release 방지)."""
        try:
            c.enableApiControl(True, vehicle_name=v)
            c.armDisarm(True, vehicle_name=v)
            c.cancelLastTask(vehicle_name=v)
        except Exception as e:
            print(f'    [reacquire {v}] warn: {e}')

    def wait_join_with_timeout(v: str, f, timeout_sec: float) -> bool:
        """msgpackrpc future 는 .join() 에 timeout 인자 없음. wall-clock polling 으로
        timeout 구현. timeout 초과 시 cancelLastTask 후 진행 (hang 방지)."""
        t_start = time.time()
        # AirSim future 가 비동기 RPC. 별 thread 에서 .join() 호출 + main 은 polling.
        import threading
        done = {'flag': False, 'err': None}
        def _wait():
            try:
                f.join()
            except Exception as e:
                done['err'] = e
            finally:
                done['flag'] = True
        t = threading.Thread(target=_wait, daemon=True)
        t.start()
        while not done['flag']:
            if time.time() - t_start > timeout_sec:
                print(f'    {v} join timeout ({timeout_sec}s) — cancel + 진행')
                try:
                    c.cancelLastTask(vehicle_name=v)
                except Exception:
                    pass
                return False
            time.sleep(0.1)
        if done['err']:
            print(f'    {v} move error: {done["err"]}')
            return False
        return True

    cycle = 0
    idx = 0
    while args.cycles == 0 or cycle < args.cycles:
        pat = pattern_seq[idx % len(pattern_seq)]
        offsets_enu = FORMATIONS_ENU[pat]
        print(f'\n  [pattern {idx+1} → {pat}]')

        # 각 드론 목표 NED 계산
        targets = []
        for i, v in enumerate(drone_names):
            if i >= len(offsets_enu):
                continue
            ox_enu, oy_enu, oz_enu = offsets_enu[i]
            ox_ned, oy_ned, oz_ned = enu_to_ned(ox_enu, oy_enu, oz_enu)
            tx = leader_x + ox_ned
            ty = leader_y + oy_ned
            tz = leader_z + oz_ned
            targets.append((v, tx, ty, tz))
            print(f'    {v} → NED({tx:+.2f}, {ty:+.2f}, {tz:+.2f})')

        # 매 패턴 명령 직전 API control + arm 재확인 + 이전 명령 취소
        for v, *_ in targets:
            reacquire(v)

        # 3대 동시 async 명령 발행, join 으로 도달 wait (timeout 20s)
        fs = []
        for v, tx, ty, tz in targets:
            print(f'    [send] {v} moveToPositionAsync...')
            f = c.moveToPositionAsync(
                tx, ty, tz,
                args.velocity,
                lookahead=args.lookahead,
                adaptive_lookahead=1,
                yaw_mode=yaw_mode,
                vehicle_name=v,
            )
            fs.append((v, f))
        print(f'    [wait] 모든 드론 도달 대기 (max {args.move_timeout}s 각)...')

        for v, f in fs:
            ok = wait_join_with_timeout(v, f, args.move_timeout)
            print(f'    [arrived] {v}: {"OK" if ok else "timeout"}')

        print(f'    [hold]   {args.hold_sec}s hover hold...')
        time.sleep(args.hold_sec)

        idx += 1
        if idx % len(pattern_seq) == 0:
            cycle += 1
            if args.cycles > 0:
                print(f'  cycle {cycle}/{args.cycles} 완료')

    print('\n[Step 5] 모든 cycle 완료. land + disarm 후 종료.')
    cleanup()
    return 0


if __name__ == '__main__':
    sys.exit(main())
