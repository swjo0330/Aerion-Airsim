#!/usr/bin/env python3
"""AERION 헬퍼: 5대(또는 N대) SimpleFlight 드론 일괄 arm + takeoff (Phase 4 진입 직전).

목적:
  formation_node가 setpoint를 발행하기 전, 모든 드론이 **armed + API control 활성 + 일정 고도 상승**된
  상태여야 함. 본 헬퍼는 AirSim Python RPC를 직접 호출해 이 사전조건을 일괄 충족시킴.

  SimpleFlight(airsim_direct) 모드 전용. Phase 5 PX4 SITL 진입 시 별도 `mavros_arm_all.py`로 대체.

순서:
  1. AirSim RPC ping
  2. 각 드론마다:
     - enableApiControl(True)
     - armDisarm(True)
     - takeoffAsync (병렬 호출)
  3. moveToZAsync(default_altitude, speed) (병렬 호출)
  4. 모든 드론이 default_altitude ±0.5m 안에 들어오면 종료

Usage:
  python3 ~/airsim/airsim_ros2_bridge/scripts/airsim_arm_all.py
  python3 ~/airsim/airsim_ros2_bridge/scripts/airsim_arm_all.py --drones 5 --altitude 5.0

주의:
  - vehicle_name = 'drone1'..'droneN' (settings.json 키와 정확히 일치)
  - settings.json의 ClockType=SteppableClock이면 step 진행을 위해 UE Play 상태여야 함
  - --no-takeoff 옵션은 arm까지만 (테스트용)

변경 이력:
  2026-05-18 v1: 최초 작성 (Phase 4 사전 자산)
"""

import argparse
import sys
import time

import airsim


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--drones', type=int, default=5, help='드론 수 (1~5)')
    p.add_argument('--altitude', type=float, default=5.0, help='이륙 고도 (m, ENU 양수=위)')
    p.add_argument('--speed', type=float, default=2.0, help='상승 속도 (m/s)')
    p.add_argument('--ip', default='127.0.0.1')
    p.add_argument('--port', type=int, default=41451)
    p.add_argument('--no-takeoff', action='store_true', help='arm만, takeoff 생략')
    p.add_argument('--prefix', default='drone', help='vehicle key prefix (기본 drone)')
    args = p.parse_args()

    if not (1 <= args.drones <= 5):
        print(f'[ERROR] --drones 1~5만 지원: {args.drones}', file=sys.stderr)
        sys.exit(2)

    vehicles = [f'{args.prefix}{i}' for i in range(1, args.drones + 1)]
    print(f'[arm_all] vehicles={vehicles}  altitude={args.altitude}m  speed={args.speed}m/s')

    # ----- 연결 -----
    c = airsim.MultirotorClient(ip=args.ip, port=args.port, timeout_value=5.0)
    if not c.ping():
        print('[ERROR] AirSim RPC ping 실패. UE Play 상태인지 확인.', file=sys.stderr)
        sys.exit(1)
    listed = c.listVehicles()
    missing = [v for v in vehicles if v not in listed]
    if missing:
        print(f'[ERROR] settings.json에 누락된 vehicle: {missing}', file=sys.stderr)
        print(f'        실제 listed: {listed}', file=sys.stderr)
        sys.exit(1)
    print(f'[arm_all] ping OK, listed vehicles={listed}')

    # ----- arm -----
    for v in vehicles:
        c.enableApiControl(True, vehicle_name=v)
        c.armDisarm(True, vehicle_name=v)
        print(f'  [{v}] enableApiControl + arm OK')

    if args.no_takeoff:
        print('[arm_all] --no-takeoff: arm까지만 완료. 종료.')
        return

    # ----- takeoff 병렬 -----
    print(f'[arm_all] takeoff 병렬 호출 (timeout 15s)...')
    futures = [c.takeoffAsync(timeout_sec=15, vehicle_name=v) for v in vehicles]
    for v, f in zip(vehicles, futures):
        try:
            f.join()
            print(f'  [{v}] takeoff OK')
        except Exception as e:
            print(f'  [{v}] takeoff FAIL: {e}', file=sys.stderr)

    # ----- 고도 상승 병렬 -----
    # AirSim NED z 음수 = 위쪽. 사용자 입력은 ENU 양수 → -altitude로 호출.
    target_z_ned = -args.altitude
    print(f'[arm_all] moveToZ NED z={target_z_ned}m 병렬 호출...')
    futures = [c.moveToZAsync(target_z_ned, args.speed, vehicle_name=v) for v in vehicles]
    for v, f in zip(vehicles, futures):
        try:
            f.join()
        except Exception as e:
            print(f'  [{v}] moveToZ FAIL: {e}', file=sys.stderr)

    # ----- 도착 확인 -----
    time.sleep(1.0)
    print(f'[arm_all] 최종 위치:')
    all_ok = True
    for v in vehicles:
        pos = c.getMultirotorState(vehicle_name=v).kinematics_estimated.position
        z_enu = -pos.z_val  # NED → ENU
        err = abs(z_enu - args.altitude)
        ok = err < 0.5
        all_ok &= ok
        mark = 'OK' if ok else 'OFF'
        print(f'  [{v}] NED=({pos.x_val:+.2f}, {pos.y_val:+.2f}, {pos.z_val:+.2f})  '
              f'ENU_alt={z_enu:.2f}m  err={err:.2f}  [{mark}]')

    if all_ok:
        print('[arm_all] All drones at target altitude. Formation 진입 가능.')
        sys.exit(0)
    else:
        print('[arm_all] WARN: 일부 드론이 목표 고도에 못 도달. Formation 진입 전 재시도 권장.', file=sys.stderr)
        sys.exit(2)


if __name__ == '__main__':
    main()
