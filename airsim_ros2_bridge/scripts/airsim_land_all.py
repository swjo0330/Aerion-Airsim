#!/usr/bin/env python3
"""AERION 헬퍼: N대 드론 일괄 land + disarm + API control release (SimpleFlight 전용).

formation_node 종료 후 또는 emergency stop 시 사용. arm_all.py 역동작.

Usage:
  python3 ~/airsim/airsim_ros2_bridge/scripts/airsim_land_all.py [--drones N]

변경 이력:
  2026-05-18 v1: 최초 작성
"""

import argparse
import sys

import airsim


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--drones', type=int, default=5)
    p.add_argument('--ip', default='127.0.0.1')
    p.add_argument('--port', type=int, default=41451)
    p.add_argument('--prefix', default='drone')
    args = p.parse_args()

    if not (1 <= args.drones <= 5):
        print(f'[ERROR] --drones 1~5만 지원: {args.drones}', file=sys.stderr)
        sys.exit(2)

    vehicles = [f'{args.prefix}{i}' for i in range(1, args.drones + 1)]
    c = airsim.MultirotorClient(ip=args.ip, port=args.port, timeout_value=5.0)
    if not c.ping():
        print('[ERROR] AirSim RPC ping 실패', file=sys.stderr)
        sys.exit(1)

    print(f'[land_all] land 병렬 호출 (timeout 20s)...')
    futures = [c.landAsync(timeout_sec=20, vehicle_name=v) for v in vehicles]
    for v, f in zip(vehicles, futures):
        try:
            f.join()
            print(f'  [{v}] land OK')
        except Exception as e:
            print(f'  [{v}] land FAIL: {e}', file=sys.stderr)

    for v in vehicles:
        c.armDisarm(False, vehicle_name=v)
        c.enableApiControl(False, vehicle_name=v)
        print(f'  [{v}] disarm + API control release')

    print('[land_all] done.')


if __name__ == '__main__':
    main()
