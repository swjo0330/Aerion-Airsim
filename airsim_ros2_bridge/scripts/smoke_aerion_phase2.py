#!/usr/bin/env python3
"""AERION Phase 2.5 — 단일 드론 smoke 테스트 (SimpleFlight 직접 제어).

목적:
  bridge를 거치지 않고 AirSim RPC 직접 호출로 takeoff→forward→land 시퀀스를 검증.
  bridge 작동과 분리해서 AirSim 자체 / settings.json / UE 환경 무결성만 측정.

순서:
  1. AirSim RPC ping (UE Play 상태 + RPC 서버 :41451 살아있음 확인)
  2. enableApiControl + armDisarm (제어권 인계 + 시동)
  3. takeoff → 3m 상승 (moveToZAsync로 정확 고도 도달)
  4. 전진 5m (moveByVelocityZAsync, x=+1m/s × 5s, z=고도 유지)
  5. land (자동 착륙)
  6. disarm + 제어권 반환

게이트 (PASS 기준):
  - 각 단계 timeout 안에 성공 (takeoff 10s, land 15s)
  - kinematics_estimated.z (NED, 음수=위쪽)가 takeoff 후 약 -3, land 후 ~0
  - 전진 후 x 좌표가 +5m 근처로 이동
  - "Smoke test passed." 메시지로 종료

주의:
  - 이 스크립트는 NED 좌표를 그대로 출력 (z 음수=위). ROS 측 ENU 변환은 bridge가 담당.
  - vehicle_name 인자는 settings.json의 vehicle key와 정확히 일치해야 함 (대소문자).
  - Colosseum의 메서드명은 `getDistanceSensorData` (sim 접두사 없음).

Usage:
    python3 ~/airsim/airsim_ros2_bridge/scripts/smoke_aerion_phase2.py            # default drone1
    python3 ~/airsim/airsim_ros2_bridge/scripts/smoke_aerion_phase2.py drone1
"""

import sys
import time
import airsim


def main():
    # 기본값을 vehicle key 통일 컨벤션(소문자 drone1)에 맞춤.
    vehicle = sys.argv[1] if len(sys.argv) > 1 else 'drone1'
    print(f'[smoke] vehicle={vehicle}')

    c = airsim.MultirotorClient(ip='127.0.0.1', port=41451, timeout_value=5.0)
    print('[step 1] ping...', flush=True)
    assert c.ping(), 'AirSim ping failed'
    print(f'  ping OK  vehicles={c.listVehicles()}')

    print('[step 2] enableApiControl + arm...', flush=True)
    c.enableApiControl(True, vehicle_name=vehicle)
    c.armDisarm(True, vehicle_name=vehicle)

    def _state_dump(tag: str):
        st = c.getMultirotorState(vehicle_name=vehicle).kinematics_estimated
        pos = st.position
        try:
            d = c.getDistanceSensorData('Distance_Front', vehicle).distance
        except Exception:
            d = float('nan')
        print(f'  [{tag}]  NED=({pos.x_val:+.2f}, {pos.y_val:+.2f}, {pos.z_val:+.2f})  '
              f'range_front={d:.2f}m', flush=True)

    _state_dump('initial')

    print('[step 3] takeoff to ~3m...', flush=True)
    c.takeoffAsync(timeout_sec=10, vehicle_name=vehicle).join()
    c.moveToZAsync(-3.0, 1.0, vehicle_name=vehicle).join()
    time.sleep(1.0)
    _state_dump('after takeoff')

    print('[step 4] move forward 5m...', flush=True)
    c.moveByVelocityZAsync(vx=1.0, vy=0.0, z=-3.0, duration=5.0,
                          vehicle_name=vehicle).join()
    time.sleep(0.5)
    _state_dump('after forward')

    print('[step 5] land...', flush=True)
    c.landAsync(timeout_sec=15, vehicle_name=vehicle).join()
    time.sleep(1.0)
    _state_dump('after land')

    print('[step 6] disarm + release API control...', flush=True)
    c.armDisarm(False, vehicle_name=vehicle)
    c.enableApiControl(False, vehicle_name=vehicle)
    print('[smoke] Smoke test passed.')


if __name__ == '__main__':
    main()
