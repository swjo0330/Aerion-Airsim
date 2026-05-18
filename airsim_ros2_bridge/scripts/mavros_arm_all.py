#!/usr/bin/env python3
"""AERION Phase 5 헬퍼: N대 PX4 SITL + MAVROS arm + OFFBOARD mode + takeoff (Phase 5 진입 직전).

목적:
  Phase 5에서 bridge가 `control_backend='px4_mavros'` 모드로 동작하려면 PX4가 OFFBOARD mode + ARMED
  상태여야 함. 본 헬퍼는 MAVROS service를 통해 N대 일괄 처리.

  SimpleFlight 모드(airsim_direct)는 별도 `airsim_arm_all.py` 사용.

순서:
  1. /drone{N}/mavros/state 구독으로 connected 대기 (최대 30초)
  2. /drone{N}/mavros/cmd/arming 서비스로 armed=True
  3. /drone{N}/mavros/set_mode 서비스로 custom_mode='OFFBOARD'
  4. (선택) /drone{N}/mavros/cmd/takeoff 서비스로 자동 이륙

사용 (Phase 5 launch 떠있는 상태에서):
  python3 ~/airsim/airsim_ros2_bridge/scripts/mavros_arm_all.py --drones 5 --altitude 5.0

주의:
  - mavros service 호출 전, formation_node가 setpoint_position/local을 publish 중이어야 함 (OFFBOARD
    mode 유지 위해 PX4가 setpoint stream 필요). leader_publisher가 자동 발행하면 OK.
  - PX4 default safety: setpoint 미수신 시 OFFBOARD 자동 종료 → arm 직전에 setpoint 흐름 보장.
  - takeoff 서비스는 PX4 펌웨어 빌드에 따라 미지원. 대안: setpoint_position/local에 z=목표고도 publish.

변경 이력:
  2026-05-18 v1: 최초 작성 (Phase 5 사전 자산, 미검증)
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State


class MavrosArmer(Node):
    def __init__(self, drones: int, takeoff_alt: float, wait_timeout: float):
        super().__init__('aerion_mavros_armer')
        self._n = drones
        self._takeoff_alt = takeoff_alt
        self._wait_timeout = wait_timeout
        self._connected = {i: False for i in range(1, drones + 1)}

        for i in range(1, drones + 1):
            self.create_subscription(
                State, f'/drone{i}/mavros/state',
                lambda msg, idx=i: self._on_state(idx, msg), 10
            )

    def _on_state(self, idx: int, msg: State):
        if msg.connected and not self._connected[idx]:
            self._connected[idx] = True
            self.get_logger().info(f'[drone{idx}] mavros state connected')

    def wait_connected(self) -> bool:
        t0 = time.time()
        while time.time() - t0 < self._wait_timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if all(self._connected.values()):
                return True
        return False

    def _call_service(self, srv_type, srv_name, request):
        client = self.create_client(srv_type, srv_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'service unavailable: {srv_name}')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()

    def arm_all(self) -> int:
        ok = 0
        for i in range(1, self._n + 1):
            req = CommandBool.Request()
            req.value = True
            res = self._call_service(CommandBool, f'/drone{i}/mavros/cmd/arming', req)
            if res and res.success:
                ok += 1
                self.get_logger().info(f'[drone{i}] arm OK')
            else:
                self.get_logger().warn(f'[drone{i}] arm FAIL: {res}')
        return ok

    def set_offboard_all(self) -> int:
        ok = 0
        for i in range(1, self._n + 1):
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            res = self._call_service(SetMode, f'/drone{i}/mavros/set_mode', req)
            if res and res.mode_sent:
                ok += 1
                self.get_logger().info(f'[drone{i}] OFFBOARD set OK')
            else:
                self.get_logger().warn(f'[drone{i}] OFFBOARD set FAIL: {res}')
        return ok

    def takeoff_all(self) -> int:
        ok = 0
        for i in range(1, self._n + 1):
            req = CommandTOL.Request()
            req.altitude = self._takeoff_alt
            res = self._call_service(CommandTOL, f'/drone{i}/mavros/cmd/takeoff', req)
            if res and res.success:
                ok += 1
                self.get_logger().info(f'[drone{i}] takeoff OK')
            else:
                self.get_logger().warn(f'[drone{i}] takeoff FAIL (펌웨어 미지원이면 setpoint 사용): {res}')
        return ok


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--drones', type=int, default=5)
    p.add_argument('--altitude', type=float, default=5.0)
    p.add_argument('--wait-timeout', type=float, default=30.0,
                   help='mavros state connected 대기 timeout (s)')
    p.add_argument('--skip-takeoff', action='store_true',
                   help='OFFBOARD까지만, takeoff 서비스 호출 생략 (setpoint로 이륙 권장)')
    args = p.parse_args()

    if not (1 <= args.drones <= 5):
        print(f'[ERROR] --drones 1~5만 지원: {args.drones}', file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    armer = MavrosArmer(args.drones, args.altitude, args.wait_timeout)

    print(f'[mavros_arm_all] {args.drones} 드론 mavros state connected 대기...')
    if not armer.wait_connected():
        print(f'[ERROR] {args.wait_timeout}s 안에 모든 드론 connected 안 됨', file=sys.stderr)
        print(f'  connected: {armer._connected}', file=sys.stderr)
        sys.exit(3)
    print('[mavros_arm_all] 모두 connected. ARM 시도...')
    arm_ok = armer.arm_all()
    print('[mavros_arm_all] OFFBOARD mode 설정...')
    off_ok = armer.set_offboard_all()

    if args.skip_takeoff:
        print(f'[mavros_arm_all] arm={arm_ok}/{args.drones}, offboard={off_ok}/{args.drones}. takeoff 생략.')
    else:
        print('[mavros_arm_all] takeoff 시도...')
        to_ok = armer.takeoff_all()
        print(f'[mavros_arm_all] arm={arm_ok}/{args.drones}, offboard={off_ok}/{args.drones}, takeoff={to_ok}/{args.drones}')

    armer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
