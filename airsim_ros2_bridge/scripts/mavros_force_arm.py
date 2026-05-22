#!/usr/bin/env python3
"""Forced arm fallback for PX4 SITL when preflight reject blocks normal arm.

mavros_arm_all.py 의 일반 `/cmd/arming` (CommandBool) 은 `MAV_CMD_COMPONENT_ARM_DISARM`
의 param2=0 으로 보내므로 PX4 가 preflight (heading invalid, system power 등) 실패
시 거부한다. 이 스크립트는 mavros `/cmd/command` (CommandLong) 로 동일 명령에
**param2=21196** (PX4 force-arm magic) 을 실어 preflight 를 우회한다.

SITL 시연/디버깅 전용. 실 비행에서는 절대 사용 금지.

settings.json 의 Parameters block (EKF2_MAG_TYPE 등) 이 Colosseum→PX4 채널에서
적용 안 되는 환경의 fallback 으로 사용.

Usage:
  ros2 run airsim_ros2_bridge mavros_force_arm \\
    --drones 3 --set-offboard --altitude 5.0
"""

import argparse
import sys
import time
from typing import Tuple

import rclpy
from rclpy.node import Node

from mavros_msgs.srv import CommandLong, SetMode


MAV_CMD_COMPONENT_ARM_DISARM = 400
FORCE_ARM_MAGIC = 21196.0


class ForcedArmer(Node):
    """Per-drone forced arm + (옵션) OFFBOARD + initial setpoint hover.

    mavros 가 connected 된 후 호출되어야 한다. service 가 graph 에 없으면 timeout.
    """

    def __init__(self, drones: int, altitude: float, set_offboard: bool):
        super().__init__('aerion_force_armer')
        self._n = drones
        self._alt = altitude
        self._set_offboard = set_offboard

    def force_arm_one(self, ns: str) -> Tuple[bool, str]:
        cli = self.create_client(CommandLong, f'/{ns}/mavros/cmd/command')
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, 'cmd/command service unavailable'
        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_COMPONENT_ARM_DISARM
        req.confirmation = 0
        req.param1 = 1.0                # arm
        req.param2 = FORCE_ARM_MAGIC    # bypass preflight
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        resp = fut.result()
        if resp is None:
            return False, 'timeout'
        return bool(resp.success), f'success={resp.success} result={resp.result}'

    def set_offboard_one(self, ns: str) -> Tuple[bool, str]:
        cli = self.create_client(SetMode, f'/{ns}/mavros/set_mode')
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, 'set_mode service unavailable'
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = 'OFFBOARD'
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        resp = fut.result()
        if resp is None:
            return False, 'timeout'
        return bool(resp.mode_sent), f'mode_sent={resp.mode_sent}'

    def run(self) -> int:
        armed = 0
        offboard = 0
        for n in range(1, self._n + 1):
            ns = f'drone{n}'
            ok, msg = self.force_arm_one(ns)
            self.get_logger().info(f'[{ns}] forced arm: {msg}')
            if ok:
                armed += 1
        for n in range(1, self._n + 1):
            if not self._set_offboard:
                break
            ns = f'drone{n}'
            ok, msg = self.set_offboard_one(ns)
            self.get_logger().info(f'[{ns}] OFFBOARD: {msg}')
            if ok:
                offboard += 1
        print(f'[forced_arm] armed={armed}/{self._n}, offboard={offboard}/{self._n}')
        # OFFBOARD 진입 후 setpoint stream 이 곧장 와야 함 (formation_node 가 publish).
        # setpoint 잠시 도착 대기 (PX4 가 OFFBOARD 유지하려면 setpoint 가 1Hz 이상 필요).
        time.sleep(2.0)
        return 0 if armed == self._n else 1


def main():
    p = argparse.ArgumentParser(description='Forced arm fallback (PX4 SITL preflight bypass).')
    p.add_argument('--drones', type=int, default=3)
    p.add_argument('--altitude', type=float, default=5.0,
                   help='OFFBOARD setpoint altitude hint (현재 formation_node 가 leader_pose 로 결정).')
    p.add_argument('--set-offboard', action='store_true', default=True,
                   help='forced arm 직후 OFFBOARD mode 도 set (기본 True).')
    args = p.parse_args()

    rclpy.init()
    armer = ForcedArmer(args.drones, args.altitude, args.set_offboard)
    try:
        rc = armer.run()
    finally:
        armer.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
