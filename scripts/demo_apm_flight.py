#!/usr/bin/env python3
"""ArduCopter MAVLink 직접 제어 + AirSim 카메라 캡처 데모.

ArduCopter는 airsim Python API의 enableApiControl/armDisarm이 미구현이므로,
pymavlink을 사용하여 MAVLink 명령을 직접 전송합니다.

사용법:
    python3 demo_apm_flight.py [--mav-url udpin:0.0.0.0:14550] [--drone-name Drone0]

요구사항:
    pip install pymavlink opencv-python numpy
    PYTHONPATH에 Colosseum/PythonClient 포함 (airsim 패키지)
"""
import argparse
import sys
import os
import time
import numpy as np

from pymavlink import mavutil

# --- AirSim 임포트 (카메라 캡처용) ---
# Colosseum PythonClient 경로 추가
_colosseum_pyclient = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "Colosseum", "PythonClient"
)
if os.path.isdir(_colosseum_pyclient):
    sys.path.insert(0, _colosseum_pyclient)

try:
    import airsim
except ImportError:
    print("[경고] airsim 패키지를 찾을 수 없습니다. 카메라 캡처를 건너뜁니다.")
    airsim = None


# ──────────────────────────────────────────────
# MAVLink 유틸리티 함수
# ──────────────────────────────────────────────

def wait_heartbeat(conn, timeout: float = 30.0):
    """Heartbeat를 수신할 때까지 대기합니다."""
    print("[1/7] Heartbeat 대기 중 ...")
    conn.wait_heartbeat(timeout=timeout)
    print(f"  -> Heartbeat 수신: system={conn.target_system}, "
          f"component={conn.target_component}")


def set_guided_mode(conn, timeout: float = 10.0):
    """GUIDED 모드(=4)로 전환합니다.

    ArduCopter의 GUIDED 모드 번호는 4입니다.
    MAV_CMD_DO_SET_MODE를 COMMAND_LONG으로 전송합니다.
    """
    print("[2/7] GUIDED 모드 전환 중 ...")

    # base_mode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1)
    # custom_mode: 4 (GUIDED)
    GUIDED_MODE = 4
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,                  # confirmation
        1,                  # param1: base_mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
        GUIDED_MODE,        # param2: custom_mode
        0, 0, 0, 0, 0      # param3-7: 미사용
    )

    # ACK 대기
    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("  -> GUIDED 모드 전환 성공")
        else:
            print(f"  -> GUIDED 모드 전환 실패 (result={ack.result})")
    else:
        print("  -> GUIDED 모드 ACK 미수신 (타임아웃)")

    # 모드가 반영될 시간을 잠깐 대기
    time.sleep(1.0)


def arm(conn, timeout: float = 10.0):
    """드론을 ARM 합니다 (MAV_CMD_COMPONENT_ARM_DISARM)."""
    print("[3/7] ARM 명령 전송 중 ...")

    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,          # confirmation
        1,          # param1: 1=ARM, 0=DISARM
        0,          # param2: 0=안전 검사 수행, 21196=강제 ARM
        0, 0, 0, 0, 0
    )

    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("  -> ARM 성공")
        else:
            print(f"  -> ARM 실패 (result={ack.result})")
    else:
        print("  -> ARM ACK 미수신 (타임아웃)")

    time.sleep(1.0)


def takeoff(conn, altitude: float = 5.0, timeout: float = 10.0):
    """MAV_CMD_NAV_TAKEOFF으로 이륙합니다.

    Args:
        altitude: 목표 고도 (미터, AGL)
    """
    print(f"[4/7] 이륙 명령 전송 (목표 고도: {altitude}m) ...")

    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,              # confirmation
        0,              # param1: pitch (미사용)
        0,              # param2: 미사용
        0,              # param3: 미사용
        0,              # param4: yaw angle (NaN이면 현재 방향 유지)
        0,              # param5: latitude (미사용)
        0,              # param6: longitude (미사용)
        altitude        # param7: altitude (미터)
    )

    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("  -> 이륙 명령 수락됨")
        else:
            print(f"  -> 이륙 명령 실패 (result={ack.result})")
    else:
        print("  -> 이륙 ACK 미수신 (타임아웃)")


def wait_altitude(conn, target_alt: float, tolerance: float = 0.5, timeout: float = 30.0):
    """드론이 목표 고도에 도달할 때까지 대기합니다.

    GLOBAL_POSITION_INT 메시지의 relative_alt 필드를 모니터링합니다.
    """
    print(f"  목표 고도 {target_alt}m 도달 대기 ...")
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # mm -> m
            if abs(current_alt - target_alt) < tolerance:
                print(f"  -> 목표 고도 도달: {current_alt:.2f}m")
                return True
    print(f"  -> 고도 도달 타임아웃 ({timeout}s)")
    return False


def move_forward_ned(conn, north: float, east: float, down: float, duration: float = 5.0):
    """SET_POSITION_TARGET_LOCAL_NED로 NED 좌표계 이동 명령을 전송합니다.

    위치 기준(position setpoint)으로 현재 위치에서 상대 이동합니다.
    type_mask 비트:
      - 0b0000_1111_1111_1000 (0x0FF8): x, y, z 위치만 사용
      실제로는 속도 기반 이동이 더 안정적이므로 속도 setpoint 사용:
      - 0b0000_1100_0111 (0x0C07): vx, vy, vz 속도만 사용

    여기서는 속도 기반으로 duration 동안 일정 속도로 이동합니다.
    """
    print(f"[6/7] NED 이동: north={north}m, east={east}m, down={down}m (약 {duration}초) ...")

    # 속도 계산 (m/s)
    vx = north / duration if duration > 0 else 0
    vy = east / duration if duration > 0 else 0
    vz = down / duration if duration > 0 else 0

    # type_mask: 위치/가속/yaw 무시, 속도만 사용
    # 비트: pos_x=1, pos_y=2, pos_z=4 무시 -> 0x07
    # 가속 ax=64, ay=128, az=256 무시 -> 0x1C0
    # yaw=1024, yaw_rate=2048 무시 -> 0xC00
    # 합산: 0x07 | 0x1C0 | 0xC00 = 0xDC7
    type_mask = 0x0DC7

    start = time.time()
    while time.time() - start < duration:
        conn.mav.set_position_target_local_ned_send(
            0,                                      # time_boot_ms (0=무관)
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # 좌표 프레임
            type_mask,
            0, 0, 0,                                # x, y, z (무시됨)
            vx, vy, vz,                             # vx, vy, vz (m/s)
            0, 0, 0,                                # afx, afy, afz (무시됨)
            0, 0                                    # yaw, yaw_rate (무시됨)
        )
        time.sleep(0.2)  # 5Hz로 명령 반복 전송

    # 정지 명령 (속도 0)
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    print("  -> 이동 완료, 정지 명령 전송됨")


def land(conn, timeout: float = 10.0):
    """MAV_CMD_NAV_LAND로 착륙합니다."""
    print("[7/7] 착륙 명령 전송 ...")

    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,          # confirmation
        0,          # param1: abort alt (0=사용안함)
        0,          # param2: land mode
        0,          # param3: 미사용
        float("nan"),  # param4: yaw (NaN=현재 방향)
        0,          # param5: latitude (0=현재)
        0,          # param6: longitude (0=현재)
        0           # param7: altitude (무시됨, 지상까지)
    )

    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("  -> 착륙 명령 수락됨")
        else:
            print(f"  -> 착륙 명령 실패 (result={ack.result})")
    else:
        print("  -> 착륙 ACK 미수신 (타임아웃)")


def wait_disarmed(conn, timeout: float = 60.0):
    """드론이 DISARM될 때까지 대기합니다 (착륙 완료 감지)."""
    print("  착륙 완료(DISARM) 대기 ...")
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:
                print("  -> DISARM 확인, 착륙 완료")
                return True
    print(f"  -> DISARM 대기 타임아웃 ({timeout}s)")
    return False


# ──────────────────────────────────────────────
# AirSim 카메라 캡처
# ──────────────────────────────────────────────

def capture_airsim_camera(drone_name: str = "Drone0", save_dir: str = "/tmp/airsim_demo"):
    """AirSim API로 카메라 이미지를 캡처하여 저장합니다.

    enableApiControl 없이 simGetImages는 사용 가능합니다.
    (카메라 캡처는 시뮬레이션 관찰용이므로 제어 API 불필요)
    """
    if airsim is None:
        print("[카메라] airsim 모듈 미설치, 캡처 건너뜀")
        return

    print(f"\n--- AirSim 카메라 캡처 ({drone_name}) ---")
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()

        # Scene 이미지 캡처 (비압축, RGBA)
        responses = client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
        ], vehicle_name=drone_name)

        if responses and responses[0].width > 0:
            r = responses[0]
            img_data = np.frombuffer(r.image_data_uint8, dtype=np.uint8)
            channels = len(r.image_data_uint8) // (r.height * r.width)
            img = img_data.reshape(r.height, r.width, channels)

            os.makedirs(save_dir, exist_ok=True)
            path = os.path.join(save_dir, f"{drone_name}_apm_flight.png")

            try:
                import cv2
                bgr = (cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                       if channels == 4
                       else cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                cv2.imwrite(path, bgr)
            except ImportError:
                # OpenCV 없으면 raw 저장
                path = os.path.join(save_dir, f"{drone_name}_apm_flight.raw")
                with open(path, "wb") as f:
                    f.write(r.image_data_uint8)

            print(f"  이미지 저장: {r.width}x{r.height} -> {path}")
        else:
            print("  이미지 캡처 실패 (빈 응답)")

        # 카메라 정보 출력
        info = client.simGetCameraInfo("front_center", vehicle_name=drone_name)
        print(f"  카메라 FOV: {info.fov:.1f}도")
        print(f"  카메라 위치: {info.pose.position}")

    except Exception as e:
        print(f"  [오류] AirSim 카메라 캡처 실패: {e}")


# ──────────────────────────────────────────────
# 메인 실행
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="ArduCopter MAVLink 직접 제어 비행 테스트"
    )
    parser.add_argument(
        "--mav-url",
        default="udpin:0.0.0.0:14550",
        help="pymavlink 연결 URL (기본: udpin:0.0.0.0:14550)"
    )
    parser.add_argument(
        "--drone-name",
        default="Drone0",
        help="AirSim 드론 이름 (기본: Drone0)"
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=5.0,
        help="이륙 목표 고도 (미터, 기본: 5.0)"
    )
    parser.add_argument(
        "--forward-distance",
        type=float,
        default=10.0,
        help="전진 거리 (미터, 기본: 10.0)"
    )
    parser.add_argument(
        "--save-dir",
        default="/tmp/airsim_demo",
        help="이미지 저장 디렉토리 (기본: /tmp/airsim_demo)"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("  ArduCopter MAVLink 비행 테스트")
    print(f"  MAVLink URL : {args.mav_url}")
    print(f"  드론 이름   : {args.drone_name}")
    print(f"  목표 고도   : {args.altitude}m")
    print(f"  전진 거리   : {args.forward_distance}m")
    print("=" * 60)

    # ── MAVLink 연결 ──
    print(f"\nMAVLink 연결: {args.mav_url}")
    conn = mavutil.mavlink_connection(args.mav_url)

    # 1. Heartbeat 수신 확인
    wait_heartbeat(conn)

    # 2. GUIDED 모드 전환
    set_guided_mode(conn)

    # 3. ARM
    arm(conn)

    # 4. 이륙 (목표 고도)
    takeoff(conn, altitude=args.altitude)

    # 이륙 후 목표 고도 도달 대기
    wait_altitude(conn, target_alt=args.altitude, tolerance=1.0, timeout=30.0)

    # 5. 5초 호버링 대기
    print("\n[5/7] 5초 호버링 대기 ...")
    time.sleep(5.0)
    print("  -> 대기 완료")

    # 6. 전진 이동 (NED 좌표: north=+X 방향)
    move_forward_ned(
        conn,
        north=args.forward_distance,
        east=0.0,
        down=0.0,
        duration=5.0
    )

    # 이동 후 안정화 대기
    time.sleep(2.0)

    # 카메라 캡처 (AirSim API)
    capture_airsim_camera(
        drone_name=args.drone_name,
        save_dir=args.save_dir
    )

    # 7. 착륙
    land(conn)

    # 착륙 완료 대기
    wait_disarmed(conn, timeout=60.0)

    print("\n" + "=" * 60)
    print("  비행 테스트 완료!")
    print("=" * 60)

    conn.close()


if __name__ == "__main__":
    main()
