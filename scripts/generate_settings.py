#!/usr/bin/env python3
"""펌웨어 유형과 드론 수에 따라 settings.json을 자동 생성.

사용법:
  python3 generate_settings.py --firmware px4 --drones 2 --mavros-ip 127.0.0.1
  python3 generate_settings.py --firmware ardupilot --drones 2 --mavros-ip 192.168.1.100
  python3 generate_settings.py --firmware px4 --drones 3 --output settings/px4_triple.json
"""
import argparse
import json
import sys


# 펌웨어별 포트 스킴 정의
PORT_SCHEMES = {
    "px4": {
        "vehicle_type": "PX4Multirotor",
        "clock_type": "SteppableClock",
        "base_ports": {
            "TcpPort": 4560,           # SITL ↔ Colosseum (TCP lockstep)
            "ControlPortLocal": 14540,  # MAVLink 수신 (local)
            "ControlPortRemote": 14555, # MAVLink 송신 (→ MAVROS)
        },
        "port_offset": 1,  # 인스턴스당 +1
        "extra_settings": {
            "UseSerial": False,
            "LockStep": True,
            "UseTcp": True,
        },
        "parameters": {
            "NAV_RCL_ACT": 0,
            "NAV_DLL_ACT": 0,
            "COM_OBL_ACT": 1,
        },
    },
    "ardupilot": {
        "vehicle_type": "ArduCopter",
        "clock_type": "SteppableClock",
        "base_ports": {
            "UdpPort": 9003,            # 센서 데이터 (Colosseum → ArduPilot)
            "ControlPortLocal": 9002,    # 모터 제어 (ArduPilot → Colosseum)
        },
        "port_offset": 10,  # ArduPilot --instance N은 N*10 오프셋
        "extra_settings": {
            "UseSerial": False,
        },
        "parameters": {},
    },
}

# MAVLink MAVROS 포워딩 포트 (펌웨어 공통)
MAVROS_BASE_PORT = 14555


def generate_vehicle(firmware: str, instance: int, mavros_ip: str, spacing: float = 5.0) -> dict:
    """단일 드론 설정 생성."""
    scheme = PORT_SCHEMES[firmware]
    offset = instance * scheme["port_offset"]

    vehicle = {
        "VehicleType": scheme["vehicle_type"],
        "X": instance * spacing,
        "Y": 0,
        "Z": 0,
        "Cameras": {
            "front_center": {
                "CaptureSettings": [{
                    "ImageType": 0,
                    "Width": 1280,
                    "Height": 720,
                    "FOV_Degrees": 90,
                    "MotionBlurAmount": 0,
                }],
                "X": 0.25, "Y": 0, "Z": -0.18,
                "Pitch": 0, "Roll": 0, "Yaw": 0,
            }
        },
    }

    # 펌웨어별 추가 설정
    vehicle.update(scheme["extra_settings"])

    # 포트 설정 (오프셋 적용)
    for port_key, base_port in scheme["base_ports"].items():
        if port_key == "ControlPortRemote":
            # MAVROS 포워딩 포트는 인스턴스당 +1
            vehicle[port_key] = MAVROS_BASE_PORT + instance
        else:
            vehicle[port_key] = base_port + offset

    # PX4: MAVROS IP 설정
    if firmware == "px4":
        vehicle["LocalHostIp"] = mavros_ip
        vehicle["Parameters"] = {
            **scheme["parameters"],
            "MAV_SYS_ID": instance + 1,
        }
    else:
        vehicle["LocalHostIp"] = "127.0.0.1"

    # ArduPilot: UdpIp 설정
    if firmware == "ardupilot":
        vehicle["UdpIp"] = "127.0.0.1"

    return vehicle


def generate_settings(firmware: str, num_drones: int, mavros_ip: str) -> dict:
    """전체 settings.json 생성."""
    scheme = PORT_SCHEMES[firmware]

    settings = {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ClockType": scheme["clock_type"],
        "Vehicles": {},
    }

    for i in range(num_drones):
        name = f"Drone{i}"
        settings["Vehicles"][name] = generate_vehicle(firmware, i, mavros_ip)

    return settings


def print_port_table(firmware: str, num_drones: int):
    """포트 매핑 테이블 출력."""
    scheme = PORT_SCHEMES[firmware]
    print(f"\n{'='*60}")
    print(f"  {firmware.upper()} 포트 매핑 ({num_drones}대)")
    print(f"{'='*60}")

    headers = list(scheme["base_ports"].keys())
    if firmware == "px4":
        headers.append("MAV_SYS_ID")

    print(f"  {'드론':<10}", end="")
    for h in headers:
        print(f"{h:<22}", end="")
    print()
    print(f"  {'-'*10}", end="")
    for _ in headers:
        print(f"{'-'*22}", end="")
    print()

    for i in range(num_drones):
        offset = i * scheme["port_offset"]
        print(f"  {'Drone'+str(i):<10}", end="")
        for key, base in scheme["base_ports"].items():
            if key == "ControlPortRemote":
                print(f"{MAVROS_BASE_PORT + i:<22}", end="")
            else:
                print(f"{base + offset:<22}", end="")
        if firmware == "px4":
            print(f"{i + 1:<22}", end="")
        print()

    if firmware == "ardupilot":
        print(f"\n  참고: ArduPilot --instance N은 포트에 N*{scheme['port_offset']} 오프셋 적용")
        print(f"  참고: ArduCopter API는 enableApiControl 미구현 → MAVROS 통한 제어 필수")
    print()


def main():
    parser = argparse.ArgumentParser(description="Colosseum settings.json 생성기")
    parser.add_argument("--firmware", "-f", choices=["px4", "ardupilot"], required=True,
                        help="펌웨어 유형 (px4 또는 ardupilot)")
    parser.add_argument("--drones", "-n", type=int, default=2,
                        help="드론 수 (기본: 2)")
    parser.add_argument("--mavros-ip", default="127.0.0.1",
                        help="원격 MAVROS IP (기본: 127.0.0.1)")
    parser.add_argument("--output", "-o", default=None,
                        help="출력 파일 경로 (기본: settings/<firmware>_<n>drones.json)")
    parser.add_argument("--deploy", "-d", action="store_true",
                        help="~/Documents/AirSim/settings.json에도 복사")
    args = parser.parse_args()

    settings = generate_settings(args.firmware, args.drones, args.mavros_ip)

    # 출력 파일 경로
    if args.output:
        output_path = args.output
    else:
        output_path = f"settings/{args.firmware}_{args.drones}drones.json"

    # 저장
    import os
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(settings, f, indent=2)
    print(f"설정 파일 생성: {output_path}")

    # 배포
    if args.deploy:
        deploy_path = os.path.expanduser("~/Documents/AirSim/settings.json")
        os.makedirs(os.path.dirname(deploy_path), exist_ok=True)
        with open(deploy_path, "w") as f:
            json.dump(settings, f, indent=2)
        print(f"배포 완료: {deploy_path}")

    # 포트 테이블 출력
    print_port_table(args.firmware, args.drones)

    # JSON 미리보기
    print(json.dumps(settings, indent=2))


if __name__ == "__main__":
    main()
