# Colosseum + PX4/APM SITL + ROS2 통합 시뮬레이션 환경 설계

**작성일:** 2026-04-07
**환경:** Ubuntu 22.04.5 LTS, NVIDIA GPU (CUDA 13.0), RAM 62GB, Disk 1.2TB free
**UE:** 5.6.1 (설치완료: `/home/clrobur/airsim/unreal-engine/`)

---

## 1. 목표

Ubuntu 22.04에서 다음을 구축:
1. Colosseum (AirSim fork) + Unreal Engine 5.6 시뮬레이션 환경
2. PX4 SITL 2대 → 원격 MAVROS 머신에 MAVLink UDP 전송
3. Python 기반 ROS2 브릿지 (카메라 이미지/파라미터/드론 제어)
4. 이후 ArduPilot(APM) SITL도 동일 구조로 구축
5. 도시 환경 에셋 + 장애물 배치 (Phase 5)

---

## 2. 아키텍처

```
┌──────────────────────────────────────────────────────────┐
│              시뮬레이션 머신 (Ubuntu 22.04)                 │
│                                                           │
│  ┌────────────────────────────────────────┐               │
│  │   Unreal Engine 5.6 + Colosseum       │               │
│  │   AirSim Plugin                       │               │
│  │   BlocksV2 환경 (→ 추후 도시 에셋)      │               │
│  │   2x 드론 물리 시뮬레이션               │               │
│  └───────┬──────────────┬─────────────────┘               │
│      TCP:4560       TCP:4561          RPC:41451           │
│          │              │                 │               │
│  ┌───────┴───┐   ┌──────┴───┐   ┌────────┴─────────────┐ │
│  │ PX4 SITL  │   │ PX4 SITL │   │  Python Bridge       │ │
│  │ -i 0      │   │ -i 1     │   │  (rclpy + airsim)    │ │
│  │ SysID=1   │   │ SysID=2  │   │                      │ │
│  └─────┬─────┘   └─────┬────┘   │  - /camera/image     │ │
│   UDP:14555        UDP:14556     │  - /camera/camera_info│ │
│        │               │        │  - 드론 제어 토픽      │ │
│        │               │        └────────┬──────────────┘ │
└────────┼───────────────┼─────────────────┼────────────────┘
         │               │          ROS2 DDS│
         ▼               ▼                  ▼
┌────────────────────────────────────────────────────────────┐
│           원격 머신 (MAVROS + embodied-drone)                │
│                                                             │
│  MAVROS Drone0 (udp:14555)   MAVROS Drone1 (udp:14556)    │
│  /camera/image ◄──── Python Bridge                         │
│  /camera/camera_info ◄──── Python Bridge                   │
│  기존 embodied-drone ROS2 노드들 (변경 없음)                  │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 핵심 결정사항

| 항목 | 결정 | 근거 |
|------|------|------|
| Colosseum 브랜치 | `main` | README에 UE 5.6 지원 명시 |
| UE 버전 | 5.6.1 (기설치) | main 브랜치 호환 |
| 컴파일러 | clang-18 (수동설치) | build.sh가 clang-18 요구 |
| MAVLink 전송 | settings.json 직접설정 | ControlPortRemote로 원격 MAVROS에 직접 UDP 전송 |
| ROS2 브릿지 | Python (rclpy + airsim client) | 빌드 간단, 커스터마이징 자유, CLAUDE.md 파이썬 기반 요구 |
| 환경 | BlocksV2 → 추후 도시에셋 | 파이프라인 검증 우선 |
| SITL 순서 | PX4 먼저 → APM 추후 구축 | PX4 문서가 더 잘 정리됨, 이후 APM은 VehicleType 변경으로 전환 |
| Python | 3.12+ (설치 필요, 현재 3.10) | CLAUDE.md 요구사항 |

---

## 4. 구축 단계

### Phase 1: 기반 설치

**1.1 시스템 의존성 설치**
- clang-18, libc++-18-dev, libc++abi-18-dev (LLVM apt repo)
- Python 3.12+ (deadsnakes PPA)
- libvulkan1, vulkan-tools (Vulkan 지원)

**1.2 Colosseum 빌드**
```bash
cd /home/clrobur/airsim
git clone https://github.com/CodexLabsLLC/Colosseum.git
cd Colosseum
./setup.sh        # rpclib, eigen3, 기본 의존성
./build.sh        # libAirLib.a, libMavLinkCom.a, librpc.a → Unreal/Plugins/AirSim/
```

**1.3 PX4-Autopilot SITL 빌드**
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl_default none_iris
```

**1.4 airsim Python 패키지 설치**
```bash
cd /home/clrobur/airsim/Colosseum/PythonClient
pip install -e .   # Python 3.12 환경에서
```

### Phase 2: PX4 SITL 2대 시뮬레이션 검증

**2.1 settings.json 구성** (`~/Documents/AirSim/settings.json`)
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "Vehicles": {
    "Drone0": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "LockStep": true,
      "UseTcp": true,
      "TcpPort": 4560,
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14555,
      "LocalHostIp": "<MAVROS_MACHINE_IP>",
      "X": 0, "Y": 0, "Z": 0,
      "Parameters": {
        "NAV_RCL_ACT": 0,
        "NAV_DLL_ACT": 0,
        "COM_OBL_ACT": 1,
        "MAV_SYS_ID": 1
      },
      "Cameras": {
        "front_center": {
          "CaptureSettings": [{
            "ImageType": 0,
            "Width": 1280,
            "Height": 720,
            "FOV_Degrees": 90,
            "MotionBlurAmount": 0
          }],
          "X": 0.25, "Y": 0, "Z": -0.18,
          "Pitch": 0, "Roll": 0, "Yaw": 0
        }
      }
    },
    "Drone1": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "LockStep": true,
      "UseTcp": true,
      "TcpPort": 4561,
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14556,
      "LocalHostIp": "<MAVROS_MACHINE_IP>",
      "X": 5, "Y": 0, "Z": 0,
      "Parameters": {
        "NAV_RCL_ACT": 0,
        "NAV_DLL_ACT": 0,
        "COM_OBL_ACT": 1,
        "MAV_SYS_ID": 2
      },
      "Cameras": {
        "front_center": {
          "CaptureSettings": [{
            "ImageType": 0,
            "Width": 1280,
            "Height": 720,
            "FOV_Degrees": 90,
            "MotionBlurAmount": 0
          }],
          "X": 0.25, "Y": 0, "Z": -0.18,
          "Pitch": 0, "Roll": 0, "Yaw": 0
        }
      }
    }
  }
}
```

**2.2 BlocksV2.uproject 수정**
- `EngineAssociation`: `"5.4"` → `"5.6"` 변경

**2.3 기동 순서**
1. UE 에디터 + BlocksV2 프로젝트 실행
2. PX4 SITL instance 0: `make px4_sitl_default none_iris`
3. PX4 SITL instance 1: `PX4_SIM_MODEL=iris ./build/px4_sitl_default/bin/px4 -i 1`
4. 시뮬레이션 Play → 두 드론이 스폰되고 PX4 SITL 연결 확인
5. 원격 MAVROS에서 MAVLink 수신 확인

### Phase 3: Python ROS2 브릿지

**3.1 브릿지 노드 구조**
```
airsim_ros2_bridge/
├── bridge_node.py          # 메인 ROS2 노드
├── camera_publisher.py     # 카메라 이미지 + CameraInfo 발행
├── drone_controller.py     # ROS2 토픽 → airsim 드론 제어
└── utils.py                # 좌표변환, 이미지 변환 유틸
```

**3.2 발행 토픽 (드론별)**
| 토픽 | 메시지 타입 | 소스 |
|------|-----------|------|
| `/drone{N}/camera/image` | `sensor_msgs/Image` | `simGetImages()` Scene 타입 |
| `/drone{N}/camera/camera_info` | `sensor_msgs/CameraInfo` | `simGetCameraInfo()` → FOV, resolution으로 fx,fy,cx,cy 계산 |

**3.3 카메라 파라미터 변환**
```python
# FOV + resolution → intrinsics
fov_rad = math.radians(fov_degrees)
fx = width / (2 * math.tan(fov_rad / 2))
fy = fx  # square pixels
cx = width / 2
cy = height / 2
# Distortion: D = [0,0,0,0,0] (pinhole model, no distortion)
```

**3.4 구독 토픽 (드론 제어)**
| 토픽 | 메시지 타입 | 동작 |
|------|-----------|------|
| `/drone{N}/cmd_vel` | `geometry_msgs/Twist` | `moveByVelocityAsync()` |
| `/drone{N}/cmd_pos` | `geometry_msgs/PoseStamped` | `moveToPositionAsync()` |

### Phase 4: ArduPilot(APM) SITL 구축

PX4 파이프라인 검증 후, APM으로 전환:

**4.1 settings.json 변경**
```json
"Drone0": {
  "VehicleType": "ArduCopter",
  "UseSerial": false,
  "UdpIp": "127.0.0.1",
  "UdpPort": 9003,
  "ControlPort": 9002,
  "LocalHostIp": "<MAVROS_MACHINE_IP>"
}
```

**4.2 ArduPilot SITL 기동**
```bash
cd ardupilot
sim_vehicle.py -v ArduCopter --no-mavproxy -f airsim-copter --instance 0
sim_vehicle.py -v ArduCopter --no-mavproxy -f airsim-copter --instance 1
```

**4.3 주의사항**
- `ClockType`을 PX4의 `SteppableClock` 대신 적절한 값으로 변경 필요
- ArduPilot은 UDP 기반 통신 (PX4는 TCP lockstep)
- MAVLink 포워딩 포트 재설정 필요
- PX4와 APM 혼합 운용은 ClockType 충돌로 비권장 → 별도 설정 프로파일로 전환 운영

### Phase 5: 도시 환경 + 장애물 (추후)

1. UE 마켓플레이스에서 도시 에셋 다운로드/설치
2. BlocksV2 대신 새 UE 프로젝트 생성, AirSim 플러그인 추가
3. 도시 맵에 장애물 배치 (UE 에디터)
4. settings.json은 동일하게 유지 (환경 독립)

---

## 5. 포트 매핑 정리

### PX4 SITL 2대 구성

| 용도 | Drone0 | Drone1 |
|------|--------|--------|
| SITL ↔ Colosseum (TCP) | 4560 | 4561 |
| MAVLink Local (UDP) | 14540 | 14541 |
| MAVLink → Remote MAVROS (UDP) | 14555 | 14556 |
| MAV_SYS_ID | 1 | 2 |
| AirSim RPC (공유) | 41451 | 41451 |

### ArduPilot SITL 2대 구성 (Phase 4)

| 용도 | Drone0 | Drone1 |
|------|--------|--------|
| SITL ↔ Colosseum Sensor (UDP) | 9003 | 9005 |
| SITL ↔ Colosseum Control (UDP) | 9002 | 9004 |
| MAVLink → Remote MAVROS (UDP) | 14555 | 14556 |

---

## 6. 파일/디렉토리 구조 (예상)

```
/home/clrobur/airsim/
├── CLAUDE.md
├── unreal-engine/                    # UE 5.6.1 (기설치)
│   └── Engine/Binaries/Linux/UnrealEditor
├── Colosseum/                        # Colosseum 소스 (Phase 1)
│   ├── setup.sh
│   ├── build.sh
│   ├── PythonClient/                 # airsim Python 패키지
│   └── Unreal/
│       ├── Plugins/AirSim/           # 빌드된 UE 플러그인
│       └── Environments/BlocksV2/    # 기본 환경
├── PX4-Autopilot/                    # PX4 SITL (Phase 1)
├── airsim_ros2_bridge/               # Python ROS2 브릿지 (Phase 3)
│   ├── bridge_node.py
│   ├── camera_publisher.py
│   ├── drone_controller.py
│   └── utils.py
└── docs/
    └── superpowers/specs/
        └── 2026-04-07-colosseum-px4-sitl-ros2-design.md
```

---

## 7. 검증 체크리스트

### Phase 1 완료 기준
- [ ] clang-18 설치 및 `clang-18 --version` 확인
- [ ] Python 3.12+ 설치 및 확인
- [ ] Colosseum `build.sh` 성공 (libAirLib.a 생성)
- [ ] PX4 SITL `make px4_sitl_default none_iris` 성공

### Phase 2 완료 기준
- [ ] UE 에디터에서 BlocksV2 프로젝트 열림
- [ ] PX4 SITL 2개 인스턴스 기동 → Colosseum 연결
- [ ] 시뮬레이션에서 드론 2대 스폰 확인
- [ ] 원격 MAVROS에서 MAVLink 수신 확인 (14555, 14556)

### Phase 3 완료 기준
- [ ] Python 브릿지에서 `/camera/image` 토픽 발행
- [ ] `/camera/camera_info` 에 올바른 intrinsics 포함
- [ ] 원격 embodied-drone 노드에서 카메라 데이터 수신
- [ ] ROS2 토픽으로 드론 제어 동작

### Phase 4 완료 기준
- [ ] ArduPilot SITL로 VehicleType 전환
- [ ] APM SITL 2대 기동 → Colosseum 연결
- [ ] 원격 MAVROS에서 APM MAVLink 수신 확인

### Phase 5 완료 기준
- [ ] 도시 에셋 설치 및 UE 프로젝트에 적용
- [ ] 장애물 배치 완료
- [ ] 전체 파이프라인 (SITL + 카메라 + MAVROS) 도시 환경에서 동작
