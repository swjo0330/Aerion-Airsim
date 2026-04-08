# Aerion-Airsim

Colosseum(AirSim) + Unreal Engine 5.6 기반 멀티드론 시뮬레이션 환경

## 개요

[Colosseum](https://github.com/CodexLabsLLC/Colosseum)(Microsoft AirSim 포크)을 **Unreal Engine 5.6**에서 동작하도록 마이그레이션하고, PX4/ArduPilot SITL 2대 + ROS2 브릿지를 통합한 시뮬레이션 환경입니다.

기존 ArduPilot + Gazebo 기반 시뮬레이션을 대체하여, 언리얼 엔진의 고품질 렌더링 환경에서 체화지능(Embodied AI) 드론 시뮬레이션을 수행합니다.

## 시스템 스펙

| 항목 | 사양 |
|------|------|
| OS | Ubuntu 22.04.5 LTS |
| GPU | NVIDIA (Driver 580, CUDA 13.0) |
| Unreal Engine | 5.6.1 |
| Colosseum | main 브랜치 (UE 5.6 지원) |
| 컴파일러 | clang-18 (UE 번들) |
| Python | 3.12 (uv venv) / 3.10 (ROS2) |
| ROS2 | Humble |

## 아키텍처

```
┌─────────────────────────────────────────────────┐
│          시뮬레이션 머신 (Ubuntu 22.04)            │
│                                                  │
│  ┌──────────────────────────────────┐            │
│  │   Unreal Engine 5.6 + Colosseum │            │
│  │   AirSim Plugin (2x 드론)       │            │
│  └──────┬──────────────┬───────────┘            │
│    TCP/UDP         RPC:41451                     │
│     │                  │                         │
│  ┌──┴────┐   ┌────────┴──────────┐              │
│  │ SITL  │   │ Python ROS2 Bridge│              │
│  │PX4/APM│   │ (rclpy + airsim)  │              │
│  └──┬────┘   └────────┬──────────┘              │
│  UDP│           ROS2 DDS│                        │
└─────┼──────────────────┼────────────────────────┘
      ▼                  ▼
┌─────────────────────────────────────────────────┐
│     원격 머신 (MAVROS + embodied-drone)           │
└─────────────────────────────────────────────────┘
```

## UE 5.6 마이그레이션 (Colosseum 패치)

Colosseum은 원래 UE 5.2 기준으로 작성되었습니다. UE 5.6에서 빌드하려면 아래 5개 파일을 수정해야 합니다:

| # | 파일 | 변경 내용 |
|---|------|----------|
| 1 | `Unreal/Plugins/AirSim/Source/PawnSimApi.cpp` | `UWorld::LineBatcher` → `DrawDebugLine()` |
| 2 | `Unreal/Plugins/AirSim/Source/WorldSimApi.cpp` | `UWorld::PersistentLineBatcher` → `DrawDebugLine()` |
| 3 | `Unreal/Plugins/AirSim/Source/SimHUD/SimHUD.cpp` | `auto*` → `APawn*` (TObjectPtr 변환) |
| 4 | `Unreal/Plugins/AirSim/Source/AirSim.Build.cs` | `CppCompileWithRpc` → `HeaderOnlyWithRpc` |
| 5 | `build.sh` | 시스템 clang → UE 번들 clang-18 경로 |

추가로 `setup.sh`에서 `vulkan-utils` → `vulkan-tools` 변경 필요 (Ubuntu 22.04).

## 펌웨어 지원

### PX4 (검증 완료)
- TCP lockstep 연결 (4560, 4561...)
- `PX4_SIM_MODEL=none_iris` 환경변수 필수
- Python API로 직접 드론 제어 가능

### ArduPilot (설정 완료, 검증 중)
- UDP JSON 프로토콜 (PX4와 다름)
- `--instance N`은 포트에 **N * 10** 오프셋 적용
- `enableApiControl` 미구현 → MAVROS 통한 제어 필수

## 포트 매핑

### PX4 SITL
| 용도 | Drone0 | Drone1 |
|------|--------|--------|
| SITL↔Colosseum TCP | 4560 | 4561 |
| MAVLink→MAVROS UDP | 14555 | 14556 |
| MAV_SYS_ID | 1 | 2 |

### ArduPilot SITL
| 용도 | Drone0 (instance 0) | Drone1 (instance 1) |
|------|--------|--------|
| Sensor UDP | 9003 | 9013 |
| Control UDP | 9002 | 9012 |

## 빠른 시작

### 1. 사전 준비

```bash
# Unreal Engine 5.6 설치 (unreal-engine/ 디렉토리에)
# → unreal-engine/README.md 참조

# Colosseum 빌드
git clone https://github.com/CodexLabsLLC/Colosseum.git
cd Colosseum
# UE 5.6 패치 적용 후:
./setup.sh && ./build.sh
# BlocksV2/Plugins/AirSim → 심볼릭 링크 생성

# PX4 SITL 빌드
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot && make px4_sitl_default none_iris
```

### 2. 설정 생성

```bash
# PX4 2대
python3 scripts/generate_settings.py -f px4 -n 2 --deploy

# ArduPilot 2대
python3 scripts/generate_settings.py -f ardupilot -n 2 --deploy

# 원격 MAVROS 연결
python3 scripts/generate_settings.py -f px4 -n 2 --mavros-ip 192.168.1.100 --deploy
```

### 3. 실행 (PX4)

```bash
# 터미널 1: UE 에디터
./scripts/launch_ue_blocksv2.sh px4

# UE 에디터에서 Play 클릭

# 터미널 2: PX4 SITL
cd PX4-Autopilot/build/px4_sitl_default
PX4_SIM_MODEL=none_iris ./bin/px4 -i 0 -d etc &
PX4_SIM_MODEL=none_iris ./bin/px4 -i 1 -d etc &

# 터미널 3: ROS2 브릿지
export PYTHONPATH="Colosseum/PythonClient:airsim_ros2_bridge:$PYTHONPATH"
source /opt/ros/humble/setup.bash
python3 airsim_ros2_bridge/airsim_ros2_bridge/bridge_node.py
```

### 4. ROS2 토픽 확인

```bash
ros2 topic list | grep Drone
# /Drone0/camera/image
# /Drone0/camera/camera_info
# /Drone0/cmd_vel
# /Drone0/cmd_pos
# /Drone1/camera/...
```

## ROS2 브릿지 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/{DroneN}/camera/image` | `sensor_msgs/Image` | RGB 1280x720 |
| `/{DroneN}/camera/camera_info` | `sensor_msgs/CameraInfo` | K, P, D 행렬 (pinhole) |
| `/{DroneN}/cmd_vel` | `geometry_msgs/Twist` | 속도 제어 (NED) |
| `/{DroneN}/cmd_pos` | `geometry_msgs/PoseStamped` | 위치 제어 (NED) |

## 디렉토리 구조

```
Aerion-Airsim/
├── CLAUDE.md                    # 프로젝트 지침 + 체크리스트
├── README.md                    # 이 파일
├── pyproject.toml               # Python 의존성 (uv)
├── airsim_ros2_bridge/          # ROS2 브릿지 패키지
│   └── airsim_ros2_bridge/
│       ├── bridge_node.py       # 메인 노드
│       ├── camera_publisher.py  # 카메라 토픽 발행
│       ├── drone_controller.py  # 드론 제어 수신
│       └── utils.py             # FOV→intrinsics 변환
├── scripts/
│   ├── generate_settings.py     # 설정 자동 생성기
│   ├── launch_ue_blocksv2.sh    # UE 에디터 실행
│   ├── launch_px4_dual.sh       # PX4 SITL 실행
│   ├── launch_apm_dual.sh       # APM SITL 실행
│   └── demo_dual_drone.py       # 드론 제어 데모
├── settings/                    # Colosseum settings.json
│   ├── px4_dual.json
│   └── apm_dual.json
├── unreal-engine/               # UE 5.6 설치 디렉토리 (별도 설치)
├── docs/
│   ├── 구축기록.md               # 상세 구축 과정 기록
│   └── superpowers/             # 설계 문서 + 구현 계획
├── Colosseum/                   # (별도 clone, .gitignore)
└── PX4-Autopilot/               # (별도 clone, .gitignore)
```

## 상세 문서

- [구축기록](docs/구축기록.md) — 전체 구축 과정, UE 5.6 패치, 알려진 문제 및 해결
- [설계문서](docs/superpowers/specs/2026-04-07-colosseum-px4-sitl-ros2-design.md) — 아키텍처 설계

## 참고 링크

- [Colosseum (CodexLabsLLC)](https://github.com/CodexLabsLLC/Colosseum)
- [Colosseum 문서](https://codexlabsllc.github.io/Colosseum/)
- [PX4 SITL 설정](https://codexlabsllc.github.io/Colosseum/px4_sitl/)
- [멀티비클 설정](https://codexlabsllc.github.io/Colosseum/px4_multi_vehicle/)
- [이미지 API](https://codexlabsllc.github.io/Colosseum/image_apis/)
