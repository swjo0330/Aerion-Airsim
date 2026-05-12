# Aerion-Airsim

Colosseum(AirSim) + Unreal Engine 5.6 기반 멀티드론 시뮬레이션 환경

## 개요

[Colosseum](https://github.com/CodexLabsLLC/Colosseum)(Microsoft AirSim 포크)을 **Unreal Engine 5.6**에서 동작하도록 마이그레이션하고, PX4/ArduPilot SITL 2대 + ROS2 브릿지를 통합한 시뮬레이션 환경입니다.

기존 ArduPilot + Gazebo 기반 시뮬레이션을 대체하여, 언리얼 엔진의 고품질 렌더링 환경에서 체화지능(Embodied AI) 드론 시뮬레이션을 수행합니다.

## 시스템 스펙

| 항목 | 사양 |
|------|------|
| 노트북 | ASUS ROG Strix G16 (G634JZR) |
| OS | Ubuntu 22.04.5 LTS |
| CPU | Intel Core i9-14900HX (24코어, 5.8GHz) |
| RAM | 64 GB |
| GPU | NVIDIA GeForce RTX 4080 Laptop (12GB) |
| NVIDIA Driver | 580.126.09 |
| CUDA | 13.0 (드라이버 지원) |
| Unreal Engine | 5.6.1 |
| Colosseum | main 브랜치 (UE 5.6 지원) |
| 컴파일러 | clang-18 (UE 번들) |
| Python | 3.12 (uv venv) / 3.10 (ROS2) |
| ROS2 | Humble |

자세한 스펙: [docs/system-specs.md](docs/system-specs.md)

## 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    시뮬레이션 머신                            │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │         Unreal Engine 5.6.1 + Colosseum 플러그인      │   │
│  │   [uav0 드론 물리 시뮬레이션]  [uav1 드론 물리 시뮬레이션]  │   │
│  └───────────────────┬─────────────────────────────────┘   │
│                      │ TCP :41451 (AirSim API)              │
│  ┌───────────────────▼─────────────────────────────────┐   │
│  │         bridge_node.py (ROS2 노드, rmw_zenoh_cpp)    │   │
│  │  /uavN/camera/image  /uavN/camera/camera_info        │   │
│  │  /uavN/cmd_vel       /uavN/cmd_pos                   │   │
│  └───────────────────┬─────────────────────────────────┘   │
│                      │                                      │
│  ┌───────────────────▼─────────────────────────────────┐   │
│  │  PX4 SITL (uav0: TCP:4560, UDP:14555)               │   │
│  │  PX4 SITL (uav1: TCP:4561, UDP:14556)               │   │
│  └───────────────────┬─────────────────────────────────┘   │
└──────────────────────┼──────────────────────────────────────┘
                       │ rmw_zenoh TCP:7447 / MAVLink UDP
┌──────────────────────▼──────────────────────────────────────┐
│                    원격 제어 머신                             │
│  MAVROS (uav0: udp://:14540@SIM_IP:14555)                   │
│  MAVROS (uav1: udp://:14541@SIM_IP:14556)                   │
│  embodied-drone 노드 (rmw_zenoh_cpp)                        │
└─────────────────────────────────────────────────────────────┘
```

자세한 아키텍처: [docs/architecture.md](docs/architecture.md)

## Zenoh 원격 통신

ROS2 미들웨어를 `rmw_zenoh_cpp`로 교체하여 WAN 환경에서 토픽을 직접 교환합니다.

```bash
# 시뮬레이션 머신 (router)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=settings/zenoh_sim.json5

# 원격 머신 (client) — 별도 브릿지 불필요, RMW 교체만으로 동작
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

설정 파일: [settings/zenoh_sim.json5](settings/zenoh_sim.json5)

### 네트워크 IP 명세 (Tailscale)

| 머신 | Tailscale IP |
|------|-------------|
| 시뮬레이션 머신 | `100.120.219.68` |
| 원격 제어 머신 | `100.67.87.116` |

Tailscale은 WireGuard 기반 VPN으로, 100.x.x.x 대역을 할당하며 NAT 통과 + 암호화를 자동 처리합니다.

## Zenoh 기반 공인망 ROS2 통신

### 현재 구성: rmw_zenoh_cpp (Tailscale 기반)

두 머신 모두 RMW를 rmw_zenoh_cpp로 교체. Tailscale(WireGuard VPN)이 NAT 통과 및
암호화를 담당하므로 별도 설정 없이 ROS2 토픽이 자동 교환됨.

| 머신 | 역할 | Tailscale IP |
|------|------|-------------|
| 시뮬레이션 머신 | Zenoh Router | 100.120.219.68 |
| 원격 제어 머신 | Zenoh Client | 100.67.87.116 |

연결: sim→ listen tcp/100.120.219.68:7447, remote→ connect tcp/100.67.87.116:7447

### 차후 구성: Zenoh Bridge (순수 공인망, Tailscale 없이)

Tailscale 없이 공인 IP로 직접 통신 시 `zenoh-bridge-ros2dds` 사용.
기존 DDS RMW를 유지하면서 Zenoh가 WAN 구간만 담당.

아키텍처:
시뮬 머신: DDS → zenoh-bridge-ros2dds → TCP:7447(공인IP) → 원격머신: zenoh-bridge-ros2dds → DDS

장점: RMW 교체 불필요, 표준 DDS 환경 유지
단점: zenoh-bridge-ros2dds 양쪽 모두 실행 필요, TLS 별도 설정 권장

실행:
```bash
# 시뮬 머신
zenoh-bridge-ros2dds -c settings/zenoh_sim.json5

# 원격 머신
zenoh-bridge-ros2dds -c settings/zenoh_remote.json5
```

필요 설정:
- 시뮬 머신 공인 IP의 7447 포트 오픈 (방화벽/포트포워딩)
- zenoh_sim.json5 listen 주소를 0.0.0.0:7447 로 변경
- TLS 인증서 설정 권장

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
# Colosseum 빌드
git clone https://github.com/CodexLabsLLC/Colosseum.git
cd Colosseum
./setup.sh && ./build.sh

# PX4 SITL 빌드
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot && make px4_sitl_default none_iris
```

### 2. 설정 생성

```bash
# PX4 2대 (로컬)
python3 scripts/generate_settings.py -f px4 -n 2 --deploy

# PX4 2대 (원격 MAVROS 연결)
python3 scripts/generate_settings.py -f px4 -n 2 --mavros-ip <REMOTE_IP> --local-ip <SIM_IP> --deploy

# ArduPilot 2대
python3 scripts/generate_settings.py -f ardupilot -n 2 --deploy
```

### 3. 실행 (PX4)

```bash
# 터미널 1: UE 에디터
bash scripts/launch_ue.sh blocks px4

# UE 에디터에서 Play 클릭

# 터미널 2: PX4 SITL 2대
bash scripts/launch_px4_dual.sh

# 터미널 3: ROS2 브릿지 (rmw_zenoh)
bash scripts/run_bridge.sh
```

### 4. ROS2 토픽 확인

```bash
ros2 topic list
# /uav0/camera/image
# /uav0/camera/camera_info
# /uav0/cmd_vel
# /uav0/cmd_pos
# /uav1/camera/...
```

## ROS2 브릿지 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/{uavN}/camera/image` | `sensor_msgs/Image` | RGB 1280x720 Raw |
| `/{uavN}/camera/camera_info` | `sensor_msgs/CameraInfo` | K, P, D 행렬 (pinhole) |
| `/{uavN}/cmd_vel` | `geometry_msgs/Twist` | 속도 제어 (NED) |
| `/{uavN}/cmd_pos` | `geometry_msgs/PoseStamped` | 위치 제어 (NED) |

## 디렉토리 구조

```
Aerion-Airsim/
├── README.md
├── pyproject.toml
├── airsim_ros2_bridge/
│   └── airsim_ros2_bridge/
│       ├── bridge_node.py       # 메인 ROS2 노드 (rmw_zenoh)
│       ├── camera_publisher.py  # 카메라 토픽 발행
│       ├── drone_controller.py  # 드론 제어 수신
│       └── utils.py
├── scripts/
│   ├── generate_settings.py     # settings.json 생성기
│   ├── run_bridge.sh            # 브릿지 실행 (rmw_zenoh 환경변수 포함)
│   ├── launch_ue.sh             # UE 에디터 실행
│   ├── launch_px4_dual.sh       # PX4 SITL 2대 실행
│   ├── launch_apm_dual.sh       # APM SITL 2대 실행
│   ├── demo_dual_drone.py       # 드론 제어 데모
│   ├── demo_apm_flight.py       # APM 비행 데모
│   └── cleanup.sh               # 프로세스 정리
├── settings/
│   ├── px4_2drones.json         # PX4 2대 설정
│   ├── px4_dual.json
│   ├── ardupilot_2drones.json   # APM 2대 설정
│   ├── apm_dual.json
│   └── zenoh_sim.json5          # Zenoh router 설정
├── docs/
│   ├── architecture.md          # 상세 시스템 아키텍처
│   ├── build-log.md             # 구축 과정 기록
│   ├── roadmap.md               # Phase 5~7 계획
│   └── system-specs.md          # 시스템 스펙
├── Colosseum/                   # (별도 clone, .gitignore)
└── PX4-Autopilot/               # (별도 clone, .gitignore)
```

## 상세 문서

- [architecture.md](docs/architecture.md) — 전체 시스템 아키텍처
- [build-log.md](docs/build-log.md) — 구축 과정, UE 5.6 패치, 알려진 문제
- [roadmap.md](docs/roadmap.md) — Phase 5~7 (도시환경, Zenoh 원격, ArduPilot)
- [system-specs.md](docs/system-specs.md) — 개발 머신 스펙

## 참고 링크

- [Colosseum (CodexLabsLLC)](https://github.com/CodexLabsLLC/Colosseum)
- [Colosseum 문서](https://codexlabsllc.github.io/Colosseum/)
- [PX4 SITL 설정](https://codexlabsllc.github.io/Colosseum/px4_sitl/)
- [멀티비클 설정](https://codexlabsllc.github.io/Colosseum/px4_multi_vehicle/)
- [rmw_zenoh](https://github.com/ros2/rmw_zenoh)
