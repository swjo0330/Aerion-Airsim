# 시스템 구조 문서

**작성일:** 2026-04-09

---

## 전체 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    시뮬레이션 머신                            │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │         Unreal Engine 5.6.1 + Colosseum 플러그인      │   │
│  │                                                     │   │
│  │   [uav0 드론 물리 시뮬레이션]  [uav1 드론 물리 시뮬레이션]  │   │
│  │        카메라 / 센서 / 모터                           │   │
│  └───────────────────┬─────────────────────────────────┘   │
│                      │ TCP :41451 (AirSim API)              │
│  ┌───────────────────▼─────────────────────────────────┐   │
│  │         airsim Python 패키지 (PythonClient)           │   │
│  │   - 이미지 캡처 / 카메라 파라미터 조회                 │   │
│  │   - 드론 이동 명령 (moveByVelocity / moveToPosition)  │   │
│  └───────────────────┬─────────────────────────────────┘   │
│                      │ Python import                        │
│  ┌───────────────────▼─────────────────────────────────┐   │
│  │              bridge_node.py (ROS2 노드)               │   │
│  │                                                     │   │
│  │  [CameraPublisher]          [DroneController]        │   │
│  │  airsim 이미지 → ROS2 발행   ROS2 토픽 → airsim 제어  │   │
│  │                                                     │   │
│  │  발행:                       구독:                   │   │
│  │  /uav0/camera/image          /uav0/cmd_vel           │   │
│  │  /uav0/camera/camera_info    /uav0/cmd_pos           │   │
│  │  /uav1/camera/image          /uav1/cmd_vel           │   │
│  │  /uav1/camera/camera_info    /uav1/cmd_pos           │   │
│  └───────────────────┬─────────────────────────────────┘   │
│                      │ RMW: rmw_zenoh_cpp                   │
│  ┌───────────────────▼─────────────────────────────────┐   │
│  │              PX4 SITL (2대)                          │   │
│  │  uav0: TCP:4560, MAVLink UDP:14555                   │   │
│  │  uav1: TCP:4561, MAVLink UDP:14556                   │   │
│  └───────────────────┬─────────────────────────────────┘   │
│                      │ UDP MAVLink                          │
└──────────────────────┼──────────────────────────────────────┘
                       │
         (네트워크: rmw_zenoh TCP:7447 / MAVLink UDP)
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                    원격 제어 머신                             │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              MAVROS (2대)                            │   │
│  │  uav0: fcu_url=udp://:14540@SIM_IP:14555            │   │
│  │  uav1: fcu_url=udp://:14541@SIM_IP:14556            │   │
│  └───────────────────┬─────────────────────────────────┘   │
│                      │ ROS2 토픽                             │
│  ┌───────────────────▼─────────────────────────────────┐   │
│  │           embodied-drone 노드                        │   │
│  │                                                     │   │
│  │  구독:                        발행:                  │   │
│  │  /uav0/camera/image           /uav0/cmd_vel          │   │
│  │  /uav0/camera/camera_info     /uav0/cmd_pos          │   │
│  │  /uav1/camera/image           /uav1/cmd_vel          │   │
│  │  /uav1/camera/camera_info     /uav1/cmd_pos          │   │
│  │                                                     │   │
│  │  RMW: rmw_zenoh_cpp (이미 사용 중)                   │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

---

## 컴포넌트별 역할

### 1. Unreal Engine + Colosseum 플러그인
- 드론의 물리 시뮬레이션 (중력, 공기저항, 모터)
- 카메라 렌더링 (1280×720 RGB, FOV 90°)
- 센서 데이터 생성 (IMU, GPS, 기압계)
- AirSim API 서버 노출 (TCP :41451)

### 2. airsim Python 패키지 (`Colosseum/PythonClient/`)
- UE/Colosseum과 TCP로 통신하는 Python API
- 이미지 캡처, 카메라 정보 조회, 드론 이동 명령
- bridge_node가 import해서 사용

### 3. bridge_node (`airsim_ros2_bridge/bridge_node.py`)
- **역할**: airsim Python API ↔ ROS2 토픽 변환기
- **실행 환경**: Python 3.10 (ROS2 Humble 시스템 Python)
- **PYTHONPATH**: user site-packages(numpy) + PythonClient(airsim)
- **RMW**: rmw_zenoh_cpp → 원격 머신과 토픽 자동 교환

| 방향 | 컴포넌트 | 토픽 | 타입 |
|------|----------|------|------|
| 발행 | CameraPublisher | `/uavN/camera/image` | `sensor_msgs/Image` (Raw RGB) |
| 발행 | CameraPublisher | `/uavN/camera/camera_info` | `sensor_msgs/CameraInfo` |
| 구독 | DroneController | `/uavN/cmd_vel` | `geometry_msgs/Twist` |
| 구독 | DroneController | `/uavN/cmd_pos` | `geometry_msgs/PoseStamped` |

### 4. PX4 SITL (2대)
- 비행 제어 펌웨어 시뮬레이션
- Colosseum과 TCP lockstep (4560/4561)
- MAVLink를 UDP로 원격 MAVROS에 전송 (14555/14556)

### 5. MAVROS (원격 머신)
- MAVLink ↔ ROS2 변환
- `fcu_url=udp://:14540@SIM_IP:14555` 형식으로 PX4 SITL 직접 연결

### 6. embodied-drone (원격 머신)
- 체화지능 제어 노드
- ROS2 토픽만 사용 (airsim/MAVLink 직접 접촉 없음)
- rmw_zenoh 이미 사용 중 → 별도 브릿지 불필요

---

## 통신 프로토콜 요약

| 구간 | 프로토콜 | 포트 |
|------|----------|------|
| Colosseum ↔ airsim API | TCP | :41451 |
| Colosseum ↔ PX4 SITL | TCP (lockstep) | :4560, :4561 |
| PX4 SITL → MAVROS | UDP MAVLink | :14555, :14556 |
| bridge_node ↔ embodied-drone | ROS2 (rmw_zenoh) | TCP :7447 |

---

## 실행 순서

```bash
# 1. 프로세스 정리
bash scripts/cleanup.sh

# 2. settings.json 생성
python3 scripts/generate_settings.py -f px4 -n 2 --deploy

# 3. Unreal Engine 실행 + Play
bash scripts/launch_ue.sh blocks px4

# 4. PX4 SITL 2대 기동
cd PX4-Autopilot
PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4 -i 0 -d ROMFS/px4fmu_common &
PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4 -i 1 -d ROMFS/px4fmu_common &

# 5. ROS2 브릿지 노드 실행
bash scripts/run_bridge.sh

# [원격 머신]
# 6. MAVROS 2대 실행
# 7. embodied-drone 노드 실행
```

---

## 파일 구조

```
/home/clrobur/airsim/
├── Colosseum/
│   ├── PythonClient/airsim/       # airsim Python API
│   └── Unreal/Environments/
│       └── BlocksV2/              # 시뮬레이션 환경 (검증됨)
├── PX4-Autopilot/                 # PX4 SITL 빌드
├── airsim_ros2_bridge/
│   └── airsim_ros2_bridge/
│       ├── bridge_node.py         # 메인 ROS2 노드
│       ├── camera_publisher.py    # 카메라 이미지/파라미터 발행
│       ├── drone_controller.py    # 드론 제어 토픽 구독
│       └── utils.py               # 메시지 변환 유틸
├── scripts/
│   ├── cleanup.sh                 # 프로세스 정리
│   ├── generate_settings.py       # settings.json 생성기
│   ├── launch_ue.sh               # UE 실행 스크립트
│   └── run_bridge.sh              # 브릿지 노드 실행 (rmw_zenoh)
└── docs/
    ├── 시스템구조.md               # 이 문서
    ├── 구축기록.md                 # 구축 과정 기록
    └── 다음단계계획.md             # Phase 5~7 계획
```
