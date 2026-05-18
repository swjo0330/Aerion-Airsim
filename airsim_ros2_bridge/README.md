# airsim_ros2_bridge

AERION 프로젝트의 AirSim/Colosseum ↔ ROS2 Humble 브릿지 패키지. 단일 머신 + 멀티 드론 환경에서 1 드론 = 1 프로세스 패턴으로 운용.

본 패키지의 근본 철학은 [swjo0330/Aerion-Airsim](https://github.com/swjo0330/Aerion-Airsim) 의 `branc/airsim-ros2-bridge` 브랜치 기조를 유지합니다.

## 빠른 시작 (Phase 3 5대 시연)

```bash
# 1) 사전 (한 번만)
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
python3 -m pip install --user --no-build-isolation -e ~/airsim/Colosseum/PythonClient
# /etc/sysctl.d/30-aerion-dds.conf 영구화 — CycloneDDS 카메라 buffer 확보

# 2) settings deploy + UE Editor 실행 + Play
cp ~/airsim/settings/sf_5drones_phase3.json ~/Documents/AirSim/settings.json
~/airsim/unreal-engine/Engine/Binaries/Linux/UnrealEditor \
  ~/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject &
# UE 로딩 후 ▶ Play 클릭, 5대 spawn 확인:
cd /tmp && python3 -c "import airsim; c=airsim.MultirotorClient(); print(c.listVehicles())"

# 3) 한 줄로 bridge×5 + formation + leader + tf publisher 기동
source /opt/ros/humble/setup.bash
source ~/airsim/install/setup.bash
ros2 launch airsim_ros2_bridge aerion_phase4_formation.launch.py drone_count:=5

# 4) 5대 일괄 takeoff
python3 ~/airsim/airsim_ros2_bridge/scripts/airsim_arm_all.py --drones 5 --altitude 5.0

# 5) 포메이션 시퀀스 시연
python3 ~/airsim/airsim_ros2_bridge/scripts/aerion_mission.py \
  --sequence "LINE:30,DIAMOND:30,ARROW:30,V:30,ECHELON:30"

# 6) RViz 시각화 (옵션)
rviz2 -d $(ros2 pkg prefix airsim_ros2_bridge)/share/airsim_ros2_bridge/config/aerion_5drones.rviz

# 7) 토픽 인터페이스 자동 검증
python3 ~/airsim/airsim_ros2_bridge/scripts/verify_topics_phase3.py --drones 5 --json-out /tmp/phase3.json

# 8) 일괄 land + disarm
python3 ~/airsim/airsim_ros2_bridge/scripts/airsim_land_all.py --drones 5
```

## 디렉토리 구조

```
airsim_ros2_bridge/
├── airsim_ros2_bridge/         # 모듈 (재사용 가능)
│   ├── bridge_node.py          # 메인 노드, ThreadSafeAirSimClient + Publisher/Controller 라이프사이클
│   ├── camera_publisher.py     # RGB 카메라 → /drone{N}/camera/{image,camera_info}
│   ├── range_publisher.py      # 거리센서 ×3 → /drone{N}/range/{front,left,right}  (sensor_msgs/Range)
│   ├── drone_controller.py     # MAVROS 호환 토픽 + AirSim API 직접 제어 (airsim_direct/px4_mavros 분기)
│   ├── formation_node.py       # Phase 4 동적 포메이션 (LINE/DIAMOND/ARROW/V/ECHELON + 회피 hook + 도착 감지)
│   ├── leader_publisher.py     # dummy leader_pose 자동 발행 (static/circle/line 모드)
│   ├── tf_publisher.py         # TF tree (map → drone{N}/odom → base_link → sensor frames)
│   └── utils.py                # FOV→intrinsics, AirSim RGB→Image 메시지 변환
├── scripts/                    # 진입점 + 헬퍼
│   ├── aerion_formation.py     # formation_node 진입점 (ros2 run aerion_formation)
│   ├── aerion_mission.py       # 단일 진입 CLI (takeoff → pattern sequence → land)
│   ├── airsim_arm_all.py       # N대 일괄 arm + takeoff (SimpleFlight 전용, hover stabilize 포함)
│   ├── airsim_land_all.py      # N대 일괄 land + disarm
│   ├── smoke_aerion_phase2.py  # Phase 2 단일 드론 smoke (takeoff/forward/land)
│   ├── verify_topics_phase3.py # Phase 3 토픽 인터페이스 자동 PASS/FAIL (JSON 출력)
│   └── run_airsim_ros2_bridge_instances.sh  # bash 멀티프로세스 (인수인계 자산, 사용은 ros2 launch 권장)
├── launch/                     # ros2 launch 파일
│   ├── aerion_single_drone.launch.py     # Phase 2 단일 드론
│   ├── aerion_multi_drone.launch.py      # Phase 3 1~5대 bridge만
│   └── aerion_phase4_formation.launch.py # Phase 3+4 통합 (bridge×N + formation + leader + tf)
├── config/
│   └── aerion_5drones.rviz     # RViz 설정 (camera ×5 + pose ×5 + range ×3 + leader)
├── docs/
│   ├── ARCHITECTURE.md         # 시스템 구조 + 모듈 책임 + 1 드론 = 1 프로세스 패턴
│   ├── CONVENTIONS.md          # vehicle/namespace/frame_id/QoS 표준
│   └── TOPIC_INTERFACE.md      # 외부 사용자용 토픽 인터페이스 스펙
├── CHANGELOG.md                # 변경 이력 (Unreleased 누적)
├── README.md                   # 이 파일
└── (package.xml, setup.py, setup.cfg, resource/)
```

## 의존성

- ROS2 Humble (`/opt/ros/humble`)
- rmw_cyclonedds_cpp (`apt install ros-humble-rmw-cyclonedds-cpp`)
- Python 3.10 (시스템) + `airsim` editable install (`~/airsim/Colosseum/PythonClient`)
- CycloneDDS XML: `~/airsim/cyclonedds.xml`
- AirSim settings: `~/Documents/AirSim/settings.json`

## 환경 변수

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/clrobur/airsim/cyclonedds.xml
```

(`~/.bashrc`에 이미 등록되어 있어 새 셸 자동 적용)

## 단일 책임 원칙

- `bridge_node` : RPC client + Publisher/Controller 라이프사이클
- `*Publisher`  : 한 가지 센서/도메인 토픽 발행
- `formation_node` : 포메이션 패턴 → setpoint publish (arm/takeoff/land 책임 X)
- `*_all.py` 헬퍼 : 5대 일괄 제어 (한 가지 동작만)
- `tf_publisher` : TF tree만 발행 (시각화용, 제어 책임 X)

새 토픽/노드 추가 시 [docs/CONVENTIONS.md](docs/CONVENTIONS.md) 준수 + [CHANGELOG.md](CHANGELOG.md)에 기록 + [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) 모듈 책임 갱신.

## 외부 사용자 (Phase 6, 계획)

토픽 인터페이스 스펙 (메시지 타입 + QoS + frame_id)은 [docs/TOPIC_INTERFACE.md](docs/TOPIC_INTERFACE.md) 참조. `domain_bridge`로 외부 `ROS_DOMAIN_ID=7`에 화이트리스트 토픽만 노출 예정 (Tailscale 활용 가능, IP `100.120.219.68`).

## 알려진 함정 (Phase 3 시연 중 발견)

1. `python3 -c "import airsim"`은 `~/airsim/`이 있는 cwd에서 실행하면 namespace 패키지로 가림. **`cd /tmp` 회피**.
2. UE Play Stop 시 bridge들은 살아있지만 RPC TimeoutError로 토픽 발행 정지. **재 Play 시 자동 복귀 시도**.
3. `~/.bashrc`에 잘못된 `CYCLONEDDS_URI` inline XML (wlo1 강제)이 있으면 모든 ROS2 노드 fail. **file:// URI 권장**.
4. settings.json의 vehicle key를 `Drone0`(대문자) 대신 **소문자 `drone1..drone5`** 로 통일하면 토픽이 자동으로 `/drone1/...` 표준 형태로 발행됨.
5. Colosseum의 distance 메서드는 `getDistanceSensorData` (sim 접두사 없음). Microsoft 본가 AirSim과 다름.

## 라이선스

본 패키지는 Aerion-Airsim 프로젝트의 일부. 본 패키지 코드는 Apache-2.0. AirSim/Colosseum 코드는 MIT.
