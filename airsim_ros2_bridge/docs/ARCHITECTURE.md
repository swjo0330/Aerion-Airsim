# AERION airsim_ros2_bridge — 아키텍처

본 문서는 패키지의 모듈 구성, 책임 분리, 토픽 인터페이스, 외부 시스템과의 관계를 명세합니다. 코드 변경 전 본 문서를 참조하고, 변경 후 [CHANGELOG.md](../CHANGELOG.md) 와 본 문서 동기화.

## 1. 시스템 전체 그림

```
┌──────────────────────────────── UE 5.6.1 (단독) / UE 5.5 (CARLA 통합) ────────────────────────────────┐
│                                                                                                       │
│   ┌────────────────────────────┐                  (Phase 1~ 진입 후 CARLA 통합)                       │
│   │ Colosseum (AirSim)         │  ◄──RPC :41451─────────────┐                                         │
│   │  - SimMode_Multirotor      │                            │                                         │
│   │  - SimpleFlight per drone  │                            │                                         │
│   │  - Camera + Distance×3     │                            │                                         │
│   └────────────────────────────┘                            │                                         │
│   ┌────────────────────────────┐                            │                                         │
│   │ CARLA 0.10.0 (Phase 1~)    │  ◄──RPC :2000──────────┐   │                                         │
│   │  - Town10HD, vehicles      │                        │   │                                         │
│   │  - synchronous_mode opt    │                        │   │                                         │
│   └────────────────────────────┘                        │   │                                         │
└─────────────────────────────────────────────────────────┼───┼─────────────────────────────────────────┘
                                                          │   │
                                                          ▼   ▼
         ┌────────────────────────────────────────────────────────────────────┐
         │                  ROS2 Humble (CycloneDDS, DOMAIN_ID=42)            │
         │                                                                    │
         │   ┌─────────────────────────┐    ┌─────────────────────────┐       │
         │   │ airsim_ros2_bridge × N  │    │ formation_node (단일)   │       │
         │   │  (1 드론 = 1 프로세스)  │    │  /aerion/formation/*    │       │
         │   │                         │    │  20Hz setpoint publish  │       │
         │   │  CameraPublisher        │    └───────────┬─────────────┘       │
         │   │  RangePublisher         │                │                     │
         │   │  DroneController        │  per-drone     │                     │
         │   │   (MAVROS alias)        │  pose/range    │ setpoint            │
         │   │                         │  subscribe     │ publish             │
         │   └────────────┬────────────┘                │                     │
         │                │ /drone{N}/*                 │                     │
         │                ▼                             ▼                     │
         │   ┌────────────────────────────────────────────────────┐           │
         │   │  /drone1..5/{camera,range,mavros}/...              │           │
         │   │  /aerion/formation/{pattern,leader_pose,status}    │           │
         │   └─────────────────────┬──────────────────────────────┘           │
         └─────────────────────────┼──────────────────────────────────────────┘
                                   │ domain_bridge (Phase 6)
                                   ▼
                ┌──────────────────────────────────────────┐
                │ 외부 사용자 (ROS_DOMAIN_ID=7, Tailscale) │
                │  RViz / 로그 / embodied-drone (Mac)      │
                └──────────────────────────────────────────┘
```

## 2. 모듈 책임 (단일 책임 원칙)

| 모듈 | 책임 | 발행하지 않는 토픽 |
|---|---|---|
| `bridge_node.py` | 진입점 + ThreadSafeAirSimClient + DroneController/Camera/Range 인스턴스 lifecycle | 자체 발행 없음 (publisher들이 발행) |
| `camera_publisher.py` | RGB 카메라 한 대 → `/drone{N}/camera/{image,camera_info}` | range, mavros |
| `range_publisher.py` | 거리센서 3개 → `/drone{N}/range/{front,left,right}` | camera, mavros |
| `drone_controller.py` | MAVROS 호환 토픽 발행 + 명령 구독 + AirSim API 직접 제어 (airsim_direct / px4_mavros 분기) | 자체 카메라/range 발행 X |
| `formation_node.py` | 포메이션 패턴 → per-drone setpoint publish + 회피 hook | arm/takeoff/land (헬퍼 책임) |
| `utils.py` | FOV→intrinsics, AirSim RGB→Image 메시지 변환 | — |
| `scripts/airsim_arm_all.py` | N대 일괄 arm + takeoff (SimpleFlight) | ROS 토픽 발행 X (AirSim RPC만) |
| `scripts/airsim_land_all.py` | N대 일괄 land + disarm | ROS 토픽 발행 X |
| `scripts/smoke_aerion_phase2.py` | Phase 2 무결성 검증 (단일 드론) | — |
| `scripts/verify_topics_phase3.py` | Phase 3 토픽 인터페이스 PASS/FAIL 자동 판정 | — |

## 3. 1 드론 = 1 프로세스 패턴 (왜?)

```
   ┌──────────────────────┐     ┌──────────────────────┐
   │ bridge_node          │     │ bridge_node          │   ... (N개)
   │  __ns:=/drone1       │     │  __ns:=/drone2       │
   │  vehicle_name=drone1 │     │  vehicle_name=drone2 │
   │                      │     │                      │
   │  airsim.MultirotorClient   airsim.MultirotorClient
   │  (자기 vehicle만 호출)     (자기 vehicle만 호출)
   └──────────┬───────────┘     └──────────┬───────────┘
              │                            │
              └─────────────┬──────────────┘
                            │ TCP :41451
                            ▼
                  ┌──────────────────┐
                  │ AirSim RPC 서버  │
                  │  (50Hz 상한)     │
                  └──────────────────┘
```

이유:
1. AirSim RPC IOLoop는 단일 클라이언트 인스턴스 안에서 동시 호출 시 'IOLoop is already running' 에러 (microsoft/AirSim#2607).
2. 한 드론 RPC 타임아웃이 다른 드론에 영향 0 (격리).
3. ROS namespace 자연 분리 → 토픽 이름 충돌 없음.
4. 노드별 CPU/메모리/로그 분리 → 디버깅 효율.

비용: 프로세스당 rclpy + airsim client 인스턴스 중복 (~150-250MB×N). 5대 = ~1GB. 노트북 62GiB라 무관.

## 4. 외부 사용자 노출 (Phase 6, 계획)

- 내부 DOMAIN_ID=42, 외부 DOMAIN_ID=7. `domain_bridge`로 화이트리스트 토픽만 export.
- Tailscale (이미 노트북 설치됨, `100.120.219.68`) → VPN 없이 즉시 외부 노출 가능.
- 외부 노출 화이트리스트 (CONVENTIONS.md에 명시):
  - `/drone{N}/{mavros/state, local_position/pose, camera/rgb/image_raw/compressed, range/*}`
  - `/aerion/formation/*`
  - `/tf`, `/tf_static`, `/clock`
  - **MAVROS 노이즈 토픽 제외** (parameter events, raw IMU 등)

## 5. Phase별 진입점

| Phase | 사용 모듈 / 스크립트 |
|---|---|
| Phase 2 (단일 드론 무결성) | `bridge_node` + `aerion_single_drone.launch.py` + `smoke_aerion_phase2.py` |
| Phase 3 (1~5대 확장) | `bridge_node` × N + `aerion_multi_drone.launch.py` + `verify_topics_phase3.py` |
| Phase 4 (포메이션) | Phase 3 위에 + `airsim_arm_all.py` → `formation_node` → `airsim_land_all.py` |
| Phase 5 (PX4 SITL) | DroneController `control_backend='px4_mavros'` + MAVROS 5개 인스턴스 + (TBD) `mavros_arm_all.py` |
| Phase 6 (외부 노출) | `domain_bridge` config + Tailscale peer 설정 + cyclonedds.xml Peers |

## 6. 외부 의존성

- ROS2 Humble (`/opt/ros/humble`)
- rmw_cyclonedds_cpp (`apt install ros-humble-rmw-cyclonedds-cpp`)
- ROS 메시지: `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `tf2_msgs`, `mavros_msgs` (Phase 5)
- Python (system 3.10): `airsim` (editable install `~/airsim/Colosseum/PythonClient`)
- Python (.venv 3.12): 옵션 (별도 테스트용)
- CycloneDDS XML: `~/airsim/cyclonedds.xml`
- AirSim settings.json: `~/Documents/AirSim/settings.json`

## 7. 향후 확장 (잠재적 모듈)

- `services_proxy.py` — `/aerion/formation/takeoff_all`, `/land_all` 등 ROS 서비스 인터페이스 (현재는 헬퍼 스크립트로 대체)
- `tf_publisher.py` — `map → drone{N}/odom → drone{N}/base_link` static + dynamic TF 발행 (현재는 robot_state_publisher 없음)
- `image_compressed_republisher.py` — Phase 6 외부 노출 시 raw → compressed (ffmpeg_image_transport)
- `obstacle_field.py` — Range 3개 → 점유 그리드 또는 vector field (Phase 4 회피 hook 고도화)
- `simulation_clock_bridge.py` — Phase 1 CARLA synchronous_mode ↔ AirSim SteppableClock 동기화 (CarlaAir 시간 동기화 부재 함정 해소)

각 확장은 별도 task로 PLAN.md에 추가 후 진행.
