# AERION airsim_ros2_bridge — 컨벤션

본 문서는 AERION 프로젝트의 ROS2/AirSim 인터페이스 컨벤션을 명문화합니다. 새 토픽/노드/모듈 추가 시 본 컨벤션을 따라야 외부 사용자(`Aerion-ardu-ws` Mac 측 + 다른 머신)와 정합됩니다.

근거: ROS REP-103 (단위) / REP-105 (좌표 frame) / image_transport / PX4 multi-vehicle / MAVROS 표준.

## 1. Vehicle key 명명

- **소문자 `drone1`..`drone5`** (1-base). settings.json의 `Vehicles` 키.
- AirSim `vehicle_name` 인자와 ROS2 namespace를 **동일 문자열**로 통일.
- 이유: bridge 측 publisher가 `f'/{vehicle_name}/...'` 형태로 토픽을 절대 경로 발행 → vehicle_name이 곧 namespace.

금지:
- `Drone0`, `Drone1` (대문자, 0-base) — 기존 인수인계 자산 호환용으로만 보존, 신규 작성 X.
- `uav0`, `uav1` — PX4 공식 launch 컨벤션이지만 AERION 외부 사용자(Mac 측)와 불일치.

## 2. ROS2 namespace

- 각 드론 toplevel: `/drone{N}`
- MAVROS 토픽: `/drone{N}/mavros/...` 하위 (PX4 공식 `multi_uav_mavros_sitl.launch` 패턴)
- 카메라: `/drone{N}/camera/...` 하위 (image_transport 컨벤션, `image_raw`/`camera_info` 같은 namespace)
- 거리센서: `/drone{N}/range/{front,left,right}`
- 포메이션 (글로벌, namespace 외부): `/aerion/formation/*`
- CARLA 환경 (Phase 1+): `/carla/world_info`, `/carla/vehicles/...`, `/carla/walkers/...`, `/carla/clock`

## 3. TF tree (REP-105)

```
map                                    (공통 글로벌, ENU, 단일 트리)
└── drone{N}/odom                      (REP-105 odom frame, drift 허용)
    └── drone{N}/base_link             (FLU: x=fwd, y=left, z=up, REP-103)
        ├── drone{N}/base_link_frd     (NED 보조, PX4 호환 디버깅용)
        ├── drone{N}/camera_link
        │   └── drone{N}/camera_optical_frame  (z=fwd, x=right, y=down, REP-103)
        ├── drone{N}/range_front_link
        ├── drone{N}/range_left_link
        └── drone{N}/range_right_link
```

- `robot_state_publisher`에 `frame_prefix:=drone{N}/` 적용 권장 (현재 미적용, 향후 작업).
- `map → drone{N}/odom`은 bridge가 static TF로 발행 (현재 미발행, 향후 작업).

## 4. 좌표/단위

- 좌표계: **REP-105 ENU** (x=East, y=North, z=Up).
- 단위: **REP-103 SI** (m, rad, s).
- AirSim 내부: NED (x=North, y=East, z=Down). bridge가 **노드 내부에서** ENU로 변환해 발행 (외부 사용자 노출 시 항상 ENU).
- MAVROS 표준 토픽(`local_position/pose`)도 자동으로 ENU.
- 시간: `/clock` (REP-2010), 모든 노드 `use_sim_time:=true`.

## 5. 표준 토픽 (외부 인터페이스)

### 5.1 상태 / 위치

| 토픽 | 타입 | QoS |
|---|---|---|
| `/drone{N}/mavros/state` | mavros_msgs/State | RELIABLE, depth=10 |
| `/drone{N}/mavros/battery` | sensor_msgs/BatteryState | RELIABLE, depth=10 |
| `/drone{N}/mavros/extended_state` | mavros_msgs/ExtendedState | RELIABLE, depth=10 |
| `/drone{N}/mavros/local_position/pose` | geometry_msgs/PoseStamped | RELIABLE, depth=10 |
| `/drone{N}/mavros/local_position/odom` | nav_msgs/Odometry | RELIABLE, depth=10 |
| `/drone{N}/mavros/local_position/velocity_local` | geometry_msgs/TwistStamped | RELIABLE, depth=10 |
| `/drone{N}/mavros/global_position/global` | sensor_msgs/NavSatFix | RELIABLE, depth=10 |

### 5.2 센서

| 토픽 | 타입 | QoS |
|---|---|---|
| `/drone{N}/mavros/imu/data` | sensor_msgs/Imu | SENSOR_DATA (BEST_EFFORT, depth=5) |
| `/drone{N}/camera/image` | sensor_msgs/Image | SENSOR_DATA (BEST_EFFORT, depth=1~5) |
| `/drone{N}/camera/camera_info` | sensor_msgs/CameraInfo | RELIABLE + **TRANSIENT_LOCAL** (지각 구독자 캘리브레이션 획득 필수) |
| `/drone{N}/camera/image/compressed` | sensor_msgs/CompressedImage | SENSOR_DATA (Phase 6 외부 노출용) |
| `/drone{N}/range/{front,left,right}` | sensor_msgs/Range | SENSOR_DATA |

### 5.3 제어

| 토픽 | 타입 | QoS |
|---|---|---|
| `/drone{N}/cmd_vel` | geometry_msgs/Twist | RELIABLE, depth=10 (AirSim 직접) |
| `/drone{N}/cmd_pos` | geometry_msgs/PoseStamped | RELIABLE, depth=10 (AirSim 직접) |
| `/drone{N}/mavros/setpoint_velocity/cmd_vel` | geometry_msgs/TwistStamped | RELIABLE (MAVROS 표준) |
| `/drone{N}/mavros/setpoint_position/local` | geometry_msgs/PoseStamped | RELIABLE (MAVROS 표준, **formation_node가 사용**) |

### 5.4 포메이션

| 토픽 | 타입 | QoS |
|---|---|---|
| `/aerion/formation/pattern` | std_msgs/String | RELIABLE + TRANSIENT_LOCAL (latched, 지각 구독자도 마지막 패턴 받음) |
| `/aerion/formation/leader_pose` | geometry_msgs/PoseStamped | RELIABLE, depth=10 |
| `/aerion/formation/status` | std_msgs/String | RELIABLE, depth=1 (2Hz publish) |

### 5.5 TF / 시계

| 토픽 | 타입 | QoS |
|---|---|---|
| `/tf` | tf2_msgs/TFMessage | RELIABLE, depth=100 |
| `/tf_static` | tf2_msgs/TFMessage | RELIABLE + TRANSIENT_LOCAL |
| `/clock` | rosgraph_msgs/Clock | RELIABLE |

## 6. 메시지 타입 (커스텀 금지 원칙)

표준 ROS2 메시지(`sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `tf2_msgs`, `mavros_msgs`)만 사용. 커스텀 메시지 패키지(`aerion_msgs`)는 **현재 만들지 않음**.

이유:
- 외부 사용자가 우리 패키지를 빌드/import 안 해도 토픽 구독 가능.
- 메시지 호환성 유지 부담 제거.
- `/aerion/formation/pattern`은 std_msgs/String으로 충분 (열거형은 코드 상수로).

향후 커스텀 메시지 필요 조건 (이 중 하나 이상이면 검토):
- `/aerion/formation/pattern`에 패턴별 동적 파라미터 (e.g. 마름모 반지름) 필요
- 멀티드론 한 토픽에 통합 publish가 자원 효율 큼 (현재는 per-drone 발행)

## 7. AirSim ↔ ROS 좌표 변환 위치

- **변환은 bridge 내부에서**. 외부에 노출되는 모든 토픽은 ENU.
- `static_transform_publisher`로 NED→ENU 회전 외부에서 처리하면 frame_id 의미 뒤섞임 (REP-103 위반).
- AirSim ROS 래퍼 옵션 (참고용): `coordinate_system_enu:=true`, `world_frame_id:=world_enu`.

## 8. QoS 프로파일

```python
# 권장 QoS profile (rclpy)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

SENSOR_DATA = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
RELIABLE_LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
RELIABLE_DEFAULT = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)
```

현재 코드는 대부분 `create_publisher(..., 10)` 형태로 기본(RELIABLE depth=10)만 사용. 향후 카메라/range를 SENSOR_DATA로, camera_info/tf_static/pattern을 LATCHED로 명시 적용 예정.

## 9. ROS_DOMAIN_ID 분리

| 용도 | DOMAIN_ID |
|---|---|
| CARLA 시뮬레이션 (5 UAV 내부) | 42 |
| 실험/검증 (HIL, 단일 드론) | 50 |
| 외부 사용자 노출 (read-only) | 7 |
| 개발 로컬 (테스트) | 0 |

5대 UAV는 동일 도메인(42) + namespace 분리. 도메인 분리는 외부 사용자/실험계 격리에만.

## 10. DDS 미들웨어

- **CycloneDDS** (rmw_cyclonedds_cpp). Zenoh는 카메라 이미지 전송 실패 경험으로 폐기 (Aerion-ardu-ws 검증).
- 환경변수: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI=file:///home/clrobur/airsim/cyclonedds.xml`.
- sysctl 영구화: `/etc/sysctl.d/30-aerion-dds.conf` (rmem_max=2147483647 등).

## 11. 새 토픽 추가 시 체크리스트

- [ ] namespace 컨벤션 (`/drone{N}/...` 또는 `/aerion/...` 또는 `/carla/...`)
- [ ] 표준 메시지 타입 (커스텀 사용 X)
- [ ] QoS 명시 (BEST_EFFORT/RELIABLE/TRANSIENT_LOCAL)
- [ ] frame_id 컨벤션 (`drone{N}/...`)
- [ ] CHANGELOG에 추가 사유 기록
- [ ] ARCHITECTURE.md 모듈 책임 갱신
- [ ] 외부 노출 화이트리스트 (Phase 6 domain_bridge config) 검토
