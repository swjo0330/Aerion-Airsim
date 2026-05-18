# AERION 토픽 인터페이스 스펙

외부 사용자(다른 머신 또는 컨테이너)가 AERION 시뮬레이션에 접속해 표준 ROS2 토픽으로 데이터를 받고 명령을 보낼 수 있도록 설계된 토픽 인터페이스 명세.

본 문서는 토픽 이름/메시지 타입/QoS/frame_id 컨벤션의 **단일 진실 출처**. 사용자 시스템 통합 시 이 스펙을 따르면 AERION 토픽과 정합.

**근거**: ROS REP-103 (단위) / REP-105 (좌표 frame) / image_transport / PX4 multi-vehicle / MAVROS 표준.

## 1. 좌표/단위

- **좌표계**: REP-105 ENU (x=East, y=North, z=Up).
- **단위**: REP-103 SI (m, rad, s).
- **시간**: `/clock` (REP-2010), 모든 외부 노드 `use_sim_time:=true` 권장.
- AirSim 내부 NED는 bridge가 ENU로 변환해 발행. 외부 사용자는 ENU만 봄.

## 2. 드론별 토픽 (N = 1..5)

### 2.1 상태 (RELIABLE, depth=10)

| 토픽 | 메시지 | frame_id (msg.header.frame_id) | 설명 |
|---|---|---|---|
| `/drone{N}/mavros/state` | `mavros_msgs/State` | n/a | MAVROS 연결/모드 (Phase 5+) |
| `/drone{N}/mavros/extended_state` | `mavros_msgs/ExtendedState` | n/a | 비행 상태 (Phase 5+) |
| `/drone{N}/mavros/battery` | `sensor_msgs/BatteryState` | `drone{N}/base_link` | 배터리 |

### 2.2 위치 / 자세

| 토픽 | 메시지 | frame_id | QoS |
|---|---|---|---|
| `/drone{N}/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `drone{N}/odom` | RELIABLE 10 |
| `/drone{N}/mavros/local_position/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | `drone{N}/odom` | RELIABLE 10 |
| `/drone{N}/mavros/local_position/odom` | `nav_msgs/Odometry` | `drone{N}/odom` | RELIABLE 10 |
| `/drone{N}/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | `drone{N}/odom` | RELIABLE 10 |
| `/drone{N}/mavros/local_position/velocity_body` | `geometry_msgs/TwistStamped` | `drone{N}/base_link` | RELIABLE 10 |
| `/drone{N}/mavros/global_position/global` | `sensor_msgs/NavSatFix` | n/a | RELIABLE 10 |
| `/drone{N}/mavros/global_position/local` | `nav_msgs/Odometry` | `drone{N}/odom` | RELIABLE 10 |
| `/drone{N}/mavros/global_position/rel_alt` | `std_msgs/Float64` | n/a | RELIABLE 10 |
| `/drone{N}/mavros/global_position/compass_hdg` | `std_msgs/Float64` | n/a | RELIABLE 10 |

### 2.3 IMU / 센서 (SENSOR_DATA = BEST_EFFORT depth=5)

| 토픽 | 메시지 | frame_id |
|---|---|---|
| `/drone{N}/mavros/imu/data` | `sensor_msgs/Imu` | `drone{N}/base_link` |
| `/drone{N}/mavros/imu/data_raw` | `sensor_msgs/Imu` | `drone{N}/base_link` |
| `/drone{N}/range/front` | `sensor_msgs/Range` | `drone{N}/range_front_link` |
| `/drone{N}/range/left` | `sensor_msgs/Range` | `drone{N}/range_left_link` |
| `/drone{N}/range/right` | `sensor_msgs/Range` | `drone{N}/range_right_link` |

거리센서: `radiation_type=INFRARED (1)`, `field_of_view=0.035 rad` (단일 ray-cast 모사), `min_range=0.2m`, `max_range=20~40m`.

### 2.4 카메라

| 토픽 | 메시지 | frame_id | QoS |
|---|---|---|---|
| `/drone{N}/camera/image` | `sensor_msgs/Image` | `drone{N}/camera_optical_frame` | BEST_EFFORT depth=1~5 |
| `/drone{N}/camera/camera_info` | `sensor_msgs/CameraInfo` | `drone{N}/camera_optical_frame` | RELIABLE + **TRANSIENT_LOCAL** (지각 구독자 캘리브레이션 획득 필수) |

기본 해상도: 320x240 RGB rgb8 (Phase 6 외부 노출 시 `image/compressed` ffmpeg_image_transport 사용 권장 — 5채널 raw는 138MB/s).

### 2.5 제어 (구독, 외부 명령용)

bridge는 다음 토픽을 구독해 AirSim API 호출:

| 토픽 | 메시지 | 의미 |
|---|---|---|
| `/drone{N}/cmd_vel` | `geometry_msgs/Twist` | AirSim NED 속도 (legacy, 호환용) |
| `/drone{N}/cmd_pos` | `geometry_msgs/PoseStamped` | AirSim NED 위치 (legacy) |
| `/drone{N}/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | ROS ENU → bridge가 NED로 변환 |
| `/drone{N}/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | 동일 |
| `/drone{N}/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | ROS ENU → NED 변환 (formation_node가 발행) |

## 3. 글로벌 토픽

### 3.1 포메이션

| 토픽 | 메시지 | 방향 | QoS |
|---|---|---|---|
| `/aerion/formation/pattern` | `std_msgs/String` (LINE/DIAMOND/ARROW/V/ECHELON) | 외부 → formation_node | RELIABLE + TRANSIENT_LOCAL (latched 권장) |
| `/aerion/formation/leader_pose` | `geometry_msgs/PoseStamped` | 외부 mission planner → formation_node | RELIABLE 10 |
| `/aerion/formation/status` | `std_msgs/String` (NO_LEADER/SETTLING/STABLE/OBSTACLE_HOVER) | formation_node → 외부 | RELIABLE 1 |

### 3.2 TF / 시계

| 토픽 | 메시지 | QoS |
|---|---|---|
| `/tf` | `tf2_msgs/TFMessage` | RELIABLE 100 VOLATILE |
| `/tf_static` | `tf2_msgs/TFMessage` | RELIABLE + **TRANSIENT_LOCAL** |
| `/clock` | `rosgraph_msgs/Clock` | RELIABLE |

## 4. TF tree (REP-105)

```
map                                    (공통 글로벌, ENU)
└── drone{N}/odom                      (spawn 위치 기반 static, drift 허용 frame)
    └── drone{N}/base_link             (FLU, REP-103: x-fwd, y-left, z-up)
        ├── drone{N}/camera_link
        │   └── drone{N}/camera_optical_frame  (REP-103: z-fwd, x-right, y-down)
        ├── drone{N}/range_front_link
        ├── drone{N}/range_left_link
        └── drone{N}/range_right_link
```

발행 주체: `aerion_tf_publisher` 노드 (`tf_publisher.py`).

## 5. ROS_DOMAIN_ID 분리 (Phase 6)

| 용도 | DOMAIN_ID | 비고 |
|---|---|---|
| 시뮬레이션 내부 (5 UAV + bridge + formation) | **42** | 노트북 + 맥 |
| 외부 사용자 (read-only) | **7** | `domain_bridge`로 화이트리스트 토픽만 |

외부 사용자가 받을 화이트리스트:
- `/drone{N}/{mavros/state, local_position/pose, camera/image/compressed, range/*}`
- `/aerion/formation/{pattern,leader_pose,status}`
- `/tf`, `/tf_static`, `/clock`

**MAVROS 노이즈 토픽 제외** (parameter events, raw IMU, log_transfer 등).

## 6. CycloneDDS 권장 설정 (외부 사용자 측)

```xml
<CycloneDDS><Domain Id="7"><General>
  <Interfaces><NetworkInterface name="<유선 NIC>" priority="10" multicast="default"/></Interfaces>
  <AllowMulticast>spdp</AllowMulticast>
  <MaxMessageSize>1400B</MaxMessageSize>
  <FragmentSize>1344B</FragmentSize>
</General>
<Discovery><Peers><Peer Address="<AERION 노트북 Tailscale IP>"/></Peers></Discovery>
<Internal><SocketReceiveBufferSize min="10MB"/></Internal>
</Domain></CycloneDDS>
```

+ sysctl 권장: `net.core.rmem_max=2147483647` (영구화 `/etc/sysctl.d/30-aerion-dds.conf`).

## 7. 메시지 타입 정책

**표준 ROS2 메시지만 사용**. 커스텀 메시지 패키지(`aerion_msgs`) 미생성.

이유:
- 외부 사용자가 우리 패키지 빌드/import 없이 토픽 구독 가능
- 메시지 호환성 유지 부담 제거
- `/aerion/formation/pattern`은 `std_msgs/String`으로 충분 (열거형은 코드 상수)

향후 커스텀 메시지 검토 조건:
- 패턴별 동적 파라미터 (예: 마름모 반지름) 필요
- 멀티드론 한 토픽에 통합 publish 자원 효율 큼

## 8. 변경 이력 / 호환성

본 인터페이스 변경 시 [CHANGELOG.md](../CHANGELOG.md)의 "Topic Interface" 섹션에 기록. 메이저 버전(`1.0` → `2.0`)은 호환성 깨짐, 마이너(`1.0` → `1.1`)는 토픽 추가만.

현재 버전: **1.0 (2026-05-18)** — Phase 3 시연 완료 시점.
