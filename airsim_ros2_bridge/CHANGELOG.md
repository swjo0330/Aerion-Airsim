# airsim_ros2_bridge 변경 이력

AERION 프로젝트의 ROS2 브릿지 패키지 변경 이력을 시간순으로 기록.

본 패키지의 근본 철학은 [swjo0330/Aerion-Airsim](https://github.com/swjo0330/Aerion-Airsim) 의 `branc/airsim-ros2-bridge` 브랜치 기조를 유지합니다. 변경 사유와 함께 모든 변경점을 본 파일에 누적합니다.

## [Unreleased] (2026-05-18~)

### Added

- **`range_publisher.py`** — AirSim Distance 센서(전/좌/우) → `sensor_msgs/Range` 토픽 발행 모듈
  - 토픽: `/drone{N}/range/{front,left,right}`
  - 폴링 20Hz (AirSim RPC 50Hz 한계 안에서 카메라+pose+IMU 공존 마진)
  - INFRARED radiation_type (단일 ray-cast 의미상 정확)
  - `_probe_sensors_once`로 settings.json 누락 즉시 경고
- **`formation_node.py`** — Phase 4 동적 포메이션 노드 (모듈)
  - 상태 머신: NO_LEADER / SETTLING / STABLE / OBSTACLE_HOVER
  - 거리센서 회피 hook (전방 < `obstacle_stop_dist` → 전체 hover)
  - 도착 감지 (모든 드론 ±0.6m 안 → STABLE)
  - 패턴 5종: LINE / DIAMOND / ARROW / V / ECHELON
  - 토픽: `/aerion/formation/{pattern,leader_pose,status}` + `/drone{N}/mavros/setpoint_position/local`
- **`scripts/aerion_formation.py`** — formation_node 진입점
- **`scripts/airsim_arm_all.py`** — N대 일괄 arm + takeoff (SimpleFlight 모드 전용)
- **`scripts/airsim_land_all.py`** — N대 일괄 land + disarm
- **`scripts/smoke_aerion_phase2.py`** — Phase 2.5 smoke (단일 드론 takeoff/forward/land)
- **`scripts/verify_topics_phase3.py`** — Phase 3 토픽 무결성 자동 검증 (5대 × 표준 토픽 Hz 측정)
- **`launch/aerion_single_drone.launch.py`** — Phase 2 단일 드론 launch
- **`launch/aerion_multi_drone.launch.py`** — Phase 3 1~5대 multi launch (1 드론 = 1 프로세스)
- **`docs/ARCHITECTURE.md`** — 전체 시스템 구조 + 책임 분리
- **`docs/CONVENTIONS.md`** — vehicle/namespace/frame_id/QoS 컨벤션 명문화

### Changed

- **`bridge_node.py`**:
  - `RangePublisher` 통합 (enable_range/range_sensors/range_publish_rate/range_field_of_view_rad 파라미터 추가)
  - `enable_ardu_compat` default `True` → `False` (AERION 트랙은 SimpleFlight/PX4 기반, ArduPilot 호환 alias 발행 불필요)
  - `ardu_compat_vehicle` default `'Drone0'` → `'drone1'` (vehicle key 통일 컨벤션)
  - 한국어 주석 보강 (ThreadSafeAirSimClient, Range publisher 통합부)
- **`setup.py`**:
  - `data_files`에 `launch/*.launch.py` 등록 (`ros2 launch` 인식)
  - `entry_points.console_scripts`에 `aerion_formation` 추가 예정
- **vehicle key 컨벤션**: `Drone0`/`Drone1` (대문자) → `drone1`/`drone2`/... (소문자, 1-base)
  - 변경 사유: publisher 토픽이 `/drone1/...` 표준 형태로 자동 발행됨 (ROS2 namespace 일치)
  - 외부 사용자 인터페이스 표준 (`Aerion-ardu-ws` 측 `drone1` 네이밍과 정합)
  - `sf_1drone_phase2.json`, `sf_5drones_phase3.json` 신규 작성 시점부터 적용
- **AirSim Python 메서드 정정**: `simGetDistanceSensorData` → `getDistanceSensorData`
  - 변경 사유: Colosseum (이 프로젝트가 사용 중인 AirSim fork)에는 `sim` 접두사 없음. 본가 Microsoft AirSim과 다름. range_publisher.py + smoke 스크립트에 반영.

### Fixed

- **launch 파일의 `vehicle_names` 인자 제거** (single + multi 둘 다)
  - 증상: ros2 launch가 `[LaunchConfiguration(...)]` 을 STRING으로 평가 → STRING_ARRAY 타입 mismatch
  - 해결: `vehicle_name` (단수)만 전달 → bridge_node._resolve_vehicle_specs가 단일 vehicle 모드 진입
- **cyclonedds.xml SocketReceiveBufferSize 호환성**: `min="10MB"` 가 `sysctl net.core.rmem_max` 의 기본값(425984)을 초과 → 노드 생성 실패
  - 해결: `/etc/sysctl.d/30-aerion-dds.conf` 로 영구화 (rmem_max=2147483647 등)

### Decided (의사결정 이력)

- **Phase 4 포메이션 라이브러리**: Aerostack2 도입 검토 → **자체 구현으로 결정** (Agent 조사 결과)
  - 이유: AirSim용 platform plugin이 커뮤니티에 없어 9개 가상 메서드 직접 작성 부담 큼. SimpleFlight + 5대 + 고정 offset 표 조건에서는 자체 80~250줄 구현이 더 단순.
  - Phase 5 PX4 SITL 진입 시 `as2_platform_mavlink` 재검토 예정 (MAVROS 경로 그대로 활용 시 최소 변경).
- **CARLA 통합 방향**: "맵만 추출" → **CarlaAir 동적 통합 패턴**으로 변경 (사용자 결정 2026-05-18)
  - 이유: 차량/보행자 동적 액터 필요. 정적 맵 임포트로는 부족.
  - 베이스라인: UE5.5 + CARLA 0.10.0 + Colosseum 5.5. CarlaAir의 ASimWorldGameMode 합성 패턴 적용.

### Migration Notes (이전 버전 → 현재 버전)

기존 main 브랜치 사용자(`vehicle_names=['Drone0', 'Drone1']`, `simGetDistanceSensorData` 사용 코드)는:
1. settings.json의 vehicle key를 `drone1..N`으로 갱신 (또는 launch 인자로 명시)
2. AirSim distance 메서드 호출을 `getDistanceSensorData`로 변경
3. `enable_ardu_compat=False`로 명시 (default 변경됨, 단 ArduPilot 사용자는 명시적으로 `True`)
