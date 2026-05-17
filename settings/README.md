# AirSim settings.json 파일 안내

AERION 프로젝트에서 사용하는 AirSim/Colosseum `settings.json` 변형들의 카탈로그. 사용 시 **하나를 골라 `~/Documents/AirSim/settings.json`에 복사 후 UE 에디터 재시작 또는 Play 다시 누르기**.

## 파일별 용도

| 파일 | 용도 | Vehicle 수 | 제어 백엔드 | 카메라 | Range 센서 | 비고 |
|---|---|---|---|---|---|---|
| `sf_1drone_phase2.json` | **Phase 2 단일 드론 무결성** | 1 (drone1) | SimpleFlight | RGB 320x240 | 3개 (전/좌/우) | AERION 표준 — vehicle key 소문자 |
| `sf_5drones_phase3.json` | **Phase 3 1~5대 점진 확장** | 5 (drone1..drone5, 5m 간격) | SimpleFlight | RGB 320x240 | 각 3개 | Phase 4 포메이션도 이걸로 |
| `simpleflight_2drones.json` | (참고용, 1.2 baseline) | 2 (Drone0, Drone1) | SimpleFlight | RGB 1280x720 | 없음 | 인수인계 자산. Range 없음 |
| `px4_dual.json` | Phase 5 PX4 SITL 2대 | 2 (Drone0, Drone1) | PX4Multirotor | (없음) | 없음 | Phase 5 사전 패턴 |
| `px4_2drones.json` | Phase 5 PX4 SITL 2대 (uav 네이밍) | 2 (uav0, uav1) | PX4Multirotor | (없음) | 없음 | uav 네이밍이라 AERION 표준과 불일치 — 사용 비권장 |
| `px4_3drones.json` | 인수인계 3대 패턴 | 3 | PX4Multirotor | RGB | 없음 | branc 인수인계 출처 |
| `apm_dual.json` | ArduPilot 2대 | 2 | ArduCopter | - | - | Phase 7 (별도 트랙) |
| `ardupilot_1drones.json` / `ardupilot_2drones.json` | ArduPilot SITL 검증 | 1/2 | ArduCopter | - | - | Phase 7 |

## vehicle key 명명 컨벤션 (AERION 표준)

- **소문자 `drone1`..`drone5`** 사용. 이유:
  - publisher가 `f'/{vehicle_name}/...'` 절대 경로로 토픽 발행
  - vehicle_name을 그대로 ROS2 namespace로 통일 → 토픽이 `/drone1/camera/image` 표준 형태로 자동 발행
  - 외부 사용자 인터페이스 표준 (Aerion-ardu-ws 측 `drone1` 네이밍과 정합)
- 기존 `Drone0/Drone1`(대문자, 인덱스 0-base) 파일들은 인수인계 자산 — 호환성 위해 보존, AERION 신규 작업은 소문자 사용

## 공통 설계 결정

- `SettingsVersion: 1.2` — 1.2 / 2.0 호환. AirSim 1.8.1 (Colosseum)에서 1.2 안정.
- `SimMode: "Multirotor"` — 멀티로터 모드 강제. CarPawn 비활성.
- `ClockType: "SteppableClock"` — AirSim 자체 step clock. (현재 CARLA `synchronous_mode`와 연결 없음 → Phase 1 통합 시 검토)
- `ViewMode: "SpringArmChase"` — 카메라 chase view. SubWindows로 추가 카메라 표시.
- `OriginGeopoint`: 서울 좌표 (37.5665, 126.9780). MAVROS `home_position` 기본값과 일치.

## 거리센서 표준 배치 (drone1..5 공통)

| 센서 | X (전후) | Y (좌우) | Yaw | MaxDistance |
|---|---|---|---|---|
| Distance_Front | +0.30 | 0 | 0 | 40m |
| Distance_Left | 0 | -0.30 | -90 | 20m |
| Distance_Right | 0 | +0.30 | +90 | 20m |

- AirSim Distance 센서는 단일 ray-cast (FOV=0). ROS sensor_msgs/Range → INFRARED, field_of_view=0.035rad.
- MinDistance 0.2m (드론 바디 회피).
- 운영 시 `DrawDebugPoints: false` (UE 화면 점 표시 끔).

## settings deploy

```bash
# Phase 2 (1대) deploy
cp ~/airsim/settings/sf_1drone_phase2.json ~/Documents/AirSim/settings.json

# Phase 3 (5대) deploy
cp ~/airsim/settings/sf_5drones_phase3.json ~/Documents/AirSim/settings.json

# UE Editor에서 Stop → Play 다시 (또는 에디터 재시작) 필요. 런타임에 settings 안 바뀜.
```

## 향후 작업

- `sf_5drones_phase3_camera_per_n.json` — N대만 카메라 활성 (자원 부담 측정용)
- `carla_5drones_phase4.json` — CARLA Town10HD에 5대 spawn (Phase 1 통합 후)
- Phase 5 진입 시 `px4_5drones.json` (PX4 SITL × 5)
