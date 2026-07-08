# AirSim GPS Mission Editor/MAVROS 사전 통합 정리

**작성일:** 2026-07-09  
**범위:** 시뮬레이션/AirSim 단독 사전 완성 작업  
**상태:** 체화지능 연동 전, 로컬 AirSim/PX4/MAVROS 미션 경로 기준 정리 완료

## 목적

체화지능 쪽과 연결하기 전에 시뮬레이션 리포 단독으로 아래 흐름을 닫는 것이 목표다.

```text
미션 에디터
  -> GPS mission JSON
  -> MAVROS mission upload
  -> PX4 AUTO.MISSION
  -> AirSim/UE 실제 이동 확인
```

기존 방식은 에디터가 `east/north/up` 로컬 좌표를 내보내고 업로드 시점에 GPS로 바꾸는 구조였다. 현재 기준은 에디터 export 단계에서 바로 MAVROS/PX4에 넣을 수 있는 GPS mission JSON을 생성하는 구조다. 로컬 좌표는 디버깅과 재생성을 위해 metadata에만 보존한다.

## 완료된 변경점

### 1. Mission editor export 기준 변경

- 기본 export는 GPS mission JSON이다.
- waypoint 구조는 아래처럼 `latitude`, `longitude`, `altitude`만 가진다.

```json
{
  "coordinate_frame": "gps",
  "waypoints": [
    {
      "latitude": 37.567422302662344,
      "longitude": 126.97821103739352,
      "altitude": 20
    }
  ]
}
```

- `altitude`는 PX4/MAVROS `FRAME_GLOBAL_REL_ALT`에 넣는 상대고도 값으로 사용한다.
- 기존 로컬 클릭점, 리샘플 결과, 축 변환 정보는 `metadata.local_source` 또는 관련 metadata로 남긴다.
- GPS 원점은 AirSim `settings.json`의 `OriginGeopoint`와 맞춘 고정값을 사용한다.

```text
latitude  = 37.5665
longitude = 126.9780
altitude  = 38
```

변환식:

```text
lat = origin_lat + north_m / R
lon = origin_lon + east_m / (R * cos(origin_lat))
R   = 6378137.0 m
```

### 2. AirSim NED 축 기준 정리

AirSim vehicle spawn의 `X/Y`는 일반 ENU가 아니라 전역 NED 기준이다.

```text
AirSim X = North
AirSim Y = East
```

따라서 에디터 export 기본 축은 아래 기준으로 고정했다.

```text
AirSim NED: map X -> North, map Y -> East
```

이 기준을 반영한 보조 스크립트:

```text
scripts/compute_map_home_for_airsim_spawn.py
scripts/compute_airsim_spawn_for_mission_node.py
```

검증된 기준 spawn:

```text
AirSim X = 240
AirSim Y = -170
AirSim Z = 0.2
```

이 spawn을 현재 AirSim NED origin에 적용하면 map home은 아래처럼 계산된다.

```text
map east  = 13.85912109
map north = 67.80341797
map up    = 27.43805176
```

관측된 MAVROS home GPS와 계산값의 차이는 cm 수준으로 확인했다.

### 3. 180도 회전/정렬 버그 수정

문제 증상:

```text
RGB topview 위에 그린 경로와 실제 시뮬레이션 경로가 맵 중앙 기준 180도 반대로 나타남
```

원인:

- `topview_control_points.json`는 에디터의 180도 회전 표시 상태에서 다시 캘리브레이션한 값이었다.
- 그런데 에디터가 alignment 변환 경로에서도 `map_rotate_180`을 한 번 더 적용하고 있었다.
- 그 결과 화면 overlay는 맞아 보이지만 저장/export되는 map 좌표가 중앙 기준 반대편으로 뒤집혔다.

수정 기준:

- alignment가 있는 RGB topview는 이미 표시 방향 기준으로 캘리브레이션된 것으로 취급한다.
- 따라서 alignment 경로에서는 `map_rotate_180`을 다시 적용하지 않는다.
- 180도 회전은 alignment가 없는 fallback 렌더링 경로에만 적용한다.

반영 파일:

```text
scripts/build_mission_editor.py
recordings/maps/mission_editor.html
```

에디터는 새 브라우저 탭으로 다시 열어야 한다. 기존 탭은 이전 JS를 들고 있을 수 있다.

### 4. GPS mission upload 로직 정리

업로드 엔트리포인트:

```text
scripts/upload_mission_json_to_mavros.sh
airsim_ros2_bridge/airsim_ros2_bridge/gps_route_mission.py
```

GPS mission일 때:

- MAVROS home을 읽어 local -> GPS 변환하지 않는다.
- mission JSON의 GPS 좌표를 그대로 MAVROS Waypoint로 만든다.
- 기본 altitude mode는 `relative`다.
- 기본 `INCLUDE_TAKEOFF=false`다.
- 기본은 upload만 수행하며, arm/start는 명시적으로 켠다.

Local legacy mission일 때:

- 하위 호환용으로 계속 지원한다.
- current MAVROS home 기준으로 GPS mission item으로 변환한다.
- 기본 altitude mode는 `relative_to_home`이다.

### 5. PX4/MAVROS 포트 정리

팀장 지시 기준의 PX4 onboard/API 채널:

```text
AirSim bind      14540
PX4 onboard bind 14580
```

단, 로컬 MAVROS는 AirSim이 쓰는 onboard 링크와 충돌하지 않도록 PX4 normal/GCS 링크를 기본으로 사용한다.

```text
MAVROS bind      14550
PX4 normal bind  18570
```

기본 MAVROS URL:

```text
udp://:14550@127.0.0.1:18570
```

관련 파일:

```text
scripts/launch_mavros_px4_instances.sh
scripts/launch_px4_instances.sh
scripts/run_gps_route_mission_experiment.sh
```

환경변수 override는 유지한다.

```text
FCU_BIND_BASE_PORT
PX4_REMOTE_BASE_PORT
FCU_REMOTE_HOST
MAVSDK_PORT
MAVSDK_URL
```

### 6. PX4 arming/preflight 대응

AirSim SITL에서 미션 arm이 `TEMPORARILY_REJECTED`로 거부되는 문제를 줄이기 위해 사전 파라미터 준비 경로를 정리했다.

주요 설정:

```text
COM_ARM_WO_GPS = 2
EKF2_MAG_TYPE  = 0
EKF2_MAG_CHECK = 0
COM_ARM_MAG_ANG = -1
COM_ARM_MAG_STR = 0
```

관련 파일:

```text
settings/px4_1drone_lidar.json
scripts/prepare_px4_manual_control.sh
scripts/run_clicked_mission_experiment.sh
```

검증된 결과:

```text
PX4 mode: AUTO.MISSION
armed: true
mission upload: success
mission progress: started
UE view: takeoff and movement observed
```

## 현재 기준 산출물

### Canonical mission

```text
recordings/missions/clicked_mission_gps.json
```

현재 구조:

```text
frame: gps
waypoints: 72
waypoint keys: latitude, longitude, altitude
axis_transform: airsim_ned_xy
rotation_fix: alignment_center_180_inverse
```

첫 waypoint:

```json
{
  "latitude": 37.567422302662344,
  "longitude": 126.97821103739352,
  "altitude": 20
}
```

마지막 waypoint:

```json
{
  "latitude": 37.567430200076046,
  "longitude": 126.97864126232591,
  "altitude": 20
}
```

### Mission editor

```text
recordings/maps/mission_editor.html
scripts/build_mission_editor.py
```

현재 에디터 입력 자산:

```text
recordings/maps/town10_live_topdown_airsim_rgb.png
recordings/maps/heightmap.json
recordings/maps/heightmap_map.png
recordings/maps/topview_alignment.json
recordings/maps/topview_heightmap_alignment.json
recordings/maps/airsim_ned_origin.json
```

### Integrated runner

기본 실행:

```bash
bash scripts/run_clicked_mission_experiment.sh
```

의도적으로 arm/start까지 수행:

```bash
MISSION_ARM=true START_MISSION=true bash scripts/run_clicked_mission_experiment.sh
```

워크플로우 wrapper:

```bash
bash scripts/aerion_mission_workflow.sh editor
bash scripts/aerion_mission_workflow.sh dry-run
bash scripts/aerion_mission_workflow.sh full
bash scripts/aerion_mission_workflow.sh start
bash scripts/aerion_mission_workflow.sh upload
bash scripts/aerion_mission_workflow.sh upload-run
```

## 권장 실행 순서

### 1. 에디터 열기

```bash
xdg-open recordings/maps/mission_editor.html
```

기존 브라우저 탭은 닫고 새로 연다.

### 2. 새 미션 export

에디터에서 `Mission JSON`을 내려받는다. 새 export는 기본적으로 GPS mission이어야 한다.

확인할 것:

```text
coordinate_frame = gps
waypoints[].latitude 존재
waypoints[].longitude 존재
waypoints[].altitude 존재
east/north/up은 최상위 waypoint에 없어야 함
```

### 3. canonical mission으로 배치

```bash
cp <exported-json> recordings/missions/clicked_mission_gps.json
```

### 4. upload dry-run

```bash
DRY_RUN=true bash scripts/upload_mission_json_to_mavros.sh
```

기대:

```text
frame: gps
altitude: relative
takeoff: false
namespace: /drone1/mavros
```

### 5. full stack upload only

```bash
bash scripts/run_clicked_mission_experiment.sh
```

기본은 upload만 하고 arm/start는 하지 않는다.

### 6. 실제 미션 실행

```bash
MISSION_ARM=true START_MISSION=true bash scripts/run_clicked_mission_experiment.sh
```

기대:

```text
MAVROS connected: true
mission push: success
mode: AUTO.MISSION
armed: true
UE 화면에서 이륙 및 waypoint 이동
```

## 검증 기록

정적 검증:

```bash
python3 -m py_compile \
  airsim_ros2_bridge/airsim_ros2_bridge/gps_route_mission.py \
  scripts/mission_smoke.py \
  scripts/convert_mission_json_to_gps.py \
  scripts/prepare_mission_for_current_fcu_home.py \
  scripts/transform_mission_frame.py \
  scripts/compute_map_home_for_airsim_spawn.py \
  scripts/compute_airsim_spawn_for_mission_node.py \
  scripts/build_mission_editor.py
```

```bash
bash -n \
  scripts/aerion_mission_workflow.sh \
  scripts/run_clicked_mission_experiment.sh \
  scripts/run_gps_route_mission_experiment.sh \
  scripts/upload_mission_json_to_mavros.sh \
  scripts/run_gps_route_mission.sh \
  scripts/run_mission_smoke.sh \
  scripts/launch_mavros_px4_instances.sh \
  scripts/launch_px4_instances.sh \
  scripts/prepare_px4_manual_control.sh \
  scripts/build_carla_live_mission_editor.sh
```

계산 검증:

```bash
python3 scripts/compute_map_home_for_airsim_spawn.py \
  --origin recordings/maps/airsim_ned_origin.json \
  --airsim-x 240 \
  --airsim-y -170 \
  --airsim-z 0.2
```

결과:

```text
home_map_enu.east  = 13.85912109
home_map_enu.north = 67.80341797
home_map_enu.up    = 27.43805176
```

Mission JSON 검증:

```text
recordings/missions/clicked_mission_gps.json
frame = gps
waypoints = 72
first waypoint keys = latitude, longitude, altitude
metadata.axis_transform = airsim_ned_xy
metadata.rotation_fix = alignment_center_180_inverse
```

시뮬레이션 검증:

```text
MAVROS connected: true
mission clear: success
mission push: success
mission set_current(0): success
PX4 AUTO.MISSION 진입 확인
UE 화면에서 이륙/이동 확인
```

## Mission smoke와의 관계

팀장 zip/PDF 기준의 MAVSDK-only smoke는 별도 채널 검증으로 유지한다.

```text
scripts/mission_smoke.py
scripts/run_mission_smoke.sh
```

이 smoke는 ROS2/MAVROS를 사용하지 않고 PX4 MAVLink mission channel을 직접 확인한다.

기본 팀장 기준 포트:

```text
udpin://0.0.0.0:14540
```

호환 옵션:

```text
--url
--bearing
--no-rtl
```

현 단계에서 MAVSDK 설치는 필수 선행조건이 아니다. 팀장 쪽 또는 MAVProxy 경로와 실제로 채널 smoke를 맞출 때 설치해서 검증하면 된다.

## 체화지능/Mac 연동 범위

이번 작업에서 실제 체화지능 연동은 수행하지 않았다.

남겨둔 후속 경로:

```bash
bash scripts/run_mavproxy_to_embodied.sh
```

원칙:

- 로컬 MAVROS와 MAVProxy가 같은 MAVLink input port를 동시에 점유하면 안 된다.
- mission upload/arm/mode/setpoint command를 보내는 writer는 하나만 둔다.
- Mac 쪽 MAVROS namespace는 로컬과 다를 수 있으므로 upload script의 `MAVROS_NAMESPACE`를 override한다.

## 주의사항

- 새로 export한 mission이 아직 `east/north/up` 최상위 waypoint를 가진다면 오래된 에디터 탭에서 export한 것이다. 에디터 HTML을 새 탭으로 다시 열어야 한다.
- RGB topview와 heightmap 정렬이 다시 틀어지면 `topview_control_points.json`와 `topview_heightmap_alignment.json`을 같은 display orientation에서 다시 뽑아야 한다.
- AirSim PlayerStart, UE map loading, vehicle spawn 기준이 바뀌면 `recordings/maps/airsim_ned_origin.json`을 다시 생성해야 한다.
- 현재 canonical mission은 기존 잘못 export된 경로를 수학적으로 보정한 버전이다. 최종 시연 전에는 수정된 에디터에서 새로 경로를 찍어 export하는 것을 권장한다.

## 팀 공유용 한 줄 공지

```text
sim/airsim에 GPS mission editor -> MAVROS/PX4 AUTO.MISSION 사전 통합 정리 문서를 올렸습니다. 현재 기본은 GPS export이며, AirSim NED 축/180도 alignment 버그/14550 MAVROS 포트 기준까지 반영되어 있습니다.
```
