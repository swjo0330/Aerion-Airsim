# AERION CARLA+AirSim 6-Drone Showcase Runbook (2026-05-26)

이 문서는 2026-05-26 기준 CARLA UE5 맵 위에서 AirSim SimpleFlight 6대 드론 포메이션 쇼케이스를 재현하고, 현재 제어 로직을 설명하기 위한 인수인계 문서다.

현재 목적은 PX4/MAVROS/ROS2 경로가 아니라 **CARLA+AirSim 통합 UE 프로젝트 + AirSim Python API 직접 제어**로 시연 가능한 포메이션 데모를 안정화하는 것이다.

---

## 1. 현재 목표

- CARLA 맵에서 AirSim SimpleFlight 드론 6대를 띄운다.
- 드론들이 삼각형, 가로 일자, 우측 echelon, 원형, 다이아몬드, 좌측 echelon 포메이션을 순환한다.
- 각 포메이션을 완성한 뒤 짧게 유지하고, 일부 포메이션은 전체 병진 이동 또는 90도 회전 액션을 수행한다.
- 시연 관찰용 3인칭 카메라가 포메이션 중심을 따라간다.
- AirSim subwindow에는 드론 내장 카메라(`back_center`)가 표시된다.
- 초기 삼각형 진입 및 포메이션 전환 중 충돌을 줄이기 위해 슬롯 최적화와 고도 레이어 staging을 사용한다.

---

## 2. 실행 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 22.04 계열 |
| 기준 날짜 | 2026-05-26 |
| Repo | `~/workspace/projects/aerion-airsim` |
| Branch | `branc/airsim-ros2-bridge` |
| CARLA UE5 repo | `~/workspace/engines/CarlaUE5` |
| UE engine | `~/workspace/engines/UE5.5_carla/UnrealEngine` |
| UE editor binary | `~/workspace/engines/UE5.5_carla/UnrealEngine/Engine/Binaries/Linux/UnrealEditor` |
| CARLA uproject | `~/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject` |
| AirSim Python client | `~/airsim/Colosseum/PythonClient/airsim` |
| AirSim RPC | `127.0.0.1:41451` |
| AirSim settings deploy path | `~/Documents/AirSim/settings.json` |

주의: AirSim settings는 UE Play 시점에 로드된다. settings를 바꾼 뒤에는 UE에서 Stop -> Play를 다시 해야 한다.

---

## 3. 핵심 파일

| 파일 | 역할 |
|---|---|
| `scripts/phase4_delta_simple.py` | 핵심 Python 제어기. AirSim API 직접 호출, 3/5/6대 포메이션 정의, 충돌 완화, 카메라 추적 포함 |
| `scripts/run_phase4_carla_showcase.sh` | settings 배포, env preset 로딩, 카메라 probe, Python 제어기 실행 wrapper |
| `configs/carla_6drones_showcase.env` | 6대 쇼케이스 고정 파라미터 preset. 명령어 파라미터 폭탄을 줄이기 위해 분리 |
| `settings/carla_6drones_showcase.json` | AirSim 6대 SimpleFlight vehicle 정의. drone1..6, X=0/10/20/30/40/50 |
| `docs/carla_airsim_6drone_showcase_2026-05-26.md` | 본 문서 |

---

## 4. UE Editor 실행

CARLA UE5 repo 안의 `Build/Unreal/CarlaUnreal/Binaries/Linux/CarlaUnrealEditor` 경로는 현재 존재하지 않는다. 에디터는 UE 엔진 바이너리로 `.uproject`를 열어야 한다.

```bash
/home/clrobur/workspace/engines/UE5.5_carla/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
  /home/clrobur/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject
```

실행 후 CARLA 맵이 로드되면, 쇼케이스 스크립트를 먼저 실행해서 settings를 배포하고, 스크립트가 `준비되면 Enter`에서 대기할 때 UE에서 Stop -> Play를 누른다.

---

## 5. 쇼케이스 실행 명령

기본 실행은 이제 env preset을 사용한다. 평소 바꿀 값은 스폰 위치 정도다.

```bash
ENV_FILE=~/workspace/projects/aerion-airsim/configs/carla_6drones_showcase.env \
SPAWN_OFFSET_X=240 SPAWN_OFFSET_Y=-170 SPAWN_OFFSET_Z=0.2 \
bash ~/workspace/projects/aerion-airsim/scripts/run_phase4_carla_showcase.sh
```

실행 흐름:

1. `settings/carla_6drones_showcase.json`을 `~/Documents/AirSim/settings.json`으로 복사한다.
2. `SPAWN_OFFSET_X/Y/Z`를 settings의 모든 vehicle spawn에 더한다.
3. `AIRSIM_VIEW_MODE`, `SubWindows`를 settings에 반영한다.
4. 사용자가 UE에서 Stop -> Play를 누를 때까지 대기한다.
5. `AUTO_CAMERA_PROBE=1`이면 AirSim 내장 카메라 응답을 확인한다.
6. `scripts/phase4_delta_simple.py`를 실행한다.

---

## 6. env preset 구조

`configs/carla_6drones_showcase.env`는 아래 그룹으로 구성되어 있다.

| 그룹 | 주요 변수 | 설명 |
|---|---|---|
| 기본 | `DRONE_COUNT`, `PATTERNS`, `CONTROL_MODE` | 6대 showcase와 포메이션 순서 |
| Motion | `VELOCITY=5.0`, `SEGMENT_SEC=5.4`, `TICK_HZ=7.0` | 사용자가 느리다고 느껴 속도를 올린 현재 기본값 |
| Safety | `MIN_SEPARATION_M=6.2`, `COLLISION_HARD_MIN_M=3.6`, `CA_NEAR_DIST_M=8.0` | 전환 경로 충돌 완화 기준 |
| Z stabilization | `Z_COMMAND_*`, `Z_COLLISION_*` | z축 출렁임 완화와 근접쌍 수직 분리 |
| Post action | `POST_ACTION_*` | 포메이션 완성 후 병진 이동/90도 회전 액션 |
| Camera | `THIRD_PERSON_*`, `CAM_*`, `SUBWINDOW_*` | 중심 추적 카메라와 AirSim subwindow 카메라 |

변수는 `: "${VAR:=default}"` 형태라서 명령어 앞에서 넘긴 값이 env preset보다 우선한다.

예: 속도만 더 올리고 싶을 때

```bash
ENV_FILE=~/workspace/projects/aerion-airsim/configs/carla_6drones_showcase.env \
SPAWN_OFFSET_X=240 SPAWN_OFFSET_Y=-170 SPAWN_OFFSET_Z=0.2 \
VELOCITY=5.3 SEGMENT_SEC=5.2 \
bash ~/workspace/projects/aerion-airsim/scripts/run_phase4_carla_showcase.sh
```

---

## 7. 포메이션 로직 요약

### 7.1 6대 포메이션 정의

`phase4_delta_simple.py`의 `FORMATIONS_NED_6`에 정의되어 있다.

현재 최소 거리 검증 결과:

| 포메이션 | 최소 거리 |
|---|---:|
| TRIANGLE | 6.46m |
| DIAMOND3 | 7.21m |
| LINE_H | 6.40m |
| ECHELON_R | 6.18m |
| ECHELON_L | 6.18m |
| CIRCLE | 7.00m |

의도적으로 모든 6대 포메이션은 약 6m 이상 간격을 확보한다. 이전에는 `ECHELON_R/L` 인접 간격이 약 5.1m라 `MIN_SEPARATION_M`보다 작아서 전환 중 충돌/보정이 자주 발생했다.

### 7.2 초기 삼각형 충돌 완화

초기 스폰은 X축으로 길게 늘어선 상태다. 이 상태에서 삼각형으로 바로 직선 이동하면 경로가 교차하거나 한 점으로 몰리는 현상이 생긴다.

현재 `settle_to_pattern()`은 6대 이상일 때 다음 절차를 수행한다.

1. 현재 위치를 읽는다.
2. `build_collision_aware_slot_assignment()`로 목표 포메이션 슬롯을 최적 배정한다.
3. XY 이동량이 큰 경우 `pre-settle-stage`를 수행한다.
4. 각 드론을 서로 다른 임시 고도 레인으로 분리한다.
5. 임시 고도에서 XY 목표로 이동한다.
6. 최종 포메이션 위치에서 z를 부드럽게 복귀한다.

이 구조는 multi-agent formation transition에서 흔히 쓰는 assignment 기반 슬롯 배정과 altitude layering 접근을 프로젝트에 맞춘 것이다.

### 7.3 전환 중 충돌 완화

showcase 전환 루프는 다음을 사용한다.

- `enforce_min_separation_points()`: 목표점 간 최소 거리 보정
- `min_pairwise_distance()`: 경로 샘플별 최소 거리 검사
- `collision_mitigation_rounds`: 위험하면 감속 및 고도 분리 재시도
- 6대 전환 전용 3-phase lane:
  - 초반: XY 유지, 고도 레인으로 이동
  - 중반: 고도 레인에서 XY 전환
  - 후반: 최종 XY에서 목표 고도로 복귀

### 7.4 z축 출렁임 완화

z축 보정은 완전히 제거하면 포메이션이 꼬였고, 너무 강하면 위아래 흔들림이 커졌다. 현재는 다음 방식으로 완화한다.

- `Z_COMMAND_DEADBAND_M`: 작은 z 오차는 재명령하지 않음
- `Z_COMMAND_MAX_STEP_M`: 한 번에 z 목표를 크게 당기지 않음
- `Z_COMMAND_HOLD_BAND_M`: 이전 z command 주변에서 히스테리시스 적용
- `Z_COMMAND_COOLDOWN_SEC`: 짧은 시간 내 z 재명령 억제

---

## 8. 카메라 구성

### 8.1 UE 관찰 카메라

`THIRD_PERSON_MODE=center_follow`를 사용한다. Python 제어기가 AirSim camera pose를 갱신해 포메이션 중심을 따라간다.

주요 값:

- `THIRD_PERSON_DISTANCE_M=24`
- `THIRD_PERSON_HEIGHT_M=10`
- `THIRD_PERSON_LOOKAHEAD_M=3`
- `THIRD_PERSON_UPDATE_HZ=6.0`
- `CAM_CENTER_SMOOTH_ALPHA=0.16`
- `CAM_DIR_SMOOTH_ALPHA=0.25`
- `CAM_YAW_RATE_LIMIT_DEG_S=24`

카메라가 어지러우면 먼저 `CAM_YAW_RATE_LIMIT_DEG_S`를 낮추고, `CAM_DIR_SMOOTH_ALPHA`를 낮춘다.

### 8.2 AirSim subwindow 내장 카메라

`SUBWINDOW_CAMERA_NAME=back_center`, `SUBWINDOW_VEHICLE=drone1`이 기본이다.

이전에는 `ImageRequest` import 경로 차이 때문에 probe가 실패했지만, 현재는 `airsim.types` fallback을 사용한다. 카메라가 검은 화면이면 `AUTO_CAMERA_PROBE=1` 상태에서 출력되는 score를 확인한다.

---

## 9. AirSim settings

`settings/carla_6drones_showcase.json`의 핵심:

- `SimMode=Multirotor`
- `ViewMode=Manual`
- `VehicleType=SimpleFlight`
- vehicle: `drone1`..`drone6`
- spawn X: `0, 10, 20, 30, 40, 50`
- `EnableCollisions=true`
- `AllowAPIAlways=true`
- CameraDefaults: Scene 640x360, motion blur off

런처가 이 파일을 복사한 뒤 `SPAWN_OFFSET_X/Y/Z`를 모든 vehicle에 더한다.

---

## 10. 문제 대응

### AirSim RPC `ECONNREFUSED`

증상:

```text
msgpackrpc.error.TransportError: Retry connection over the limit
```

대응:

1. UE Editor가 열려 있는지 확인한다.
2. CARLA 맵에서 Play 상태인지 확인한다.
3. settings 배포 후 Stop -> Play를 다시 한다.
4. `AIRSIM_PORT=41451`이 맞는지 확인한다.

### 드론이 땅 아래로 떨어짐

이전 CARLA 맵 통합에서 collision/ground 문제가 있었다. 현재 settings는 `EnableCollisions=true`이며, Python에는 `SAFETY_FLOOR_Z` 복구 로직이 있다. 그래도 재현되면 CARLA 맵에서 AirSim pawn collision/ground collision이 살아 있는지 UE 쪽을 확인해야 한다.

### 초기 삼각형에서 충돌

현재 수정 후에는 `pre-settle-plan`과 `pre-settle-stage` 로그가 나와야 한다.

예상 로그:

```text
[pre-settle-plan] optimized slot_map={...}
[pre-settle-stage] lane_gap=...m, max_xy_err=...m, paths=6
```

이 로그가 안 나오면 `DRONE_COUNT=6`, `PRE_SETTLE=1`인지 확인한다.

### 카메라가 검은 화면

`AUTO_CAMERA_PROBE=1`로 실행하면 `drone1:back_center size=640x360 score=...` 형태의 로그가 나와야 한다. 모든 score가 낮으면 UE Play 직후 카메라가 아직 준비되지 않았거나, AirSim camera name이 다른 상태일 수 있다.

---

## 11. 검증 명령

코드 문법 검증:

```bash
cd ~/workspace/projects/aerion-airsim
python3 -m py_compile scripts/phase4_delta_simple.py
bash -n scripts/run_phase4_carla_showcase.sh
bash -n configs/carla_6drones_showcase.env
```

6대 포메이션 최소 거리 검증:

```bash
cd ~/workspace/projects/aerion-airsim
python3 - <<'PY'
import importlib.util, math
p='scripts/phase4_delta_simple.py'
spec=importlib.util.spec_from_file_location('phase4', p)
m=importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
for name, pts in m.FORMATIONS_NED_6.items():
    mn=999
    pair=None
    for i in range(len(pts)):
        for j in range(i+1,len(pts)):
            d=math.dist(pts[i], pts[j])
            if d<mn:
                mn=d; pair=(i+1,j+1)
    print(f'{name:10s} min3d={mn:.2f} pair={pair}')
PY
```

---

## 12. 참고 자료

- AirSim Multi-Vehicle documentation: https://microsoft.github.io/AirSim/multi_vehicle/
- AirSim issue #1538, multi-vehicle RPC thread issue: https://github.com/microsoft/AirSim/issues/1538
- AirSim issue #991, SimpleFlight hover issue: https://github.com/microsoft/AirSim/issues/991
- AirSim issue #4421, moveToPosition shaky behavior: https://github.com/microsoft/AirSim/issues/4421
- Multi-UAV assignment and altitude-layered formation planning example: https://www.mdpi.com/2504-446X/6/8/192
- UAV formation with collision avoidance / repulsive term example: https://springerplus.springeropen.com/articles/10.1186/s40064-016-2476-y
- Multi-drone collision reduction via partitioned/non-overlapping regions: https://www.mdpi.com/1424-8220/22/5/1855

---

## 13. 현재 남은 리스크

- 실제 UE/AirSim runtime 검증은 시뮬레이터가 떠 있을 때만 가능하다. 문법 검증과 경로/거리 사전검증은 통과했지만, 최종 안정성은 CARLA Play 상태에서 확인해야 한다.
- SimpleFlight의 multi-vehicle 명령 처리 특성상, 아주 빠른 속도/짧은 segment로 더 올리면 충돌 회피보다 동역학 지연이 커질 수 있다.
- `ROLE_ASSIGNMENT=fixed`는 시연 중 드론 번호 역할을 보존한다. 단, 초기 진입에서는 안전을 위해 pre-settle 전용 optimized slot assignment를 사용한다.
- UE 쪽 카메라 워크는 Python `simSetCameraPose` 기반이다. 더 영화적인 카메라가 필요하면 UE Blueprint/C++ spectator camera actor로 옮기는 것이 다음 단계다.
