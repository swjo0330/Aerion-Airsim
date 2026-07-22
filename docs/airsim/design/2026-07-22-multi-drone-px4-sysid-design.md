# 멀티 PX4 SITL 드론 — System ID / 포트 배분 설계 & 검토

> 작성 2026-07-22 · 병렬 3-agent 코드/설정 조사 결과 통합 + 라이브 실측 정정.
> 목적: settings.json에 여러 드론을 정의해 각기 다른 MAV_SYS_ID로 띄우는 것을 **어디까지 지원하는지** 설계·검토하고,
> N드론 embodied 연동으로 확장하기 위한 갭·스킴을 명세한다.
> 연관: [`2026-07-22-aerion-embodied-execution-guide.md`](../guide/2026-07-22-aerion-embodied-execution-guide.md).

---

## 0. 결론 요약 (TL;DR)

| 계층 | N드론 지원 | 비고 |
|---|---|---|
| **PX4 SITL 런처** (`launch_px4_instances.sh`) | ✅ **완비** | `-i N` 인스턴스 루프, 포트/sysid 자동 배분 |
| **settings.json 멀티드론** | ✅ 정의 존재 | 2/3/5드론 프리셋. 단 **sysid 지정 방식 2가지 혼재**(아래) |
| **bridge_node** | ✅ **아키텍처 준비됨** | "1드론=1프로세스", `aerion_multi_drone.launch.py`가 N개 스폰, per-drone namespace |
| **embodied 런타임** (`test_embodied_link.sh` + `run_mavproxy_to_embodied.sh`) | ❌ **단일드론 하드와이어** | mavproxy 1개·MAVROS 1개·`bare` namespace(N>1 충돌) |

**핵심:** PX4/bridge 하부는 이미 N드론 가능. 갭은 **embodied 셸 2개**와 **`bare` namespace 선택**뿐.

---

## 1. MAV_SYS_ID 지정 방식 — 2가지 컨벤션 (혼재, 통일 권장)

| 방식 | 어디 | sysid 결정 | 사용 파일 |
|---|---|---|---|
| **A. settings.json Parameters** | AirSim이 lockstep으로 PX4에 param push | `Parameters.MAV_SYS_ID` 명시 (1,2,4…) | 레거시 `px4_dual`, `px4_2drones`, `px4_3drones` |
| **B. PX4 인스턴스 번호 파생 (권장)** | `px4 -i N` CLI 플래그 | PX4가 `MAV_SYS_ID = N+1` 자동 | AERION 현행 `phase4_delta`, `phase5` (JSON에 sysid 없음) |

- **방식 B가 AERION 표준.** `launch_px4_instances.sh`가 `px4 -i "$px4_instance"`로 기동 → sysid는 PX4가 결정.
  스크립트의 `system_id=$((px4_instance + 1))`는 **표시·MAVROS `tgt_system`용**일 뿐 PX4에 주입하지 않음.
- 방식 A는 AirSim param 채널이 PX4에 확실히 도달해야 하나, "heading invalid"류로 param 반영이 불안정한 사례 있음
  → 방식 B가 더 견고.

---

## 2. 포트 배분 산식 (검증됨, 0-based instance `i`)

### 2.1 런처 산식 (`launch_px4_instances.sh` L48–51, PX4 rc 권위)

```bash
tcp_port=$((4560 + i))            # PX4 SITL ↔ AirSim lockstep TCP (HIL)
mavros_bind_port=$((14550 + i))   # GCS/Normal 채널: 클라(MAVROS/mavproxy)가 bind
px4_remote_port=$((18570 + i))    # PX4 Normal 인스턴스 listen
system_id=$((i + 1))              # 표시/tgt_system
```

### 2.2 PX4 MAVLink 인스턴스 2채널 (실행 중 px4 로그 실측)

| PX4 mode | PX4 listen | remote(클라 bind) | 클라이언트 | 용도 |
|---|---|---|---|---|
| **Normal (GCS)** | `18570+i` | **`14550+i`** | **mavproxy / MAVROS** | 전체 텔레메트리+제어 |
| Onboard | `14580+i` | `14540+i` | **AirSim** (`ControlPortLocal`) | AirSim↔PX4 offboard |

> ★ **정정(라이브 검증):** embodied mavproxy는 **GCS 채널 `14550+i`를 tap**한다. offboard `14540+i`는
> **AirSim이 점유**하므로 쓰면 충돌(`UDP 14540 local MAVROS 점유` 거부). 단일드론 실측: 14540 실패 →
> **14550에서 `Detected vehicle 1:1` 성공.** (px4-rc.mavlink만 보면 offboard=14540+i로 오독하기 쉬움 — AirSim 소비 간과 주의.)

### 2.3 settings.json 포트 (드론 i)

| 컨벤션 | TcpPort | ControlPortLocal | ControlPortRemote |
|---|---|---|---|
| 레거시(`px4_dual/2drones`) | `4560+i` | `14540+i` | `14555+i` |
| AERION 현행(`phase4_delta/phase5`) | `4560+i` | `15040+i` | `15045+i` |
| spawn | `X=5.0*i`, Y=Z=0 (5m 선형) | | |

> ⚠️ ControlPort는 **AirSim↔PX4 offboard 링크**용(GCS 아님). embodied mavproxy와 무관. GCS tap은 항상 `14550+i`.

---

## 3. 발견된 불일치 (정리 필요 항목)

| # | 파일 | 문제 | 권장 |
|---|---|---|---|
| 1 | `settings/README.md` | `px4_2drones`가 `uav0/uav1`라 기재 — 실제는 `Drone0/Drone1` | README 수정 |
| 2 | `px4_3drones.json` | Drone2가 인덱스 건너뜀(TcpPort 4563, sysid 4), `LocalHostIp 172.23.80.1`(WSL) | 4562/sysid3/127.0.0.1로 정규화 |
| 3 | `phase4_delta`·`phase5` | 주석은 `14555~`인데 실제 JSON은 `15045~` | 주석/값 일치 |
| 4 | 전역 | sysid 방식 A/B 혼재 | **방식 B(인스턴스 파생)로 통일** |
| 5 | `run_mavproxy_to_embodied.sh` | 기본 in-port 14540(AirSim 충돌) | 기본 **14550**로 변경(+ i 오프셋) |

---

## 4. bridge_node 멀티드론 (코드 준비 완료)

- **1드론=1프로세스** 패턴 (`bridge_node.py:1-8,28-33`) — AirSim RPC IOLoop 재진입(`AirSim#2607`) 회피 위해
  `SingleThreadedExecutor`. N-in-one 대신 **N 프로세스**.
- **namespace** (`topic_naming.py`, `bridge_node.py:137-143`):
  - `__vehicle__`(기본) → `/{vehicle}/camera/...` (예 `/drone1/camera/image`)
  - `''`/`bare` → bare `/camera/image` **(단일드론 전용, N>1 충돌)**
- **멀티 런치 존재:** `aerion_multi_drone.launch.py:47-74`가 `n=1..drone_count` 루프로 드론별 `bridge_node`(namespace
  `drone{n}`) 스폰. 1–5드론 캡.
- **⚠ `topic_naming.py:14-16` 명시:** *"비네임스페이스는 단일 드론에서만 안전. 다중 드론이 같은 `/camera/image`를
  같은 ROS graph에 올리면 충돌"* → N드론은 `/drone{n}/*` 또는 `ROS_DOMAIN_ID` 분리 필수.

---

## 5. embodied 런타임 — 단일드론 하드와이어 (일반화 체크리스트)

| 위치 | 하드와이어 값 | N드론에 필요 |
|---|---|---|
| `test_embodied_link.sh:39` | `VEHICLE=drone1` | `drone1..droneN` 루프 |
| `test_embodied_link.sh:51` | `TOPIC_NAMESPACE=bare` | `/drone{n}/*` + Mac relay 또는 `ROS_DOMAIN_ID` 분리 |
| `step_bridge` | 단일 `bridge_node` | N 프로세스 (`aerion_multi_drone.launch.py` 재사용) |
| `step_mavproxy` | `MAVPROXY_IN_PORT=14540`, MAVROS 1개, `tgt_system:=1` | mavproxy N개(bind **`14550+i`**), MAVROS N개(`tgt_system i+1`) |
| `run_mavproxy_to_embodied.sh:10-12,31` | master/out 1쌍, `exec` 1회 | per-instance 파라미터화 또는 N개 기동 |
| `step_zenoh` | zenoh client 1개 | 그대로 OK(1 client가 전 DDS 토픽 브릿지). 단 allow-list에 `/drone{n}/*` 포함 |

---

## 6. 권장 N드론 스킴 (i = 0..N-1)

| 항목 | 산식 | Drone0 | Drone1 | 근거 |
|---|---|---|---|---|
| vehicle / ROS ns | `drone{i+1}` | drone1 | drone2 | `aerion_multi_drone.launch.py:48` |
| PX4 instance / SysID | `i` / `i+1` | 0/1 | 1/2 | `launch_px4_instances.sh:51` |
| PX4↔AirSim TCP | `4560+i` | 4560 | 4561 | `:48` |
| **GCS: PX4 out / mavproxy bind** | `18570+i` / **`14550+i`** | 18570/14550 | 18571/14551 | 라이브 검증 |
| mavproxy → Mac MAVROS bind | `14555+i` (Mac 자유선택) | 14555 | 14556 | — |
| Mac MAVROS ns / tgt_system | `/drone{i+1}/mavros` / `i+1` | 1 | 2 | — |
| 센서 토픽(bridge) | `/drone{i+1}/{camera,range}` | /drone1/* | /drone2/* | `topic_naming.py:41` |
| Mac 관측 relay | ns 유지(`/drone{n}/*`), **bare로 붕괴 금지** | — | — | 충돌 방지 |

### mavproxy N드론 (예: drone i)
```bash
mavproxy.py --master=udpin:0.0.0.0:$((14550+i)) \
            --out=udpout:100.67.87.116:$((14555+i)) \
            --state-basedir=/tmp/mav_embodied_$i --daemon
```
### Mac MAVROS (드론 i)
```bash
ROS_DOMAIN_ID=0 ros2 run mavros mavros_node -r __ns:=/drone$((i+1))/mavros --ros-args \
  -p fcu_url:="udp://:$((14555+i))@100.120.219.68:$((14550+i))" \
  -p tgt_system:=$((i+1)) -p tgt_component:=1
```

---

## 7. 필수 게이트 & 함정 (런처)

- **`PX4_SIM_MODEL=none_iris`** 매 `px4` 기동에 필수 (`make px4_sitl_default none_iris` 선행).
- **per-instance 별도 cwd** `/tmp/px4_inst_$i` — PX4가 cwd에 `./etc`·dataman 생성 → 공유 시 충돌(rm-rf 후 재생성).
- **`PX4_DIR` 기본값 오류** — 런처 기본 `~/PX4-Autopilot` 틀림 → `PX4_DIR=/home/clrobur/airsim/PX4-Autopilot`.
- **UE Play 선행** — `4560..4560+N-1` LISTEN 후 PX4 기동(lockstep). `run_phase4_delta.sh` Step2가 검사.
- **staggered `sleep 2`** 인스턴스 간; `run_phase4_delta.sh`는 EKF heading 수렴 위해 추가 대기.
- **arming** — AirSim param 채널 불안정 시 force-arm(`param2=21196`) 폴백(`mavros_force_arm.py`). 근본픽스 `EKF2_MAG_CHECK=0`.
- **`PX4_INSTANCE_IDS`** env로 인스턴스 식별자 커스텀 매핑 가능(개수는 DRONE_COUNT와 일치해야 함).
- **PX4 자체 캡:** 인스턴스 `>9`면 offboard 포트가 14549로 collapse(`px4-rc.mavlink:6`) — 10대 이상은 별도 처리.

---

## 8. 검토 결론 & 다음 작업

**질문("settings.json에 여러 대 정의 → 각 sysid로 뜨는 것 지원?")에 대한 답:**
- ✅ **지원한다.** settings.json 멀티드론 프리셋 + `launch_px4_instances.sh -i N` + `aerion_multi_drone.launch.py`로
  **PX4 N대 각기 다른 MAV_SYS_ID(=instance+1)로 기동 + 드론별 namespace 센서 토픽**까지 이미 동작.
- ❌ **embodied(Mac) 연동만 단일드론 하드와이어.** N드론 관측/제어를 Mac에 보내려면 §5 체크리스트대로
  embodied 셸 2개를 루프화하고 `bare`→`/drone{n}/*`로 전환해야 함.

**권장 다음 작업 (우선순위):**
1. `run_mavproxy_to_embodied.sh` 기본 in-port 14540→**14550**, `-i` 오프셋 파라미터화.
2. `test_embodied_link.sh`에 `DRONE_COUNT` 루프 + `TOPIC_NAMESPACE=/drone{n}` 적용, mavproxy/MAVROS N개.
3. settings 불일치 정리(§3), sysid 방식 B로 통일.
4. Mac 측 N MAVROS + N relay 런치 스크립트.

---

## 9. 인스턴스별 초기 GPS/위치 설정

### 9.0 결론 (Feasibility Verdict)

- **각 드론에게 서로 다른 실제 GPS 시작점을 주는 것은 이미 자동으로 됩니다.** 방법은 단 하나 — **하나의 전역 `OriginGeopoint` + 드론별 NED `X/Y/Z` 오프셋**. AirSim이 vehicle마다 `home = nedToGeodetic(스폰 NED, OriginGeopoint)`를 계산하므로, `X/Y`만 다르면 lat/lon이 자동으로 달라집니다.
- **"드론별 독립 lat/lon 필드"는 AirSim에 없습니다.** `OriginGeopoint`는 항상 settings 최상위(전역) 키이며, `Vehicles.<name>` 안에 넣는 geo 필드는 존재하지 않습니다. → 상대(relative) 방식만 지원, 절대(per-vehicle lat/lon) 방식은 미지원.
- **`PX4_HOME_LAT/LON/ALT` 환경변수 방식은 이 스택에서 무의미합니다.** AirSim GPS 센서 + LockStep 구성에서 PX4 EKF2 home은 AirSim이 주입하는 GPS로 latch되므로, `PX4_HOME_*`는 덮어써집니다. `launch_px4_instances.sh`에도 `PX4_HOME_*`는 전혀 설정돼 있지 않습니다(grep 0건).

### 9.1 메커니즘 (코드 근거)

전역 origin은 vehicle 밖에만 존재:
- `~/Documents/AirSim/settings.json` → `"OriginGeopoint": {Latitude 37.5665, Longitude 126.978, Altitude 38}` (최상위)
- `settings/px4_3drones_phase4_delta.json`, `settings/px4_5drones_phase5.json` — 동일하게 최상위 1개. 각 vehicle 블록엔 `X/Y/Z/Yaw`만 있고 geo 필드 없음.

vehicle별 home GPS는 AirSim이 스폰 시 계산:
- `Colosseum/Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp:664`
  `home_geopoint = EarthUtils::nedToGeodetic(pawn_ned_pos, getSettings().origin_geopoint);`
  → **드론의 NED 스폰 위치 + 전역 origin_geopoint** 로 그 드론만의 home lat/lon/alt 산출.
- `Colosseum/AirLib/include/common/EarthUtils.hpp:291` `nedToGeodetic()` — 구면 투영식으로 `x`(북), `y`(동) 오프셋을 lat/lon으로 변환, `alt = origin.altitude − v.z()`.
- GPS 센서가 이 home 기준 상태를 발행: `Colosseum/AirLib/include/sensors/gps/GpsSimple.hpp:80`.

즉 settings의 vehicle `X`(NED 북)·`Y`(NED 동)·`Z`(NED 하, 음수가 위)만 다르게 주면 각 드론이 자동으로 서로 다른 GPS home을 갖는다. 현행 `settings.json`의 `X:108.03, Y:-250.89`도 이미 origin(서울)에서 북 +108m·동 −251m 떨어진 고유 GPS로 스폰됨.

> ⚠️ bridge_node의 `home_latitude/home_longitude`(`bridge_node.py:117-118`, `drone_controller.py:1288-1293`)는 AirSim GPS가 아니라 ENU pose→NavSatFix 합성용 **단일 상수**다. vehicle마다 안 갈라지므로, 멀티드론 합성 GPS 토픽 사용 시 별도 대응 필요(§9.5).

### 9.2 AirSim 측 / PX4 측 상호작용

| 질문 | 답 | 근거 |
|---|---|---|
| vehicle별 geo 필드? | **없음.** 전역 `OriginGeopoint` + vehicle `X/Y/Z`(NED)만 | 전 settings 구조 + `SimModeBase.cpp:664` |
| `launch_px4_instances.sh`가 `PX4_HOME_*` 설정? | **안 함** (env는 `PX4_SIM_HOSTNAME`/`PX4_SIM_MODEL`만) | `launch_px4_instances.sh` L46-82 |
| 설정하면 먹히나? | **덮어써짐.** AirSim GPS 센서(SensorType 3)가 lockstep로 GPS 주입→EKF2 home latch. `PX4_HOME_*`는 gazebo/jmavsim처럼 자체 GPS 생성 시에만 유효 | `GpsSimple.hpp:80` |

**home은 AirSim이 권위.** PX4 쪽이 아니라 AirSim `X/Y/Z`로 제어.

### 9.3 가장 깨끗한 방법

**(i) 단일 `OriginGeopoint` + vehicle별 NED 오프셋**을 사용. AirSim이 실제로 지원하는 유일한 방식이며 코드가 자동으로 vehicle별 GPS 생성. **(ii) 드론별 진짜 독립 lat/lon은 AirSim에서 불가**(그런 필드 없음). 드론들이 수 km 떨어져야 하면 `OriginGeopoint`를 군집 중심으로 잡고 각 드론을 큰 NED 오프셋으로 배치.

### 9.4 N드론 구체 스펙

**산식 (드론 i, i=0..N-1):** 원하는 실제 GPS `(lat_i, lon_i, alt_i)` → 전역 origin `(lat0, lon0, alt0)` 대비 NED 오프셋:

```
X_i (north, m) = (lat_i − lat0) × 111320
Y_i (east,  m) = (lon_i − lon0) × 111320 × cos(lat0)
Z_i (down,  m) = −(alt_i − alt0)        # 지면이면 0.2 정도(약간 아래)
```

**예시 (서울 origin, 3드론 각기 다른 GPS):**

| 드론 | 목표 GPS (lat, lon) | X(north) | Y(east) | Z |
|---|---|---|---|---|
| drone1 | 37.5665, 126.9780 (origin) | 0.0 | 0.0 | 0.2 |
| drone2 | 37.56695, 126.9780 | +50.0 | 0.0 | 0.2 |
| drone3 | 37.5665, 126.97857 | 0.0 | +50.0 | 0.2 |

**예시 JSON (전역 origin 1개 + vehicle별 X/Y/Z):**

```json
{
  "OriginGeopoint": { "Latitude": 37.5665, "Longitude": 126.978, "Altitude": 38 },
  "Vehicles": {
    "drone1": { "VehicleType": "PX4Multirotor", "X": 0.0,  "Y": 0.0,  "Z": 0.2, "TcpPort": 4560 },
    "drone2": { "VehicleType": "PX4Multirotor", "X": 50.0, "Y": 0.0,  "Z": 0.2, "TcpPort": 4561 },
    "drone3": { "VehicleType": "PX4Multirotor", "X": 0.0,  "Y": 50.0, "Z": 0.2, "TcpPort": 4562 }
  }
}
```

현행 `px4_3drones_phase4_delta.json`의 `X:0/5/10, Y:0`도 이미 이 방식(5m 일렬). GPS를 벌리려면 X/Y를 원하는 거리로 바꾸면 끝.

### 9.5 함정 (Caveats)

- **origin은 반드시 전역 1개.** vehicle 안에 lat/lon 넣어도 무시.
- **큰 NED 오프셋 정확도:** `nedToGeodetic`은 구면 투영이라 수 km까지 정확, 수십 km급 초장거리는 origin을 군집 중심으로 재배치 권장.
- **PX4 EKF origin은 AirSim GPS가 latch.** `PX4_HOME_*` 세팅 금지(무효+혼란). home 이상 시 `OriginGeopoint`·vehicle `X/Y`·`Gps` 센서 enabled 확인(`mission_smoke.py:287`이 이 셋을 지목).
- **`Z`(down) 부호:** 위로 갈수록 음수. 지면 스폰 관례 `Z≈0.2`.
- **bridge 합성 GPS 상수:** 합성 GPS 발행 모드면 `home_latitude/longitude`가 vehicle별로 안 갈라짐(§9.1) → 드론별 bridge 인스턴스에 각자 home 파라미터 주입 필요.
- **API home 거부 이슈(별개):** AirSim Python API `getHomeGeoPoint=nan` → 제어는 MAVLink offboard 경로. GPS origin 설정과 무관하나 home 디버깅 시 혼동 주의.
