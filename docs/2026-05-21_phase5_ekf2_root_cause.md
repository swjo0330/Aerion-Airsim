# Phase 5 — PX4 SITL EKF2 sensor 미수신 root cause 진단 + 수정

**작성일:** 2026-05-21
**작업자:** Claude Code (Opus 4.7) + 김영훈
**대상 트랙:** `aerion-airsim` (PX4 SITL + Colosseum/UE5.6 + MAVROS, Linux native)
**관련 브랜치:** `branc/airsim-ros2-bridge`

---

## 1. 배경

### 1.1 Phase 5 정의 (이 문서 한정)

본 문서에서 "Phase 5"는 메모리/`project-aerion-final-plan` 의 Phase 5 = **PX4 SITL 5대 + MAVROS + AirSim Colosseum + lockstep** 스택을 의미한다.  
(repo `CLAUDE.md` 의 Phase 5 = "도시 환경 + 장애물 배치" 와는 다른 분류이므로 혼동 주의.)

목표: Phase 3 SimpleFlight 백엔드에서 5대 동시 CIRCLE 시 2/5 ARRIVED 한계가 있던 부분을 PX4 PID controller로 5/5 보장 가능하게 백엔드 교체.

### 1.2 이전 세션(~2026-05-20 새벽) 종료점

이전 세션에서 PX4 인프라 자체는 Linux native에서 다음까지 검증 완료:

- ✅ UE↔PX4 TCP 4560~4564 lockstep connect
- ✅ MAVROS 5/5 `connected: true`
- ✅ OFFBOARD mode set
- ❌ **arm 시 `Preflight Fail: ekf2 missing data` + `heading estimate invalid` 지속**, arm `result=1 (TEMPORARILY_REJECTED)`

→ Phase 5 진척이 EKF2 sensor pipeline 문제로 막힘. 본 문서는 이 막힘의 원인 규명과 수정 과정을 담는다.

### 1.3 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| UE | 5.6.1 source build (`~/airsim/unreal-engine/`) |
| Colosseum | `main` branch (~/airsim/Colosseum, libAirLib.a 빌드 완료) |
| PX4 | `~/airsim/PX4-Autopilot`, `make px4_sitl_default none_iris` 빌드, Flight software `01110040` = PX4 v1.17.0 |
| ROS2 | Humble + `rmw_cyclonedds_cpp` (`ROS_DOMAIN_ID=0`) |
| MAVROS | 2.x (humble) |
| 대상 UE 프로젝트 | `~/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject` |
| `~/Documents/AirSim/settings.json` | PX4Multirotor × 5, lockstep, TCP 4560~4564, UDP 14540~14544/14580~14584 |

---

## 2. 진단 진행 순서

### Step 1 — 환경 검증

전 세션 종료 후 상태를 확인. 다음 명령으로 모든 컴포넌트 상태를 한 번에 점검:

```bash
md5sum ~/Documents/AirSim/settings.json ~/airsim/settings/px4_5drones_phase5.json
pgrep -af 'UnrealEditor|UE5Editor|BlocksV2'
pgrep -af 'px4|mavros|formation|tf_publisher|airsim_ros'
ss -tlnp | grep -E ':456[0-9]|:1454[0-9]|:1458[0-9]|:1504[0-9]'
ls -la ~/airsim/Colosseum/Unreal/Plugins/AirSim/Source/Vehicles/Multirotor/
```

결과:

| 항목 | 상태 |
|---|---|
| settings.json md5 (v4) | `f89df34a...` — PX4 v4 형태 (의심 param 포함) |
| UE Editor (BlocksV2) | 살아있음 (PID 73403), Play 상태 아님 (TCP 4560+ unlisten) |
| PX4/MAVROS/bridge 잔여 | 모두 정리됨 |
| Colosseum source canonical 경로 | `Vehicles/Multirotor/` 가 아니라 `AirLib/include/vehicles/multirotor/firmwares/mavlink/` (메모리 경로 정정 필요) |

### Step 2 — Colosseum HIL_SENSOR 송신 경로 정밀 분석

Explore agent에게 다음 파일들의 HIL_SENSOR 송신 path를 추적시킴:

- `~/airsim/Colosseum/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`
- `~/airsim/Colosseum/AirLib/deps/MavLinkCom/include/MavLinkMessages.hpp`

핵심 발견:

| line | 내용 |
|---|---|
| `MavLinkMultirotorApi.hpp:1784` | `sendHILSensor()` — IMU/Mag/Baro 패킹 + `hil_node_->sendMessage(hil_sensor)` |
| `MavLinkMultirotorApi.hpp:126` | `update()` — sensor send 진입점 (UE tick 주기) |
| `MavLinkMultirotorApi.hpp:140-154` | **lockstep skip 메커니즘** — `lock_step_active_ && !received_actuator_controls_` 이면 sensor send를 SKIP. PX4가 HIL_ACTUATOR_CONTROLS 응답을 못 보내면 deadlock 발생 가능. |
| `MavLinkMultirotorApi.hpp:1638` | `handleLockStep()` — actuator 수신 시 flag set |

→ **가설 A**: PX4가 actuator response를 못 보내 lockstep deadlock → sensor stream 전체 중단.

### Step 3 — TCP 4560 트래픽 실측 (가설 A 검증)

다른 터미널에서 (PX4 SITL + UE Play 상태 유지):

```bash
sudo timeout 10 tcpdump -i lo -nn 'tcp port 4560' 2>/dev/null | wc -l
```

**결과: 13,622 packets / 10s (≈ 1.3K pps)** — 양방향 매우 활발한 흐름.

→ **가설 A 기각**: lockstep deadlock 아님. HIL_SENSOR/ACTUATOR_CONTROLS 모두 정상 흐름. 데이터는 들어오는데 EKF가 fuse를 안 함.

### Step 4 — settings.json EKF2 parameter 검토 (가설 B)

`~/Documents/AirSim/settings.json` v4 의 각 vehicle `Parameters` 블록:

```json
"Parameters": {
  "NAV_RCL_ACT": 0,
  "NAV_DLL_ACT": 0,
  "COM_OBL_ACT": 1,
  "COM_ARM_WO_GPS": 1,
  "EKF2_AID_MASK": 0,       // ← 의심
  "EKF2_HGT_REF": 1,
  "EKF2_GPS_CHECK": 0,      // ← 의심
  "CBRK_USB_CHK": 197848,
  "CBRK_IO_SAFETY": 22027,
  "CBRK_SUPPLY_CHK": 894281
}
```

PX4 1.14+ 에서 `EKF2_AID_MASK` 의 bitmask 의미:

| bit | 값 | 효과 |
|---|---|---|
| 0 | 1 | GPS aiding |
| 1 | 2 | optical flow |
| 2 | 4 | inhibit IMU bias estimation |
| 3 | 8 | vision position |
| 4 | 16 | vision yaw |
| 5 | 32 | multi-rotor drag |
| 6 | 64 | rotate external vision |

`EKF2_AID_MASK = 0` 의 의도는 "외부 aiding 없이 동작" 이었으나, 실제 PX4 1.17.0 에서는 **모든 aiding source 비활성화** → EKF2 가 sensor data 받아도 fusion 하지 않음 → `Preflight Fail: ekf2 missing data`.

조합:
- `EKF2_AID_MASK=0` (모든 aiding 비활성)
- `EKF2_GPS_CHECK=0` (GPS sanity check off)
- `COM_ARM_WO_GPS=1` (GPS 없이 arm 시도)

세 개가 충돌. **GPS 없이 arm 하려고 했는데 정작 EKF 자체를 disable 시킨 상태.**

→ **가설 B 확정**: settings.json 의 `EKF2_AID_MASK=0`, `EKF2_GPS_CHECK=0` 가 EKF2 fusion 자체를 막은 root cause.

### Step 5 — settings v5 작성 + deploy

수정 내용: 모든 5대 vehicle 의 `Parameters` 에서 `EKF2_AID_MASK`, `EKF2_GPS_CHECK` 두 줄 제거 (PX4 default 사용 = GPS aiding 활성). 나머지 (`EKF2_HGT_REF=1`, `COM_ARM_WO_GPS=1`, `CBRK_*`) 는 유지.

| 파일 | md5 (v4) | md5 (v5) |
|---|---|---|
| `settings/px4_5drones_phase5.json` | `f89df34a3d1bfb2f3d1c604d921d26fc` | `a34d5a92e06a793d87c3c85dc89bd95f` |
| `~/Documents/AirSim/settings.json` | (v4 deploy 상태) | v5 deploy 완료 |

backup 보존: `settings/px4_5drones_phase5.v4.bak.json` (repo 안에 함께 commit).

### Step 6 — 효과 검증 (PX4 SITL 1대 manual)

같은 시퀀스로 PX4 1대만 콘솔에서 직접 띄워 v4 vs v5 startup log 비교.

**v4 (수정 전):**
```
INFO  [simulator_mavlink] Simulator connected on TCP port 4560.
INFO  [lockstep_scheduler] setting initial absolute time to 1779293290577895 us
WARN  [health_and_arming_checks] Preflight Fail: ekf2 missing data
WARN  [health_and_arming_checks] Preflight Fail: system power unavailable
WARN  [health_and_arming_checks] Preflight Fail: heading estimate invalid
WARN  [health_and_arming_checks] Preflight: GPS fix too low
WARN  [health_and_arming_checks] Preflight Fail: heading estimate invalid
WARN  [health_and_arming_checks] Preflight Fail: heading estimate invalid
   ↓ (이 메시지들이 반복적으로 출력 — EKF가 데이터 못 받음)
```

**v5 (수정 후):**
```
INFO  [simulator_mavlink] Simulator connected on TCP port 4560.
INFO  [lockstep_scheduler] setting initial absolute time to 1779294103483577 us
WARN  [health_and_arming_checks] Preflight Fail: ekf2 missing data   ← 초기 1회만
WARN  [health_and_arming_checks] Preflight Fail: system power unavailable
... (startup)
WARN  [health_and_arming_checks] Preflight Fail: heading estimate invalid
WARN  [health_and_arming_checks] Preflight Fail: heading estimate invalid
   ↓ (이후 30초 이상 추가 warning 없음 — EKF 수렴)
```

차이:

| 항목 | v4 | v5 |
|---|---|---|
| `ekf2 missing data` | 반복 출력 | **초기 1회만** (이후 EKF가 sensor fusion 시작) |
| `GPS fix too low` | 출력됨 | **사라짐** (GPS aiding 정상화 → fix 획득) |
| `heading estimate invalid` | 지속 반복 | startup 직후 2회 → 잠잠 (수렴 진행) |

### Step 7 — MAVROS 측 양성 신호 확인

같은 PX4 인스턴스에 MAVROS 1대 (drone1, `udp://:14540@127.0.0.1:14580`) 띄워 다음 로그 확인:

```
[drone1.mavros] MAVROS UAS via /uas1 started. MY ID 1.191, TARGET ID 1.1
[drone1.imu]    IMU: High resolution IMU detected!
[drone1.imu]    IMU: Attitude quaternion IMU detected!   ← EKF가 attitude estimate publish 중!
[drone1.mavros] CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
[drone1.sys]    VER: 1.1: Flight software: 01110040 (PX4 v1.17.0)
[drone1.mission]    WP: mission received
[drone1.rallypoint] RP: mission received
[drone1.geofence]   GF: mission received
```

`Attitude quaternion IMU detected!` 는 PX4 가 ATTITUDE_QUATERNION MAVLink 메시지를 publish 한다는 의미 = EKF2 가 attitude estimate 계산 중. **fix 효과 확정**.

---

## 3. 작성/수정한 코드

### 3.1 `settings/px4_5drones_phase5.json` (신규)

PX4 SITL 5대 + Colosseum + lockstep 설정. v5 형태로 commit (의심 param 제거 상태). 핵심 구조:

- `SimMode: Multirotor`, `ClockType: SteppableClock`
- `OriginGeopoint`: Seoul (37.5665, 126.978, 38m)
- 5개 vehicle (drone1~drone5):
  - `VehicleType: PX4Multirotor`, `UseSerial: false`, `LockStep: true`, `UseTcp: true`
  - `TcpPort: 4560~4564`
  - `ControlPortLocal: 15040~15044`, `ControlPortRemote: 15045~15049`
  - `Sensors: Imu/Magnetometer/Barometer/Gps` 모두 Enabled
  - `Parameters`: `COM_ARM_WO_GPS=1`, `EKF2_HGT_REF=1`, `CBRK_*` safety bypass. **EKF2_AID_MASK / EKF2_GPS_CHECK 제거**.

이 파일을 `~/Documents/AirSim/settings.json` 에 deploy 하여 UE Play 시점에 로드.

### 3.2 `settings/px4_5drones_phase5.v4.bak.json` (신규)

문제 원인이 됐던 v4 형태 (회귀 검증용으로 보존). 이 파일은 deploy 하지 않음.

### 3.3 메모리 업데이트 (Claude auto-memory)

`/home/clrobur/.claude/projects/-home-clrobur-workspace-projects/memory/project_aerion_phase5_progress.md` 의 다음 항목 정정/추가:

- Colosseum source canonical 경로 정정 (`AirLib/include/vehicles/multirotor/firmwares/mavlink/`)
- `MavLinkMultirotorApi.hpp` 핵심 line number 추가 (1784/126/140-154/1638)
- 세션 종료 시점 process 상태 + 다음 세션 시작 순서 추가
- 본 root cause 진단 결과를 반영 (commit 후 별도 update 예정)

---

## 4. 재현 절차 (다른 사람이 따라 실행 가능)

### 사전 정리

```bash
pkill -KILL -f 'bin/px4' 2>/dev/null
pkill -KILL -f 'mavros_node' 2>/dev/null
sleep 2
```

### 터미널 1 — UE Editor

```bash
# 만약 죽어있으면:
~/airsim/unreal-engine/Engine/Binaries/Linux/UnrealEditor \
  ~/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject &
```

`~/Documents/AirSim/settings.json` 이 v5 인지 확인:

```bash
md5sum ~/Documents/AirSim/settings.json
# 기대값: a34d5a92e06a793d87c3c85dc89bd95f
```

UE Editor 안에서 **Play** 버튼은 터미널 2 시작 후에 누른다.

### 터미널 2 — PX4 SITL (1대 콘솔)

```bash
mkdir -p /tmp/px4_inst_diag && cd /tmp/px4_inst_diag && \
  PX4_SIM_HOSTNAME=localhost PX4_SIM_MODEL=none_iris \
  ~/airsim/PX4-Autopilot/build/px4_sitl_default/bin/px4 \
  -i 0 -d ~/airsim/PX4-Autopilot/build/px4_sitl_default/etc
```

대기 메시지:
```
INFO [simulator_mavlink] Waiting for simulator to accept connection on TCP port 4560
```

이 시점에 **터미널 1 UE Editor에서 Play 누름**. 터미널 2에 다음 출력 확인:
```
INFO [simulator_mavlink] Simulator connected on TCP port 4560.
INFO [lockstep_scheduler] setting initial absolute time to ...
```

이후 30초 정도 대기. v5 정상 동작 신호는:
- `Preflight Fail: ekf2 missing data` 가 **1회만** (반복 안 됨)
- `GPS fix too low` 가 **출력되지 않음**
- `heading estimate invalid` 가 1~2회 후 잠잠

### 터미널 3 — MAVROS + arm 검증

```bash
source /opt/ros/humble/setup.bash

nohup ros2 run mavros mavros_node --ros-args \
  -r __ns:=/drone1 \
  -p fcu_url:='udp://:14540@127.0.0.1:14580' \
  -p target_system_id:=1 -p target_component_id:=1 \
  > /tmp/mavros_drone1.log 2>&1 &
disown

sleep 20

pgrep -af mavros_node                                         # 살아있는지
timeout 5 ros2 topic echo /drone1/mavros/state --once         # connected:true 확인
timeout 10 ros2 service call /drone1/mavros/cmd/arming \
  mavros_msgs/srv/CommandBool "{value: true}"                 # arm 시도
tail -30 /tmp/mavros_drone1.log
```

**판정**:

| 결과 | 의미 |
|---|---|
| `state.connected: true` + `state.system_status: 3 (STANDBY)` + arm `success: true` | ✅ v5 fix 완전 통과 |
| `connected: true` + arm `success: false, result: 4` | 부분 fail (mag 등) — 추가 조정 필요 |

---

## 5. 알려진 함정 / 주의사항

| 함정 | 회피 |
|---|---|
| PX4 SITL 콘솔 stdin 응답 없음 (`commander check` 입력해도 출력 안 보임) | PX4 work queue 가 lockstep clock 진행에 묶임. 외부에서 MAVROS topic/service 로 검증. |
| MAVROS 를 `&` 로만 백그라운드 실행 시 일정 시간 후 죽는 사례 | `nohup ... > log 2>&1 &` + `disown` 으로 SIGHUP 차단. |
| `EKF2_AID_MASK=0` 을 "aiding 없이 동작" 으로 잘못 해석 | PX4 1.14+ 에서는 **모든 aiding 비활성** 이고 EKF2 fusion 자체 정지. GPS 없이 arm 하려면 `COM_ARM_WO_GPS=1` 만 두고 AID_MASK 는 default. |
| `EKF2_GPS_CHECK=0` 로 GPS sanity check 끄는 것도 잘못 | GPS aiding 자체를 enable 시켜야 EKF 가 sensor fusion 한다. SITL HIL_GPS 는 정상 송신되므로 default 가 안전. |
| 메모리에 `Vehicles/Multirotor/MavLinkMultirotorApi.hpp` 로 적힌 경로는 잘못된 곳 | canonical 은 `AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`. UE Plugin 안 (`Unreal/Plugins/AirSim/Source/AirLib/...`) 에도 동일 파일 복사본. |
| Colosseum lockstep deadlock 가능성 (`MavLinkMultirotorApi.hpp:140-154`) | 이번 케이스에서는 발현 안 함 (tcpdump 1.3K pps 확인) 하지만 향후 다른 증상의 후보로 기억. |

---

## 6. 다음 작업 (next steps)

### 6.1 즉시 (v5 final 검증)

**§4 의 터미널 3 시퀀스로 arm 시도**. 결과 두 갈래:

- **Path A — arm success: true**
  1. 1대 mavros + PX4 kill (단, UE Play 유지)
  2. 기존 launcher 로 PX4 SITL × 5 일괄 기동: `PX4_DIR=~/airsim/PX4-Autopilot DRONE_COUNT=5 BACKGROUND_MODE=true bash ~/airsim/scripts/launch_px4_instances.sh`
  3. UE Stop → Play 다시 (5대 모두 새 connection 받기)
  4. Phase 5 launch.py (5대 MAVROS + formation + tf): 메모리에 적힌 `aerion_phase5_px4.launch.py` 이 repo 에 아직 없음 → 별도 작성 필요
  5. 5대 동시 OFFBOARD + arm + CIRCLE 시연 → 5/5 ARRIVED 검증

- **Path B — arm success: false (heading invalid 잔존)**
  - `EKF2_MAG_TYPE` 명시 (0 = AUTO, 1 = 3D yaw fusion) → settings.json 에 추가
  - 또는 `EKF2_HGT_REF=1` (baro) 제거하고 GPS height (default) 로 전환 → mag heading 수렴 시간 단축 가능
  - 또는 시간을 60초로 늘려 EKF 수렴 대기

### 6.2 코드 작업 (repo 안 — 기존 자산 활용)

본 root cause 진단 commit 작성 직후 origin/branc/airsim-ros2-bridge 를 fetch 해 보니 협업자 `wnsgud5813` 가 **2026-05-18 에 이미 다음 자산을 commit 해 둠** (당시 로컬은 fetch 전이라 보이지 않았음):

- `airsim_ros2_bridge/launch/aerion_phase5_px4.launch.py` (commit d19c092, Round 8) — PX4 SITL 5대 + MAVROS 5대 + bridge×5 + formation + leader + tf 통합 launch
- `airsim_ros2_bridge/config/domain_bridge_external.yaml` (commit d19c092, Round 8) — Phase 6 외부 노출 (DOMAIN 42→7, 화이트리스트 + QoS)
- `airsim_ros2_bridge/scripts/mavros_arm_all.py` (commit 2f78c34, Round 9) — N대 동시 arm + OFFBOARD + takeoff

다음 세션에서는 **재작성 대신 위 자산 활용**. 주의할 정렬 포인트:

- Round 8 commit message 의 fcu_url 매핑은 `udp://:14555+N@127.0.0.1:14540+N` 으로 기재되어 있으나, 본 문서 §4 의 단일 mavros 검증에서 동작한 매핑은 `udp://:14540@127.0.0.1:14580` (PX4 default GCS 14550 / Onboard 14580). settings.json v5 의 `ControlPortLocal=15040+`, `ControlPortRemote=15045+` 는 Colosseum↔PX4 control 채널 (PX4 SITL의 mavlink Onboard 와 무관). **5대 확장 시 launch.py 의 udp 매핑을 v5 settings + PX4 SITL Onboard(14580+) 와 정렬해야 함** — 정렬 미스 시 mavros connect 실패 또는 wrong drone 매핑.
- Round 11 (`bec63b3`) 의 `SingleThreadedExecutor + kinematic_z_ned 정합` 도 phase5 5대 실행 시 영향. bridge_node 실행 시 executor 선택 확인 필요.

### 6.3 문서화

- 본 문서 commit 후 메모리 `project_aerion_phase5_progress.md` 의 "다음 세션 시작 순서" 갱신
- §6.2 의 launch.py 가 완성되면 `docs/bridge_validation_runbook.md` 에 Phase 5 5대 절차 추가

---

## 7. 참고 메모리 / 관련 자료

- `~/.claude/projects/-home-clrobur-workspace-projects/memory/project_aerion_phase5_progress.md` — Phase 5 진척 + 다음 세션 시작점
- `~/.claude/projects/-home-clrobur-workspace-projects/memory/feedback_aerion_airsim_multivehicle_quirks.md` — Colosseum 5/5 동시 100% 어려움 (이 문서로 PX4 stack 가능성 확보)
- `~/.claude/projects/-home-clrobur-workspace-projects/memory/reference_aerion_airsim_rpc_limits.md` — AirSim RPC 50Hz / IOLoop 한계
- Colosseum source: `~/airsim/Colosseum/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`
- PX4 EKF2 parameter doc: https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2
