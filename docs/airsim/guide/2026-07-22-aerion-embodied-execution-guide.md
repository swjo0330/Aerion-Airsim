# AERION ↔ 체화지능(Mac) 통합 실행 가이드 (단일 드론, 2채널)

> 작성 2026-07-22 · 기존 문서 통합본. 환경 스펙 · 경로 · 명령어 · 아키텍처 · 런북을 한 문서로 병합.
> **단일 진입점:** `scripts/test_embodied_link.sh <명령>`. 직접 `pkill` 금지(self-match로 셸 죽음, exit 144) → 항상 `stop`.
> **권위 원본:** [`2026-06-18-embodied-link-runbook.md`](../runbook/2026-06-18-embodied-link-runbook.md) ·
> [`2026-06-18-embodied-topic-integration-design.md`](../design/2026-06-18-embodied-topic-integration-design.md) ·
> [`2026-06-17-sim-comm-extension-design.md`](../design/2026-06-17-sim-comm-extension-design.md)

---

## 0. 이번 세션(2026-07-22) 현재 상태 & 신규 발견

| 항목 | 상태 |
|---|---|
| UE 환경 | **Town10 사용**(권장). BlocksV2는 플러그인 BuildId stale로 크래시(§4.1) |
| UE Play / drone1 | ✅ Play 됨, drone1 스폰, RPC 41451 · lockstep 4560 LISTEN |
| PX4 SITL | ✅ 기동, AirSim 4560 ESTAB |
| bridge_node | ✅ 기동, `/camera/*`·`/range/*` 로컬 발행 |
| zenoh client | ✅ Mac Router 7447 세션 ESTABLISHED, DDS 라우트 생성 |
| mavproxy | ✅ `MAVPROXY_IN_PORT=14550`, `Detected vehicle 1:1`, `--out udpout:100.67.87.116:14555` |
| **Mac 관측 수신** | ✅ **이미지(camera/compressed) 수신 확인** (Router rx 528MB, 12:20 카메라 0.8Hz 실측 이력) |
| **Mac 비행데이터** | ✅ **`/mavros/*` 수신 확인** |
| 제어(control) | 경로 확정 — Mac MAVROS→MAVLink(§2.8). zenoh control-토픽 scoping은 미완(defense-in-depth) |

### 0.1 현재 라이브 설정 명세 (실행 중 프로세스 argv)

| 컴포넌트 | 실행 설정 |
|---|---|
| **UE** | `run_ue_carla_town10.sh` → UE5.5_carla 엔진 + CarlaUnreal(Town10HD_Opt), `town10_stable` 프로필 |
| **AirSim settings** | `~/Documents/AirSim/settings.json` = drone1 PX4Multirotor(§1.6), `EKF2_MAG_CHECK=0` |
| **PX4 SITL** | `px4 -i 0` (MAV_SYS_ID=1), lockstep TCP 4560, Normal/GCS remote 14550 |
| **CycloneDDS** | `settings/cyclone_dds_airsim_client.xml`, `NetworkInterface=lo` (★enp108s0 down 대응) |
| **bridge_node** | `vehicle_name:=drone1 control_backend:=px4_mavros enable_camera:=true camera_fps:=5.0 image_compressed:=true jpeg_quality:=70 controller_pose_rate:=10.0 topic_namespace:=bare publish_mavros_state:=false range_mode:=points enable_range:=true enable_lidar:=false` (카메라 로컬 ~4.9Hz 실측) |
| **zenoh client** | `settings/zenoh_bridge_dds_airsim_client.json5` → connect `tcp/100.67.87.116:7447` |
| **mavproxy** | `--master=udpin:0.0.0.0:14550 --out=udpout:100.67.87.116:14555 --out=udpout:127.0.0.1:14601` |
| **Mac MAVROS** | `fcu_url:="udp://:14555@100.120.219.68:14550" tgt_system:=1` |

**신규 발견 2건 (반드시 반영):**

1. **CycloneDDS NIC를 `lo`로 전환.** 하드코딩됐던 `enp108s0`(유선 LAN)이 **DOWN** 상태여서
   `bridge_node`가 `rmw_create_node: failed to create domain` (`enp108s0 does not match an available interface`)로
   죽었다. 로컬 DDS discovery는 peers=127.0.0.1 + 멀티캐스트 off이므로 **루프백 `lo` 바인딩으로 해결**
   (`settings/cyclone_dds_airsim_client.xml`). cross-host는 어차피 zenoh TCP/Tailscale이라 물리 NIC 무관.
2. **mavproxy in-port는 `14540`이 아니라 `14550`.** (스크립트 기본값 `MAVPROXY_IN_PORT=14540`은 gazebo 시절 값 →
   AirSim+PX4에선 틀림.) PX4 SITL은 MAVLink 인스턴스를 **2개** 연다 — **Onboard(remote 14540)**는 AirSim
   (`ControlPortLocal=14540`)이 정당하게 점유, **Normal/GCS(remote 14550)**가 mavproxy/MAVROS 자리다.
   → **`MAVPROXY_IN_PORT=14550`로 기동하면 충돌 없이 `Detected vehicle 1:1`.** (§1.5·§2.5에 전체 토폴로지.)

---

## 1. 환경 스펙 & 경로 (Environment & Paths)

### 1.1 시스템 스펙

| 항목 | 값 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| GPU / Driver | NVIDIA (Driver 580, CUDA 13.0) |
| Compiler | clang-18 (Ubuntu clang 18.1.8) + libc++-18-dev |
| Python | 3.12.13 (CPython, uv 0.10.12 관리, `.venv/`) |
| cmake | `/usr/bin/cmake` |
| uv | `/home/clrobur/.local/bin/uv` |

### 1.2 UE 엔진 2벌 (서로 호환 안 됨 — 절대 혼용 금지)

프로젝트는 **독립된 UE 엔진 설치 2개**를 쓴다. 각 프로젝트는 반드시 짝이 맞는 엔진으로만 실행한다.

| 환경 | 엔진 버전 | 엔진 경로 | 프로젝트(.uproject) | 맵 |
|---|---|---|---|---|
| **BlocksV2** (baseline) | UE 5.6 | `~/airsim/unreal-engine` | `~/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject` | 기본 Blocks |
| **Town10** (city, 권장) | UE 5.5 | `~/workspace/engines/UE5.5_carla/UnrealEngine` | `~/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject` | `Town10HD_Opt` |

> **CRITICAL GOTCHA — BlocksV2는 현재 실행 시 크래시:**
> BlocksV2의 AirSim 플러그인 BuildId `a123c54b-bcac-45ee-b279-5ee9c8d0919e`가 UE5.6 엔진의
> CompatibleChangelist `43139311`과 **불일치(stale)**. 실행 즉시 `Incompatible or missing module: AirSim`
> → `Exiting abnormally error code 1`로 크래시 → **플러그인 재빌드 필요.**
> Town10 쪽은 모든 플러그인 BuildId가 `a123c54b`로 **일치**하여 클린 실행. **2026-07-22 기준 권장 = Town10.**

### 1.3 핵심 경로

| 대상 | 경로 |
|---|---|
| PX4-Autopilot | `/home/clrobur/airsim/PX4-Autopilot` (실 바이너리: `build/px4_sitl_default/bin/px4`) |
| Colosseum repo | `/home/clrobur/airsim/Colosseum` |
| AirLib 산출물 | `libAirLib.a` (Colosseum build.sh 산출) |
| UE DDC 캐시 | `~/workspace/cache/ue_ddc` |
| UE 로그 | `~/workspace/logs/ue/` |

> **GOTCHA — PX4_DIR:** `launch_px4_instances.sh` 기본값 `~/PX4-Autopilot`은 **틀림**. 반드시
> `PX4_DIR=/home/clrobur/airsim/PX4-Autopilot` 명시.

### 1.4 네트워크

| 항목 | 이 PC (AirSim) | Mac (체화지능) |
|---|---|---|
| Tailscale IP | `100.120.219.68` | `100.67.87.116` |
| NIC | `enp108s0` (⚠️ 현재 DOWN → DDS는 `lo` 사용) | `en4` |
| ROS_DOMAIN_ID | 0 | 0 |
| RMW | `rmw_cyclonedds_cpp` | `rmw_cyclonedds_cpp` |

### 1.5 포트 맵

| 포트 | 용도 | 방향 |
|---|---|---|
| 41451 | AirSim RPC (client API) | 이 PC local |
| 4560 | PX4 SITL ↔ AirSim lockstep (TCP, HIL 센서/액추에이터) | 이 PC local |
| **14550** | **PX4 Normal/GCS MAVLink → mavproxy udpin** (제어/비행데이터 tap) | 이 PC local |
| 14540 | **AirSim onboard 링크** (`ControlPortLocal`) — mavproxy 아님, 건드리지 말 것 | 이 PC local |
| 18570 / 14580 | PX4 Normal / Onboard 인스턴스 listen 포트 | 이 PC local |
| 14555 | Mac MAVROS bind (mavproxy `--out` 목적지) | 이 PC → Mac |
| 14601 | 로컬 자율제어(pymavlink offboard) tap (mavproxy extra `--out`) | 이 PC local |
| 7447 | zenoh Router listen | Mac |

> **PX4 SITL MAVLink 인스턴스 (px4 로그 실측, localhost only):**
>
> | PX4 mode | PX4 listen | remote(클라 bind) | 클라이언트 |
> |---|---|---|---|
> | Normal (GCS) | 18570 | **14550** | **mavproxy / Mac MAVROS (embodied)** |
> | Onboard | 14580 | 14540 | AirSim (`ControlPortLocal=14540`, `ControlPortRemote=14580`) |
>
> 14540을 mavproxy에 쓰면 AirSim onboard 링크와 충돌한다. embodied tap은 반드시 **14550(Normal/GCS)**.

### 1.6 활성 settings.json 요약

- **런타임 위치:** `/home/clrobur/Documents/AirSim/settings.json`
- **정본(canonical) 소스:** `settings/px4_1drone_lidar.json` (런타임은 Town10 스폰 X/Y/Z 및 `EnableRpc:true`만 차이)

| 필드 | 값 |
|---|---|
| SimMode / ClockType | `Multirotor` / `SteppableClock` |
| OriginGeopoint | 37.5665, 126.978, alt 38 |
| Vehicle | `drone1` — `PX4Multirotor`, LockStep, UseTcp, TcpPort 4560, LocalHostIp 127.0.0.1 |
| 제어 포트 | ControlIp 127.0.0.1, ControlPortLocal 14540, ControlPortRemote 14580 |
| Distance 센서 | `Distance_Front`(0.2~40m), `Distance_Left`/`Right`(0.2~20m) |
| LiDAR | `Lidar_Front`: 16ch, 100k pts/s, 10rps, HFOV±45°, VFOV+10/−30°, Range 50m |
| 기타 센서 | Imu, Magnetometer, Barometer, Gps |
| 카메라 | `front_center`: ImageType 0, 1280×720, FOV 78° |
| PX4 Params | **EKF2_MAG_CHECK=0**, EKF2_MAG_TYPE=0, COM_ARM_WO_GPS=2, NAV_RCL/DLL_ACT=0 등 |

### 1.7 UE 실행 스크립트

| 스크립트 | 동작 |
|---|---|
| `scripts/run_ue_blocksv2.sh` | 범용 UE Editor 런처. `UE_BIN`/`UE_PROJECT`/`UE_MAP`/`UE_RENDER_PROFILE` env로 대상 지정. `setsid` 백그라운드 기동, 중복 차단, 로그 tee |
| `scripts/run_ue_carla_town10.sh` | Town10 전용 래퍼 — Town10 env를 export 후 `run_ue_blocksv2.sh`를 `exec` 위임 (`UE_RENDER_PROFILE=town10_stable`) |

> **운영 규칙:** 두 스크립트 모두 **UE Editor만** 띄운다. **▶ Play는 GUI에서 사용자가 직접** 눌러야 한다
> (AirSim은 Editor PIE 모드에서만 안정, standalone game은 별도 패키징). Play 후 PX4 SITL이 TCP 4560에 connect.

---

## 2. 연동 아키텍처 & 토픽 계약 & Mac 측 실행

### 2.1 2-track 아키텍처 (관측=Zenoh read-only · 제어=MAVLink via mavproxy)

```
[AirSim PC — Ubuntu 22.04]                                     [Mac — 체화지능]
 Tailscale 100.120.219.68                                       Tailscale 100.67.87.116

 UE5.x + AirSim ──lockstep TCP 4560── PX4 SITL
   │              (onboard 14580↔14540 = AirSim 전용)  │ Normal/GCS: PX4 18570 → remote 14550
 bridge_node ─ camera/range RPC(41451)                 ▼
   │ bare DDS (CycloneDDS, lo 바인딩)  [Track 2 · 제어/비행데이터]
   ▼                                 mavproxy  udpin:14550
 zenoh-bridge-dds 1.2.1 (CLIENT)         │ --out udpout → 100.67.87.116:14555
   │                                     │
   │  [Track 1 · 관측 read-only]         │        ┌───────────────────────────────────┐
   └────── Tailscale TCP 7447 ───────────┼──────► │ zenoh Router 1.8.0 (:7447) ─► detector
          (image/camera_info/range)      │        └───────────────────────────────────┘
                                         └── Tailscale UDP ─► MAVROS(bind 14555) ─► /mavros/*
```

- **Track 1 (관측):** AirSim RPC(41451) → `bridge_node` bare DDS 발행 → CycloneDDS(localhost/lo) →
  `zenoh-bridge-dds 1.2.1`(client) → Tailscale TCP `7447` → Mac `zenoh Router 1.8.0` → detector. **단방향 읽기.**
- **Track 2 (제어/비행데이터):** PX4 SITL Normal/GCS(→remote 14550) → mavproxy(`--master=udpin:0.0.0.0:14550`
  → `--out=udpout:100.67.87.116:14555`) → Mac MAVROS(remote `fcu_url`) → `/mavros/*`. **제어는 이 경로만.**
- zenoh 버전 상이(Ubuntu=1.2.1, Mac=1.8.0)하나 1.x wire 호환 e2e 검증됨. 양단 `ROS_DOMAIN_ID=0`.

### 2.2 one-writer 안전 규칙 (필수)

- **제어 토픽은 절대 Zenoh 미경유.** allow-list = 관측(camera/range/lidar)만. `setpoint`·`cmd`·`mavros`·`rt/fmu/in/*` 차단.
- **AirSim PC의 local MAVROS는 OFF.** 로컬 MAVROS가 `14550`(mavproxy udpin) 점유 시 충돌(heartbeat 없음의 주원인). `14540`은 AirSim onboard 전용이라 건드리지 않는다.
- **bridge는 camera+range만**, `PUBLISH_MAVROS_STATE=false`. `/mavros/*`의 유일 소유자 = **Mac MAVROS**.
  `/ap/*`·`/clock`도 발행 금지.

### 2.3 토픽 계약 (gap = 0) — bare namespace

| 채널 | 토픽 | 타입 | 발행 주체 | 실측 rate |
|---|---|---|---|---|
| 관측(zenoh) | `/camera/image/compressed` | `sensor_msgs/CompressedImage`(JPEG) | bridge | ~5–7Hz |
| 관측(zenoh) | `/camera/camera_info` | `sensor_msgs/CameraInfo` | bridge | ~30Hz |
| 관측(zenoh) | `/range/front/points` | `sensor_msgs/PointCloud2`(3D, 전방 1점) | bridge | ~3–9Hz |
| 제어/비행데이터 | `/mavros/*` | MAVROS 표준 | **Mac MAVROS** | 50Hz |

- **raw 720p(2.76MB/frame)는 zenoh/Tailscale 단편 드롭** → 전송은 **JPEG CompressedImage 필수**.
  Mac은 `image_transport republish compressed raw`로 detector에 raw 공급.
- 모드 문자열(PX4 `OFFBOARD`/`AUTO.*` vs APM `GUIDED`/`AUTO`)만 체화지능 측 어댑터 필요.

### 2.4 Mac 측 실행 명령

**(1) zenoh Router `:7447`** — 가장 먼저 기동. 안 떠 있으면 `zenoh` 단계가 `Mac Router 7447 미도달`로 실패.
```bash
zenohd --listen tcp/0.0.0.0:7447     # + detector 노드, ROS_DOMAIN_ID=0
```
**(2) MAVROS**:
```bash
ROS_DOMAIN_ID=0 ros2 run mavros mavros_node --ros-args \
  -p fcu_url:="udp://:14555@100.120.219.68:14550" \
  -p tgt_system:=1 -p tgt_component:=1
```
- `:14555` = 로컬 bind(수신), `@100.120.219.68:14550` = 명령 송신 원격 엔드포인트(=우리 mavproxy udpin **14550**).
- mavproxy 재기동 시 수신(14555)·명령(14550) 엔드포인트 고정 → **Mac MAVROS는 재시작 없이 자동 재연결.**

### 2.5 mavproxy 명령 명세 (AirSim PC, 제어/비행데이터 forwarder)

`scripts/run_mavproxy_to_embodied.sh`가 실제 실행하는 argv (검증됨, `Detected vehicle 1:1`):

```bash
mavproxy.py \
  --master=udpin:0.0.0.0:14550 \        # PX4 Normal/GCS MAVLink 수신 (PX4 18570 → remote 14550)
  --out=udpout:100.67.87.116:14555 \    # → Mac MAVROS (fcu_url bind 14555)
  --out=udpout:127.0.0.1:14601 \        # → 로컬 자율제어(pymavlink offboard) tap. EXTRA_OUT 없으면 생략
  --state-basedir=/tmp/mav_embodied --daemon
```

| mavproxy 인자 | 값 | 의미 | 오케스트레이터 env |
|---|---|---|---|
| `--master` | `udpin:0.0.0.0:14550` | PX4 GCS MAVLink 수신 (★ 14540 아님) | `MAVPROXY_IN_PORT=14550` |
| `--out` (1) | `udpout:100.67.87.116:14555` | Mac MAVROS 목적지 | `MAC_TAILSCALE_IP` / `MAC_MAVROS_PORT` |
| `--out` (2) | `udpout:127.0.0.1:14601` | 로컬 offboard(pymavlink) | `MAVPROXY_EXTRA_OUT` |

> ⚠️ 기본 `MAVPROXY_IN_PORT=14540`은 AirSim onboard 링크와 충돌 → precheck `UDP 14540 local MAVROS 점유`로 거부.
> **반드시 `MAVPROXY_IN_PORT=14550`로 실행:** `MAVPROXY_IN_PORT=14550 ./scripts/test_embodied_link.sh mavproxy`.
> (근본 수정으로 스크립트/settings 기본값을 14550으로 바꾸는 것을 권장 — TODO §0.)

### 2.6 검증 명령 (Mac, typed echo — `ros2 topic list`는 zenoh 너머 불신)

```bash
ROS_DOMAIN_ID=0 ros2 topic hz   /camera/image/compressed                                  # ~5–7Hz
ROS_DOMAIN_ID=0 ros2 topic echo --once /camera/camera_info  sensor_msgs/msg/CameraInfo    # 1280×720
ROS_DOMAIN_ID=0 ros2 topic echo --once /range/front/points  sensor_msgs/msg/PointCloud2   # width=1
ROS_DOMAIN_ID=0 ros2 topic echo --once /mavros/state                                       # connected: true
ROS_DOMAIN_ID=0 ros2 topic hz   /mavros/local_position/pose                                # ~50Hz
```

### 2.8 제어 토픽으로 드론 제어 (Mac MAVROS → MAVLink 경로)

**질문 "제어 토픽으로도 제어가 가능한가?" → YES.** 단, 경로는 **zenoh가 아니라 MAVLink**다. gazebo+ArduPilot 시절과
동일하게 **Mac 체화지능 노드가 표준 MAVROS 제어 토픽/서비스를 쓰면, Mac MAVROS가 MAVLink로 변환해 PX4를 제어**한다.

```
[Mac] 체화지능 노드 ─► /mavros/setpoint_velocity/cmd_vel   (또는 /mavros/setpoint_position/local)
                       /mavros/set_mode (svc: OFFBOARD)      /mavros/cmd/arming (svc)
                                │
                          Mac MAVROS ─ MAVLink ─► mavproxy(14550) ─► PX4 SITL ─► AirSim(lockstep 4560)
```

- **단일 writer = Mac MAVROS.** 제어 명령은 Mac MAVROS의 `fcu_url` 명령 반환경로(`@100.120.219.68:14550`)로
  mavproxy에 전달되어 PX4에 도달한다.
- **arming/mode 주의(PX4):** OFFBOARD 진입엔 setpoint 스트림(>2Hz) 선행 필요, arming은 AirSim mag self-check로
  실패할 수 있어 **`EKF2_MAG_CHECK=0`(적용됨)** + 필요 시 force-arm(`param2=21196`). PX4 모드 문자열은
  `OFFBOARD`/`AUTO.*` (APM `GUIDED`와 다름) → 체화지능 어댑터 확인.
- **bridge의 제어 콜백은 이 경로가 아님:** bridge는 `/cmd_*`·`/mavros/setpoint_*`를 구독하지만 콜백이 **AirSim API**
  (`enableApiControl`/`moveByVelocity`)를 호출 → **PX4에선 "valid GPS home" 거부로 무력**, 또한 `control_backend=px4_mavros`
  에서 `allow_local_motion=False`라 **실제로 아무 동작 안 함(inert)**. 즉 PX4 embodied에서 제어의 실효 경로는 **Mac MAVROS뿐**.
- **★ zenoh control-토픽 scoping (권장 조치):** 현재 zenoh가 `rt/cmd_vel`·`rt/mavros/setpoint_*`를 Zenoh→DDS로
  역라우팅한다. px4_mavros에서 bridge가 inert라 당장 이중제어는 없지만, **혼선·미래 위험 방지를 위해 zenoh allow-list에서
  control 토픽을 제외**(관측 camera/range/lidar만 허용)하는 것을 권장. (기존 미완 항목 "Phase C2 zenoh scoping".)

**로컬 대안(자율비행 스크립트):** Mac 없이 이 PC에서 직접 제어하려면 pymavlink offboard(`fly_drone1_city_demo.py`,
mavproxy extra out `udpout:127.0.0.1:14601`) 사용 — §4.2.

### 2.9 성능 비교 (2026-07-22 vs 06-18 정본, Mac 수신 실측)

| 지표 | 06-18 정본 | 2026-07-22 (20s 안정구간) | 판정 |
|---|---|---|---|
| 카메라 fps | 7.2Hz | **5.0Hz** | ⚠️ ~70% — 유일한 저하 (원인=설정값, 손실 아님) |
| 프레임 크기 | 187KB/f | 162KB/f | ✅ 동급 (JPEG 압축 정상) |
| 대역폭 | ~1.35MB/s | 817KB/s | fps 저하분 그대로 |
| 프레임 간격 안정성 | — | p50 199ms·p95 283ms·max 318ms | ✅ 매우 규칙적 (드롭/버스트 없음) |
| range 3D (PointCloud2) | 23KB급 통과 | 안정 | ✅ |
| detect_frame | — | 7.4Hz | ✅ 기준선 도달 |
| GPS/MAVLink | — | 풀스트림 | ✅ |

**결론:** 비행데이터·검출·range는 과거와 **동급**, **카메라만 7.2→5.0Hz**.

**원인 (전송 저하 아님):** 프레임 간격이 **정확히 199ms(=5Hz)로 극히 규칙적** → 손실로 깎이는 패턴(불규칙 간격·버스트)이
아니다. 현재 bridge `camera_fps:=5.0` **설정값 자체**가 5Hz 발행이고(§0.1), 이는 AirSim 로컬 캡처 주기 설정이다.
전송 경로(zenoh/Tailscale)는 무손실. RPC 천장은 ~9.4fps(§4.1).

**복원 방법:** `CAMERA_FPS`를 올려 재기동하면 됨 — 예 `CAMERA_FPS=7.5 ./scripts/test_embodied_link.sh bridge`
(최대 RPC 천장 ~9.4fps). 데모 용도로는 현 5.0Hz도 뷰어·검출 체인이 문제없이 소화. (언리얼 씬 부하가 높으면
설정만큼 안 나올 수 있으므로 fps 상향 후 `ros2 topic hz`로 재실측 권장.)

---

## 3. 실행 (파이프라인 쉘 + 단일 진입점)

### 3.0 ★ 전체 실행 파이프라인 한 방 — `scripts/run_embodied_pipeline.sh`

언리얼 기동 → Play 대기 → 관측 → 제어까지 순서대로 묶은 오케스트레이터(신규). `test_embodied_link.sh`를 내부에서 호출한다.

```bash
cd ~/workspace/projects/aerion-airsim
bash scripts/run_embodied_pipeline.sh            # Town10 + 전체(관측+제어)
UE_ENV=none     bash scripts/run_embodied_pipeline.sh   # UE 이미 실행중 → 링크만
SKIP_MAVPROXY=1 bash scripts/run_embodied_pipeline.sh   # 관측만
```

**동작(자동):** ① UE Editor 기동(Town10 기본, 41451 이미 뜨면 skip) → ② **▶ Play 폴링 대기**(GUI 수동, 41451+4560 LISTEN까지)
→ ③ `px4` → **Mac Router 7447 도달 재시도** → `zenoh` → `bridge` → ④ `mavproxy`(**`MAVPROXY_IN_PORT=14550` 자동**)
→ ⑤ `status` + Mac 실행/검증 명령 출력.

| env | 기본 | 의미 |
|---|---|---|
| `UE_ENV` | `town10` | `town10`\|`blocksv2`\|`none`(UE 재기동 안 함) |
| `MAVPROXY_IN_PORT` | `14550` | ★ 14540 금지(AirSim onboard) |
| `SKIP_MAVPROXY` | `0` | `1`=관측만 |
| `PLAY_WAIT_TIMEOUT` | `300` | Play 폴링 최대초 |
| `ROUTER_WAIT_TIMEOUT` | `120` | Mac Router 재시도 최대초(초과 시 zenoh 생략하고 진행) |

> ⚠️ UE ▶ Play는 GUI 수동(AirSim PIE 전용) — 스크립트는 대신 못 누르고 "대기"만 한다.
> ⚠️ Mac zenoh Router(:7447) 선행 권장. 미가동이면 Router 재시도 후에도 없으면 zenoh 생략(관측 Mac 미전달), 나중에 `test_embodied_link.sh zenoh && ... bridge`로 이어 붙임.

### 3.1 서브커맨드 전체 (`scripts/test_embodied_link.sh`)

| 명령 | 하는 일 |
|---|---|
| `preflight` | Tailscale/Mac ping · `nc :7447` 도달 · zenoh 버전 · RMW/DOMAIN/CYCLONEDDS_URI 출력 |
| `px4` | PX4 SITL 기동(`PX4_DIR`/`DRONE_COUNT`). 4560 LISTEN 없으면 UE Play 먼저 하라며 실패 |
| `zenoh` | zenoh-bridge-dds **client** 기동(→ Mac Router). Mac Router 미도달 시 실패 |
| `pub` | `/airsim_test` 2Hz 발행(연결 검증용) |
| `bridge` | `bridge_node` 기동 — camera/range(옵션 lidar). AirSim/PX4 필요 |
| `mavproxy` | PX4 MAVLink → Mac MAVROS UDP 포워드. 14540 점유 시 실패 |
| `verify` | zenoh 세션 · 로컬 topic list · Mac typed echo 안내 · mavproxy 링크 |
| `status` | zenoh/pub/bridge/mavproxy/px4 pid, 4560, Mac Router 세션 현황 |
| `stop` | 자원 해제(graceful→SIGKILL), 포트/세션 해제 검증(`✓/⚠`) |
| `connect` | `preflight → zenoh → pub → verify` |
| `pipeline` | `preflight → px4 → zenoh → bridge → verify` (전체 1커맨드) |

### 3.2 권장 실행 순서

**Step 0 — 수동 선행 (스크립트가 안 함)**
1. **Mac:** zenoh Router `:7447` 먼저 기동 (안 되면 zenoh/pipeline 실패), detector, ROS_DOMAIN_ID=0.
2. **AirSim PC(UE):** Town10 에디터 Play → drone1 spawn. `ss -tlnp | grep -E ':41451|:4560'` 둘 다 LISTEN 확인.
3. `tailscale status | grep 100.67.87.116`.

**Step 1 — 관측 채널**
```bash
cd ~/workspace/projects/aerion-airsim
./scripts/test_embodied_link.sh pipeline
```

**Step 2 — 제어/비행데이터 채널** (★ in-port 14550 명시 — §2.5)
```bash
MAVPROXY_IN_PORT=14550 ./scripts/test_embodied_link.sh mavproxy
# → Detected vehicle 1:1, --out udpout:100.67.87.116:14555
```

**재시작만 (PX4/UE 보존)**
```bash
STOP_PX4=false ./scripts/test_embodied_link.sh stop
./scripts/test_embodied_link.sh zenoh && \
./scripts/test_embodied_link.sh bridge && \
MAVPROXY_IN_PORT=14550 ./scripts/test_embodied_link.sh mavproxy
```

### 3.3 환경변수 오버라이드

| 변수 | 기본값 | 의미 / 주의 |
|---|---|---|
| `PX4_DIR` | `/home/clrobur/airsim/PX4-Autopilot` | **런처 기본 `~/PX4-Autopilot`은 틀림** |
| `DRONE_COUNT` | `1` | PX4 SITL 인스턴스 수 |
| `VEHICLE` | `drone1` | AirSim vehicle 이름 |
| `CAMERA_FPS` | `5.0` | ★ **DOUBLE 필수**(정수 시 `.0` 자동 보정). RPC 천장 ~9.4fps |
| `RANGE_MODE` | `points` | `/range/front/points` PointCloud2. `range`/`laserscan`/`both` |
| `ENABLE_LIDAR` | `false` | 카메라와 RPC 경합 → 검증 중 off |
| `IMAGE_COMPRESSED` | `true` | JPEG 압축(raw 2.76MB 단편 드롭) |
| `JPEG_QUALITY` | `70` | JPEG 화질 |
| `TOPIC_NAMESPACE` | `bare` | `/camera/*`·`/range/*`. `__vehicle__`=`/drone1/*` 레거시 |
| `PUBLISH_MAVROS_STATE` | `false` | bridge는 `/mavros/*` 발행 안 함(Mac MAVROS 담당) |
| `MAC_TAILSCALE_IP` | `100.67.87.116` | Mac |
| `AIRSIM_TAILSCALE_IP` | 자동/`100.120.219.68` | Mac MAVROS `fcu_url` 원격 엔드포인트 |
| `MAVPROXY_IN_PORT` | 기본 `14540`(❌ AirSim 충돌) → **`14550` 사용** | mavproxy udpin = PX4 Normal/GCS remote 포트 |
| `MAC_MAVROS_PORT` | `14555` | Mac MAVROS bind(= mavproxy `--out` 목적지) |
| `MAVPROXY_EXTRA_OUT` | `udpout:127.0.0.1:14601` | 로컬 자율제어(pymavlink offboard) 출력 |
| `STOP_PX4` / `KEEP_TLOG` | `true` / `false` | stop 보존 옵션 |
| `ROS_DOMAIN_ID` | `0` | Mac과 동일 |

### 3.4 종료 / 자원 해제

```bash
./scripts/test_embodied_link.sh stop                  # 전체 종료
STOP_PX4=false ./scripts/test_embodied_link.sh stop   # PX4/UE 보존(링크만)
KEEP_TLOG=true ./scripts/test_embodied_link.sh stop   # tlog 보존
```
`stop`은 graceful(SIGTERM ~1.5s)→SIGKILL 후 포트(14540/7447/4560)·세션 해제를 `✓/⚠`로 보고. UE는 GUI라 수동 종료.

> ⚠️ **절대 금지:** raw `pkill -f mavproxy` — 자기 셸 매칭으로 오케스트레이터 셸을 죽이고 **exit 144**. 반드시 `stop`.

---

## 4. 트러블슈팅 & 자율 비행 제어

### 4.1 트러블슈팅 (증상 | 원인 | 해결)

| 증상 | 원인 | 해결 |
|---|---|---|
| **bridge `rmw_create_node: failed to create domain` / `<nic> does not match an available interface`** | CycloneDDS config가 하드코딩한 NIC가 DOWN(2026-07-22: enp108s0 down) | `settings/cyclone_dds_airsim_client.xml`의 `NetworkInterface name`을 UP NIC 또는 **`lo`**로 교체(로컬 discovery는 lo로 충분). zenoh·bridge 재기동 |
| **`mavproxy` `UDP 14540 local MAVROS 점유`** | 스크립트 기본 in-port 14540이 AirSim onboard 링크(`ControlPortLocal=14540`)와 충돌. embodied는 PX4 Normal/GCS(remote 14550)를 tap해야 함 | **`MAVPROXY_IN_PORT=14550 ./scripts/test_embodied_link.sh mavproxy`** → `Detected vehicle 1:1`. Mac fcu_url 원격도 `...:14550` (§2.5·§2.6) |
| 셸이 갑자기 종료(exit 144) | raw `pkill -f mavproxy` self-match | `stop` 명령 사용, 부득이하면 `[m]avproxy` |
| Mac 토픽 0개(세션 ESTAB) | bridge 다회 재시작 후 zenoh 라우트 stale | zenoh client 재기동(`stop`→`zenoh`), 필요시 Mac Router도 |
| `zenoh` `Mac Router 7447 미도달 / Connection refused` | Mac zenoh Router 미가동 | Mac에서 `zenohd --listen tcp/0.0.0.0:7447` 먼저 |
| `camera_fps InvalidParameterType` | DOUBLE 기대에 INTEGER | `CAMERA_FPS`에 `.0` |
| `set -u`에서 ROS source 실패 | `AMENT_TRACE_SETUP_FILES` unbound | `source_ros()`가 `set +u`로 감쌈(적용됨) |
| camera 720p ≥9fps 안 나옴 | AirSim `simGetImages` RPC 천장 ~9.4fps(해상도 무관, ~106ms/call) | JPEG 압축으로 전송 해소. 30fps는 PixelStreaming |
| `/range/front` 타입 불일치 | detector는 3D PointCloud2 구독 | `RANGE_MODE=points` |
| `4560 없음(UE off)` 오탐 | status가 LISTEN만 체크(실제 ESTAB) | mavproxy `Detected vehicle` / `ss -tn :4560` 교차확인 |
| mavproxy heartbeat 없음 | PX4 미가동/UE Play 안 함/14540 점유 | PX4 먼저, local MAVROS kill |
| **BlocksV2 크래시 `Incompatible or missing module: AirSim`** | UE5.6 재빌드로 BlocksV2 AirSim 플러그인 stale(BuildId mismatch) | 플러그인 재빌드 또는 **Town10 사용**(플러그인 일치) |

### 4.2 자율 비행 제어 (drone1, PX4 OFFBOARD)

관련 스크립트: `fly_drone1_city_demo.py`(시퀀스), `control_drone1.py`(정밀), `_diag_vel_vs_pos.py`(진단),
`prepare_px4_manual_control.sh`(파라미터 준비).

**제어 경로 (왜 MAVLink offboard인가):**
- AirSim Python API는 PX4에서 "valid GPS home" 거부(`getHomeGeoPoint=nan`) → **사용 불가.**
- **pymavlink MAVLink OFFBOARD**: ① force-arm(`COMPONENT_ARM_DISARM` param2=`21196`) ② OFFBOARD(`DO_SET_MODE` main=`6`)
  ③ `SET_POSITION_TARGET_LOCAL_NED` **20Hz** 상시 스트리밍(offboard 유지 >2Hz 필수). 좌표계 `MAV_FRAME_LOCAL_NED`(z 아래 음수).

**연결 (one-writer):**
| 구간 | 엔드포인트 |
|---|---|
| mavproxy 추가출력 | `EXTRA_OUT=udpout:127.0.0.1:14601` (env `MAVPROXY_EXTRA_OUT`) |
| pymavlink 수신/명령 | `udpin:127.0.0.1:14601` |
- 단일 제어원 = 비행 스크립트 하나. Mac MAVROS는 read-only 유지.

**★ 필수 픽스 `EKF2_MAG_CHECK=0`:**
- AirSim 시뮬 자력계가 PX4 mag sanity check 실패(`mag_ratio=2.0` 거부) → heading 미수렴 → 수평 OFFBOARD 차단.
- 끄면 `mag_ratio~0.01`, heading 정상. `prepare_px4_manual_control.sh`에 포함.
- 런타임 적용: `param set EKF2_MAG_CHECK 0` → `PREFLIGHT_STORAGE` 저장 → **reboot**.

**상태(2026-06-18 체크포인트):**
| 항목 | 상태 |
|---|---|
| arm / 이륙 / 고도유지 / AUTO.LAND / disarm | ✅ 안정 |
| velocity 수평 이동(3m/s) | ✅ 즉시·정밀 |
| position-setpoint-from-hover 수평 항법 | ⚠️ 초기 ~2–3분 지연(사실상 미작동) |
- **다음 작업 = velocity 기반 waypoint 항법 재작성**(`VEL_MASK = 0b0000110111000111`).
- 운영 팁: 비행 전 PX4 fresh reboot + `mag_ratio<1` 확인. 스트림 끊기면 offboard failsafe 자동착륙(안전).
