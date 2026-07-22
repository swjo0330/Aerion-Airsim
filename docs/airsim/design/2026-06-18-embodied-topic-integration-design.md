# AERION AirSim ↔ 체화지능 토픽 통합 설계안 (2026-06-18)

> 체화지능(Gazebo+ArduPilot 기반, Mac) 스택의 실제 토픽 목록(2026-04-09)과
> 우리 AirSim/PX4 bridge 출력을 교차분석해, **단일 드론 PoC에서 토픽을 통일·연결**하는 실행 설계.
> 근거: 병렬분석 A(체화지능 소비자 계약, status §15.2/§15.3 + topic-mapping), B(bridge 실출력+px4_mavros 게이팅).
> 상위 문서: `docs/airsim/design/2026-06-17-sim-comm-extension-design.md`, `docs/status_2026-06-17_single_drone_zenoh_memory.md` §15.

---

## 0. 한 줄 요약
**2채널**: 관측(카메라/range)은 bridge가 AirSim RPC→**bare 토픽**으로 빼서 **zenoh**로, FCU 상태·제어는 **mavproxy MAVLink→Mac MAVROS**가 표준 `/mavros/*`로. 토픽 "통일(래핑)"은 카메라/range만 bridge가, FCU 토픽은 **MAVROS가 펌웨어 무관하게 자동** 처리.

---

## 0c. ✅ 관측 채널 e2e 종결 (Mac 수신 확인, 2026-06-18)
| 토픽 | Mac 수신 | 비고 |
|---|---|---|
| `/camera/image/compressed` | 7.1fps, 188KB/f ✅ | JPEG. raw는 단편 드롭, 압축이 정상경로 |
| `/camera/camera_info` | 7.0fps, 1280×720 ✅ | |
| `/range/front/points` | 3.6fps, PointCloud2(pts=1) ✅ | detector 3D range fusion 직결 |

detector 계약 충족 — 그대로 붙음. **관측 채널 종결.**
- **후속(추론 시만)**: 메시지 `frame_id`에 `drone1_` prefix 잔존(예 `drone1_front_center_optical`, `drone1_range_front_link`). 토픽 매칭엔 무관하나 실추론/TF 시 bare로 통일 필요(tf_publisher frame 불일치 버그와 함께).
- **다음 = 채널2** mavproxy→Mac MAVROS (비행데이터/제어). 체화지능 측 firmware 작업(`AUTOPILOT_TYPE=px4` + MAVROS px4.launch 분기)은 실코드 변경이라 그때 승인 후 진행.

## 0d. 채널2(MAVLink 제어/비행데이터) 포워더 가동 (2026-06-18)
`scripts/test_embodied_link.sh mavproxy` → `run_mavproxy_to_embodied.sh`: PX4 SITL ─MAVLink(onboard, rc.mavlink `-o 14540`)→ mavproxy `udpin:0.0.0.0:14540` → `udpout:100.67.87.116:14555` → Mac MAVROS.
- **IP**: AirSim PC Tailscale `100.120.219.68`, Mac `100.67.87.116`. PX4 inst0: offboard remote=14540(=mavproxy bind), local=14580.
- **검증(forwarder)**: `Detected vehicle 1:1`, `online system 1`, `COMMAND_ACK ACCEPTED`(양방향) ✅. EKF 워밍업(heading invalid) 정상.
- **Mac fcu_url**: `udp://:14555@100.120.219.68:14540` (14555 bind 수신, 명령은 우리 14540 mavproxy udpin→PX4 브릿지). 대안 reply-to-source `udp://:14555@`.
- **오케스트레이터 통합**: `step_mavproxy`(pre-kill+백그라운드+링크확인+Mac명령 출력), `status`/`stop`/`verify`에 mavproxy 반영. 자기매칭 pkill은 `[m]avproxy` 브래킷 필수.
- **✅ 채널2 e2e 종결(Mac 확인)**: `/mavros/state` `connected=True, mode='AUTO.LOITER'`(PX4 모드문자열), gps lat/lon 정상(fix=0=EKF 워밍업), pose frame=map. one-writer: AirSim PC local MAVROS(14540) 금지.
- **발견**: Mac이 **`apm.launch`로도 PX4 모드문자열을 정확히 디코드** → `px4.launch` 분기 불필요(firmware 작업 축소). 핵심은 fcu_url **원격 엔드포인트** `@100.120.219.68:14540`(명령 반환경로). ⚠️ 단 OFFBOARD setpoint/arming 등 *제어 플러그인* 동작은 apm vs px4 pluginlist 차이가 있을 수 있어, 실제 원격제어(arm/takeoff) 단계에서 재검증 필요.
- **현황: 양 채널(관측 zenoh + 비행데이터/제어 MAVROS) 모두 가동.**

## 1. 2채널 아키텍처
```
[채널1 — 관측 / zenoh 토픽]
  UE/CARLA+AirSim ─RPC 41451→ bridge_node ─bare(/camera/image, /camera/image/compressed,
                                              /camera/camera_info, /range/front)→
     CycloneDDS(localhost) → zenoh CLIENT ─Tailscale TCP 7447→ Mac zenoh ROUTER → Mac DDS → AERION AI 노드

[채널2 — FCU 상태+제어 / mavproxy MAVLink]
  PX4 SITL ─MAVLink→ mavproxy(run_mavproxy_to_embodied.sh) ─UDP/Tailscale→ Mac MAVROS
     → /mavros/* (state, local_position/pose, global_position/*, vfr_hud, setpoint_*, 서비스)
```
- 관측 = 단방향 읽기. 제어 = MAVLink만(절대 zenoh 미경유).
- 단일 드론 = bare 토픽(Gazebo 규약 동일). 다중 드론은 `ROS_DOMAIN_ID` 분리(차후).

---

## 2. 체화지능 토픽 계약 — 소스 패밀리 분류 (병렬분석 A)
체화지능 AI 노드가 실제 *구독*하는 것만 추림(§15.2):

| 소비자 | 구독 토픽 | 소스 패밀리 |
|---|---|---|
| vision_detector | `/camera/image`, `/camera/camera_info`, `/range/front`(opt) | **sim-센서** |
| vision_detector | `/mavros/global_position/{global,rel_alt,compass_hdg}`, `/mavros/vfr_hud`, `/mavros/home_position/home` | MAVROS(Mac) |
| self_destruct | `/range/front`, `/mavros/local_position/pose`, `/mavros/global_position/global`, `/mavros/vfr_hud`, `/mavros/mission/waypoints`; setpoint·서비스 발행 | MAVROS(Mac) |
| reasoning/planner/uas_link | `/mavros/state`, `/mavros/local_position/pose`, `/perception/state` | MAVROS(Mac)+AI |
| (모든 AI) | `/vision/*`, `/perception/state`, `/replan_waypoints`, `/a2a/*` | **AI 자체 생성** |

**판정:**
- **sim-센서(=bridge 책임)**: `/camera/image`, `/camera/camera_info` (필수), `/range/front` (옵션). 끝.
- **MAVROS(=Mac, mavproxy)**: 모든 `/mavros/*` (state·pose·global·vfr_hud·home·waypoints·setpoint·서비스).
- **/ap/* (ArduPilot DDS)**: 체화지능 **직접 구독 없음** → PX4는 `/mavros/*`로 대체. **갭 없음**.
- **Gazebo bare `/imu`,`/navsat`,`/odometry`,`/battery`,`/magnetometer`,`/air_pressure`,`/clock`,`/range/{left,right}`**: **구독 AI 노드 없음** → bridge 제공 불필요(AI는 IMU/GPS/배터리를 `/mavros/*`로 받음).

---

## 3. 매치 / 갭 표
| 항목 | 체화지능 요구 | 우리 bridge (bare, px4_mavros) | 상태 |
|---|---|---|---|
| `/camera/image` | 필수 | active ✅ | **일치** |
| `/camera/image/compressed` | (전송용) | active ✅ (JPEG) | **일치+** |
| `/camera/camera_info` | 필수 | active ✅ | **일치** |
| `/range/front` | 옵션 | active ✅ | **일치** |
| `/mavros/*` | Mac MAVROS | 빈 advertise만(px4_mavros 게이팅) | **무충돌**(데이터 0) |
| `/ap/*` | 미구독 | 미발행 | **갭 없음** |
| bare `/imu`·`/navsat`·… | 미구독 | 미발행 | **불필요** |
| `/range/{left,right}`, `/lidar/points` | 미구독 | active(잉여) | 무해 |

→ **관측 갭 = 0.** 요구 3종(camera/image·camera_info·range_front)을 bridge가 bare로 정확히 충족.

---

## 4. 토픽 통일(bare 발행) — ✅ 구현 완료 (2026-06-18)
- `topic_namespace` 파라미터를 4개 발행자(camera/range/lidar/DroneController)+bridge_node에 통과. `topic_naming.py` 헬퍼.
- bridge_node param: `__vehicle__`(기본=`/drone1/*` 레거시) / `bare`(=`/camera/image` 등) / 그외=ns.
- DroneController는 `topic_prefix`(line 93) 한 곳이 mavros/cmd 전체 지배. bare시 mavros 인스턴스ns도 `mavros`.
- 오케스트레이터 `TOPIC_NAMESPACE`(기본 bare). **실측: 전 토픽 bare, /drone1 0개.**
- 절대경로 f-string이라 launch `__ns:=/` remap은 불가 → 코드 파라미터화가 정답이었음(검증).

---

## 5b. ✅ 확정(2026-06-18 병렬검토): bridge는 비행데이터를 래핑하지 않는다
- **MAVROS = MAVLink↔ROS 변환기**. PX4 ─MAVLink→ mavproxy(UDP)→ Mac MAVROS가 MAVLink(ATTITUDE/GLOBAL_POSITION_INT/SYS_STATUS/VFR_HUD…)를 파싱해 **`/mavros/*`를 스스로 publish**. → 우리는 **MAVLink만 포워드**, 비행데이터 래핑 0.
- bare `/imu`·`/navsat`·`/odometry`·`/battery`는 원래 Gazebo가 **FCU에 주입한 센서 입력**(AI 소비 아님). AirSim+PX4에선 **AirSim이 PX4에 직접 주입(lockstep 4560)→EKF2→Mac MAVROS** → bridge 관여 없음.
- **bridge 발행 = `/camera/*` + `/range/*` 만**(영상·거리센서raw는 MAVLink에 없으므로). `/mavros/*`·`/imu`·`/navsat`·`/odometry`·`/battery`·`/ap/*` 발행 금지.
- ⚠️ **`/clock` 발행 금지** — Mac MAVROS/시스템 시간과 충돌.
- `publish_mavros_state`는 **off 유지**(param은 옵션으로만 존재).

## 5. FCU 토픽은 MAVROS가 자동 통일
- MAVLink→MAVROS는 **펌웨어 무관**: PX4든 APM이든 Mac MAVROS가 동일한 `/mavros/*`를 냄. → bridge가 PX4 상태를 `/ap/*`나 `/mavros/*`로 **흉내 낼 필요 없음**(중복·충돌만 유발).
- ⚠️ MAVROS가 **못** 통일하는 것 = **모드 문자열**: PX4 `OFFBOARD`/`AUTO.MISSION`/`AUTO.RTL` vs APM `GUIDED`/`AUTO`/`RTL`. → 체화지능 self_destruct/mission_forward에 **PX4 모드 어댑터** 필요(status §15.7, 체화지능 repo 측 작업).
- px4_mavros 기본에서 bridge의 `/mavros/*`는 **빈 advertise**(B 확정: `_publish_local_pose`가 `airsim_direct`에만 publish) → 데이터 충돌은 없으나 그래프 중복 → **embodied 모드에선 끄기 권장**.

---

## 6. 카메라 전송 (기구현 + 한계)
- raw 720p = 2.76MB/frame → zenoh/Tailscale 단편 드롭(실증). → **JPEG CompressedImage**(`/camera/image/compressed`, ~150–190KB, 1/15) 구현.
- 실측 **~7fps**(720p). 병목 = AirSim `simGetImages` RPC(~106–130ms). JPEG 인코딩 3ms·DroneController 폴링은 병목 아님. **720p>9fps는 RPC로 불가** → 필요시 해상도↓ 또는 PixelStreaming.
- Mac: `image_transport republish compressed raw`로 detector에 raw 공급(detector 무변경).

---

## 7. 제어 one-writer + zenoh 관측 scoping (필수 안전장치)
- ⚠️ B 발견: bridge의 `/cmd_pos` + `/mavros/setpoint_position/local` 콜백은 **backend 게이트 없이 AirSim `moveToPosition` 직접 구동**(px4_mavros에서도). (`/cmd_vel`·setpoint_velocity는 드롭)
- 목표 구조 = Mac MAVROS→MAVLink→PX4가 **유일 컨트롤러**. 만약 zenoh가 Mac의 `/mavros/setpoint_position/local`을 우리 측으로 포워드하면 → bridge가 또 moveToPosition → **이중 제어(one-writer 위반)**.
- **조치 = zenoh 관측 전용 scoping**: allow `camera/*`·`range/front`만, deny `setpoint`·`cmd`·`mavros`·`rt/fmu/in`. → 제어 이중구동 + uXRCE-DDS 누출 동시 차단.

---

## 8. 구현 상태 / 남은 작업
- [x] 채널1 zenoh 연결(1.2.1↔1.8.0 e2e PASS)
- [x] 토픽 bare 통일(camera/range/lidar/mavros), /drone1 0
- [x] 카메라 JPEG 압축(~7fps, 1.3MB/s)
- [ ] **zenoh 관측 scoping**(allow camera/range, deny control/mavros/rt-fmu) ← 다음, 안전 필수
- [ ] bridge 빈 `/mavros/*` 발행 끄기(embodied 모드 옵션) — 그래프 청소
- [ ] 채널2: `run_mavproxy_to_embodied.sh` 실행(local MAVROS off 선행) + Mac MAVROS `fcu_url=udp://:14555@<AIRSIM>:14540`
- [ ] PX4 모드 어댑터(체화지능 repo: OFFBOARD/AUTO.MISSION 분기)
- [ ] 카메라 색감/republish Mac 검증

---

## 9. 산출물 / 실행
- 단일 진입점: `scripts/test_embodied_link.sh` (preflight/px4/zenoh/pub/bridge/mavproxy/verify/status/stop/pipeline). bare 기본.
- `scripts/run_mavproxy_to_embodied.sh` (채널2).
- `settings/{zenoh_bridge_dds_airsim_client.json5, cyclone_dds_airsim_client.xml}`.
- zenoh 1.2.1: `~/workspace/tools/zenoh-bridge-dds-v1.2.1/`.
- 실행: UE Play → `./scripts/test_embodied_link.sh pipeline` → Mac에서 bare typed echo.

---

## 10. 미해결 / 실측 필요
- `/mavros/compass_hdg` 타입(bridge Float64 vs 체화지능 UInt16) — 단 이건 Mac MAVROS가 내므로 무관(MAVROS 표준 타입 사용).
- HOME/rel_alt/vfr_hud 필드 — Mac MAVROS+PX4 실측.
- 다중 드론 시 `ROS_DOMAIN_ID` 분리(drone0=10, drone1=11…) — 별도 설계.

---

## 부록. 채널별 토픽 귀속 (전체)
| 토픽 | 채널 | 소스 |
|---|---|---|
| `/camera/image`, `/camera/image/compressed`, `/camera/camera_info` | 1 (zenoh) | bridge (RPC) |
| `/range/front` (+left/right 잉여) | 1 (zenoh) | bridge (RPC) |
| `/lidar/points` (옵션, AI 미사용) | 1 (zenoh) | bridge (RPC) |
| `/mavros/state,pose,global,vfr_hud,home,waypoints,setpoint_*,서비스` | 2 (MAVLink) | Mac MAVROS |
| `/vision/*`, `/perception/state`, `/replan_waypoints`, `/a2a/*` | (Mac local) | AI 노드 |
| `/ap/*`, bare `/imu`·`/navsat`·`/odometry`·`/battery`·`/clock`·`/magnetometer`·`/air_pressure` | — | 불필요(미사용) |

## 부록 B. 전체 토픽 4-버킷 분류 (전수 스윕, 2026-06-18 병렬검토 확정)
원본 체화지능(Gazebo+ArduPilot) 토픽 전부를 분류 → **bridge가 래핑할 것 = BRIDGE-WRAP 뿐.**

| 버킷 | 토픽 | 처리 |
|---|---|---|
| **BRIDGE-WRAP** (우리가 래핑·발행) | `/camera/image`, `/camera/camera_info`, `/range/front` (+옵션 `/range/{left,right}`, `/lidar/points`) | AirSim RPC→bare 토픽→zenoh. MAVLink에 없는 sim 센서 + §15.2 AI 소비. **✅ 구현 완료** |
| **MAC-MAVROS** (Mac MAVROS가 MAVLink로 생성) | `/mavros/*` 전체(state·local_position·global_position·vfr_hud·home·waypoints·setpoint·서비스), `/imu`·`/navsat`·`/odometry`·`/air_pressure`·`/magnetometer`·`/battery`(→ `/mavros/*`로 도달), `/tf`(FCU), `/uas1/mavlink_*` | mavproxy→Mac MAVROS. **우리 미발행** |
| **AI-OUTPUT** (체화지능 노드 자체 생성) | `/vision/detect_information`·`/vision/detect_frame`, `/perception/state`, `/a2a/drone1/*`, `/replan_waypoints` | 소비자측, 무관 |
| **INFRA/IGNORE** (도구/sim내부) | `/clock`, `/gz/tf`·`/gz/tf_static`, `/robot_description`, `/joint_states`, `/ap/*` 전부, `/clicked_point`·`/goal_pose`·`/initialpose`·`/move_base_simple/goal`, `/parameter_events`·`/rosout`·`/diagnostics` | 양측 통합 무관. **미발행** |

**확정 wrap 셋(이것만):** `/camera/image`, `/camera/camera_info`, `/range/front` [+옵션 range L/R, lidar]. 그 외 전부 제외.
- `/ap/*`·`/gz/*`·`/tf` = AI 미소비(viz/FCU/ArduPilot 전용) → **래핑 안 함**.
- TF는 어떤 AI 노드도 구독 안 함(월드투영은 `/mavros/global_position`+`compass_hdg`+`local_position`로). `aerion_tf`는 rviz 옵션.

---
**문서 버전:** 2026-06-18 v1.1 · 근거: 병렬분석 A/B(2회) + 전수 토픽 스윕 + 체화지능 토픽목록(2026-04-09) + status §15.
