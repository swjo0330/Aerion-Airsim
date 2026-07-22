# AERION 시뮬레이션 통신 확장 설계 (2026-06-17 · v2 · END-TO-END 검증 완료)

> **한 줄 요약:** 관측(카메라이미지·camera_info·range·lidar)은 **zenoh가 토픽 그대로 포워드**,
> 제어(MAVLink)는 **mavproxy가 UDP 포워드**. 그 이상의 relay/namespace 변환 장치는 불필요.
> 권위 상태 문서: `docs/status_2026-06-17_single_drone_zenoh_memory.md` §15.

---

## 0. 검증 상태 (2026-06-17)

**① 연결 e2e PASS** — `AirSim(zenoh 1.2.1) → Zenoh/Tailscale → Mac Router(1.8.0) → Mac DDS → ros2 echo` 완전 관통. `/airsim_test` Mac 수신 확인.

**② 실센서 토픽 흐름 ✅(우리쪽 확인)** — UE Play + PX4 SITL(4560 ESTAB, lockstep) + bridge_node(drone1) →
zenoh가 `rt/drone1/{camera/image, camera/camera_info, lidar/points, range/front|left|right, mavros/*}` 전부 DDS→Zenoh 라우트 생성·전송. (Camera 1280×720/FOV89.9, Range 20Hz, LiDAR 10Hz). Mac echo 수신 확인 단계.

**진행 순서(확정):** 센서 흐름 → **③ mavproxy(MAVLink) 테스트** → **④ 토픽 이름 갭 정합**(체화지능 기존 토픽 ↔ `/drone1/*`) → ⑤ 관측 scoping.

---

## 1. 두 side
| | 시뮬 side (AirSim PC) | 체화지능 side (Mac) |
|---|---|---|
| OS/IP | Ubuntu 22.04 / Tailscale `100.120.219.68` (NIC enp108s0) | macOS / `100.67.87.116` (NIC **en4**) |
| zenoh | **Client** (connect), `1.2.1` | **Router** (listen 0.0.0.0:7447), `1.8.0` |
| 역할 | UE/CARLA+AirSim+PX4+bridge_node+mavproxy | MAVROS + vision_detector + reasoning/self_destruct |

---

## 2. 통신 구조 (2-track)
```text
[관측]  UE/CARLA+AirSim → bridge_node(/drone1/camera|range|lidar)
          → CycloneDDS(localhost) → zenoh CLIENT ──Tailscale TCP 7447──► Mac zenoh ROUTER → Mac DDS → 체화지능 노드
[제어]  PX4 SITL ── mavproxy (udpin:14540 → udpout:Mac:14555) ──Tailscale UDP──► Mac MAVROS(/mavros/* 네이티브)
```
- 관측 = Zenoh 단방향(읽기). 제어 = MAVLink만. **제어는 절대 Zenoh 경유 금지.**

---

## 3. 버전 — OS 차이로 다름, interop 검증됨
- Ubuntu 22.04(glibc 2.35) prebuilt 상한 = **1.2.1** (1.4.0+는 glibc 2.38 요구). Mac = **1.8.0**.
- **버전 일치 불필요** — zenoh 1.x wire 호환, e2e로 실증됨. 설치: `~/workspace/tools/zenoh-bridge-dds-v1.2.1/`.

---

## 4. 토픽 — 발행 이름 그대로 포워드
- bridge_node가 내는 `/drone1/camera/image`, `/drone1/camera/camera_info`, `/drone1/range/*`, `/drone1/lidar/points`가 zenoh로 **그대로** Mac에 도착.
- 체화지능이 비네임스페이스 `/camera/image`를 기대하면 → **필요 시 Mac 쪽 `topic_tools relay` 한 줄**로 변환(양쪽 노드 코드 무수정). AirSim 쪽엔 relay/param 안 만듦.
- 단일 드론 기준. 다중 드론은 차후 `ROS_DOMAIN_ID` 분리(설계 §15.8).
- **검증은 `ros2 topic list` 아님 → typed echo.** zenoh-bridge-dds는 **로컬 DDS Reader가 생겨야 Zenoh→DDS 라우트를 lazy 생성**하므로, list엔 안 보이고 `ros2 topic echo <topic> <type>`로 Reader를 띄워야 흐른다.

---

## 5. ⚠️ 관측 scoping (§11.4 — 토픽 정합 단계와 함께)
현재 zenoh client는 **scope 없이 domain 0의 모든 DDS 토픽을 포워드**한다. 실제로 Mac Router 로그에
PX4 uXRCE-DDS `rt/fmu/out/*`(vehicle_local_position, attitude, sensor_combined, battery 등)가 관측됨.
- 위험: `rt/fmu/in/*`(제어)까지 새어나가면 one-writer 원칙 위반.
- 조치: zenoh client에 **allow regex(관측 토픽만)** 적용 — camera/range/lidar/camera_info 허용, `*/setpoint*`·`*/cmd/*`·`rt/fmu/in/*` 차단.
- PX4 uXRCE-DDS 채널은 우리 MAVLink `/mavros/*` 경로와 **별개** → 중복/충돌 정리 필요.

---

## 6. 제어 — mavproxy MAVLink (Mac-local MAVROS)
- `scripts/run_mavproxy_to_embodied.sh`: PX4 `udpin:14540` → `udpout:Mac:14555`. **local MAVROS off 선행**(14540 충돌).
- Mac: MAVROS `fcu_url=udp://:14555@<AIRSIM>:14540` → `/mavros/*` Mac 네이티브 생성.
- **one-writer**: local manual publisher와 remote 제어 동시 금지. TCP 4560(lockstep)은 MAVLink master 아님.

---

## 7. 카메라 (딥리서치 2026-06-18 — [[airsim-camera-transport-design]])
- **전송 메커니즘 = 풀 방식 msgpack-RPC**(`simGetImages`). 스트리밍 아님.
- **RPC fps 천장 ~9-12fps**(720p 실측 9.4fps/106ms). 병목 = per-call(GPU readback+msgpack+IOLoop) → **해상도 낮춰도 30fps 불가**(픽셀 수 무관).
- **압축 `compress=True`=PNG**(CPU↑, fps 도움 X). JPEG도 캡처 fps 무관(네트워크 전용).
- **720p 30fps 실시간 = Unreal PixelStreaming(H.264/WebRTC)만**(RPC 우회, ROS-native 아님).
- **인지주기≠제어주기**: PX4 FC 50-1000Hz 분리. 10-15fps 인지+분리제어가 표준, fps보다 **e2e latency**가 폐루프 관건.
- **옵션**: A) PixelStreaming(720p 30-60fps, 구현 큼) / **B) 720p ~9fps RPC + JPEG + Mac decompress republish(권장 시작)** / C) 저해상도(비권장).
- ⚠️ **raw 720p는 zenoh에서 단편 드롭 실증**(9MB/s→Mac 0프레임) → **전송엔 JPEG 압축이 사실상 필수**(raw는 fps 무관 실패).
- 확정 기준: detector 요구 fps + 허용 latency.

---

## 8. 산출물
| 파일 | 역할 |
|---|---|
| **`scripts/test_embodied_link.sh`** | **단일 테스트 오케스트레이터** (preflight/zenoh/pub/bridge/mavproxy/verify/status/stop/connect). 실행은 이걸로. |
| `settings/zenoh_bridge_dds_airsim_client.json5` | zenoh client config (connect Mac:7447) |
| `settings/cyclone_dds_airsim_client.xml` | localhost DDS discovery (NIC enp108s0, 1.x `<Interfaces>`) |
| `scripts/run_zenoh_airsim_client.sh` | zenoh client 단독 실행(컴포넌트) |
| `scripts/run_mavproxy_to_embodied.sh` | MAVLink 포워더 |
| `scripts/verify_zenoh_airsim_client.sh` | 단독 검증(컴포넌트) |
| (보존) 0.5 peer 파일 | 미사용, 보존 |
| zenoh 1.2.1 바이너리 | `~/workspace/tools/zenoh-bridge-dds-v1.2.1/` |

---

## 9. 실행 (오케스트레이터 셸 하나로)
```bash
./scripts/test_embodied_link.sh preflight   # Mac 도달/버전/환경
./scripts/test_embodied_link.sh zenoh       # client 기동 (Mac Router 떠 있어야)
./scripts/test_embodied_link.sh pub         # /airsim_test 발행 → Mac에서 typed echo
./scripts/test_embodied_link.sh bridge      # bridge_node (camera/range/lidar; sim 필요)
./scripts/test_embodied_link.sh mavproxy    # MAVLink → Mac MAVROS
./scripts/test_embodied_link.sh status|stop
```
> 단계는 phase 진행에 따라 이 셸을 업데이트한다(예: scoping, typed-echo verify).

---

## 10. 검증에서 확정한 것
- 1.2.1↔1.8.0 interop OK (버전 일치 불필요).
- typed echo 필수(lazy route). `ros2 topic list` 신뢰 금지.
- Mac NIC=en4, Router/echo 동일 `cyclone_dds_local.xml`+DOMAIN 0 사용(별도 en0 config 불필요).

---

## 11. 남은 일 (확정 진행 순서)
1. ✅ **연결 e2e + 실센서 토픽 흐름(우리쪽)** — 완료. Mac echo 수신 확인만 남음.
2. **mavproxy(MAVLink) 테스트** ← *다음* — local MAVROS off → `run_mavproxy_to_embodied.sh` forward → Mac MAVROS `/mavros/*` 네이티브 생성 확인 (제어 명령 전송 아님, 상태 토픽 수신까지).
3. **토픽 이름 갭 정합** (mavproxy 이후 단계) — 체화지능 기존 토픽 계약과 우리 `/drone1/*` 차이를 **실측·문서화 후 맞춤**. 체화지능 기대(status §15.2): 비네임스페이스 `/camera/image`·`/camera/camera_info`·`/range/front`·`/mavros/*` 등. 정합 = **Mac 쪽 topic_tools relay / launch remap**(AirSim 코드 무수정 원칙). compass_hdg 타입(Float64 vs UInt16) 등 타입 갭도 이때 점검.
4. **관측 scoping**(§5) — allow regex로 camera/range/lidar/camera_info만 허용, 제어·`rt/fmu/in/*` 차단.
5. PX4 uXRCE-DDS(`rt/fmu/*`)와 MAVLink `/mavros/*` 중복 정리.

---

## 부록. 포트 / IP
| 용도 | 값 |
|---|---|
| Mac zenoh Router | `tcp/100.67.87.116:7447` (NIC en4) |
| AirSim PC | `100.120.219.68` (NIC enp108s0) |
| AirSim RPC | `127.0.0.1:41451` · PX4 SITL TCP `4560`(master 아님) |
| MAVLink → Mac MAVROS | PX4 `14540` → Mac `14555` |
| ROS_DOMAIN_ID | `0` (양단 동일) |

---
**v2.1 변경:** 실센서 토픽 흐름(camera/range/lidar/mavros) 우리쪽 확인 반영 · 진행 순서 확정(센서→mavproxy→토픽갭 정합→scoping) · 토픽 이름 갭 정합을 mavproxy 이후 단계로 명시 · 런타임 환경 사실(PX4 경로 등은 CLAUDE.md 환경 정보 참조).
**v2 변경:** end-to-end 검증 반영 · 2-track 단순화 · 버전 일치 불필요 확정 · typed-echo/lazy-route · Mac en4 정정 · uXRCE-DDS scoping 항목 추가 · 단일 오케스트레이터 셸 도입.
