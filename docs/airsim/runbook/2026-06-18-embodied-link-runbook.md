# AERION ↔ 체화지능(Mac) 실행 런북 (단일 드론, 2채널)

> 작성 2026-06-18 · 검증 완료(양 채널 e2e). 설계 권위 문서: [`2026-06-18-embodied-topic-integration-design.md`](../design/2026-06-18-embodied-topic-integration-design.md).
> **단일 진입점:** `scripts/test_embodied_link.sh <명령>` — 모든 실행은 이 셸로. 직접 `pkill`은 self-match로 셸이 죽으니 `stop` 명령 사용.

## 0. 구성 한눈에

```
[AirSim PC]                                              [Mac (체화지능)]
 UE5.6 + AirSim ──lockstep TCP 4560── PX4 SITL
                                          │ MAVLink (onboard, -o 14540)
 bridge_node ─ camera/range RPC          ▼
   │ bare DDS                        mavproxy  udpin:14540
   ▼                                   │ udpout → 100.67.87.116:14555
 zenoh-bridge-dds 1.2.1 ─TCP 7447─────────────────► zenoh Router 1.8.0 ─► detector
   (관측: image/range)              Tailscale          │ (관측 토픽)
                                                       └► MAVROS (bind 14555) ─► /mavros/*
```

| 항목 | AirSim PC | Mac |
|---|---|---|
| Tailscale IP | `100.120.219.68` | `100.67.87.116` |
| zenoh | bridge-dds **client** 1.2.1 | Router 1.8.0 (`:7447`) |
| ROS_DOMAIN_ID | 0 | 0 |
| RMW | rmw_cyclonedds_cpp | (Mac 설정) |

**포트:** PX4 lockstep 4560 · AirSim RPC 41451 · mavproxy in 14540 · Mac MAVROS bind 14555 · zenoh 7447.

## 1. 채널 / 토픽 계약 (갭 0)

| 채널 | 토픽 | 타입 | 발행 주체 | 실측 |
|---|---|---|---|---|
| 관측(zenoh) | `/camera/image/compressed` | CompressedImage(JPEG) | bridge | ~5–7Hz |
| 관측(zenoh) | `/camera/camera_info` | CameraInfo | bridge | ~30Hz |
| 관측(zenoh) | `/range/front/points` | **PointCloud2**(3D, 전방 1점) | bridge | ~3–9Hz |
| 제어/비행데이터 | `/mavros/*` | (표준) | **Mac MAVROS** | 50Hz |

- bridge는 **camera+range만** 발행. `/mavros/*`·`/ap/*`·`/clock`은 발행 금지(Mac MAVROS가 MAVLink로 생성).
- **one-writer:** 제어는 MAVLink 경로(mavproxy→MAVROS)로만. zenoh로 제어 토픽 금지. AirSim PC의 local MAVROS는 끈다(14540 충돌).

## 2. 사전 준비 (수동, 1회)

1. **Mac:** zenoh Router(`:7447`) + 디텍터 노드 가동, ROS_DOMAIN_ID=0.
2. **AirSim PC:** UE5.6에서 BlocksV2 등 **Play** → drone1 spawn (AirSim RPC 41451 LISTEN, lockstep 4560 LISTEN 확인).
3. Tailscale 연결 확인: `tailscale status | grep 100.67.87.116`.

## 3. 실행 (AirSim PC)

### 3-A. 관측 채널 (한 번에)
```bash
cd ~/workspace/projects/aerion-airsim
./scripts/test_embodied_link.sh pipeline      # preflight→px4→zenoh→bridge→verify
```
`pipeline`은 UE Play(41451)·PX4·zenoh·bridge를 순서대로 띄우고 로컬 토픽까지 확인한다. 단계별로 하려면:
```bash
./scripts/test_embodied_link.sh preflight     # Tailscale/Mac도달/버전 점검
./scripts/test_embodied_link.sh px4           # PX4 SITL (UE Play 선행)
./scripts/test_embodied_link.sh zenoh         # zenoh client → Mac Router
./scripts/test_embodied_link.sh bridge        # camera/range 발행
```

### 3-B. 제어/비행데이터 채널
```bash
./scripts/test_embodied_link.sh mavproxy      # PX4 MAVLink → Mac:14555
```
출력에 **Mac에서 실행할 MAVROS 명령**이 자동으로 찍힌다(우리 Tailscale IP 자동 탐지). `Detected vehicle 1:1` 보이면 링크 OK.

## 4. Mac 측 실행

```bash
# zenoh Router는 이미 가동 중이라고 가정. MAVROS만 기동:
ROS_DOMAIN_ID=0 ros2 run mavros mavros_node --ros-args \
  -p fcu_url:="udp://:14555@100.120.219.68:14540" \
  -p tgt_system:=1 -p tgt_component:=1
```
- `udp://:14555@...:14540` = 로컬 14555 bind(수신) + 명령은 우리 14540(mavproxy udpin)으로 송신 → PX4 브릿지. **이 원격 엔드포인트가 핵심**(reply-to-source 대안 `udp://:14555@`).
- Mac은 `apm.launch`로도 PX4 모드문자열(`AUTO.LOITER` 등)을 정확히 디코드 → px4.launch 불필요. ⚠ 단 OFFBOARD setpoint·arming 등 *제어 플러그인*은 원격제어 단계에서 재검증.

## 5. 검증 (Mac, typed echo — `ros2 topic list`는 zenoh 너머로 불신)

```bash
ROS_DOMAIN_ID=0 ros2 topic hz   /camera/image/compressed                                  # ~5–7Hz
ROS_DOMAIN_ID=0 ros2 topic echo --once /camera/camera_info  sensor_msgs/msg/CameraInfo
ROS_DOMAIN_ID=0 ros2 topic echo --once /range/front/points  sensor_msgs/msg/PointCloud2   # width=1
ROS_DOMAIN_ID=0 ros2 topic echo --once /mavros/state                                       # connected: true
ROS_DOMAIN_ID=0 ros2 topic hz   /mavros/local_position/pose
```
AirSim PC 측 상태/링크 확인: `./scripts/test_embodied_link.sh status` · `verify`.

## 6. 종료 / 자원 해제

```bash
./scripts/test_embodied_link.sh stop          # zenoh/pub/bridge/mavproxy/px4 graceful→SIGKILL,
                                               # tlog 정리, 포트(14540/7447/4560)·세션 해제 검증
STOP_PX4=false ./scripts/test_embodied_link.sh stop   # PX4/UE 보존(링크만 내림)
KEEP_TLOG=true ./scripts/test_embodied_link.sh stop   # mavproxy tlog 보존
```
- stop은 종료 후 `✓/⚠`로 포트·세션·프로세스가 실제 풀렸는지 보고한다(프로세스 종료 비동기성 고려 ~3s 재시도).
- UE는 GUI라 수동 종료.

### 재시작 (PX4/UE 유지, 링크만)
```bash
STOP_PX4=false ./scripts/test_embodied_link.sh stop
./scripts/test_embodied_link.sh zenoh && ./scripts/test_embodied_link.sh bridge && ./scripts/test_embodied_link.sh mavproxy
```
mavproxy 재기동 시 송신 ephemeral 포트만 바뀌고 수신(14555)·명령(14540) 엔드포인트는 고정 → **Mac MAVROS는 재시작 없이 자동 재연결**.

## 7. 주요 환경변수 (오버라이드)

| 변수 | 기본 | 의미 |
|---|---|---|
| `CAMERA_FPS` | `5.0` | ★ DOUBLE 필수(정수면 `.0` 자동 보정) |
| `RANGE_MODE` | `points` | `/range/front/points` PointCloud2(3D). `range`/`laserscan`/`both` 가능 |
| `IMAGE_COMPRESSED` | `true` | JPEG 발행(raw 2.76MB는 단편 드롭) |
| `JPEG_QUALITY` | `70` | |
| `TOPIC_NAMESPACE` | `bare` | `/camera/*`·`/range/*`. `__vehicle__`=`/drone1/*` 레거시 |
| `PUBLISH_MAVROS_STATE` | `false` | bridge는 `/mavros/*` 발행 안 함(Mac MAVROS 담당) |
| `ENABLE_LIDAR` | `false` | 카메라와 RPC 경합 → 검증 중 off |
| `MAC_TAILSCALE_IP` | `100.67.87.116` | Mac |
| `AIRSIM_TAILSCALE_IP` | 자동/`100.120.219.68` | mavproxy 원격 엔드포인트 |
| `MAVPROXY_IN_PORT` / `MAC_MAVROS_PORT` | `14540` / `14555` | |
| `STOP_PX4` / `KEEP_TLOG` | `true` / `false` | stop 보존 옵션 |

## 8. 트러블슈팅 (실제 겪은 함정)

| 증상 | 원인 | 해결 |
|---|---|---|
| 셸이 갑자기 종료(exit 144) | 직접 `pkill -f 'mavproxy.py'`가 자기 셸 매칭 | `stop` 명령 사용, 부득이하면 `[m]avproxy` 브래킷 |
| Mac 토픽 0개(세션은 ESTAB) | bridge 다회 재시작 후 zenoh 세션/라우트 stale | zenoh client 재기동(`stop`→`zenoh`), 필요시 Mac Router도 |
| `camera_fps InvalidParameterType` | DOUBLE 기대에 INTEGER 전달 | `CAMERA_FPS`에 `.0`(셸이 자동 보정) |
| `set -u`에서 ROS source 실패 | `AMENT_TRACE_SETUP_FILES` unbound | `source_ros()`가 `set +u`로 감쌈(이미 적용) |
| camera 720p ≥9fps 안 나옴 | AirSim simGetImages RPC 천장 ~9.4fps(해상도 무관) | JPEG 압축으로 전송 해결, 30fps는 PixelStreaming 필요 |
| `/range/front` 타입 불일치 | detector는 3D PointCloud2 구독 | `RANGE_MODE=points`(`/range/front/points`) |
| `4560 없음(UE off)` 오탐 | status가 LISTEN만 체크, 실제론 ESTAB | mavproxy `Detected vehicle`/`ss -tn :4560`로 교차확인 |
| mavproxy heartbeat 없음 | PX4 미가동/UE Play 안 함, 또는 14540 local MAVROS 점유 | `px4` 먼저, local MAVROS 끄기 |

## 9. 자율 비행 제어 (drone1, PX4 offboard) — 2026-06-18 체크포인트

상세/근거: 메모리 `px4-airsim-flight-control`. 스크립트 `scripts/fly_drone1_city_demo.py`(시퀀스), `control_drone1.py`(정밀), `_diag_vel_vs_pos.py`(진단).

- **제어 경로**: AirSim Python API는 PX4에서 "valid GPS home" 거부 → 사용 불가. **MAVLink offboard**(pymavlink)로 force-arm(`COMPONENT_ARM_DISARM` p2=21196), OFFBOARD(`DO_SET_MODE` main=6), `SET_POSITION_TARGET_LOCAL_NED` 20Hz 스트리밍.
- **연결**: mavproxy `EXTRA_OUT=udpout:127.0.0.1:14601`(오케스트레이터 `MAVPROXY_EXTRA_OUT` 기본) ↔ pymavlink `udpin:127.0.0.1:14601`. one-writer(Mac=read-only).
- **★ 필수 픽스 `EKF2_MAG_CHECK=0`**: AirSim 시뮬 자력계가 PX4 mag sanity check 실패 → mag_ratio=2.0 거부 → heading 미수렴. 끄면 mag_ratio~0.01, heading 정상. `prepare_px4_manual_control.sh`에 포함. (런타임 적용: `param set EKF2_MAG_CHECK 0` + `PREFLIGHT_STORAGE` 저장 + reboot.)
- **상태**: ✅ arm/이륙/고도유지/AUTO.LAND/disarm 안정. ✅ velocity 수평 이동 즉시·정밀(3m/s). ⚠️ **position-setpoint-from-hover 수평 항법은 초기 ~2-3분 지연(사실상 미작동)** — 도형 비행 미완성. **다음 작업 = velocity 기반 waypoint 항법으로 재작성**(VEL_MASK `0b0000110111000111`).
- **운영 팁**: 비행 전 PX4 fresh reboot + mag_ratio<1 확인 권장(EKF가 누적으로 재diverge·LOCAL z offset 발생). 스트림 끊기면 offboard failsafe로 자동착륙(안전).
