# AirSim PX4 SITL + Zenoh Bridge 연동 설계 (2026-06-09)

> 현 Gazebo+ArduPilot(APM) SITL 대신 **원격 AirSim PX4 SITL**을 쓰되, 그쪽에도 zenoh bridge를 두어
> Tailscale 너머로 ROS2 토픽을 공유 — 지금 Gazebo와 토픽 공유하듯 동작시키는 것이 목표.
> 병렬 분석 2종(네트워크/Zenoh, PX4↔ArduPilot 스택) + 사용자 정정 반영.

## 결론: **조건부 가능**
- **네트워크/Zenoh**: 가능 (Zenoh는 순수 TCP, 현재 이미 `multicast=false` 유니캐스트라 Tailscale 오버레이와 호환).
- **제어 로직**: PX4 호환은 firmware 어댑터로 구현 수정하면 됨 (self_destruct에 `PX4Handler`/`ArduHandler` 분기가 이미 존재).
- **최대 리스크**: **카메라 이미지를 네트워크(zenoh/Tailscale)로 전송** — vision_detector를 시뮬 머신에 둘 수 없어 이미지가 반드시 네트워크를 타야 함. 과거 `rmw_zenoh_cpp`를 폐기시킨 사유가 정확히 "대용량 이미지 전송 실패"(2026-04-08).

---

## 핵심 결정 사항
1. **RMW**: 초기 도식의 `rmw_zenoh_cpp`(Zenoh를 RMW로 직접)는 **사용 금지**. AERION 표준인 **`rmw_cyclonedds_cpp` + `zenoh-bridge-dds`(v1.8.0, 양측 동일)** 사용. (rmw_zenoh는 대용량 이미지 실패로 2026-04-08 폐기됨.)
2. **vision_detector 위치**: **원격 제어 머신(Mac 측)**. 시뮬 머신에 둘 수 없음 → `/camera/image`(비네임스페이스)가 zenoh로 전송됨.
3. **카메라 전송**: zenoh-bridge-dds 경유. **BEST_EFFORT KL(1) QoS + 압축 이미지(image_transport/compressed) + 해상도/FPS 하향** 필수 (대역폭/지연 완화). ※현재 Gazebo 하이브리드에서 카메라는 zenoh를 안 거치고 직접 LAN DDS로 가므로, **zenoh 카메라 전송은 미검증 영역 — PoC 필수**.
4. **MAVROS**: 원격 제어 머신에서 MAVLink UDP로 PX4 연결 (MAVLink 경량, 원격 OK).
5. **다중 UAV**: 드론별 **ROS_DOMAIN_ID** 분리(비네임스페이스 유지, /uavN/ 리네임 아님) + bridge_node·zenoh-bridge 1:1/PX4 — 군집 검증 계획과 정합.

---

## 전체 통합 도식 (텍스트)

> **핵심**: embodied-drone(ROS2 로컬 노드)과 aerion-drone(Docker 에이전트)는 **전부 로컬 Mac**에 뜬다.
> AirSim 컴퓨터(원격)와는 **zenoh 브릿지로 토픽만 공유** + MAVROS는 MAVLink UDP로 PX4 직접 연결.
> = 현 Gazebo 하이브리드에서 **SITL PC만 AirSim 컴퓨터로 교체**, Mac 스택은 그대로.

가로형(좌=AirSim 컴퓨터 / 우=로컬 Mac), 토픽은 비네임스페이스(/camera/image). **MAVROS는 Mac에만**(AirSim 측 MAVLink 소스=PX4 SITL, bridge_node는 RPC 전용).

```
┌──────────────────────────────────────────┐                              ┌────────────────────────────────────────────────┐
│ AirSim 컴퓨터 (원격, Tailscale 100.x.x.A) │                              │ 내 로컬 Mac — embodied-drone + aerion-drone 전부 │
│                                          │                              │                                                │
│ UE 5.6.1 + Colosseum  [uav0][uav1]       │                              │ zenoh-bridge-dds (Router, listen 0.0.0.0:7447) │
│   │ RPC 41451 (vehicle_name, 공유)        │     Zenoh TCP:7447           │   ◄ AirSim Client    ◄ Docker Client           │
│   ▼                                      │   ════════════════════►      │ CycloneDDS (로컬)                               │
│ bridge_node xN (커스텀,1:1/PX4,cyclonedds)│   (Tailscale, ★카메라 포함)   │ ┌── embodied-drone (ROS2 로컬) ───────────────┐ │
│   pub /camera/image, /camera/camera_info │                              │ │ mavros (udp-in :14555) ↔ PX4 (양방향)       │ │
│   (비네임스페이스=가제보 동일, RPC만)     │                              │ │ vision_detector ← /camera/image (zenoh ★)   │ │
│                                          │                              │ │   → /vision/detect_information               │ │
│ PX4 SITL xN (udp-in) uav0:4560 uav1:4561  │   MAVLink UDP (양방향)       │ │ self_destruct (PX4Handler/OFFBOARD)         │ │
│   ▲▼ 로컬 UDP (mavproxy ↔ PX4 udp-in)      │   ◄══════════════════►       │ │   ↑ /mavros/* 서비스·setpoint 로컬(Mac)     │ │
│ mavproxy --out udpout:<Mac_TS>:14555       │  mavproxy ⇄ mavros(udp-in)   │ │ perception_memory, uas_link_a2a_adapter     │ │
│ CycloneDDS(mcast off)+zenoh-bridge(Client)│                              │ ┌── aerion-drone (Docker, aerion-net) ────────┐ │
│   (MAVROS 없음)                           │                              │ │ reasoning/planner/supervisor/qdrant/ollama   │ │
│                                          │                              │ │ zenoh-bridge-dds(Client→host.docker:7447)   │ │
│                                          │                              │ └─────────────────────────────────────────────┘ │
└──────────────────────────────────────────┘                              └────────────────────────────────────────────────┘
        └──────────────── Tailscale VPN (WireGuard, unicast) ────────────────┘

데이터 흐름:
  [感] AirSim 카메라 → /camera/image ═(zenoh/Tailscale ★)═► Mac vision_detector
        → /vision/detect_information ─(Mac CycloneDDS→zenoh)─► reasoning(Docker)
  [斷] reasoning → planner → /replan_waypoints ─► self_destruct(Mac)
        → mavros(Mac) OFFBOARD setpoint ═MAVLink UDP═► mavproxy(AirSim) ► PX4 SITL
  [復] self_destruct → /agent/evasion_complete → reasoning
  [監] /mavros/state, /perception/state → supervisor/reasoning
  ※ MAVLink는 zenoh 미경유: PX4 SITL ↔ mavproxy(AirSim) ↔(UDP/Tailscale)↔ mavros(Mac). AirSim측 mavros 없음.
  ※ self_destruct↔mavros 서비스(set_mode/arming)는 Mac 로컬 (zenoh 서비스 브릿지 미검증 회피)
  ※ 다중 드론: 토픽명 동일(비네임스페이스), 드론별 ROS_DOMAIN_ID 분리(/uavN/ 리네임 아님).
              RPC 41451 공유(vehicle_name), MAVLink/PX4 포트는 차량별
```

---

## 현재 Gazebo 구조 → AirSim 구조 매핑

| 현재 (Gazebo+APM) | AirSim+PX4 버전 |
|---|---|
| SITL PC (Gazebo+ArduPilot), 같은 LAN | **시뮬 머신** (UE+Colosseum+PX4 SITL), 원격 Tailscale |
| 카메라: SITL PC → Mac **직접 LAN CycloneDDS** (zenoh 미경유) | 카메라: 시뮬 머신 → Mac **zenoh/Tailscale** (★신규 부하) |
| Mac: mavros/self_destruct/vision/perception + zenoh Router | **동일 유지** (vision_detector도 Mac) |
| Docker: 에이전트 + zenoh Client | **동일 유지** |
| ArduPilot GUIDED/AUTO | PX4 OFFBOARD/AUTO.MISSION (firmware 어댑터) |
| 단일 기체 | 다중: **드론별 ROS_DOMAIN_ID** 분리(비네임스페이스 유지, /uavN/ 리네임 아님). bridge_node·zenoh-bridge 1:1/PX4 |

---

## PX4 ↔ ArduPilot 제어 호환 (firmware 어댑터로 해결)
self_destruct에 이미 `PX4Handler`/`ArduHandler` 분기 + GUIDED↔OFFBOARD 조건분기 존재. 수정 필요 지점:
1. `mode_auto()` APM 하드코딩("AUTO"/param2=3.0) → PX4 "AUTO.MISSION" 분기 (self_destruct ~L4794, mission_forward ~L193).
2. **OFFBOARD는 모드 전환 前 ≥2Hz setpoint 스트림 필수** → 발행 순서 조정.
3. setpoint 타입 통일 (ArduHandler=PoseStamped vs PX4Handler=PositionTarget).
4. DO_CHANGE_SPEED param1, HOME WP 등 미션 명령 firmware 분기.
- 호환 OK(수정 불필요): vision_detector/DetectInfo(NED, firmware-agnostic), planner /replan_waypoints, ENU↔NED 변환, 짐벌(CommandLong 205).
- ⚠️ self_destruct는 상용 검증 노드 → control-path 변경 시 재검증/승인 필요.

---

## 리스크 & 완화
| 리스크 | 심각도 | 완화 |
|---|:---:|---|
| **카메라 이미지 zenoh/Tailscale 전송** (rmw_zenoh 폐기 사유) | ★★★ | 압축(compressed) + BEST_EFFORT KL(1) + 해상도/FPS 하향 + **PoC로 실측 검증** |
| Tailscale 지연(+10~50ms)/MTU 단편화 | ★★ | 경량 토픽 우선, batch_size 모니터링, MTU 튜닝 |
| zenoh-bridge-dds **v1.8.0 양측 일치** | ★★ | 명시적 v1.8.0 설치, apt 자동업그레이드 방지 |
| AirSim 머신 RMW=rmw_cyclonedds_cpp 강제 | ★★ | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| PX4 제어 호환 | ★★ | firmware 어댑터 (위) |

---

## 구현 로드맵
1. **네트워크 PoC**: 시뮬 머신 zenoh Client(v1.8.0) → Mac Router(Tailscale:7447) TCP 연결 + `/mavros`·경량 토픽 공유 검증 (`nc -zv <MAC_TS_IP> 7447`).
2. **★카메라 PoC**: `/camera/image`(압축+BE) zenoh/Tailscale 전송 → Mac vision_detector 수신 FPS/지연/손실 실측. (성패가 전체 타당성의 관건; 목표 예: ≥5fps, 지연<500ms, 손실<5%)
3. **firmware 어댑터**: mode/ setpoint/ 미션 PX4 분기.
4. **멀티 UAV**: `/uavN/` 네임스페이싱 + PX4 다중 인스턴스.
5. **통합 T1-airsim** 테스트.

---

## 아키텍처 다이어그램 (PNG)

![AirSim PX4 Zenoh 연동 아키텍처](2026-06-09-airsim-px4-zenoh-integration.png)

---

---

## 설계 확정 (2026-06-09 추가 검토 — 위 도식의 /uavN/ 표기 대체)

병렬 검토(설정/네임스페이싱/충돌/detector) 결과 **사용자 정정 방향이 옳음**. 확정안:

### 1. 카메라 토픽 = `/camera/image` (가제보와 동일, **비네임스페이스**)
- vision_detector는 `/camera/image`(raw, BEST_EFFORT KL1) + `/camera/camera_info` 구독 (vision_detector_node.py:3695-3706).
- bridge_node가 **동일 토픽명·타입·QoS로 발행하면 detector 코드 변경 0**. 위 도식의 `/uavN/camera/image`는 **`/camera/image`로 대체**.
- CameraInfo는 fallback(a8_mini FOV 81°) 있어, AirSim은 width/height/K만 채우면 됨.

### 2. 압축 = 선택 (필요 시 **Mac republish 노드**로, detector 무변경)
- detector는 raw만 지원(cv_bridge imgmsg_to_cv2). 압축 필요 시 3안:
  (a) detector를 image_transport로 변경(~5-10줄) (b) **Mac에 decompress republish 노드 추가 → detector 무변경 [권장]** (c) 압축 안 함(raw).
- 우선 raw로 PoC → 대역폭 부족하면 (b).

### 3. 브릿지 = **PX4 SITL 1대당 zenoh-bridge 1개 (1:1 다중 브릿지)**
- Mac Router 1개에 드론별 Client N개 + Docker Client 연결 (TCP 다중 수락 + zenoh 세션 멀티플렉싱). 한 브릿지로 여러 SITL 멀티플렉싱 ✗.

### 4. 드론 분리 = **IP(머신) 분리 + 단일/다중 구분**
- **단일 드론(현 목표)**: 비네임스페이스(/camera/image, /mavros/*) + 브릿지 1개 → **즉시 가능, 충돌 없음, detector 무변경**.
- **다중 드론(군집, 향후)**: ⚠️ IP 분리만으론 **부족** — Zenoh는 토픽 키로 라우팅(IP 무관)하므로 같은 `/camera/image`가 Mac에서 **충돌**. 해법:
  - **권장: `ROS_DOMAIN_ID` 도메인 분리** (drone0=10, drone1=11...). 각 드론 머신 + 그에 대응하는 Mac측 노드세트(vision_detector_N 등)를 같은 도메인에 둠. ROS2 표준, detector 코드 무변경(인스턴스만 분리).
  - 대안: zenoh-bridge scope/key-prefix (현 설정에 없음, v1.8.0 지원 미확인 → PoC 필요).
  - 비권장: per-drone 전체 스택 복제(과도).

### 단일 드론 확정 도식 (즉시 적용)
```
[AirSim 컴퓨터] PX4 SITL + bridge_node → /camera/image, /camera/camera_info, /mavros/*(비네임스페이스)
   + CycloneDDS(로컬,mcast off) + zenoh-bridge-dds(Client) ──zenoh/Tailscale──┐
[Mac] zenoh-bridge-dds(Router 7447) ◄── AirSim Client ◄── Docker Client       ┘
   embodied-drone(mavros→MAVLink UDP→PX4, vision_detector←/camera/image, self_destruct PX4Handler, perception)
   + Docker aerion-drone(reasoning/planner/supervisor)
```

---

---

## 양단 설정 파일 정의 (CycloneDDS XML + Zenoh json5)

> 양단 모두 **로컬 노드(CycloneDDS) ↔ zenoh-bridge-dds** 구조. 두 zenoh-bridge가 Tailscale TCP:7447로 통신.
> `CYCLONEDDS_URI`(DDS 측 xml)와 `-c json5`(zenoh 측)를 **둘 다** 줘서 켠다. xml은 같은 호스트 내 노드↔브릿지 DDS discovery용, json5는 브릿지 간 zenoh 연결용.

```
[AirSim 컴퓨터]                                    [Mac]
 bridge_node (CycloneDDS) ──DDS(localhost)──┐       ┌──DDS(localhost)── mavros/vision/self_destruct/perception
                                            ▼       ▼
 zenoh-bridge-dds(Client) ═══ Zenoh TCP:7447 (Tailscale) ═══ zenoh-bridge-dds(Router) ──┐
   CYCLONEDDS_URI=cyclone_dds_airsim.xml                CYCLONEDDS_URI=cyclone_dds_local.xml │
   -c zenoh_bridge_dds_airsim_client.json5             -c zenoh_bridge_dds_local.json5       ▼
                                                                              Docker zenoh Client(host.docker.internal:7447)
```

### A) AirSim 컴퓨터 — 신규 2개

**`cyclone_dds_airsim.xml`** (bridge_node + AirSim zenoh-bridge 공유, 같은 호스트 discovery용)
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface name="eth0"/></Interfaces>   <!-- AirSim 호스트 NIC (Win이면 "Ethernet"/"Wi-Fi") -->
      <AllowMulticast>false</AllowMulticast>                     <!-- Tailscale 미지원 + 로컬 unicast로 충분 -->
      <MaxMessageSize>1400B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer address="localhost"/>     <!-- bridge_node ↔ zenoh-bridge (같은 호스트) -->
        <Peer address="127.0.0.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

**`zenoh_bridge_dds_airsim_client.json5`** (Docker Client 미러링, Mac Router로 connect)
```json5
{
  mode: "client",
  connect: { endpoints: ["tcp/<MAC_TAILSCALE_IP>:7447"] },   // 예: tcp/100.x.x.M:7447
  scouting: { multicast: { enabled: false } },
  transport: { shared_memory: { enabled: false },
    link: { tx: { batch_size: 65535 }, rx: { buffer_size: 16777216 } } },   // v1.8.0
}
```

**기동 (AirSim 컴퓨터)**
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///path/cyclone_dds_airsim.xml"
# 1) zenoh-bridge (Client) — Mac Router 먼저 떠있어야 함
zenoh-bridge-dds -c /path/zenoh_bridge_dds_airsim_client.json5 &
# 2) PX4 SITL + bridge_node (1:1), 같은 CYCLONEDDS_URI 상속
ros2 run <pkg> bridge_node   # /camera/image, /camera/camera_info, ... 발행
# (다중 드론: 드론별 ROS_DOMAIN_ID=10/11..., bridge_node+zenoh-bridge 쌍 1:1)
```

### B) Mac — 기존 파일 그대로 (변경 없음)
- `cyclone_dds_local.xml` (en4, AllowMulticast=spdp, Peers) — **AirSim를 DDS peer로 추가 불필요** (AirSim 토픽은 zenoh로 들어옴).
- `zenoh_bridge_dds_local.json5` (Router, listen `tcp/0.0.0.0:7447`) — 0.0.0.0이라 Tailscale IP로 들어오는 AirSim Client 자동 수락.
- 전제: Mac에 Tailscale 활성 + `lsof -iTCP:7447`가 Mac Router(zenoh-bri) 점유 (com.docker.backend면 안 됨, full.yml 금지).

### MAVLink 경로 — **mavros는 Mac, AirSim측은 mavproxy로 UDP 포워드** (확정)
**왜 mavros를 AirSim측에 두지 않나**: 리서치 결과 `zenoh-bridge-dds`의 **ROS2 서비스(request/reply) 브릿지는 미검증·위험**(공식 "to some extent", 1.x 서비스 회귀 이력). self_destruct(Mac)가 호출하는 mavros **서비스(set_mode/arming/takeoff/command)**가 zenoh를 건너면 깨질 수 있음. → **mavros를 Mac에 유지**해 서비스를 **로컬**로 처리(=현재 Gazebo와 동일).

**확정 구조** (현재 Gazebo의 mavproxy 패턴 그대로):
- AirSim 컴퓨터: PX4 SITL → **mavproxy**(또는 mavlink-router)가 PX4 MAVLink를 **Mac Tailscale IP로 UDP 포워드**.
- Mac: `mavros`가 받아(`udp://:14555@` 등) `/mavros/*` 발행 + `/mavros/setpoint_*`/서비스로 제어. self_destruct↔mavros **모두 Mac 로컬** → zenoh-서비스 문제 없음.
- MAVLink(경량)만 Tailscale UDP, 카메라+에이전트 토픽은 zenoh.
#### PX4 SITL MAVLink 포트 (리서치 확정, 2026-06-10)
> ※ AirSim/Colosseum은 표준 PX4 SITL과 포트 체계가 다름 — `settings.json`의 `QgcHostIp`/`QgcPort`(기본 127.0.0.1:14550), `ControlPortLocal`(14540)/`ControlPortRemote`(14580)로 제어. **AirSim/PX4가 능동 송신(unicast/broadcast)하므로 받는 쪽(mavproxy/mavros)은 반드시 listen(udp-in)**.

| 용도 | 포트 | 방향(PX4/AirSim 관점) | 비고 |
|------|------|----------------------|------|
| GCS (QGC) | 14550 | PX4/AirSim **송신** → 수신측 listen | `QgcPort` |
| Offboard API (MAVSDK/MAVROS) | 14540 | PX4 **송신** → API listen | `ControlPortLocal`; mavproxy master 권장 |
| Offboard remote | 14580 | — | `ControlPortRemote` |
| 시뮬 lockstep | **4560 (TCP)** | AirSim↔PX4 HIL 전용 | **MAVLink GCS 무관, master로 쓰지 말 것** |

- 멀티비클: 인스턴스별 `TcpPort` 4560→4561…, `ControlPortLocal` 14540→14541… 로 분리(AirSim 공식). sysid(`VehicleSysID`)도 드론별로 다르게 줄 것.
- 멀티머신: `settings.json`의 `LocalHostIp`를 실제 NIC IP로(127.0.0.1 그대로면 외부로 안 나감).

```bash
# ── AirSim 컴퓨터 (SITL, 현재 10.130.200.31 / 원격 시 Tailscale) ──
# mavproxy: PX4 offboard 채널(14540) listen → Mac mavros(udp-in)로 양방향 포워드
#   현재 LAN 테스트: MAC=10.130.200.32  /  원격 전환: MAC=100.67.87.116(Tailscale)
mavproxy.py \
  --master=udpin:0.0.0.0:14540 \
  --out=udpout:10.130.200.32:14555 \
  --daemon --state-basedir=/tmp/mav0
#  ※ AirSim 설정상 GCS가 14550(QgcPort)으로만 나오면 --master=udpin:0.0.0.0:14550 으로.
#    실제 도착 포트는 `tcpdump -i any udp port 14540 or udp port 14550` 로 1회 검증(버전 차이 주의).

# 다중 드론(1:1): 드론당 PX4 인스턴스 + mavproxy 1개, 포트·state-basedir 분리
# uav0: --master=udpin:0.0.0.0:14540 --out=udpout:10.130.200.32:14555 --state-basedir=/tmp/mav0
# uav1: --master=udpin:0.0.0.0:14541 --out=udpout:10.130.200.32:14556 --state-basedir=/tmp/mav1

# ── Mac (mavros, udp-in으로 listen) ──
ros2 run mavros mavros_node --ros-args -p fcu_url:="udp://:14555@10.130.200.31:14540"
#   udp://:14555@  = Mac이 14555 bind/listen(자동 lock 시 @뒤 생략 가능)
#   다중: uav1 → udp://:14556@ , mavros_node 별 ROS_DOMAIN_ID(10/11) 분리

# ── mavproxy 없는 대안: PX4가 직접 Mac으로 unicast ──
#   AirSim settings.json: "QgcHostIp":"10.130.200.32", "QgcPort":14555, "LocalHostIp":"10.130.200.31"
#   또는 PX4 init: mavlink start -x -u 14550 -t 10.130.200.32 -r 4000000
#   (단일 목적지엔 OK, QGC+mavros 동시/유연 매핑은 mavproxy 우위)
```
> **주의**: ① lockstep TCP 4560은 절대 master로 쓰지 말 것(HIL 전용). ② broadcast→unicast이므로 수신측 UDP 인바운드 방화벽 허용. ③ TCP 포워드는 지연 → UDP 사용. ④ 드론별 sysid 충돌 금지.
> bridge_node는 MAVLink 미관여(AirSim RPC 카메라 전용). AirSim 측엔 **mavros 없음**(mavproxy 포워더만). 서비스 zenoh 미경유.
> ※ mavros-on-AirSim을 굳이 쓰려면 `zenoh-bridge-ros2dds`(서비스 공식지원) 교체 + 서비스 왕복 실측 필요 — 비권장.

---

## bridge_node 1:1 설계 (★우리 설계 선택 + 기억 포인트)

### bridge_node의 역할 = AirSim 소스 토픽의 매핑 재발행
- bridge_node는 **AirSim RPC(41451)에서 카메라/포즈를 읽어 [[2026-06-09-airsim-px4-topic-mapping]] 명세대로 ROS2 토픽으로 재발행**한다. 그 매핑 표가 곧 bridge_node의 발행 스펙(토픽명·타입·QoS).
- 발행 대상(=AirSim이 소스인 것만): `/camera/image`(raw, BEST_EFFORT KL1), `/camera/camera_info`, (필요 시 pose/odom). **비네임스페이스**(가제보와 동일) → vision_detector 코드 변경 0.
- **`/mavros/*`는 bridge_node가 아니라 MAVROS(Mac)가 mavproxy 경유로 제공** — bridge_node 소관 아님. (제어/MAVLink 경로와 카메라/RPC 경로는 분리.)

### 왜 bridge_node ↔ PX4 SITL = 1:1 인가 (우리 설계 선택)
1. **처리량(throughput)**: 한 bridge_node가 N대 SITL의 카메라를 모두 처리하면 **단일 RPC(41451) 폴링이 직렬 병목** + 이미지 디코드/발행 부하 집중. 드론당 1개로 쪼개면 병렬 분산.
2. **기본 ROS2 PX4/MAVROS 토픽이 단일 드론 기준**: MAVROS는 FCU당 1인스턴스, PX4 네이티브 토픽(`/fmu/*` 등)도 단일/도메인 기준. 비네임스페이스 표준 토픽명을 그대로 쓰려면 **드론당 독립 노드세트 + ROS_DOMAIN_ID 분리**가 자연스럽다.
3. **AERION 군집 정합**: 토픽명 동일(비네임스페이스) + 드론별 `ROS_DOMAIN_ID`(drone0=10, drone1=11…) → /uavN/ 리네임 회피, detector 등 노드는 인스턴스만 분리.

> ※ **AirSim 하드 제약은 아님**: RPC 41451은 vehicle_name 라우팅으로 공유되어 한 노드가 N대를 다룰 수도 있다(공식 airsim_ros_pkgs는 1노드+네임스페이싱). 그러나 위 ①②③ 때문에 **AERION은 1:1을 채택**.

### 1:1 묶음 단위 (드론당 동반 기동)
`PX4 SITL  +  mavproxy(1개)  +  bridge_node(1개)  +  zenoh-bridge-dds Client(1개)`  →  Mac Router 1개에 Client N개 다중 접속 + Mac측 노드세트(vision_detector_N 등)는 같은 ROS_DOMAIN_ID.

### 기억 포인트 (recall)
- bridge_node = **카메라/RPC 전용**, MAVLink 미관여. `/mavros/*`는 Mac MAVROS.
- 1:1 = **우리 설계 선택**(throughput + 단일드론 기준 토픽), AirSim 강제 아님.
- 다중 드론 = **IP 분리만으론 부족** → Zenoh는 토픽키 라우팅이라 같은 `/camera/image` 충돌 → `ROS_DOMAIN_ID` 분리 필수.
- 한 zenoh-bridge로 여러 SITL 멀티플렉싱 ✗ → **브릿지도 1:1**.

---

**상태**: 설계 확정(단일 드론) + 양단 설정 정의, PoC 전. bridge_node(AirSim) 미구현. **관련**: [[2026-06-09-airsim-px4-topic-mapping]], `experiments/T1-airsim/airsim-zenoh.png`, [[project_swarm_verification_plan]], rmw_zenoh 폐기(2026-04-08).
