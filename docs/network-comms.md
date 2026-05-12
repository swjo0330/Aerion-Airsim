# 네트워크 통신 설계

## 머신 구성

| 머신 | 역할 | Tailscale IP |
|------|------|-------------|
| 시뮬레이션 머신 | UE5 + Colosseum + PX4 SITL + bridge_node | 100.120.219.68 |
| 원격 제어 머신 | MAVROS + embodied-drone 노드 | 100.67.87.116 |

## Phase 1 (현재): rmw_zenoh_cpp + Tailscale

### 구조

DDS를 rmw_zenoh_cpp로 완전 교체. Tailscale(WireGuard)이 공인망 터널 담당.

```
시뮬 머신 (100.120.219.68)               원격 머신 (100.67.87.116)
┌─────────────────────┐                 ┌─────────────────────┐
│ bridge_node.py      │                 │ embodied-drone      │
│ rmw_zenoh_cpp       │                 │ rmw_zenoh_cpp       │
│ [Zenoh Router]      │◄──Tailscale────►│ [Zenoh Client]      │
│ listen :7447        │   WireGuard     │ connect →           │
│                     │   암호화 P2P    │ 100.120.219.68:7447 │
└─────────────────────┘                 └─────────────────────┘
```

### 설정 파일

settings/zenoh_sim.json5 (시뮬 머신):
```json5
{
  mode: "router",
  listen: { endpoints: ["tcp/100.120.219.68:7447"] },
  connect: { endpoints: ["tcp/100.67.87.116:7447"] },
  scouting: { multicast: { enabled: false }, gossip: { enabled: true } }
}
```

### 실행

시뮬 머신:
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=settings/zenoh_sim.json5
bash scripts/run_bridge.sh
```

원격 머신:
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# zenoh_remote.json5: mode=client, connect→100.120.219.68:7447
ros2 run mavros apm_node ...
```

### Tailscale 직접 연결 확인

```bash
tailscale status
# "direct" 표시 시 P2P 직접 연결 (최적)
# "relay" 표시 시 DERP 서버 경유 (지연 증가)
```

### 대역폭 예상

| 카메라 FPS | 2드론 Raw RGB 대역폭 |
|-----------|-------------------|
| 30fps | ~166 MB/s |
| 15fps | ~83 MB/s |
| 10fps | ~55 MB/s |

Tailscale WireGuard 오버헤드: 약 3~5% 추가

---

## Phase 2 (차후): zenoh-bridge-ros2dds + 순수 공인망

Tailscale 없이 공인 IP로 직접 통신. DDS RMW는 그대로 유지하고 Zenoh가 WAN 구간 브릿지.

### 구조

```
시뮬 머신                                      원격 머신
┌─────────────────────────┐               ┌─────────────────────────┐
│ bridge_node (rmw_dds)   │               │ embodied-drone (rmw_dds)│
│      ↕ DDS 로컬          │               │      ↕ DDS 로컬          │
│ zenoh-bridge-ros2dds    │               │ zenoh-bridge-ros2dds    │
│ [router] listen :7447   │◄──TCP 공인망──►│ [client] connect→       │
│                         │               │ <SIM_PUBLIC_IP>:7447    │
└─────────────────────────┘               └─────────────────────────┘
```

### rmw_zenoh_cpp 대비 차이점

| 항목 | rmw_zenoh_cpp | zenoh-bridge-ros2dds |
|------|--------------|---------------------|
| RMW 교체 | 필요 | 불필요 |
| 추가 프로세스 | 없음 | 양쪽 bridge 실행 필요 |
| 호환성 | Zenoh 전용 | 기존 DDS 환경과 호환 |
| 암호화 | 기본 없음(TLS 설정 가능) | 기본 없음(TLS 설정 가능) |

### 사전 조건

1. 시뮬 머신 공인 IP에서 7447 포트 오픈
```bash
sudo ufw allow 7447/tcp
```

2. 공유기/방화벽에서 7447 포트포워딩 설정

3. (권장) TLS 인증서 설정
```json5
tls: {
  server_certificate: "certs/server.pem",
  server_private_key: "certs/server_key.pem"
}
```

### settings/zenoh_remote.json5 (원격 머신용, 미생성)

```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/<SIM_PUBLIC_IP>:7447"],
  },
  scouting: {
    multicast: { enabled: false },
  },
}
```

### 실행

```bash
# 시뮬 머신
zenoh-bridge-ros2dds -c settings/zenoh_sim.json5 &
bash scripts/run_bridge.sh

# 원격 머신
zenoh-bridge-ros2dds -c settings/zenoh_remote.json5 &
ros2 run mavros apm_node ...
```

---

## 방화벽 설정 요약

```bash
# 양쪽 머신: Zenoh
sudo ufw allow 7447/tcp

# 원격 머신: MAVLink 수신
sudo ufw allow 14580/udp   # Drone0
sudo ufw allow 14581/udp   # Drone1
```

---

## 전송되는 ROS2 토픽

| 방향 | 토픽 | 타입 |
|------|------|------|
| 시뮬→원격 | /uav0/camera/image | sensor_msgs/Image |
| 시뮬→원격 | /uav0/camera/camera_info | sensor_msgs/CameraInfo |
| 시뮬→원격 | /uav1/camera/image | sensor_msgs/Image |
| 시뮬→원격 | /uav1/camera/camera_info | sensor_msgs/CameraInfo |
| 원격→시뮬 | /uav0/cmd_vel | geometry_msgs/Twist |
| 원격→시뮬 | /uav0/cmd_pos | geometry_msgs/PoseStamped |
| 원격→시뮬 | /uav1/cmd_vel | geometry_msgs/Twist |
| 원격→시뮬 | /uav1/cmd_pos | geometry_msgs/PoseStamped |
