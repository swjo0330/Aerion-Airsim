# Gazebo APM → AirSim PX4 토픽 매핑 설계 (2026-06-09)

> 현재 Gazebo+ArduPilot(APM)에서 우리 노드들이 쓰는 토픽 전수 인벤토리 + AirSim PX4 연동 시 매핑/변환 설계.
> 기준: `TOPICS.md`(repo 루트) + 노드 코드 실측 + 병렬 리서치/설계/검토 3종.
> 연동 토폴로지: [[2026-06-09-airsim-px4-zenoh-integration]] 참조 (단일=비네임스페이스/1:1브릿지, 다중=ROS_DOMAIN_ID).

## 핵심 원칙
- **비네임스페이스 유지**(/camera/image, /mavros/*) → vision_detector/노드 **코드 변경 0**.
- 우리 토픽을 **PX4 mavros + bridge_node**가 동일 토픽명·타입·QoS로 채워주면 됨.
- 무거운 건 로컬, 충돌은 ROS_DOMAIN_ID로.

---

## 1. 토픽 인벤토리 (현 Gazebo APM) — 우리 노드 활용분

| 토픽 | 타입 | pub | sub | QoS | 비고 |
|------|------|-----|-----|-----|------|
| `/camera/image` | sensor_msgs/Image | ros_gz_bridge | vision_detector(L3695) | BE/1 | raw bgr8 |
| `/camera/camera_info` | sensor_msgs/CameraInfo | ros_gz_bridge | vision_detector(L3702) | BE/10 | K행렬/FOV |
| `/vision/detect_information` | ec_edge_msgs/DetectInfoArray | vision_detector(L3533) | reasoning, perception_memory, self_destruct | RELIABLE/10 | 핵심 감지 결과 |
| `/vision/detect_frame` | sensor_msgs/Image | vision_detector(L2939) | (모니터) | BE | 오버레이 |
| `/perception/state` | ec_edge_msgs/PerceptionState | perception_memory(L91) | uas_link_adapter | BE/10 | 통합 인지 |
| `/replan_waypoints` | ec_edge_msgs/ReplanWaypoints | planner/uas_link(L117) | self_destruct(L1256) | - | NED 회피경로 |
| `/agent/evasion_complete` | std_msgs/Bool | self_destruct(L1120) | reasoning | - | 회피완료 |
| `/mavros/state` | mavros_msgs/State | MAVROS | reasoning,uas_link | RELIABLE/TL | mode/armed |
| `/mavros/local_position/pose` | geometry_msgs/PoseStamped | MAVROS | self_destruct,planner(L73),uas_link | BE | **ENU** |
| `/mavros/global_position/global` | sensor_msgs/NavSatFix | MAVROS | vision_detector,perception,self_destruct | BE | GPS(WGS84) |
| `/mavros/global_position/rel_alt` | std_msgs/Float64 | MAVROS | vision_detector(L3063) | BE | HOME 상대고도 |
| `/mavros/global_position/compass_hdg` | std_msgs/UInt16 | MAVROS | vision_detector | BE | heading(deg) |
| `/mavros/vfr_hud` | mavros_msgs/VfrHud | MAVROS | vision_detector,perception,self_destruct | BE/10 | groundspeed/alt/hdg |
| `/mavros/home_position/home` | mavros_msgs/HomePosition | MAVROS | vision_detector,self_destruct | RELIABLE/TL | HOME 기준점 |
| `/mavros/mission/waypoints` | mavros_msgs/WaypointList | MAVROS | self_destruct(L1264) | - | 미션 WP |
| `/mavros/setpoint_velocity/cmd_vel` | geometry_msgs/TwistStamped | self_destruct(L1224) | MAVROS | - | 속도 제어 |
| `/mavros/setpoint_position/local` | geometry_msgs/PoseStamped | self_destruct | MAVROS | - | 위치 제어 |
| `/mavros/gimbal_control/device/attitude_status` | GimbalDeviceAttitudeStatus | MAVROS | vision_detector(L2968) | BE | 짐벌 자세 |
| **서비스** `/mavros/set_mode`, `/mavros/cmd/arming`, `/mavros/cmd/takeoff`, `/mavros/cmd/command`(짐벌205), `/mavros/mission/push` | | | self_destruct/mission_forward | - | 모드/ARM/이륙/짐벌/미션 |
| `/ap/*` (airspeed,navsat,pose/filtered…) | micro_ros DDS | micro_ros_agent | (직접구독 코드 없음) | - | **ArduPilot 전용** |
| `/range/front` | sensor_msgs/Range | ros_gz_bridge | vision_detector,self_destruct | BE | 전방거리(옵션) |

DetectInfo 핵심 필드(매핑 의존): drone_lat/lon, drone_heading_deg, drone_position_ned[3], target_local_north/east, hfov/vfov, distance_m, threat_level, semantic_tag.

---

## 2. PX4/AirSim → 우리 토픽 매핑

| 우리 토픽 | PX4/AirSim 소스 | 매핑 주체 | 변환 | 난이도 |
|-----------|-----------------|-----------|------|:---:|
| `/camera/image`,`/camera/camera_info` | AirSim 카메라 API(TCP:41451) | **bridge_node** | raw→(옵션 압축), K/FOV 채움 | ★★★ (zenoh 대역폭) |
| `/mavros/state` | PX4 HEARTBEAT | MAVROS | 직접(mode 문자열만 PX4) | ★ |
| `/mavros/local_position/pose` | PX4 LOCAL_POSITION_NED | MAVROS | NED→ENU(MAVROS 자동) | ★ |
| `/mavros/global_position/global` | PX4 GLOBAL_POSITION_INT | MAVROS | 자동 | ★ |
| `/mavros/global_position/rel_alt` | PX4 | MAVROS | 자동 (단, HOME 초기화 의존) | ★★ |
| `/mavros/global_position/compass_hdg` | PX4 | MAVROS | 자동 | ★ |
| `/mavros/vfr_hud` | PX4 VFR_HUD | MAVROS | 직접 (throttle 필드 채움 확인) | ★★ |
| `/mavros/home_position/home` | PX4 HOME | MAVROS | **PX4 자동설정 시점 차이** | ★★ |
| `/mavros/mission/waypoints` | PX4 미션 | MAVROS | HOME seq=0 처리 차이 | ★★ |
| `/mavros/setpoint_*` | self_destruct→MAVROS | self_destruct | ENU→NED(자동), **OFFBOARD 필요** | ★★ |
| 짐벌 status | PX4 MOUNT/gimbal | MAVROS | PX4 짐벌 지원 확인 | ★★ |
| `/ap/*` | **PX4 미발행** | — | → `/mavros/*`로 대체 (현 코드 /ap/ 미구독이라 다행) | ★ |
| DetectInfo의 drone_lat/lon/heading/local | `/mavros/global_position/global`,`compass_hdg`,`local_position/pose` | vision_detector | 그대로(비네임스페이스) | ★ |

### 책임 분리
- **bridge_node (AirSim 측, ROS2 노드)**: AirSim API→ `/camera/image`,`/camera/camera_info` 발행 (+옵션 `/cmd_vel`,`/cmd_pos` 수동). DDS로 발행만.
- **MAVROS (Mac 측)**: PX4와 MAVLink UDP 직접 → `/mavros/*` 전부 제공. self_destruct가 setpoint/서비스 사용.

---

## 3. PX4 vs ArduPilot 갭 (검토 결과 — 매핑 위험)

| 갭 | 내용 | 영향 노드 | 대응 |
|----|------|----------|------|
| **모드 문자열** | GUIDED→OFFBOARD, AUTO→AUTO.MISSION, RTL→AUTO.RTL | self_destruct(mode_auto APM하드코딩), mission_forward(L193) | firmware 어댑터(PX4Handler 존재) |
| **OFFBOARD setpoint 타이밍** | 모드 전환 前 ≥2Hz 스트림 필수, 끊기면 failsafe | self_destruct | 발행 순서 조정(전환 전 스트림) |
| **HOME 초기화** | PX4는 ARM 시 자동(초기 0 가능), APM은 WP seq=0 필수 | vision/self_destruct 고도·NED 변환 | bridge_node에서 DO_SET_HOME(179) 보장 |
| **좌표/고도 기준** | local_position frame_id, rel_alt 기준 | vision_detector 거리/고도 추정 | frame_id 실측 확인 |
| **vfr_hud throttle 등 필드** | PX4 mavros가 일부 필드 비울 수 있음 | vision/self_destruct | 실측 확인 |
| **카메라 intrinsics** | AirSim K/FOV가 거리추정 가정과 일치해야 | vision_detector | CameraInfo 채움 + fallback(a8_mini 81°) |
| **/ap/* 전용** | PX4 미발행 | (현 코드 미구독) | 무영향 |

---

## 4. 검증 우선순위 (PoC)
1. **★카메라 /camera/image (raw/압축) zenoh·Tailscale 전송** — FPS/지연/손실 실측 (최대 리스크).
2. **PX4 HOME 초기화 + rel_alt/local_position frame** — vision 고도/NED 정상인지.
3. **OFFBOARD setpoint over Tailscale** — ≥2Hz 유지, failsafe 없는지.
4. **vfr_hud/compass_hdg/global_position 필드 채움** — DetectInfo drone pose 정상인지.

---

**상태**: 매핑 설계 완료(PoC 전). detector/노드 무변경 전제(비네임스페이스). bridge_node(AirSim) 미구현 → 작성 필요.
