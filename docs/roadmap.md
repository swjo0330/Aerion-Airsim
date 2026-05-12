# Phase 5~7 통합 계획

**작성일:** 2026-04-08
**전제:** PX4 2대 + ROS2 브릿지 검증 완료

---

## Phase 5: 도시 환경 구축

### 접근법: BlocksV2 프로젝트 확장 (가장 단순)

기존 동작 중인 BlocksV2에 새 맵(Level)을 추가하여 도시 환경 구성. AirSim 통합이 이미 완료되어 있으므로 추가 설정 최소화.

### 작업 순서

1. **BlocksV2 프로젝트 복사**
   ```bash
   cp -r Colosseum/Unreal/Environments/BlocksV2 Colosseum/Unreal/Environments/CityEnv
   # CityEnv.uproject 파일 이름 변경 + 내용 수정
   ```

2. **UE 에디터에서 새 맵 생성**
   - File → New Level → Default
   - 건물, 도로, 나무 등 배치

3. **에셋 소스 (Linux에서 사용 가능한 것들)**
   - **Quixel Megascans**: UE5 에디터 내 Quixel Bridge로 건물/도로/콘크리트 무료 다운로드
   - **FBX 임포트**: 외부 3D 모델(TurboSquid, CGTrader)에서 도시 건물 다운로드 → UE 임포트
   - **Modeling Tools Plugin**: 이미 활성화됨, 간단한 건물/벽 직접 모델링
   - **UE Starter Content**: 기본 에셋 활용

4. **필수 설정**
   - World Settings → GameMode Override → `ColosseumGameMode`
   - PlayerStart 액터 배치 (드론 스폰 위치)
   - Edit → Editor Preferences → "Use Less CPU when in Background" 해제

5. **GPS 좌표 설정** (settings.json)
   ```json
   "OriginGeopoint": {
     "Latitude": 37.5665,
     "Longitude": 126.9780,
     "Altitude": 38
   }
   ```

6. **장애물 배치**
   - Static Mesh 드래그 + Transform 도구
   - 전선줄: Spline Mesh / Cable Component
   - 충돌 메쉬 자동 생성 (UE 기본)

---

## Phase 6: Zenoh DDS + MAVROS 원격 연동

### 6.1 아키텍처

```
시뮬레이션 머신 (SIM_IP)              원격 머신 (MAVROS_IP)
┌──────────────────────┐           ┌──────────────────────┐
│ PX4 SITL 0 ──UDP:14555────────→ │ MAVROS Drone0        │
│ PX4 SITL 1 ──UDP:14556────────→ │ MAVROS Drone1        │
│                      │           │                      │
│ airsim_ros2_bridge   │           │ embodied-drone 노드  │
│   ↓ DDS              │           │   ↑ DDS              │
│ zenoh-bridge ──TCP:7447────────→ │ zenoh-bridge         │
│ (router)             │           │ (client)             │
└──────────────────────┘           └──────────────────────┘
```

### 6.2 MAVLink 원격 전송 (settings.json)

**핵심 발견: `ControlIp` 키가 필요** (`LocalHostIp`만으로는 안 됨)

```json
"Drone0": {
  "ControlIp": "<MAVROS_IP>",
  "ControlPortRemote": 14555,
  "ControlPortLocal": 14540,
  "LocalHostIp": "<SIM_IP>"
}
```

generate_settings.py에 `--mavros-ip`와 `--local-ip` 옵션 추가 필요.

### 6.3 MAVROS 설정 (원격 머신)

```python
# fcu_url 형식
'fcu_url': 'udp://:14555@SIM_IP:14540'
#                ^^^^^ 수신포트  ^^^^^^^^ ^^^^ 송신대상
```

2대 MAVROS launch:
- Drone0: `fcu_url=udp://:14555@SIM_IP:14540`, `target_system_id=1`
- Drone1: `fcu_url=udp://:14556@SIM_IP:14541`, `target_system_id=2`

### 6.4 ROS2 토픽 통신: rmw_zenoh (DDS 직접 교체)

양쪽 머신에서 이미 rmw_zenoh를 사용 중이므로, **zenoh-bridge 없이** ROS2 토픽이 자동 교환됨.

```bash
# 양쪽 머신에서 환경변수 설정 (이미 사용 중)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

zenoh router 엔드포인트만 양쪽이 연결되도록 설정하면 됨.

### 6.5 카메라 이미지

Raw RGB (sensor_msgs/Image) 그대로 전송. JPEG 압축 없음 — 비전 처리에 손실 압축은 불필요.
대역폭 조절은 `camera_fps` 파라미터로 FPS 제한 (10~30fps).

| FPS | 2대 Raw 대역폭 | 비고 |
|-----|---------------|------|
| 30 | ~166 MB/s | 1Gbps LAN에서 주의 |
| 15 | ~83 MB/s | 1Gbps 여유 |
| 10 | ~55 MB/s | 안전 |

### 6.6 방화벽

```bash
# MAVROS 측: MAVLink 수신
sudo ufw allow 14580/udp   # PX4 SITL Drone0 MAVLink
sudo ufw allow 14581/udp   # PX4 SITL Drone1 MAVLink

# Zenoh: rmw_zenoh 라우터 포트 (양쪽)
sudo ufw allow 7447/tcp
```

### 6.7 전체 기동 순서

```
[시뮬레이션 머신]
1. cleanup.sh
2. generate_settings.py -f px4 -n 2 --mavros-ip <MAVROS_IP> --local-ip <SIM_IP> --deploy
3. UE 에디터 실행 + Play
4. PX4 SITL 2대 기동
5. airsim_ros2_bridge 실행
6. zenoh-bridge-ros2dds -c settings/zenoh_sim.json5

[원격 MAVROS 머신]
7. zenoh-bridge-ros2dds -c zenoh_mavros.json5
8. MAVROS 2대 launch
9. embodied-drone 노드 실행
```

---

## Phase 7: ArduPilot 통합 (별도)

### 모터 인덱스 패치 (이미 적용)

ArduCopterParams.hpp에 `std::swap(params.rotor_poses[2], params.rotor_poses[3])` 추가 완료.

### 남은 작업

1. Colosseum 리빌드 (`build.sh`)
2. UE Binaries 삭제 → 재컴파일
3. APM 2대 테스트 (모터 패치 + 비블로킹 recvRotorControl 패치)
4. MAVLink 수신 확인 + 제어 테스트

---

## 우선순위

1. **Phase 6.2~6.5** (MAVROS + Zenoh) — generate_settings.py 수정 + 카메라 압축
2. **Phase 5** (도시 환경) — UE 에디터에서 맵 구축
3. **Phase 7** (ArduPilot) — 모터 패치 테스트
