# Phase 4-Δ — 3대 포메이션 quality 향상 (4패턴 + linear morphing) — Design

**작성일:** 2026-05-22
**작성자:** Claude Code (Opus 4.7) + 김영훈
**대상 트랙:** `aerion-airsim` (코드 변경), 실행 환경은 `aerion-carlaair` Phase 1 진입 후 CARLA+AirSim 통합 빌드
**관련 브랜치:** `branc/airsim-ros2-bridge`
**참고:** [microsoft/AirSim#1538](https://github.com/microsoft/AirSim/issues/1538) — `RpcLibServerBase.cpp:260` 의 `pimpl_->server.async_run(4); //4 threads` 한계 진단

---

## 1. Motivation

[`formation_node.py` v2](../../airsim_ros2_bridge/airsim_ros2_bridge/formation_node.py) 는 **5대 드론 가정**으로 작성되어 있고, 5 패턴 (LINE/DIAMOND/ARROW/V/ECHELON) 모두 5요소 offset 테이블에 hardcode. 그러나:

- AirSim 측 한계 (issue #1538): RPC 서버 `async_run(4)` 가 4 thread 만 사용 → 5번째 차량부터 동시 명령 직렬화 → 5대 동시 takeoff/setpoint 안정성 부족.
- 메모리 [`feedback_aerion_airsim_multivehicle_quirks`](memory): 5/5 100% 어려움 — Phase 3 SimpleFlight CIRCLE 시연에서 2/5 ARRIVED 한계 발생.
- 사용자 관측 (2026-05-22): **3대까지는 동시 제어 안정** (4 thread 안전마진).

**전략 결정**: Colosseum source 의 RPC patch (issue #1538 의 fix commit `262cb28` 이식) 는 별도 phase 로 분리하고, 본 phase 에서는 **3대 한도 안에서 포메이션 quality 자체를 향상**시킨다. 패턴 set 을 3대 전용으로 재설계 + 패턴 간 부드러운 전환 (morphing) 도입.

CARLA+AirSim 통합 환경 (aerion-carlaair Phase 1 진입 단계) 에서 3대 호위/추격/근접 비행 같은 시연이 가능하도록 baseline 을 정비한다.

## 2. Goals / Non-goals

### Goals

1. `formation_node.py` 의 baseline 을 **5대 가정 → 3대 default** 로 전환 (drone_count 파라미터 1~5 범위 유지하되 default 3).
2. 3대 전용 4패턴 (TRIANGLE / V3 / COLUMN / DIAMOND3) 의 offset table 재설계.
3. 패턴 간 **linear interpolation morphing** (1.5초) 도입 — 기존 step transition 제거.
4. FSM 에 신규 상태 `MORPHING` 추가, 신규 토픽 `/aerion/formation/morph_progress` (Float32, 0~1) publish.
5. 기존 외부 토픽 인터페이스 100% backward compatible (외부 mission planner 영향 없음).
6. 검증 시나리오: leader_publisher `circle` 모드 + 4패턴 순환 + morph_progress curve 검증.

### Non-goals (별도 phase)

- multi-axis 회피 (좌/우/후 range 센서)
- CARLA dynamic actor (차량/보행자) 추종 leader
- `RpcLibServerBase.cpp` 의 `async_run` thread patch (issue #1538 fix 의 Colosseum fork 이식)
- 5대 mode 복원 (위 RPC patch 이후 별도 phase 에서 다룸)
- depth camera / lidar 활용 동적 obstacle map

## 3. Architecture

기존 단일 노드 (`formation_node`) 안에서 변경. 새 패키지/모듈 분리 없음.

```
┌──────────────────┐  leader_pose            ┌─────────────────────┐  setpoint_position/local
│ leader_publisher │ ────────────────────▶  │   formation_node    │ ──────────────────────▶  drone1~3 (mavros)
│ (static/circle/  │                          │ (FSM + morphing)    │
│  line)           │                          └─────────────────────┘  morph_progress
└──────────────────┘                                   │                ───────────▶  외부 (관찰/로깅용)
                                                       │
                          /aerion/formation/pattern    │  status (incl. MORPHING)
                          ───────────────────────────▶ │  ───────────▶ 외부 (rviz/CLI)
                          (외부 mission/CLI)            │
                                                       │  range/front (drone1~3)
                                                       │  local_position/pose (drone1~3)
                                                       └◀─── 구독
```

격리 원칙:
- `formation_node` 책임: 패턴/morphing 계산 → per-drone setpoint publish. arm/offboard/takeoff 는 별 책임 외 (기존 그대로).
- `leader_publisher` 책임: 가상 leader_pose 생성 (변경 없음).
- mission planner / RViz / CLI (외부): 본 phase 변경 없음 (외부 토픽 100% backward compat).

## 4. Detailed Design

### 4.1 패턴 offset table

ENU 좌표, leader 기준 상대. 모든 안전거리 ≥ 1.5m (현재 v2 와 동일 기준).

```python
FORMATIONS_3 = {
    'TRIANGLE': [
        (0.0,  0.0, 0.0),    # D1 (leader 위치)
        (-1.7, -1.0, 0.0),   # D2 (좌후방)
        (-1.7, +1.0, 0.0),   # D3 (우후방)
    ],
    'V3': [
        (0.0,  0.0, 0.0),
        (-1.5, -1.5, 0.0),
        (-1.5, +1.5, 0.0),
    ],
    'COLUMN': [
        (0.0,  0.0, 0.0),
        (-2.0, 0.0, 0.0),
        (-4.0, 0.0, 0.0),
    ],
    'DIAMOND3': [
        (0.0,  0.0, +1.0),   # D1 lead 위 1m
        (-1.5, -1.0, 0.0),
        (-1.5, +1.0, 0.0),
    ],
}
```

### 4.2 Morphing 알고리즘

**Linear interp, duration = 1.5 초**.

```python
@dataclass
class MorphState:
    src_pattern: str             # 이전 패턴 이름
    dst_pattern: str             # 새 패턴 이름
    t0: rclpy.time.Time          # morphing 시작 시각
    duration: float = 1.5        # 초
    paused_elapsed: float = 0.0  # OBSTACLE_HOVER 동안 누적된 일시정지 시간 (sec)
    paused_at: Optional[rclpy.time.Time] = None  # pause 진입 시각, resume 시 누적

    def progress(self, now) -> float:
        if self.paused_at is not None:
            # pause 중이면 progress 동결 (paused_at 시점의 elapsed 로 고정).
            # paused_elapsed 는 누적된 pause 시간이므로 elapsed 에서 빼야 함.
            elapsed = ((self.paused_at - self.t0).nanoseconds * 1e-9) - self.paused_elapsed
        else:
            elapsed = ((now - self.t0).nanoseconds * 1e-9) - self.paused_elapsed
        return max(0.0, min(1.0, elapsed / self.duration))

    def pause(self, now):
        if self.paused_at is None:
            self.paused_at = now

    def resume(self, now):
        if self.paused_at is not None:
            self.paused_elapsed += (now - self.paused_at).nanoseconds * 1e-9
            self.paused_at = None
```

각 tick (20Hz) 에서:

```python
if self._morph is not None:
    p = self._morph.progress(now)
    src = FORMATIONS_3[self._morph.src_pattern]
    dst = FORMATIONS_3[self._morph.dst_pattern]
    offsets = [
        ((1-p)*src[i][0] + p*dst[i][0],
         (1-p)*src[i][1] + p*dst[i][1],
         (1-p)*src[i][2] + p*dst[i][2])
        for i in range(self._n)
    ]
    self._morph_pub.publish(Float32(data=p))
    if p >= 1.0:
        self._pattern = self._morph.dst_pattern   # 완료 (None 처리 전 먼저 캡처)
        self._morph = None
        self._fsm = 'SETTLING'                    # 후속 도착 감지
else:
    offsets = FORMATIONS_3[self._pattern]
    self._morph_pub.publish(Float32(data=1.0))   # idle = 1.0
```

### 4.3 FSM 변경

기존 v2 의 FSM (`NO_LEADER / SETTLING / STABLE / OBSTACLE_HOVER`) 에 `MORPHING` 추가:

| 현재 상태 | 트리거 | 다음 상태 |
|---|---|---|
| STABLE | `_on_pattern(new)` 수신 (`new != self._pattern`) | MORPHING (morph 시작) |
| SETTLING | `_on_pattern(new)` 수신 | MORPHING (morph 시작, settling 중단) |
| MORPHING | morph progress ≥ 1.0 | SETTLING (도착 감지 재개) |
| MORPHING | range < obstacle_stop_dist | OBSTACLE_HOVER (morph 중단, src/dst 보존) |
| OBSTACLE_HOVER | range ≥ obstacle_stop_dist | MORPHING 재개 (남은 progress) 또는 SETTLING (morph 완료 상태) |
| 기타 | (기존 v2 그대로) | (기존 v2 그대로) |

### 4.4 토픽 인터페이스 (변경/유지 명시)

**Subscribe (변경 없음)**:
- `/aerion/formation/pattern` (std_msgs/String)
- `/aerion/formation/leader_pose` (geometry_msgs/PoseStamped)
- `/drone{1..3}/range/front` (sensor_msgs/Range)
- `/drone{1..3}/mavros/local_position/pose` (geometry_msgs/PoseStamped)

**Publish**:
- `/drone{1..3}/mavros/setpoint_position/local` (geometry_msgs/PoseStamped, 20Hz) — 변경 없음
- `/aerion/formation/status` (std_msgs/String) — 값 set 확장: `NO_LEADER / SETTLING / STABLE / OBSTACLE_HOVER / **MORPHING**`
- `/aerion/formation/morph_progress` (std_msgs/Float32, 20Hz) — **신규**

**Parameter (변경/추가)**:
- `drone_count`: default `5 → 3`. 범위 `1~5` (5 는 RPC patch 이후 별도 phase 까지 deprecated 표시)
- `morph_duration_sec`: 신규, default `1.5`
- `default_pattern`: default `'LINE' → 'TRIANGLE'`
- 기타 (`publish_rate, obstacle_stop_dist, enable_arrival_check, default_altitude`) 변경 없음

### 4.5 파일 변경 단위 (commits 분할)

| commit | 범위 |
|---|---|
| 1 | `formation_node.py`: `FORMATIONS_3` 도입, `N_DRONES=3`, `MorphState`, FSM `MORPHING`, morph publisher. drone_count default 3. |
| 2 | `airsim_ros2_bridge/launch/aerion_phase4_formation.launch.py`: 신규 인자 `default_pattern`, `morph_duration_sec`, `drone_count` default 정렬. |
| 3 | `airsim_ros2_bridge/airsim_ros2_bridge/formation_demo.py` (신규) + `setup.py` entry point `aerion_formation_demo`: 4패턴 순환 데모 (`--demo morphing-cycle`). 기존 `aerion_formation` entry point 는 무영향. |
| 4 | docs/spec 본 문서 + verification runbook (`docs/phase4_delta_runbook.md`) |

총 4 commits, 예상 ~2일.

## 5. Verification

### 5.1 Smoke (단일 노드 단위)

```bash
ros2 launch airsim_ros2_bridge aerion_phase4_formation.launch.py \
  drone_count:=3 default_pattern:='TRIANGLE' morph_duration_sec:=1.5

# 다른 터미널:
ros2 topic echo /aerion/formation/status --once   # → "STABLE" 또는 "SETTLING"
ros2 topic pub --once /aerion/formation/pattern std_msgs/String '{data: "V3"}'
# 즉시 status → "MORPHING" → 1.5s 후 → "SETTLING" → "STABLE"
ros2 topic echo /aerion/formation/morph_progress  # 0→1 증가 (20Hz)
```

### 5.2 Integration (CARLA+AirSim 환경)

전제: aerion-carlaair Phase 1 통합 빌드 완료, settings 3대 PX4Multirotor 또는 SimpleFlight, MAVROS 3대 connected, OFFBOARD set.

```bash
# Step 1: leader_publisher (circle 모드)
ros2 run airsim_ros2_bridge leader_publisher \
  --ros-args -p mode:=circle -p circle_radius:=5.0 -p circle_angular_vel:=0.1 \
             -p init_x:=0.0 -p init_y:=0.0 -p init_z:=5.0

# Step 2: formation_node + 순환 데모
ros2 run airsim_ros2_bridge aerion_formation \
  --ros-args -p drone_count:=3 -p default_pattern:='TRIANGLE'

# Step 3: CLI 로 4패턴 순환 (T4 신규 entry point)
ros2 run airsim_ros2_bridge aerion_formation_demo --demo morphing-cycle \
  --patterns 'TRIANGLE,V3,COLUMN,DIAMOND3' --hold-sec 10
```

검증 PASS 조건:

- [ ] 각 패턴 hold 구간에서 도착 오차 < `ARRIVAL_TOLERANCE_M` (0.6m) → status 가 `STABLE`
- [ ] 각 morph 구간에서 `morph_progress` 가 0→1 단조 증가 (20Hz 샘플 기준 +0.066±0.01/sample)
- [ ] `MORPHING` 상태가 1.5±0.1초 지속
- [ ] 각 드론의 `local_position/pose` 시계열에서 위치 jerk < 임계 (시각적 부드러움)
- [ ] `range/front < 1.0m` 트리거 시 `OBSTACLE_HOVER` 우선 발동, morph 보존 (resume 시 progress 이어짐)

### 5.3 rosbag 분석

```bash
ros2 bag record /drone{1,2,3}/mavros/local_position/pose \
                /aerion/formation/{pattern,leader_pose,status,morph_progress}
# 분석 script (별도 PR):
python3 scripts/analyze_morphing.py <bag> --plot
```

## 6. Risks

| 위험 | 영향 | 완화 |
|---|---|---|
| morph 중간에 `OBSTACLE_HOVER` 진입 시 resume 로직이 progress 를 어떻게 이어갈지 미정 | morph 중단 → resume 시 jump 가능 | §4.3 표 에 명시 — `OBSTACLE_HOVER` 진입 시 `src/dst` 보존, resume 시 elapsed 재계산 (`t0` 를 paused_at 만큼 보정). 검증 §5.2 마지막 항목으로 포함. |
| MAVROS setpoint stream 의 단조성 가정과 morph linear 충돌 (가속도 불연속) | PX4 EKF estimator 에 미세 noise | duration 1.5s + leader 자체 속도 ω·R = 0.5 m/s 합산 시 setpoint jump < 0.15m/tick → 안전 마진. Verification §5.2 의 jerk 임계로 가드. |
| 기존 5대 launch / CLI 회귀 | 외부 사용자 영향 | `drone_count` 파라미터 1~5 그대로 지원. 5 입력 시 v2 패턴 fallback 로직 유지 (deprecated warning 만 추가). |
| RPC 4-thread (issue #1538) 한계 안에서 3대 보장 했지만 4번째 노드 (formation_node) RPC 추가 사용 시 영향 | formation_node 는 RPC 안 씀 (mavros topic only) | formation_node 는 ROS2 topic 기반, RPC 미사용. mavros 도 별 프로세스. → 영향 없음. |

## 7. Out of Scope (다음 phase 후보)

- **Phase 4-ε**: multi-axis 회피 (좌/우/후 range 추가) + dynamic obstacle map
- **Phase 4-ζ**: CARLA actor leader 어댑터 (`/carla/<actor>/pose` → `/aerion/formation/leader_pose` 변환 노드)
- **Phase 4-η (별도)**: Colosseum fork 에 issue #1538 fix (`async_run(num_vehicles)`) 이식 + 5대 mode 복원
- **Phase 4-θ**: leader yaw 변화 시 패턴 회전의 jerk-limited 버전 (현재 linear interp 도 quaternion slerp 적용 필요할 수 있음)

## 8. Open Questions

(현재 해결 — 모든 default 사용자 confirm)

- ~~Morphing 보간 알고리즘~~ → Linear interp 1.5s 채택
- ~~패턴 set~~ → 4패턴 채택
- ~~정적/동적/회피 중 scope~~ → 정적 + 동적 채택

## 9. References

- [microsoft/AirSim#1538](https://github.com/microsoft/AirSim/issues/1538) — RPC 4-thread async_run 한계 진단 + fix commit `262cb28`
- [`formation_node.py` v2 (current)](../../airsim_ros2_bridge/airsim_ros2_bridge/formation_node.py) — 5대 baseline
- [`leader_publisher.py`](../../airsim_ros2_bridge/airsim_ros2_bridge/leader_publisher.py) — 변경 없음
- 메모리: `feedback_aerion_airsim_multivehicle_quirks`, `reference_aerion_airsim_rpc_limits`, `reference_aerostack2_phase4_analysis`
- [Phase 5 EKF2 root cause doc (2026-05-21)](../2026-05-21_phase5_ekf2_root_cause.md) — 동일 baseline 작업 흐름
