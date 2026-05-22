# Phase 4-Δ — 3-Drone Formation Verification Runbook

**Date:** 2026-05-22
**Spec:** [`docs/superpowers/specs/2026-05-22-phase4-delta-3drone-formation-design.md`](superpowers/specs/2026-05-22-phase4-delta-3drone-formation-design.md)
**Plan:** [`docs/superpowers/plans/2026-05-22-phase4-delta-formation.md`](superpowers/plans/2026-05-22-phase4-delta-formation.md)

Verifies the Phase 4-Δ implementation — 3-drone baseline + 4 patterns + linear morphing.

## Prerequisites

- `airsim_ros2_bridge` package built (`colcon build --packages-select airsim_ros2_bridge`)
- 3 drones (PX4 SITL or SimpleFlight) in the AirSim/Colosseum scene
- MAVROS 3 instances connected (`/drone{1,2,3}/mavros/state.connected: true`)
- All 3 drones armed + OFFBOARD mode set

## A. Smoke test (formation_node alone)

Terminal 1 — formation_node:

```bash
source ~/workspace/projects/aerion-airsim/install/setup.bash
ros2 launch airsim_ros2_bridge aerion_phase4_formation.launch.py \
  drone_count:=3 default_pattern:=TRIANGLE morph_duration_sec:=1.5
```

Terminal 2 — leader_publisher (circle):

```bash
ros2 run airsim_ros2_bridge aerion_leader --ros-args \
  -p mode:=circle -p circle_radius:=5.0 -p circle_angular_vel:=0.1 \
  -p init_x:=0.0 -p init_y:=0.0 -p init_z:=5.0
```

> 위 launch는 `enable_leader_publisher:=true` 가 default 이므로 leader 노드가 자동 기동된다. mission planner 가 있거나 별도 leader 를 띄우려면 `enable_leader_publisher:=false` 인자를 launch 명령에 추가하고 Terminal 2 의 leader 를 별도로 띄운다.

Terminal 3 — pattern transition + observe:

```bash
# 즉시 status 확인
ros2 topic echo /aerion/formation/status --once
# 기대: data: 'SETTLING' (leader 받자마자) → 잠시 후 'STABLE'

# 패턴 전환
ros2 topic pub --once /aerion/formation/pattern std_msgs/String '{data: "V3"}'

# morph_progress 가 0→1 부드럽게 증가 (20Hz, 1.5초)
ros2 topic echo /aerion/formation/morph_progress
```

**PASS 조건:**

- [ ] `status` 가 `MORPHING` 으로 전이 후 약 1.5초 뒤 `SETTLING` → `STABLE`
- [ ] `morph_progress` 가 0.0 → 1.0 단조 증가 (10 Hz 샘플 기준 +0.066±0.01)
- [ ] 패턴 전환 중 setpoint 가 끊김 없이 20Hz 발행 (`ros2 topic hz /drone1/mavros/setpoint_position/local`)

## B. 순환 데모 (CLI)

위 Terminal 1, 2 그대로 두고, T4 에서 추가된 `aerion_formation_demo` entry point 호출:

```bash
ros2 run airsim_ros2_bridge aerion_formation_demo \
  --demo morphing-cycle \
  --patterns TRIANGLE,V3,COLUMN,DIAMOND3 \
  --hold-sec 10.0
```

**PASS 조건:**

- [ ] 10초마다 패턴 전환, 각 전환에서 1.5초 morphing 후 STABLE
- [ ] 4패턴 모두 도착 오차 < 0.6m (`ARRIVAL_TOLERANCE_M`)

## C. 회피 hook + morphing 일시정지

morphing 중에 obstacle 트리거 시 morph pause 검증.

Terminal 3 (B 진행 중):

```bash
# 가짜 range 메시지 발행 (실제 sensor 없을 때)
ros2 topic pub --once /drone2/range/front sensor_msgs/Range \
  '{radiation_type: 1, field_of_view: 0.5, min_range: 0.1, max_range: 10.0, range: 0.5}'
```

**PASS 조건:**

- [ ] `status` 즉시 `OBSTACLE_HOVER` 로 전이, `morph_progress` 값 동결 (변화 없음 — OBSTACLE_HOVER 동안 _tick 이 early-return 하므로 stream gap 발생; pause 직전 마지막 값 유지)
- [ ] 다시 정상 range publish 시 (range > obstacle_stop_dist 임계, 기본 1.0m) `MORPHING` 복귀, `morph_progress` 일시정지된 지점부터 계속 증가

## D. rosbag 분석 (시각화)

```bash
ros2 bag record /drone1/mavros/local_position/pose \
                /drone2/mavros/local_position/pose \
                /drone3/mavros/local_position/pose \
                /aerion/formation/pattern \
                /aerion/formation/leader_pose \
                /aerion/formation/status \
                /aerion/formation/morph_progress \
                -o /tmp/phase4_delta_demo
# 10분 정도 진행 후 Ctrl+C

# 분석 (python script 작성은 별 PR; 우선 ros2 bag info 로 토픽 행수만 확인)
ros2 bag info /tmp/phase4_delta_demo
```

**기록 항목:**

- 4 패턴 × 10초 hold = 40초/cycle
- morph_progress curve 가 각 패턴 전환 후 0→1 4 번 반복

## E. Backward compat 회귀

5대 drone_count 가 여전히 동작하는지:

```bash
ros2 launch airsim_ros2_bridge aerion_phase4_formation.launch.py drone_count:=5
# 로그: "drone_count=5 is deprecated ... Fallback to v2 5-pattern table."
ros2 topic pub --once /aerion/formation/pattern std_msgs/String '{data: "LINE"}'
# LINE/DIAMOND/ARROW/V/ECHELON 모두 인식 (FORMATIONS_5)
```

**PASS 조건:**

- [ ] 로그에 deprecation warning 출력
- [ ] 5패턴 모두 인식, morphing 도 동일 동작

## F. 단위 테스트 (MorphState 직접 검증, ROS2 환경 불필요)

```bash
cd ~/workspace/projects/aerion-airsim/airsim_ros2_bridge
python3 -m pytest test/test_formation_morph.py -v
```

**PASS 조건:**

- [ ] 12 tests pass (progress boundary 5 + pause/resume 5 + duration guard 2)
- [ ] T1 commit `0c1cfb7` 의 pyproject.toml `[tool.pytest.ini_options]` 가 ROS2 launch plugin 들을 비활성화하여 plain `python3 -m pytest` 가 동작.

## 알려진 한계

- AirSim RPC `async_run(4)` (issue #1538) 한계로 drone_count=5 시 동시 takeoff 안정성 부족 — 별도 Phase 4-η 에서 Colosseum fork 패치로 해결 예정.
- multi-axis 회피 미지원 — Phase 4-ε 별도 작업.
- CARLA dynamic actor leader 추종 미지원 — Phase 4-ζ 별도 작업.
- Rapid pattern 요청 (morph 진행 중 새 patten request): `_on_pattern` 이 src 를 마지막 committed pattern 으로 reset → 가시적 setpoint jump 가능. 의도된 트레이드오프. Phase 4-η 후속.
