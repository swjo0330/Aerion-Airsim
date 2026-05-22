# Phase 4-Δ — 3-Drone Formation Quality (4 patterns + linear morphing) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Shift `formation_node` baseline from 5-drone to 3-drone, redesign 4 patterns (TRIANGLE / V3 / COLUMN / DIAMOND3) and replace step-jump pattern transition with 1.5s linear-interp morphing — with backward compatibility for the existing external topic interface.

**Architecture:** Single-node modification inside `airsim_ros2_bridge` package. Add `MorphState` dataclass (pure Python, no rclpy dependency for testability), extend FSM with `MORPHING` state, publish new `/aerion/formation/morph_progress` topic. Keep `leader_publisher.py` untouched. Verification via pytest unit tests + manual ros2 launch smoke + rosbag trajectory analysis.

**Tech Stack:** Python 3.12, rclpy (ROS2 Humble), ament_python, pytest. No new third-party dependencies.

**Spec reference:** [`docs/superpowers/specs/2026-05-22-phase4-delta-3drone-formation-design.md`](../specs/2026-05-22-phase4-delta-3drone-formation-design.md) (commit `d503c0f`)

---

## File Structure

| Path | Action | Responsibility |
|---|---|---|
| `airsim_ros2_bridge/airsim_ros2_bridge/formation_node.py` | Modify | `MorphState` dataclass (Task 1), `FORMATIONS_3` + `N_DRONES=3` + FSM `MORPHING` + morph publisher (Task 2) |
| `airsim_ros2_bridge/test/test_formation_morph.py` | Create | Pure pytest unit tests for `MorphState` semantics (progress, pause, resume) |
| `airsim_ros2_bridge/launch/aerion_phase4_formation.launch.py` | Modify | New launch args (`default_pattern`, `morph_duration_sec`, `drone_count` default 3) |
| `airsim_ros2_bridge/scripts/aerion_formation.py` | Modify | New `--demo morphing-cycle` mode that publishes pattern transitions on a fixed schedule |
| `docs/phase4_delta_runbook.md` | Create | Verification runbook: 3-terminal sequence + rosbag analysis cheat-sheet |

5 commits total, ~1 per task. Each task is self-contained — produces working/testable code.

---

## Task 1: MorphState dataclass with pure-pytest unit tests (TDD)

**Files:**
- Create: `airsim_ros2_bridge/test/test_formation_morph.py`
- Modify: `airsim_ros2_bridge/airsim_ros2_bridge/formation_node.py` (add MorphState dataclass + Optional import only — do NOT touch FORMATIONS / FSM / pub-sub yet)

**Rationale:** `MorphState` is the only piece with non-trivial state semantics (progress, pause, resume). Keep it pure Python (float-second times instead of `rclpy.time.Time`) so it is unit-testable without ROS2 runtime. The node calls it with `self.get_clock().now().nanoseconds * 1e-9`.

- [ ] **Step 1: Create test file with failing tests**

Create `airsim_ros2_bridge/test/test_formation_morph.py`:

```python
"""Unit tests for MorphState — pure Python, no ROS2 runtime needed."""
import pytest

from airsim_ros2_bridge.formation_node import MorphState


def test_progress_zero_at_start():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    assert ms.progress(100.0) == 0.0


def test_progress_half_at_midpoint():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    assert ms.progress(100.75) == pytest.approx(0.5, rel=1e-3)


def test_progress_one_at_end():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    assert ms.progress(101.5) == 1.0


def test_progress_clamped_above_one():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    assert ms.progress(200.0) == 1.0


def test_progress_clamped_below_zero():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    assert ms.progress(99.0) == 0.0


def test_pause_freezes_progress():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    ms.pause(100.75)              # paused at midpoint
    assert ms.progress(101.5) == pytest.approx(0.5, rel=1e-3)
    assert ms.progress(105.0) == pytest.approx(0.5, rel=1e-3)


def test_resume_continues_from_pause_point():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    ms.pause(100.75)              # paused at midpoint
    ms.resume(103.0)              # 2.25s gap
    # After resume, elapsed = (now - t0) - paused_elapsed = (now - 100) - 2.25
    # At now=103.75: elapsed = 3.75 - 2.25 = 1.5 → progress = 1.0
    assert ms.progress(103.75) == 1.0
    # At now=103.0 (just resumed): elapsed = 3.0 - 2.25 = 0.75 → progress = 0.5
    assert ms.progress(103.0) == pytest.approx(0.5, rel=1e-3)


def test_double_pause_is_idempotent():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    ms.pause(100.75)
    ms.pause(101.0)               # 2nd pause should be no-op
    ms.resume(103.0)              # gap should be 103.0 - 100.75 = 2.25, not 2.0
    assert ms.progress(103.75) == 1.0


def test_resume_without_pause_is_noop():
    ms = MorphState(src_pattern='TRIANGLE', dst_pattern='V3', t0_sec=100.0, duration=1.5)
    ms.resume(101.0)              # no-op
    assert ms.progress(101.5) == 1.0    # progress as if no pause/resume happened
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd ~/workspace/projects/aerion-airsim/airsim_ros2_bridge
python3 -m pytest test/test_formation_morph.py -v
```

Expected: `ImportError: cannot import name 'MorphState' from 'airsim_ros2_bridge.formation_node'` (collection error)

- [ ] **Step 3: Add MorphState dataclass to formation_node.py**

Modify `airsim_ros2_bridge/airsim_ros2_bridge/formation_node.py`. In the existing imports near the top, ensure `Optional` is imported (it already is — line 55). Add the following dataclass right after the existing `DroneState` dataclass (around line 99), before the `# ---------- 유틸 ----------` comment:

```python
@dataclass
class MorphState:
    """패턴 간 부드러운 전환 상태. 순수 Python (float-second time) 으로 단위 테스트 가능.

    노드 안에서는 `self.get_clock().now().nanoseconds * 1e-9` 로 호출.
    OBSTACLE_HOVER 진입 시 pause(), 해소 시 resume() 로 progress 동결/재개.
    """
    src_pattern: str
    dst_pattern: str
    t0_sec: float
    duration: float = 1.5
    paused_elapsed: float = 0.0
    paused_at_sec: Optional[float] = None

    def progress(self, now_sec: float) -> float:
        if self.paused_at_sec is not None:
            # pause 중이면 paused_at 시점의 elapsed 로 동결. paused_elapsed 는 누적된 pause 시간.
            elapsed = (self.paused_at_sec - self.t0_sec) - self.paused_elapsed
        else:
            elapsed = (now_sec - self.t0_sec) - self.paused_elapsed
        return max(0.0, min(1.0, elapsed / self.duration))

    def pause(self, now_sec: float) -> None:
        if self.paused_at_sec is None:
            self.paused_at_sec = now_sec

    def resume(self, now_sec: float) -> None:
        if self.paused_at_sec is not None:
            self.paused_elapsed += now_sec - self.paused_at_sec
            self.paused_at_sec = None
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest test/test_formation_morph.py -v
```

Expected: all 9 tests PASS.

- [ ] **Step 5: Commit**

```bash
cd ~/workspace/projects/aerion-airsim
git add airsim_ros2_bridge/airsim_ros2_bridge/formation_node.py \
        airsim_ros2_bridge/test/test_formation_morph.py
git commit -m "feat(formation): add MorphState dataclass for pattern transitions (phase4-Δ)

Pure-Python float-second time API so progress/pause/resume can be unit
tested without rclpy runtime. Node will call it with
self.get_clock().now().nanoseconds * 1e-9.

Tests: 9 cases — progress at boundaries, clamping, pause freezes
progress, resume continues from pause point, double-pause idempotent,
resume-without-pause is no-op.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Replace 5-drone FORMATIONS with FORMATIONS_3, integrate MorphState into FSM, add morph publisher

**Files:**
- Modify: `airsim_ros2_bridge/airsim_ros2_bridge/formation_node.py`

**Rationale:** Core behavior change. Replace 5-element `FORMATIONS` dict with 3-element `FORMATIONS_3` (4 patterns), shift default `N_DRONES` to 3, integrate `MorphState` into the tick loop, add `MORPHING` FSM state and `/aerion/formation/morph_progress` publisher. Keep `drone_count` parameter range `1~5` (with deprecation warning when 5 is selected) for backward compat.

- [ ] **Step 1: Update top-level constants (lines ~66–84)**

In `formation_node.py`, replace the `FORMATIONS` dict and `N_DRONES` constant.

Find (around line 64-75):
```python
N_DRONES = 5

# ENU 좌표, leader 기준 상대 offset (dx, dy, dz). 충돌 안전거리 ≥1.5m 유지.
FORMATIONS: Dict[str, list] = {
    'LINE':    [( 0,  0, 0), ( 0, -2, 0), ( 0, -4, 0), ( 0, -6, 0), ( 0, -8, 0)],
    'DIAMOND': [( 0,  0, 0), ( 2, -2, 0), ( 0, -4, 0), (-2, -2, 0), ( 0, -2, 1.0)],
    'ARROW':   [( 0,  0, 0), (-1.5, -1.5, 0), (-3, -3, 0), (-1.5, 1.5, 0), (-3, 3, 0)],
    'V':       [( 0,  0, 0), (-1.5, -1.5, 0), (-3, -3, 0), (-1.5,  1.5, 0), (-3,  3, 0)],
    'ECHELON': [( 0,  0, 0), (-1.5, -1.5, 0), (-3, -3, 0), (-4.5, -4.5, 0), (-6, -6, 0)],
}
```

Replace with:
```python
N_DRONES = 3  # Phase 4-Δ baseline (AirSim issue #1538 RPC 4-thread limit safe margin).

# ENU 좌표, leader 기준 상대 offset (dx, dy, dz). 충돌 안전거리 ≥1.5m 유지.
# Phase 4-Δ: 3-drone 전용 4패턴.
FORMATIONS_3: Dict[str, list] = {
    'TRIANGLE': [( 0.0,  0.0, 0.0), (-1.7, -1.0, 0.0), (-1.7,  1.0, 0.0)],
    'V3':       [( 0.0,  0.0, 0.0), (-1.5, -1.5, 0.0), (-1.5,  1.5, 0.0)],
    'COLUMN':   [( 0.0,  0.0, 0.0), (-2.0,  0.0, 0.0), (-4.0,  0.0, 0.0)],
    'DIAMOND3': [( 0.0,  0.0, 1.0), (-1.5, -1.0, 0.0), (-1.5,  1.0, 0.0)],
}

# Phase 4 (v2) 5-drone fallback. drone_count=5 일 때만 사용. deprecated.
FORMATIONS_5: Dict[str, list] = {
    'LINE':    [( 0,  0, 0), ( 0, -2, 0), ( 0, -4, 0), ( 0, -6, 0), ( 0, -8, 0)],
    'DIAMOND': [( 0,  0, 0), ( 2, -2, 0), ( 0, -4, 0), (-2, -2, 0), ( 0, -2, 1.0)],
    'ARROW':   [( 0,  0, 0), (-1.5, -1.5, 0), (-3, -3, 0), (-1.5, 1.5, 0), (-3, 3, 0)],
    'V':       [( 0,  0, 0), (-1.5, -1.5, 0), (-3, -3, 0), (-1.5,  1.5, 0), (-3,  3, 0)],
    'ECHELON': [( 0,  0, 0), (-1.5, -1.5, 0), (-3, -3, 0), (-4.5, -4.5, 0), (-6, -6, 0)],
}

DEFAULT_MORPH_DURATION_SEC = 1.5
```

- [ ] **Step 2: Add Float32 import**

Find the import block (line 60-61):
```python
from std_msgs.msg import String
```

Replace with:
```python
from std_msgs.msg import Float32, String
```

- [ ] **Step 3: Modify FormationNode.__init__ to select pattern table + add morph publisher**

Find (around line 137-152) the parameter declaration and pattern lookup block. Modify it as follows.

Find:
```python
        self.declare_parameter('drone_count', N_DRONES)
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter('obstacle_stop_dist', DEFAULT_OBSTACLE_STOP_DISTANCE)
        self.declare_parameter('enable_arrival_check', True)
        self.declare_parameter('default_altitude', 5.0)  # leader_pose 미수신 시 기본 고도 (ENU z)
        self.declare_parameter('default_pattern', 'LINE')

        self._n = int(self.get_parameter('drone_count').value)
        self._rate = float(self.get_parameter('publish_rate').value)
        self._obstacle_stop_dist = float(self.get_parameter('obstacle_stop_dist').value)
        self._enable_arrival = bool(self.get_parameter('enable_arrival_check').value)
        self._default_alt = float(self.get_parameter('default_altitude').value)
        self._pattern = str(self.get_parameter('default_pattern').value).upper()
        if self._pattern not in FORMATIONS:
            self.get_logger().warn(f'Unknown default_pattern={self._pattern}, fallback to LINE')
            self._pattern = 'LINE'

        if not (1 <= self._n <= 5):
            raise ValueError(f'drone_count 1~5만 지원: {self._n}')
```

Replace with:
```python
        self.declare_parameter('drone_count', N_DRONES)
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter('obstacle_stop_dist', DEFAULT_OBSTACLE_STOP_DISTANCE)
        self.declare_parameter('enable_arrival_check', True)
        self.declare_parameter('default_altitude', 5.0)  # leader_pose 미수신 시 기본 고도 (ENU z)
        self.declare_parameter('default_pattern', 'TRIANGLE')
        self.declare_parameter('morph_duration_sec', DEFAULT_MORPH_DURATION_SEC)

        self._n = int(self.get_parameter('drone_count').value)
        self._rate = float(self.get_parameter('publish_rate').value)
        self._obstacle_stop_dist = float(self.get_parameter('obstacle_stop_dist').value)
        self._enable_arrival = bool(self.get_parameter('enable_arrival_check').value)
        self._default_alt = float(self.get_parameter('default_altitude').value)
        self._morph_duration = float(self.get_parameter('morph_duration_sec').value)

        if not (1 <= self._n <= 5):
            raise ValueError(f'drone_count 1~5만 지원: {self._n}')

        # Phase 4-Δ: drone_count 에 따라 패턴 테이블 선택. 5 는 deprecated v2 fallback.
        if self._n == 5:
            self._formations = FORMATIONS_5
            self.get_logger().warn(
                'drone_count=5 is deprecated in Phase 4-Δ (AirSim RPC 4-thread limit). '
                'Fallback to v2 5-pattern table. See spec phase4-delta + Phase 4-η.'
            )
        else:
            # 1~4 인 경우 모두 FORMATIONS_3 사용. drone_count < 3 이면 앞 N 개만 사용.
            self._formations = FORMATIONS_3

        self._pattern = str(self.get_parameter('default_pattern').value).upper()
        if self._pattern not in self._formations:
            fallback = 'TRIANGLE' if self._n != 5 else 'LINE'
            self.get_logger().warn(
                f'Unknown default_pattern={self._pattern} for drone_count={self._n}, '
                f'fallback to {fallback}'
            )
            self._pattern = fallback
```

- [ ] **Step 4: Add morph state field + morph publisher**

Find (around line 162-164) the runtime state block:
```python
        self._state: Dict[int, DroneState] = {i: DroneState() for i in range(1, self._n + 1)}
        self._fsm = 'NO_LEADER'  # NO_LEADER | SETTLING | STABLE | OBSTACLE_HOVER
        self._lock = threading.RLock()
```

Replace with:
```python
        self._state: Dict[int, DroneState] = {i: DroneState() for i in range(1, self._n + 1)}
        self._fsm = 'NO_LEADER'  # NO_LEADER | SETTLING | STABLE | OBSTACLE_HOVER | MORPHING
        self._morph: Optional[MorphState] = None
        self._lock = threading.RLock()
```

Find (around line 185-186) the status publisher block:
```python
        self._status_pub = self.create_publisher(String, '/aerion/formation/status', 10)
```

Replace with:
```python
        self._status_pub = self.create_publisher(String, '/aerion/formation/status', 10)
        self._morph_pub = self.create_publisher(Float32, '/aerion/formation/morph_progress', 10)
```

- [ ] **Step 5: Modify _on_pattern callback to start a morph**

Find (around line 198-209):
```python
    def _on_pattern(self, msg: String):
        name = msg.data.strip().upper()
        if name not in FORMATIONS:
            self.get_logger().warn(f'Unknown formation pattern: {msg.data!r}')
            return
        with self._lock:
            old = self._pattern
            self._pattern = name
            # 패턴이 바뀌면 SETTLING 상태로 진입 (도착 감지 → STABLE 재판정).
            if self._fsm == 'STABLE':
                self._fsm = 'SETTLING'
        self.get_logger().info(f'Formation pattern: {old} -> {name}')
```

Replace with:
```python
    def _on_pattern(self, msg: String):
        name = msg.data.strip().upper()
        if name not in self._formations:
            self.get_logger().warn(f'Unknown formation pattern: {msg.data!r}')
            return
        with self._lock:
            old = self._pattern
            if name == old:
                return    # no-op
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            self._morph = MorphState(
                src_pattern=old,
                dst_pattern=name,
                t0_sec=now_sec,
                duration=self._morph_duration,
            )
            self._fsm = 'MORPHING'
        self.get_logger().info(f'Formation pattern: {old} -> {name} (morphing {self._morph_duration:.1f}s)')
```

- [ ] **Step 6: Modify _tick to compute interpolated offsets when MORPHING**

Find (around line 234-277) the `_tick` method. Replace the whole method:

```python
    def _tick(self):
        """20Hz: leader_pose + pattern (또는 morph 진행) 기반으로 각 드론 setpoint 발행."""
        with self._lock:
            if not self._leader_received:
                # leader 없으면 publish 안 함. 외부에서 leader_pose 발행 시작 대기.
                return

            now_sec = self.get_clock().now().nanoseconds * 1e-9

            # 거리센서 회피 hook: 어느 드론이라도 전방 거리 < 임계 → OBSTACLE_HOVER
            min_range = min(s.range_front for s in self._state.values())
            if min_range < self._obstacle_stop_dist:
                if self._fsm != 'OBSTACLE_HOVER':
                    if self._morph is not None:
                        self._morph.pause(now_sec)
                    self._fsm = 'OBSTACLE_HOVER'
                self._publish_hover_setpoints()
                return
            elif self._fsm == 'OBSTACLE_HOVER':
                # 장애물 해소 → 이전 상태 복귀.
                if self._morph is not None:
                    self._morph.resume(now_sec)
                    self._fsm = 'MORPHING'
                else:
                    self._fsm = 'SETTLING'

            # Morphing 활성 시 interp, 아니면 정적 offset.
            if self._morph is not None:
                p = self._morph.progress(now_sec)
                src = self._formations[self._morph.src_pattern]
                dst = self._formations[self._morph.dst_pattern]
                offsets = [
                    ((1 - p) * src[i][0] + p * dst[i][0],
                     (1 - p) * src[i][1] + p * dst[i][1],
                     (1 - p) * src[i][2] + p * dst[i][2])
                    for i in range(self._n)
                ]
                morph_progress_value = p

                if p >= 1.0:
                    # 완료: dst 캡처 후 morph 정리, SETTLING 으로 전이.
                    self._pattern = self._morph.dst_pattern
                    self._morph = None
                    self._fsm = 'SETTLING'
            else:
                offsets = self._formations[self._pattern]
                # idle 시 morph_progress=1.0 (외부 관찰자가 항상 stream 받을 수 있게)
                morph_progress_value = 1.0

            lx, ly, lz, yaw = self._leader

        # publish (lock 밖에서 publish → 콜백 backpressure 영향 최소화, rclpy publisher 는 thread-safe 이지만 v2 패턴 일관성 위해)
        mp = Float32()
        mp.data = float(morph_progress_value)
        self._morph_pub.publish(mp)
        stamp = self.get_clock().now().to_msg()
        for i in range(1, self._n + 1):
            dx, dy, dz = offsets[i - 1]
            rx, ry = rotate_xy(dx, dy, yaw)
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x = lx + rx
            ps.pose.position.y = ly + ry
            ps.pose.position.z = lz + dz
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            self._setpoint_pubs[i].publish(ps)

        # 도착 감지 → STABLE 전환 (morph 종료 직후의 SETTLING 도 같은 로직 적용)
        if self._enable_arrival and self._fsm == 'SETTLING':
            if self._all_arrived(offsets):
                with self._lock:
                    self._fsm = 'STABLE'
                self.get_logger().info('All drones arrived → STABLE')
```

- [ ] **Step 7: Update __init__ log line**

Find (around line 191-194):
```python
        self.get_logger().info(
            f'AERION formation node ready. drones={self._n}, pattern={self._pattern}, '
            f'rate={self._rate}Hz, obstacle_stop={self._obstacle_stop_dist}m'
        )
```

Replace with:
```python
        self.get_logger().info(
            f'AERION formation node ready. drones={self._n}, pattern={self._pattern}, '
            f'rate={self._rate}Hz, obstacle_stop={self._obstacle_stop_dist}m, '
            f'morph_duration={self._morph_duration}s, table='
            f'{"FORMATIONS_5(deprecated)" if self._n == 5 else "FORMATIONS_3"}'
        )
```

- [ ] **Step 8: Run unit tests to verify no regression**

```bash
cd ~/workspace/projects/aerion-airsim/airsim_ros2_bridge
python3 -m pytest test/test_formation_morph.py -v
```

Expected: 9 tests PASS (same as Task 1 — MorphState semantics unchanged).

- [ ] **Step 9: Syntax check the full file**

```bash
python3 -m py_compile airsim_ros2_bridge/airsim_ros2_bridge/formation_node.py
echo "exit=$?"
```

Expected: `exit=0` (no syntax errors).

- [ ] **Step 10: Commit**

```bash
git add airsim_ros2_bridge/airsim_ros2_bridge/formation_node.py
git commit -m "feat(formation): 3-drone baseline + 4 patterns + linear morphing (phase4-Δ)

Replace FORMATIONS (5-drone) with FORMATIONS_3 (TRIANGLE / V3 / COLUMN /
DIAMOND3) as the active table when drone_count != 5. drone_count=5 keeps
the v2 5-pattern table as deprecated fallback (with logged warning).

FSM gains MORPHING state. _on_pattern() starts a MorphState; _tick()
interpolates src→dst offsets linearly over morph_duration_sec (default
1.5s) and publishes /aerion/formation/morph_progress (Float32, 20Hz).
OBSTACLE_HOVER pauses the morph (paused_at) and resume() accumulates
the gap on clearance so the curve continues smoothly.

New parameter: morph_duration_sec (default 1.5).
default_pattern default: LINE → TRIANGLE.

External topic interface is backward compatible — only addition is
/aerion/formation/morph_progress.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Update launch override with new args

**Files:**
- Modify: `airsim_ros2_bridge/launch/aerion_phase4_formation.launch.py`

- [ ] **Step 1: Read current launch file**

```bash
cat airsim_ros2_bridge/launch/aerion_phase4_formation.launch.py
```

This tells us the existing structure. The plan below assumes the typical pattern of `DeclareLaunchArgument` + `Node` blocks. Adapt parameter wiring to existing convention if different.

- [ ] **Step 2: Add three new DeclareLaunchArgument entries near existing args**

Add the following arguments (insert near the existing `drone_count` declaration if present; otherwise near the top of `generate_launch_description()` after other `DeclareLaunchArgument` calls):

```python
    DeclareLaunchArgument(
        'drone_count',
        default_value='3',
        description='Number of drones (1~5). Phase 4-Δ default: 3 (AirSim RPC 4-thread safe margin).',
    ),
    DeclareLaunchArgument(
        'default_pattern',
        default_value='TRIANGLE',
        description='Initial formation pattern. For drone_count != 5: TRIANGLE / V3 / COLUMN / DIAMOND3.',
    ),
    DeclareLaunchArgument(
        'morph_duration_sec',
        default_value='1.5',
        description='Pattern-to-pattern morphing duration in seconds (linear interp).',
    ),
```

- [ ] **Step 3: Wire the args into the formation_node Node block**

Find the `Node(...)` entry for `aerion_formation` (entry_point `aerion_formation` from setup.py). Locate its `parameters=[{...}]` block and ensure these three keys are present (add if missing, leave others untouched):

```python
        parameters=[{
            'drone_count':         LaunchConfiguration('drone_count'),
            'default_pattern':     LaunchConfiguration('default_pattern'),
            'morph_duration_sec':  LaunchConfiguration('morph_duration_sec'),
            # ... 기존 parameters 유지 (publish_rate, obstacle_stop_dist 등)
        }],
```

If the existing file uses a different parameter-passing style (e.g. arguments list), apply the same three keys using that style.

- [ ] **Step 4: Sanity check launch file syntax**

```bash
python3 -m py_compile airsim_ros2_bridge/launch/aerion_phase4_formation.launch.py
echo "exit=$?"
```

Expected: `exit=0`.

- [ ] **Step 5: Verify launch description resolves (no runtime)**

```bash
cd ~/workspace/projects/aerion-airsim
python3 -c "
import sys
sys.path.insert(0, 'airsim_ros2_bridge/launch')
import aerion_phase4_formation
ld = aerion_phase4_formation.generate_launch_description()
print('actions:', len(ld.describe_sub_entities()))
"
```

Expected: prints `actions: N` with N > 0, no exception.

- [ ] **Step 6: Commit**

```bash
git add airsim_ros2_bridge/launch/aerion_phase4_formation.launch.py
git commit -m "feat(launch): expose default_pattern + morph_duration_sec + drone_count default 3 (phase4-Δ)

aerion_phase4_formation.launch.py gains three new launch arguments that
wire into formation_node parameters. drone_count default is shifted
from 5 to 3 to match the new baseline.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Add morphing-cycle demo mode to aerion_formation CLI

**Files:**
- Modify: `airsim_ros2_bridge/scripts/aerion_formation.py`

- [ ] **Step 1: Inspect current CLI structure**

```bash
cat airsim_ros2_bridge/scripts/aerion_formation.py
```

Identify the argparse setup. Locate `--demo` (or equivalent) argument if it exists, or note where to add it.

- [ ] **Step 2: Add `morphing-cycle` demo handler**

In the script, add this function near the other demo functions (or as a new section if none exist):

```python
def demo_morphing_cycle(args):
    """Phase 4-Δ 데모: TRIANGLE → V3 → COLUMN → DIAMOND3 순환.

    각 패턴 hold-sec 초 유지 후 다음 패턴으로 전환 (morphing 은 노드 측이 처리).
    /aerion/formation/pattern 토픽에 std_msgs/String 으로 발행만 한다.
    """
    import time
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    patterns = args.patterns.split(',') if args.patterns else \
               ['TRIANGLE', 'V3', 'COLUMN', 'DIAMOND3']
    hold_sec = float(args.hold_sec)

    rclpy.init()
    node = Node('aerion_formation_morphing_cycle')
    pub = node.create_publisher(String, '/aerion/formation/pattern', 10)
    # publisher 가 graph 에 보일 때까지 잠시 대기
    time.sleep(1.0)

    try:
        idx = 0
        while rclpy.ok():
            name = patterns[idx % len(patterns)]
            msg = String()
            msg.data = name
            pub.publish(msg)
            node.get_logger().info(f'Pattern → {name} (hold {hold_sec:.1f}s)')
            # hold_sec 동안 spin (Ctrl+C 응답 위해 짧은 sleep 반복)
            t_end = time.time() + hold_sec
            while time.time() < t_end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
            idx += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 3: Register `morphing-cycle` in the argparse**

Find the existing `argparse.ArgumentParser` setup. Add (or extend the existing `--demo` choices to include) `morphing-cycle`:

```python
    parser.add_argument(
        '--demo',
        choices=['legacy-5pattern', 'morphing-cycle'],
        default=None,
        help='Pre-built demo sequence. morphing-cycle = TRIANGLE→V3→COLUMN→DIAMOND3 순환 (phase4-Δ).',
    )
    parser.add_argument(
        '--patterns',
        default='TRIANGLE,V3,COLUMN,DIAMOND3',
        help='Comma-separated pattern names for --demo morphing-cycle (overridable).',
    )
    parser.add_argument(
        '--hold-sec',
        default=10.0,
        type=float,
        help='Seconds to hold each pattern before next transition.',
    )
```

If the existing CLI does not have `--demo` at all, add the argparse skeleton:

```python
import argparse


def main():
    parser = argparse.ArgumentParser(description='AERION formation CLI (phase4-Δ).')
    parser.add_argument('--demo', choices=['morphing-cycle'], default='morphing-cycle')
    parser.add_argument('--patterns', default='TRIANGLE,V3,COLUMN,DIAMOND3')
    parser.add_argument('--hold-sec', default=10.0, type=float)
    args = parser.parse_args()

    if args.demo == 'morphing-cycle':
        demo_morphing_cycle(args)
    else:
        raise SystemExit(f'Unknown --demo: {args.demo}')


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Verify script syntax**

```bash
python3 -m py_compile airsim_ros2_bridge/scripts/aerion_formation.py
echo "exit=$?"
```

Expected: `exit=0`.

- [ ] **Step 5: Verify CLI parses without invoking ros2**

```bash
python3 airsim_ros2_bridge/scripts/aerion_formation.py --help
```

Expected: usage text including `--demo morphing-cycle`, `--patterns`, `--hold-sec`.

- [ ] **Step 6: Commit**

```bash
git add airsim_ros2_bridge/scripts/aerion_formation.py
git commit -m "feat(cli): add morphing-cycle demo to aerion_formation (phase4-Δ)

Publishes TRIANGLE → V3 → COLUMN → DIAMOND3 on /aerion/formation/pattern
with configurable hold duration (--hold-sec, default 10s). The
formation_node handles the actual morphing transition.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Verification runbook docs

**Files:**
- Create: `docs/phase4_delta_runbook.md`

- [ ] **Step 1: Create runbook**

Create `docs/phase4_delta_runbook.md`:

````markdown
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
- [ ] `morph_progress` 가 0.0 → 1.0 단조 증가 (10 hz 샘플 기준 +0.066±0.01)
- [ ] 패턴 전환 중 setpoint 가 끊김 없이 20Hz 발행 (`ros2 topic hz /drone1/mavros/setpoint_position/local`)

## B. 순환 데모 (CLI)

위 Terminal 1, 2 그대로 두고:

```bash
ros2 run airsim_ros2_bridge aerion_formation --demo morphing-cycle \
  --patterns TRIANGLE,V3,COLUMN,DIAMOND3 --hold-sec 10.0
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

- [ ] `status` 즉시 `OBSTACLE_HOVER` 로 전이, `morph_progress` 값 동결 (변화 없음)
- [ ] 다시 정상 range publish 시 `MORPHING` 복귀, `morph_progress` 일시정지된 지점부터 계속 증가

## D. rosbag 분석 (시각화)

```bash
ros2 bag record /drone1/mavros/local_position/pose \
                /drone2/mavros/local_position/pose \
                /drone3/mavros/local_position/pose \
                /aerion/formation/{pattern,leader_pose,status,morph_progress} \
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

## 알려진 한계

- AirSim RPC `async_run(4)` (issue #1538) 한계로 drone_count=5 시 동시 takeoff 안정성 부족 — 별도 Phase 4-η 에서 Colosseum fork 패치로 해결 예정.
- multi-axis 회피 미지원 — Phase 4-ε 별도 작업.
- CARLA dynamic actor leader 추종 미지원 — Phase 4-ζ 별도 작업.
````

- [ ] **Step 2: Commit**

```bash
git add docs/phase4_delta_runbook.md
git commit -m "docs(runbook): phase4-Δ verification runbook (smoke + cycle + obstacle pause + bag analysis + 5-drone regression)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Final: Push all 5 commits to origin

- [ ] **Step 1: Verify all 5 commits land on top of origin**

```bash
cd ~/workspace/projects/aerion-airsim
git log --oneline origin/branc/airsim-ros2-bridge..HEAD
```

Expected: 5 lines, the commits from Task 1 → Task 5 in order.

- [ ] **Step 2: Push**

```bash
git push git@github.com:swjo0330/Aerion-Airsim.git branc/airsim-ros2-bridge
```

Expected: `origin/branc/airsim-ros2-bridge..HEAD` updates cleanly (no rejection).

- [ ] **Step 3: Final verification**

```bash
git log --oneline -7
```

Expected: 5 new phase4-Δ commits + the prior `d503c0f` spec commit + the prior `85b8e96` phase5 commit (or whatever the previous head was).
