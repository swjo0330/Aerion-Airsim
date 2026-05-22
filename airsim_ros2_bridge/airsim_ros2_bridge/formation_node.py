"""AERION Phase 4-Δ — 3대 드론 동적 포메이션 노드 (Virtual Leader + morphing).

## 설계 철학

1. **Aerion-Airsim 단일 패키지 기조 유지** — 새 패키지 분리 없이 `airsim_ros2_bridge` 안에 추가.
2. **단일 책임 원칙** — 이 노드는 "포메이션 패턴 → per-drone setpoint 발행" 만 담당.
   arm/offboard/takeoff/land는 별도 헬퍼(`scripts/airsim_arm_all.py`, `scripts/airsim_land_all.py`).
3. **재사용 가능 모듈** — 진입 함수는 `scripts/aerion_formation.py`가 호출. 다른 launch에서도 import 가능.
4. **거리센서 회피 hook 포함** — 가장 가까운 장애물 거리 < `obstacle_stop_distance` 이면 전체 hover.
   Phase 5 PX4 시 동일 인터페이스 유지하기 위해 setpoint publish는 그대로, hover만 leader_pose 정지로.

## 토픽 인터페이스 (외부 노출 표준)

### Subscribe (외부 → 노드)

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/aerion/formation/pattern` | std_msgs/String | "TRIANGLE"/"V3"/"COLUMN"/"DIAMOND3" (drone_count=5 시 LINE/DIAMOND/ARROW/V/ECHELON v2 fallback) |
| `/aerion/formation/leader_pose` | geometry_msgs/PoseStamped | 가상 leader 위치 (ENU, 외부 mission planner) |
| `/drone{N}/range/front` | sensor_msgs/Range | 회피 hook 입력 (전방만 사용, 좌/우는 별도 검토) |
| `/drone{N}/mavros/local_position/pose` | geometry_msgs/PoseStamped | 도착 감지 입력 (transition 종료 판정) |

### Publish (노드 → 외부)

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/drone{N}/mavros/setpoint_position/local` | geometry_msgs/PoseStamped | 각 드론의 목표 위치 (20Hz) |
| `/aerion/formation/status` | std_msgs/String | "SETTLING" / "STABLE" / "OBSTACLE_HOVER" / "NO_LEADER" / "MORPHING" |
| `/aerion/formation/morph_progress` | std_msgs/Float32 | 패턴 전환 진행도 (0.0~1.0, 20Hz, idle=1.0) |

## 동작 상태 (status 토픽 값은 모두 UPPERCASE)

- `NO_LEADER`      : leader_pose 미수신 (기본 leader_pose 사용 안 함, setpoint publish 정지)
- `SETTLING`       : 패턴 전환 직후 → 모든 드론이 목표 위치 ±tolerance 안에 들어올 때까지
- `STABLE`         : 모든 드론이 목표 위치 안정
- `OBSTACLE_HOVER` : 어느 드론이라도 전방 거리 < obstacle_stop_distance → 전체 hover
- `MORPHING`       : 패턴 전환 보간 중 (linear interp, default 1.5s; OBSTACLE_HOVER 진입 시 일시정지, 해소 후 resume).

## 좌표

REP-105 ENU (x=East, y=North, z=Up). offset 테이블도 동일.

## 의존성

- ROS2 Humble + rclpy
- 외부에서 각 드론이 armed + offboard mode 인 상태여야 함 (이 노드 책임 외)

## 변경 이력

- 2026-05-18 v1: 80줄 초기 골격 (Aerostack2 대신 자체 구현 결정 직후)
- 2026-05-18 v2: 모듈화, 회피 hook, 도착 감지, status publisher, 상태 머신, 한국어 주석 강화
- 2026-05-22 v3: Phase 4-Δ — 3대 baseline + 4패턴 (TRIANGLE/V3/COLUMN/DIAMOND3) + MorphState + MORPHING FSM. 5대는 deprecated fallback.
"""

import math
import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, String


# ---------- 상수 ----------

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

# 도착 감지 허용 오차 (m). 모든 드론의 위치 오차가 이 값보다 작으면 STABLE.
ARRIVAL_TOLERANCE_M = 0.6

# 거리센서 회피 임계값 (m). 어느 드론이라도 전방 거리 < 이 값 → 전체 hover.
DEFAULT_OBSTACLE_STOP_DISTANCE = 1.0

# Setpoint publish 주기 (Hz). bridge 의 cmd_vel 모드와 정합.
DEFAULT_PUBLISH_RATE_HZ = 20.0


# ---------- 데이터 클래스 ----------

@dataclass
class DroneState:
    """각 드론의 런타임 상태 (구독 토픽 캐시)."""
    pose_xyz: Optional[Tuple[float, float, float]] = None  # ENU
    range_front: float = float('inf')                       # m, +∞ = 미수신 또는 max

    @property
    def has_pose(self) -> bool:
        return self.pose_xyz is not None


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

    def __post_init__(self):
        if self.duration <= 0:
            raise ValueError(f'MorphState.duration must be > 0, got {self.duration}')

    def progress(self, now_sec: float) -> float:
        if self.paused_at_sec is not None:
            # paused 중: paused_at 시점의 elapsed 로 동결. paused_elapsed 는 누적된 pause 시간이므로 elapsed 에서 차감.
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


# ---------- 유틸 ----------

def yaw_from_quaternion(q) -> float:
    """ROS quaternion (w,x,y,z) → ENU yaw radian. roll/pitch 0 가정."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def rotate_xy(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
    """leader yaw에 따라 offset (dx, dy)를 회전. ENU 평면, CCW 양수."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx - s * dy, s * dx + c * dy


def distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


# ---------- 노드 ----------

class FormationNode(Node):
    """3대 드론 동적 포메이션 컨트롤러 (Phase 4-Δ; drone_count 1~5 지원, 5는 deprecated v2 fallback).

    각 드론에 ros2 publisher/subscriber를 생성하고, 20Hz timer로 setpoint 발행.
    런타임 상태는 _state[drone_idx] 에 캐시. _lock으로 동시성 격리.

    Parameters (ros2 launch override 가능):
      - drone_count          : 1~5 (기본 3, Phase 4-Δ; 5는 deprecated)
      - publish_rate         : Hz (기본 20.0)
      - obstacle_stop_dist   : m (기본 1.0)
      - enable_arrival_check : bool (기본 True, 도착 감지 → status STABLE)
      - default_altitude     : ENU z (기본 5.0m, leader_pose 미수신 시 기본 고도)
      - morph_duration_sec   : 패턴 전환 보간 시간 (기본 1.5초)
    """

    def __init__(self):
        super().__init__('aerion_formation')

        # ----- 파라미터 -----
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
        if self._morph_duration <= 0:
            raise ValueError(f'morph_duration_sec must be > 0, got {self._morph_duration}')

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

        # ----- 런타임 상태 -----
        # leader_pose 가 한 번이라도 publish 되면 _leader_received=True.
        # False면 NO_LEADER 상태로 setpoint 발행 정지 (안전).
        self._leader_received = False
        self._leader = (0.0, 0.0, self._default_alt, 0.0)  # (x, y, z, yaw)
        self._state: Dict[int, DroneState] = {i: DroneState() for i in range(1, self._n + 1)}
        self._fsm = 'NO_LEADER'  # NO_LEADER | SETTLING | STABLE | OBSTACLE_HOVER | MORPHING
        self._morph: Optional[MorphState] = None
        self._lock = threading.RLock()

        # ----- Pub/Sub -----
        self._setpoint_pubs: Dict[int, any] = {}
        for i in range(1, self._n + 1):
            self._setpoint_pubs[i] = self.create_publisher(
                PoseStamped, f'/drone{i}/mavros/setpoint_position/local', 10
            )
            self.create_subscription(
                PoseStamped, f'/drone{i}/mavros/local_position/pose',
                lambda msg, idx=i: self._on_pose(idx, msg), 10
            )
            self.create_subscription(
                Range, f'/drone{i}/range/front',
                lambda msg, idx=i: self._on_range(idx, msg), 10
            )

        self.create_subscription(String, '/aerion/formation/pattern',
                                 self._on_pattern, 10)
        self.create_subscription(PoseStamped, '/aerion/formation/leader_pose',
                                 self._on_leader, 10)
        self._status_pub = self.create_publisher(String, '/aerion/formation/status', 10)
        self._morph_pub = self.create_publisher(Float32, '/aerion/formation/morph_progress', 10)

        # ----- 메인 timer -----
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self._status_timer = self.create_timer(0.5, self._publish_status)  # 2Hz 상태 보고

        self.get_logger().info(
            f'AERION formation node ready. drones={self._n}, pattern={self._pattern}, '
            f'rate={self._rate}Hz, obstacle_stop={self._obstacle_stop_dist}m, '
            f'morph_duration={self._morph_duration}s, table='
            f'{"FORMATIONS_5(deprecated)" if self._n == 5 else "FORMATIONS_3"}'
        )

    # ----- 구독 콜백 -----

    def _on_pattern(self, msg: String):
        name = msg.data.strip().upper()
        if name not in self._formations:
            self.get_logger().warn(f'Unknown formation pattern: {msg.data!r}')
            return
        with self._lock:
            old = self._pattern
            if name == old:
                return    # no-op
            # leader 미수신 상태에서는 morph 시작 안 함. 패턴 target 만 갱신 — leader 수신 시 SETTLING 으로 진입.
            if self._fsm == 'NO_LEADER':
                self._pattern = name
                self.get_logger().info(f'Formation pattern (pre-leader): {old} -> {name}')
                return
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            # 이미 morph 중일 때 새 pattern 요청이 들어오면 src=현재 committed pattern (= old).
            # 의도된 동작: 트레이드오프 — 현재 보간 위치가 아닌 마지막 committed pattern 위치에서 새 morph 시작.
            # 시각적으로 약간의 setpoint jump 가능 (특히 morph 진행도 >50% 일 때). Phase 4-η 후속 작업 후보.
            self._morph = MorphState(
                src_pattern=old,
                dst_pattern=name,
                t0_sec=now_sec,
                duration=self._morph_duration,
            )
            self._fsm = 'MORPHING'
        self.get_logger().info(f'Formation pattern: {old} -> {name} (morphing {self._morph_duration:.1f}s)')

    def _on_leader(self, msg: PoseStamped):
        yaw = yaw_from_quaternion(msg.pose.orientation)
        with self._lock:
            self._leader = (msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z, yaw)
            if not self._leader_received:
                self._leader_received = True
                # defensive: leader 미수신 상태에서 _morph 가 살아있으면 정리 (B1 guard 이후엔 사실 발생 안 함).
                if self._morph is not None:
                    self._morph = None
                self._fsm = 'SETTLING'
                self.get_logger().info('Leader pose received → SETTLING')

    def _on_pose(self, drone_idx: int, msg: PoseStamped):
        with self._lock:
            self._state[drone_idx].pose_xyz = (
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            )

    def _on_range(self, drone_idx: int, msg: Range):
        with self._lock:
            self._state[drone_idx].range_front = float(msg.range)

    # ----- 메인 tick -----

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
                # idle (morph 없음): morph_progress=1.0. OBSTACLE_HOVER 진입 시 _tick 이
                # early-return 하므로 그 동안엔 stream 끊김 (의도된 동작 — pose update 와 동기).
                morph_progress_value = 1.0

            lx, ly, lz, yaw = self._leader
            fsm_snapshot = self._fsm    # publish/도착 감지 블록에서 사용할 일관 스냅샷

        # publish (lock 밖에서 publish — 콜백 backpressure 영향 최소화).
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

        # 도착 감지 → STABLE 전환 (morph 종료 직후의 SETTLING 도 같은 로직 적용).
        # FSM 스냅샷 사용 — lock 밖이므로 self._fsm 직접 읽으면 콜백 mid-mutation 위험.
        if self._enable_arrival and fsm_snapshot == 'SETTLING':
            if self._all_arrived(offsets):
                with self._lock:
                    self._fsm = 'STABLE'
                self.get_logger().info('All drones arrived → STABLE')

    def _publish_hover_setpoints(self):
        """OBSTACLE_HOVER 상태에서 각 드론의 현재 위치를 setpoint로 발행 (제자리 유지)."""
        stamp = self.get_clock().now().to_msg()
        for i in range(1, self._n + 1):
            s = self._state[i]
            if not s.has_pose:
                continue
            x, y, z = s.pose_xyz
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            self._setpoint_pubs[i].publish(ps)

    def _all_arrived(self, offsets) -> bool:
        """모든 드론이 목표 위치 ±ARRIVAL_TOLERANCE_M 안에 들어왔는지."""
        lx, ly, lz, yaw = self._leader
        for i in range(1, self._n + 1):
            s = self._state[i]
            if not s.has_pose:
                return False
            dx, dy, dz = offsets[i - 1]
            rx, ry = rotate_xy(dx, dy, yaw)
            target = (lx + rx, ly + ry, lz + dz)
            if distance(s.pose_xyz, target) > ARRIVAL_TOLERANCE_M:
                return False
        return True

    # ----- 상태 보고 -----

    def _publish_status(self):
        with self._lock:
            state = self._fsm
        msg = String()
        msg.data = state
        self._status_pub.publish(msg)


# ---------- 진입 함수 ----------

def main(args=None):
    rclpy.init(args=args)
    node = FormationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
