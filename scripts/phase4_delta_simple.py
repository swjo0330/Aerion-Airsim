#!/usr/bin/env python3
"""AERION Phase 4-Δ — 3-drone formation runner (AirSim 공식 패턴).

설계 원칙 (이전 누적 시도들의 실패에서 학습):
  - AirSim Colosseum PythonClient/multirotor/multi_agent_drone.py 와 동일한 호출 패턴
    그대로 사용. 추가 방어 (yaw_mode 명시, cancelLastTask, armDisarm 반복, future
    timeout polling) 는 모두 제거 — 공식 예제에 없는 호출은 부작용만 만들었음.
  - settings.json 의 ClockType: SteppableClock 은 PX4 lockstep 전용. SimpleFlight 는
    default ScalableClock 을 써야 시뮬 시간이 자동 흐름. (이전엔 SteppableClock 이라
    .moveToPositionAsync future 가 영원히 완료 안 되는 hang 가능성)
  - 매 단계 명시적 print + getMultirotorState() 로 위치 보고 → 정확한 진단.

흐름:
  1. confirmConnection
  2. 3대 enableApiControl + armDisarm + takeoffAsync 동시 → 모두 .join()
  3. 3대 자기 spawn 위 5m 로 안정 hover → .join() + sleep(3)
  4. 패턴 순환: 3대 동시 moveToPositionAsync → 모두 .join() → sleep(hold)
  5. Ctrl+C → land + disarm

Usage:
  python3 scripts/phase4_delta_simple.py
  python3 scripts/phase4_delta_simple.py --hold-sec 10 --velocity 1.5
"""

import argparse
import itertools
import json
import math
import signal
import sys
import time

import airsim
from airsim.types import DrivetrainType, Pose, Quaternionr, Vector3r, YawMode


# settings.json (sf_3drones_phase4_delta.json) 의 vehicle spawn 위치 (NED, world frame).
# AirSim multi-vehicle 의 moveToPositionAsync / getMultirotorState 가 **vehicle local
# frame** 을 사용하므로 world ↔ local 변환은 이 spawn 좌표를 빼고/더해서 수행한다.
SPAWN_NED = {
    'drone1': (0.0,  0.0, 0.0),
    'drone2': (5.0,  0.0, 0.0),
    'drone3': (10.0, 0.0, 0.0),
}

# leader 기준 상대 offset (NED, world frame). AirSim 의 world 가 NED → z 음수 = 위.
FORMATIONS_NED = {
    # 충돌/교차를 줄이기 위해 좌우 역할이 유지되는 대형 위주.
    'TRIANGLE':     [(0.0,  0.0, 0.0), (-4.4, -3.0, 0.0), (-4.4,  3.0, 0.0)],
    'DIAMOND3':     [(0.0,  0.0, -0.8), (-4.0, -2.4, 0.0), (-4.0,  2.4, 0.0)],
    'LINE_H':       [(-5.2, 0.0, 0.0), (0.0, 0.0, 0.0), ( 5.2, 0.0, 0.0)],
    'ECHELON_R':    [(-2.0, -3.8, 0.0), (0.0, 0.0, 0.0), ( 2.0,  3.8, 0.0)],
    'ECHELON_L':    [(-2.0,  3.8, 0.0), (0.0, 0.0, 0.0), ( 2.0, -3.8, 0.0)],
    'CIRCLE':       [(0.0, 0.0, 0.0), (-4.4, 0.0, 0.0), ( 4.4, 0.0, 0.0)],
}

# 5-drone 전용 대형 (배치 최적화/충돌 회피를 위해 충분한 간격 유지).
FORMATIONS_NED_5 = {
    'TRIANGLE': [
        (0.0, 0.0, 0.0),
        (-4.6, -3.2, 0.0),
        (-4.6, 3.2, 0.0),
        (-9.0, -1.8, 0.0),
        (-9.0, 1.8, 0.0),
    ],
    'DIAMOND3': [
        (0.0, 0.0, -0.8),
        (-4.4, -2.6, 0.0),
        (-4.4, 2.6, 0.0),
        (-8.6, 0.0, 0.0),
        (-11.8, 0.0, 0.2),
    ],
    'LINE_H': [
        (-10.4, 0.0, 0.0),
        (-5.2, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (5.2, 0.0, 0.0),
        (10.4, 0.0, 0.0),
    ],
    'ECHELON_R': [
        (-4.0, -7.6, 0.0),
        (-2.0, -3.8, 0.0),
        (0.0, 0.0, 0.0),
        (2.0, 3.8, 0.0),
        (4.0, 7.6, 0.0),
    ],
    'ECHELON_L': [
        (-4.0, 7.6, 0.0),
        (-2.0, 3.8, 0.0),
        (0.0, 0.0, 0.0),
        (2.0, -3.8, 0.0),
        (4.0, -7.6, 0.0),
    ],
    'CIRCLE': [
        (0.0, 0.0, 0.0),
        (0.0, 5.6, 0.0),
        (5.3, 1.7, 0.0),
        (3.3, -4.5, 0.0),
        (-3.3, -4.5, 0.0),
    ],
}

# 6-drone 전용 대형 (5대 대비 간격/대칭 강화).
FORMATIONS_NED_6 = {
    'TRIANGLE': [
        (0.0, 0.0, 0.0),
        (-6.6, -5.2, 0.0),
        (-6.6, 5.2, 0.0),
        (-12.8, -3.4, 0.0),
        (-12.8, 3.4, 0.0),
        (-18.6, 0.0, 0.0),
    ],
    'DIAMOND3': [
        (0.0, 0.0, -0.6),
        (-6.0, -4.0, 0.0),
        (-6.0, 4.0, 0.0),
        (-12.0, 0.0, 0.0),
        (-18.0, -4.0, 0.2),
        (-18.0, 4.0, 0.2),
    ],
    'LINE_H': [
        (-16.0, 0.0, 0.0),
        (-9.6, 0.0, 0.0),
        (-3.2, 0.0, 0.0),
        (3.2, 0.0, 0.0),
        (9.6, 0.0, 0.0),
        (16.0, 0.0, 0.0),
    ],
    'ECHELON_R': [
        (-7.5, -13.5, 0.0),
        (-4.5, -8.1, 0.0),
        (-1.5, -2.7, 0.0),
        (1.5, 2.7, 0.0),
        (4.5, 8.1, 0.0),
        (7.5, 13.5, 0.0),
    ],
    'ECHELON_L': [
        (-7.5, 13.5, 0.0),
        (-4.5, 8.1, 0.0),
        (-1.5, 2.7, 0.0),
        (1.5, -2.7, 0.0),
        (4.5, -8.1, 0.0),
        (7.5, -13.5, 0.0),
    ],
    'CIRCLE': [
        (0.0, 7.0, 0.0),
        (6.1, 3.5, 0.0),
        (6.1, -3.5, 0.0),
        (0.0, -7.0, 0.0),
        (-6.1, -3.5, 0.0),
        (-6.1, 3.5, 0.0),
    ],
}


def get_formations_for_count(drone_count: int) -> dict[str, list[tuple[float, float, float]]]:
    if drone_count >= 6:
        return FORMATIONS_NED_6
    if drone_count >= 5:
        return FORMATIONS_NED_5
    return FORMATIONS_NED

ALTITUDE_M = 9.0   # 이륙 고도 (ENU). AirSim NED z = -ALTITUDE_M


def pos_str(p) -> str:
    return f'({p.x_val:+.2f}, {p.y_val:+.2f}, {p.z_val:+.2f})'


def world_pos(state, v_name: str) -> tuple[float, float, float]:
    """vehicle local frame → world NED 변환 (spawn 좌표 더하기)."""
    p = state.kinematics_estimated.position
    sx, sy, sz = SPAWN_NED.get(v_name, (0.0, 0.0, 0.0))
    return p.x_val + sx, p.y_val + sy, p.z_val + sz


def teleport_world_ned(client: airsim.MultirotorClient, v_name: str, x: float, y: float, z: float):
    """World NED 목표를 vehicle local NED 로 바꿔 simSetVehiclePose 적용."""
    sx, sy, sz = SPAWN_NED.get(v_name, (0.0, 0.0, 0.0))
    local_x = x - sx
    local_y = y - sy
    local_z = z - sz
    pose = Pose(Vector3r(local_x, local_y, local_z), Quaternionr(0.0, 0.0, 0.0, 1.0))
    client.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=v_name)


def move_world_ned(
    client: airsim.MultirotorClient,
    v_name: str,
    x: float,
    y: float,
    z: float,
    velocity: float,
):
    """World NED 목표를 vehicle local NED 로 변환해 moveToPositionAsync 발행."""
    sx, sy, sz = SPAWN_NED.get(v_name, (0.0, 0.0, 0.0))
    local_x = x - sx
    local_y = y - sy
    local_z = z - sz
    return client.moveToPositionAsync(local_x, local_y, local_z, velocity, vehicle_name=v_name)


def world_path_to_local_points(v_name: str, pts_world: list[tuple[float, float, float]]) -> list[Vector3r]:
    sx, sy, sz = SPAWN_NED.get(v_name, (0.0, 0.0, 0.0))
    return [Vector3r(x - sx, y - sy, z - sz) for x, y, z in pts_world]


def dist3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def enforce_min_separation_points(
    points: list[tuple[str, float, float, float]],
    min_sep_m: float,
    iterations: int = 3,
) -> list[tuple[str, float, float, float]]:
    """동일 스텝 target 간 최소 간격 강제 (xy 평면 repulsion)."""
    out = points[:]
    for _ in range(iterations):
        changed = False
        for i in range(len(out)):
            vi, xi, yi, zi = out[i]
            for j in range(i + 1, len(out)):
                vj, xj, yj, zj = out[j]
                dx = xi - xj
                dy = yi - yj
                d = math.sqrt(dx * dx + dy * dy)
                if d < 1e-3:
                    dx, dy, d = 1.0, 0.0, 1.0
                if d < min_sep_m:
                    push = 0.5 * (min_sep_m - d)
                    ux, uy = dx / d, dy / d
                    xi += ux * push
                    yi += uy * push
                    xj -= ux * push
                    yj -= uy * push
                    out[i] = (vi, xi, yi, zi)
                    out[j] = (vj, xj, yj, zj)
                    changed = True
        if not changed:
            break
    return out


def min_pairwise_distance(points: list[tuple[float, float, float]]) -> float:
    if len(points) < 2:
        return 9999.0
    m = 9999.0
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            dx = points[i][0] - points[j][0]
            dy = points[i][1] - points[j][1]
            dz = points[i][2] - points[j][2]
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d < m:
                m = d
    return m


def min_pairwise_distance_xy(points: list[tuple[float, float, float]]) -> float:
    if len(points) < 2:
        return 9999.0
    m = 9999.0
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            dx = points[i][0] - points[j][0]
            dy = points[i][1] - points[j][1]
            d = math.sqrt(dx * dx + dy * dy)
            if d < m:
                m = d
    return m


def has_path_crossings_xy(
    drone_names: list[str],
    curr_world: dict[str, tuple[float, float, float]],
    slot_map: dict[str, int],
    leader_end_xy: tuple[float, float],
    next_offsets: list[tuple[float, float, float]],
) -> bool:
    n = min(len(drone_names), len(next_offsets))
    segs: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for i, v in enumerate(drone_names[:n]):
        sidx = slot_map.get(v, i)
        if sidx >= len(next_offsets) or v not in curr_world:
            continue
        ox, oy, _oz = next_offsets[sidx]
        start = (curr_world[v][0], curr_world[v][1])
        end = (leader_end_xy[0] + ox, leader_end_xy[1] + oy)
        segs.append((start, end))
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            if segments_intersect_2d(segs[i][0], segs[i][1], segs[j][0], segs[j][1]):
                return True
    return False


def enforce_vertical_separation_for_close_xy(
    points: list[tuple[str, float, float, float]],
    min_xy_m: float,
    min_dz_m: float,
    dz_gain_per_xy_m: float = 0.30,
    iterations: int = 2,
) -> list[tuple[str, float, float, float]]:
    """XY가 가까운 쌍은 z를 벌려 수직 충돌을 회피."""
    out = points[:]
    if len(out) < 2 or min_dz_m <= 0.0:
        return out
    for _ in range(max(1, iterations)):
        changed = False
        for i in range(len(out)):
            vi, xi, yi, zi = out[i]
            for j in range(i + 1, len(out)):
                vj, xj, yj, zj = out[j]
                dxy = math.sqrt((xi - xj) * (xi - xj) + (yi - yj) * (yi - yj))
                if dxy < min_xy_m:
                    dz_req = min_dz_m + max(0.0, (min_xy_m - dxy)) * max(0.0, dz_gain_per_xy_m)
                    dz = abs(zi - zj)
                    if dz < dz_req:
                        push = 0.5 * (dz_req - dz)
                        # 이름 정렬로 일관된 상하 배치 유지(진동/flip 방지)
                        if vi <= vj:
                            zi -= push
                            zj += push
                        else:
                            zi += push
                            zj -= push
                        out[i] = (vi, xi, yi, zi)
                        out[j] = (vj, xj, yj, zj)
                        changed = True
        if not changed:
            break
    return out


def rotate_offset_xy(ox: float, oy: float, deg: float) -> tuple[float, float]:
    r = math.radians(deg)
    c = math.cos(r)
    s = math.sin(r)
    return (ox * c - oy * s, ox * s + oy * c)


def symmetric_layer_bias(i: int, n: int) -> float:
    """인덱스를 대칭 bias [-1, +1]로 정규화."""
    if n <= 1:
        return 0.0
    raw = float(i) - 0.5 * float(n - 1)
    scale = max(1e-6, 0.5 * float(n - 1))
    return max(-1.0, min(1.0, raw / scale))


def centered_layer_index(i: int, n: int) -> float:
    if n <= 1:
        return 0.0
    return float(i) - 0.5 * float(n - 1)


def stabilize_target_z(
    current_z: float,
    target_z: float,
    deadband_m: float,
    max_step_m: float,
) -> float:
    dz = target_z - current_z
    if abs(dz) <= max(0.0, deadband_m):
        return current_z
    step = max(0.05, max_step_m)
    dz = max(-step, min(step, dz))
    return current_z + dz


def stabilize_target_z_with_state(
    v_name: str,
    current_z: float,
    target_z: float,
    deadband_m: float,
    max_step_m: float,
    hold_band_m: float,
    cooldown_sec: float,
    now_t: float,
    state: dict[str, dict[str, float]],
) -> float:
    ent = state.get(v_name, {})
    last_cmd_z = ent.get("last_cmd_z", current_z)
    last_cmd_t = ent.get("last_cmd_t", -1e9)

    # 짧은 시간 내 재명령 억제: 이전 z command 유지
    if now_t - last_cmd_t < max(0.0, cooldown_sec):
        return last_cmd_z

    dz = target_z - current_z
    if abs(dz) <= max(0.0, deadband_m):
        # 충분히 가까우면 현재 z 유지 (명령 갱신 없음)
        return current_z

    # 이전 명령점 주변에서는 불필요한 반대 보정 억제
    if abs(target_z - last_cmd_z) <= max(0.0, hold_band_m):
        cmd = last_cmd_z
    else:
        step = max(0.05, max_step_m)
        cmd = current_z + max(-step, min(step, dz))

    state[v_name] = {"last_cmd_z": cmd, "last_cmd_t": now_t}
    return cmd


def execute_rigid_formation_action(
    client: airsim.MultirotorClient,
    drone_names: list[str],
    slot_map: dict[str, int],
    base_offsets: list[tuple[float, float, float]],
    leader_start_xy: tuple[float, float],
    leader_end_xy: tuple[float, float],
    leader_z: float,
    segment_sec: float,
    tick_hz: float,
    velocity: float,
    command_stagger_ms: int,
    min_separation_m: float,
    collision_hard_min_m: float,
    collision_mitigation_rounds: int,
    transition_z_lift: float,
    z_lift_gain: float,
    safety_floor_z: float,
    safety_recovery_z: float,
    action_name: str,
    rotate_deg: float = 0.0,
    z_collision_xy_thresh_m: float = 4.6,
    z_collision_min_dz_m: float = 1.25,
    z_collision_dz_gain_per_xy_m: float = 0.35,
) -> tuple[bool, float]:
    steps = max(3, int(segment_sec * max(tick_hz, 1.0)))
    per_drone_world_path: dict[str, list[tuple[float, float, float]]] = {v: [] for v in drone_names}
    path_vel_scale = 1.0
    z_lift_scale = 1.0
    worst_min_d = 9999.0

    for mitigate_round in range(collision_mitigation_rounds + 1):
        for v in per_drone_world_path:
            per_drone_world_path[v].clear()
        worst_min_d = 9999.0
        for s in range(steps + 1):
            t = s / steps
            et = smoothstep(t)
            lx = lerp(leader_start_xy[0], leader_end_xy[0], et)
            ly = lerp(leader_start_xy[1], leader_end_xy[1], et)
            step_targets: list[tuple[str, float, float, float]] = []
            for i, v in enumerate(drone_names):
                sidx = slot_map.get(v, i)
                if sidx >= len(base_offsets):
                    continue
                bo = base_offsets[sidx]
                rox, roy = rotate_offset_xy(bo[0], bo[1], rotate_deg * et)
                bias = symmetric_layer_bias(i, len(drone_names))
                # 전환 동안에는 z 레이어를 항상 유지해 XY 교차 시 수직 충돌을 회피.
                oz = bo[2] + bias * transition_z_lift * z_lift_scale * z_lift_gain
                tx = lx + rox
                ty = ly + roy
                tz = leader_z + oz
                step_targets.append((v, tx, ty, tz))
            step_targets = enforce_min_separation_points(step_targets, min_separation_m)
            step_targets = enforce_vertical_separation_for_close_xy(
                step_targets,
                z_collision_xy_thresh_m,
                z_collision_min_dz_m,
                dz_gain_per_xy_m=z_collision_dz_gain_per_xy_m,
            )
            step_xyz = [(tx, ty, tz) for _v, tx, ty, tz in step_targets]
            dmin = min_pairwise_distance(step_xyz)
            if dmin < worst_min_d:
                worst_min_d = dmin
            for v, tx, ty, tz in step_targets:
                per_drone_world_path[v].append((tx, ty, tz))

        if worst_min_d >= collision_hard_min_m:
            if mitigate_round > 0:
                print(
                    f'    [action-collision-mitigated] {action_name} round={mitigate_round}, '
                    f'min_step_dist={worst_min_d:.2f}m'
                )
            break
        if mitigate_round < collision_mitigation_rounds:
            path_vel_scale *= 0.86
            z_lift_scale *= 1.30
            print(
                f'    [action-collision-risk] {action_name} min_step_dist={worst_min_d:.2f}m '
                f'< {collision_hard_min_m:.2f}m -> retry(round={mitigate_round+1})'
            )

    # 회전 액션은 "하나씩 움직이는 느낌"을 없애기 위해
    # 드론별 경로 길이에 맞춘 속도로 moveOnPathAsync를 동시 발행한다.
    # (모든 드론이 같은 segment_sec 안에 시작/종료)
    if abs(rotate_deg) > 1e-6:
        rot_fs = []
        for v in drone_names:
            pts_world = per_drone_world_path.get(v, [])
            if len(pts_world) < 2:
                continue
            path_len = 0.0
            for i in range(1, len(pts_world)):
                path_len += dist3(pts_world[i - 1], pts_world[i])
            # 같은 segment_sec 내 완료되도록 드론별 속도 산출.
            sync_vel = path_len / max(0.5, segment_sec)
            sync_vel = max(1.4, min(6.2, sync_vel))
            local_pts = world_path_to_local_points(v, pts_world)
            f = client.moveOnPathAsync(
                local_pts,
                sync_vel,
                timeout_sec=max(8.0, segment_sec * 2.0),
                drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=YawMode(is_rate=False, yaw_or_rate=0.0),
                lookahead=-1.0,
                adaptive_lookahead=1.0,
                vehicle_name=v,
            )
            rot_fs.append((v, f))
        for _v, f in rot_fs:
            f.join()
        enforce_safety_floor(client, drone_names, safety_floor_z, safety_recovery_z, velocity)
    else:
        path_vel = max(1.3, velocity * 0.78 * path_vel_scale)
        fs = []
        for v in drone_names:
            pts_world = per_drone_world_path.get(v, [])
            if len(pts_world) < 2:
                continue
            local_pts = world_path_to_local_points(v, pts_world)
            f = client.moveOnPathAsync(
                local_pts,
                path_vel,
                timeout_sec=max(8.0, segment_sec * 2.0),
                drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=YawMode(is_rate=False, yaw_or_rate=0.0),
                lookahead=-1.0,
                adaptive_lookahead=1.0,
                vehicle_name=v,
            )
            fs.append((v, f))
            if command_stagger_ms > 0:
                time.sleep(command_stagger_ms / 1000.0)
        for _v, f in fs:
            f.join()
    enforce_safety_floor(client, drone_names, safety_floor_z, safety_recovery_z, velocity)
    return (worst_min_d >= collision_hard_min_m), worst_min_d


def _orient(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> float:
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def segments_intersect_2d(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> bool:
    o1 = _orient(a0[0], a0[1], a1[0], a1[1], b0[0], b0[1])
    o2 = _orient(a0[0], a0[1], a1[0], a1[1], b1[0], b1[1])
    o3 = _orient(b0[0], b0[1], b1[0], b1[1], a0[0], a0[1])
    o4 = _orient(b0[0], b0[1], b1[0], b1[1], a1[0], a1[1])
    return (o1 * o2 < 0.0) and (o3 * o4 < 0.0)


def build_collision_aware_slot_assignment(
    drone_names: list[str],
    curr_world: dict[str, tuple[float, float, float]],
    leader_end_xy: tuple[float, float],
    next_offsets: list[tuple[float, float, float]],
    min_sep_m: float,
    crossing_penalty: float,
    prev_slot_map: dict[str, int] | None = None,
    continuity_penalty: float = 4.0,
) -> dict[str, int]:
    """드론 -> next formation slot index 할당 최적화 (교차/근접 비용 포함)."""
    n = min(len(drone_names), len(next_offsets))
    names = drone_names[:n]
    slots = list(range(n))
    best_cost = float("inf")
    best = {names[i]: i for i in range(n)}
    for perm in itertools.permutations(slots):
        # perm[k] = names[k]에 할당될 slot index
        endpoints: list[tuple[str, tuple[float, float, float]]] = []
        cost = 0.0
        for k, v in enumerate(names):
            sidx = perm[k]
            ox, oy, oz = next_offsets[sidx]
            tx = leader_end_xy[0] + ox
            ty = leader_end_xy[1] + oy
            tz = -ALTITUDE_M + oz
            cw = curr_world[v]
            dx = cw[0] - tx
            dy = cw[1] - ty
            dz = cw[2] - tz
            cost += math.sqrt(dx * dx + dy * dy + dz * dz)
            if prev_slot_map is not None and v in prev_slot_map and prev_slot_map[v] != sidx:
                cost += continuity_penalty
            endpoints.append((v, (tx, ty, tz)))

        # pairwise crossing + endpoint separation penalty
        for i in range(n):
            va, ta = endpoints[i]
            sa = curr_world[va]
            for j in range(i + 1, n):
                vb, tb = endpoints[j]
                sb = curr_world[vb]
                if segments_intersect_2d((sa[0], sa[1]), (ta[0], ta[1]), (sb[0], sb[1]), (tb[0], tb[1])):
                    cost += crossing_penalty
                ddx = ta[0] - tb[0]
                ddy = ta[1] - tb[1]
                d = math.sqrt(ddx * ddx + ddy * ddy)
                if d < min_sep_m:
                    cost += (min_sep_m - d) * (crossing_penalty * 0.6)

        if cost < best_cost:
            best_cost = cost
            best = {names[k]: perm[k] for k in range(n)}
    return best


def estimate_leader_from_actual(
    client: airsim.MultirotorClient,
    drone_names: list[str],
    slot_map: dict[str, int],
    offsets: list[tuple[float, float, float]],
    leader_z: float,
) -> tuple[float, float, float] | None:
    est = []
    for i, v in enumerate(drone_names):
        sidx = slot_map.get(v, i)
        if sidx >= len(offsets):
            continue
        nox, noy, noz = offsets[sidx]
        st = client.getMultirotorState(vehicle_name=v)
        wx, wy, wz = world_pos(st, v)
        # world = leader + offset  => leader = world - offset
        est.append((wx - nox, wy - noy, wz - noz))
    if not est:
        return None
    lx = sum(p[0] for p in est) / len(est)
    ly = sum(p[1] for p in est) / len(est)
    lz = sum(p[2] for p in est) / len(est)
    # z는 기존 고도 기준 유지
    return (lx, ly, leader_z if abs(lz - leader_z) < 1.5 else lz)


def update_spawn_from_settings(settings_path: str, drone_names: list[str]) -> bool:
    try:
        with open(settings_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        vehicles = data.get("Vehicles", {})
        loaded = 0
        for v in drone_names:
            info = vehicles.get(v)
            if not isinstance(info, dict):
                continue
            x = float(info.get("X", 0.0))
            y = float(info.get("Y", 0.0))
            z = float(info.get("Z", 0.0))
            SPAWN_NED[v] = (x, y, z)
            loaded += 1
        return loaded > 0
    except Exception:
        return False


def settle_to_pattern(
    client: airsim.MultirotorClient,
    drone_names: list[str],
    pattern_name: str,
    leader_xyz: tuple[float, float, float],
    velocity: float,
    tol_m: float,
    tol_xy_m: float,
    tol_z_m: float,
    retries: int,
    z_lift_for_settle: float = 0.0,
    min_separation_m: float = 4.0,
    crossing_penalty: float = 20.0,
) -> tuple[bool, dict[str, tuple[float, float, float]]]:
    formations = get_formations_for_count(len(drone_names))
    offsets = formations.get(pattern_name, [])
    targets: dict[str, tuple[float, float, float]] = {}
    lx, ly, lz = leader_xyz
    if len(drone_names) >= 6 and len(offsets) >= len(drone_names):
        curr_world: dict[str, tuple[float, float, float]] = {}
        for v in drone_names:
            st = client.getMultirotorState(vehicle_name=v)
            curr_world[v] = world_pos(st, v)
        slot_map = build_collision_aware_slot_assignment(
            drone_names,
            curr_world,
            (lx, ly),
            offsets,
            min_separation_m,
            crossing_penalty,
            prev_slot_map=None,
            continuity_penalty=0.0,
        )
        print(f'    [pre-settle-plan] optimized slot_map={slot_map}')
        for i, v in enumerate(drone_names):
            sidx = slot_map.get(v, i)
            if sidx >= len(offsets):
                continue
            ox, oy, _oz = offsets[sidx]
            # 6대 pre-settle은 최종 z를 당기지 않고 XY 역할만 맞춘다.
            targets[v] = (lx + ox, ly + oy, curr_world[v][2])

        max_xy_err = 0.0
        for v, (tx, ty, _tz) in targets.items():
            sx, sy, _sz = curr_world[v]
            max_xy_err = max(max_xy_err, math.hypot(sx - tx, sy - ty))
        needs_stage = max_xy_err > max(2.5, tol_xy_m * 1.8)
        if needs_stage:
            # 큰 재진입은 모범사례(Hungarian assignment + altitude layering)처럼
            # 각 기체를 고유 고도 레인으로 분리한 뒤 XY를 이동시킨다.
            lane_gap = max(0.85, min(1.25, z_lift_for_settle if z_lift_for_settle > 0.01 else 0.85))
            stage_fs = []
            for i, v in enumerate(drone_names):
                if v not in targets:
                    continue
                sx, sy, sz = curr_world[v]
                tx, ty, tz = targets[v]
                lane_z = lz + centered_layer_index(i, len(drone_names)) * lane_gap
                pts_world = [
                    (sx, sy, sz),
                    (sx, sy, lane_z),
                    (tx, ty, lane_z),
                    (tx, ty, tz),
                ]
                f = client.moveOnPathAsync(
                    world_path_to_local_points(v, pts_world),
                    max(1.6, velocity * 0.62),
                    timeout_sec=max(8.0, 2.0 * len(pts_world)),
                    drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=YawMode(is_rate=False, yaw_or_rate=0.0),
                    lookahead=-1.0,
                    adaptive_lookahead=1.0,
                    vehicle_name=v,
                )
                stage_fs.append((v, f))
                time.sleep(0.016)
            if stage_fs:
                print(f'    [pre-settle-stage] lane_gap={lane_gap:.2f}m, max_xy_err={max_xy_err:.2f}m, paths={len(stage_fs)}')
            for _v, f in stage_fs:
                f.join()
    else:
        for i, v in enumerate(drone_names):
            if i >= len(offsets):
                continue
            ox, oy, oz = offsets[i]
            targets[v] = (lx + ox, ly + oy, lz + oz)

    for attempt in range(1, retries + 1):
        if len(drone_names) >= 6:
            fs = []
            for v, (tx, ty, tz) in targets.items():
                fs.append((v, move_world_ned(client, v, tx, ty, tz, max(1.4, velocity * 0.70))))
            for _v, f in fs:
                f.join()
        # 5대 이하 초기 정렬은 2단계 접근(고도 분리 -> 최종 z 정렬)로 충돌 리스크를 낮춘다.
        elif z_lift_for_settle > 0.01:
            names_sorted = sorted(targets.keys())
            center = 0.5 * (len(names_sorted) - 1)
            layer_amp = max(0.25, min(1.2, z_lift_for_settle))
            fs_l1 = []
            for idx, v in enumerate(names_sorted):
                tx, ty, tz = targets[v]
                tz_layer = tz + (idx - center) * layer_amp
                fs_l1.append((v, move_world_ned(client, v, tx, ty, tz_layer, max(1.5, velocity * 0.72))))
            for _v, f in fs_l1:
                f.join()
            fs = []
            for v, (tx, ty, tz) in targets.items():
                fs.append((v, move_world_ned(client, v, tx, ty, tz, max(1.5, velocity * 0.82))))
            for _v, f in fs:
                f.join()
        else:
            fs = []
            for v, (tx, ty, tz) in targets.items():
                fs.append((v, move_world_ned(client, v, tx, ty, tz, velocity)))
            for _v, f in fs:
                f.join()
        all_ok = True
        for v, (tx, ty, tz) in targets.items():
            reached, pos, err, err_xy, err_z = verify_target(
                client, v, (tx, ty, tz), tol_m, tol_xy_m, tol_z_m
            )
            if len(drone_names) >= 6:
                reached = err_xy <= tol_xy_m
            if not reached:
                all_ok = False
                wx, wy, wz = pos
                print(
                    f'    [pre-settle-miss] {v} world=({wx:+.2f},{wy:+.2f},{wz:+.2f}) '
                    f'target=({tx:+.2f},{ty:+.2f},{tz:+.2f}) err={err:.3f}m '
                    f'xy={err_xy:.3f} z={err_z:.3f} attempt={attempt}/{retries}'
                )
        if all_ok:
            print(f'    [pre-settle-ok] {pattern_name} aligned in {attempt}/{retries}')
            return True, targets
    return False, targets


def verify_target(
    client: airsim.MultirotorClient,
    v_name: str,
    target: tuple[float, float, float],
    tol_m: float,
    tol_xy_m: float,
    tol_z_m: float,
) -> tuple[bool, tuple[float, float, float], float, float, float]:
    state = client.getMultirotorState(vehicle_name=v_name)
    wx, wy, wz = world_pos(state, v_name)
    dx = wx - target[0]
    dy = wy - target[1]
    dz = wz - target[2]
    err_xy = math.sqrt(dx * dx + dy * dy)
    err_z = abs(dz)
    err = dist3((wx, wy, wz), target)
    reached = (err <= tol_m) or (err_xy <= tol_xy_m and err_z <= tol_z_m)
    return reached, (wx, wy, wz), err, err_xy, err_z


def enforce_safety_floor(
    client: airsim.MultirotorClient,
    drone_names: list[str],
    safety_floor_z: float,
    recovery_z: float,
    velocity: float,
) -> int:
    """지면 관통(월드 NED z가 floor보다 커짐) 시 즉시 복구."""
    recovered = 0
    for v in drone_names:
        state = client.getMultirotorState(vehicle_name=v)
        wx, wy, wz = world_pos(state, v)
        if wz > safety_floor_z:
            # 물리 붕괴 시에는 move보다 pose 복구가 확실함.
            teleport_world_ned(client, v, wx, wy, recovery_z)
            move_world_ned(client, v, wx, wy, recovery_z, max(1.5, velocity * 0.6))
            recovered += 1
            print(
                f'    [safety-floor] {v} z={wz:+.2f} > floor({safety_floor_z:+.2f}) '
                f'-> recover z={recovery_z:+.2f}'
            )
    return recovered


def calc_look_quaternion(
    from_xyz: tuple[float, float, float],
    to_xyz: tuple[float, float, float],
) -> Quaternionr:
    dx = to_xyz[0] - from_xyz[0]
    dy = to_xyz[1] - from_xyz[1]
    dz = to_xyz[2] - from_xyz[2]
    yaw = math.atan2(dy, dx)
    horizontal = max(1e-3, math.sqrt(dx * dx + dy * dy))
    pitch = math.atan2(-dz, horizontal)
    return airsim.to_quaternion(pitch, 0.0, yaw)


def get_swarm_center(
    client: airsim.MultirotorClient,
    drone_names: list[str],
) -> tuple[float, float, float] | None:
    points = []
    for v in drone_names:
        st = client.getMultirotorState(vehicle_name=v)
        points.append(world_pos(st, v))
    if not points:
        return None
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    cz = sum(p[2] for p in points) / len(points)
    return (cx, cy, cz)


def update_third_person_follow_camera(
    client: airsim.MultirotorClient,
    drone_names: list[str],
    follow_vehicle: str,
    camera_name: str,
    distance_m: float,
    height_m: float,
    lookahead_m: float,
) -> None:
    center = get_swarm_center(client, drone_names)
    if center is None:
        return

    anchor_st = client.getMultirotorState(vehicle_name=follow_vehicle)
    ax, ay, az = world_pos(anchor_st, follow_vehicle)
    vx = center[0] - ax
    vy = center[1] - ay
    n = math.sqrt(vx * vx + vy * vy)
    if n < 1e-3:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = vx / n, vy / n

    tx = center[0] + ux * lookahead_m
    ty = center[1] + uy * lookahead_m
    tz = center[2]
    cam_x = center[0] - ux * distance_m
    cam_y = center[1] - uy * distance_m
    cam_z = center[2] - height_m
    q = calc_look_quaternion((cam_x, cam_y, cam_z), (tx, ty, tz))

    sx, sy, sz = SPAWN_NED.get(follow_vehicle, (0.0, 0.0, 0.0))
    local_pose = Pose(Vector3r(cam_x - sx, cam_y - sy, cam_z - sz), q)
    client.simSetCameraPose(camera_name, local_pose, vehicle_name=follow_vehicle)


def update_external_follow_camera(
    client: airsim.MultirotorClient,
    drone_names: list[str],
    camera_name: str,
    distance_m: float,
    height_m: float,
    lookahead_m: float,
    view_azimuth_deg: float,
    view_elevation_bias_m: float,
    state: dict,
    center_smooth_alpha: float,
    dir_smooth_alpha: float,
    yaw_rate_limit_deg_s: float,
    motion_deadband_mps: float,
    dt_s: float,
) -> None:
    points = []
    for v in drone_names:
        st = client.getMultirotorState(vehicle_name=v)
        points.append(world_pos(st, v))
    if not points:
        return
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    cz = sum(p[2] for p in points) / len(points)
    center = (cx, cy, cz)

    prev_scenter = state.get("prev_scenter")
    if prev_scenter is None:
        scenter = center
    else:
        a = max(0.0, min(1.0, center_smooth_alpha))
        scenter = (
            lerp(prev_scenter[0], center[0], a),
            lerp(prev_scenter[1], center[1], a),
            lerp(prev_scenter[2], center[2], a),
        )

    prev_center = state.get("prev_center")
    if prev_center is None:
        vx, vy = 1.0, 0.0
    else:
        vx = scenter[0] - prev_center[0]
        vy = scenter[1] - prev_center[1]

    speed_xy = math.sqrt(vx * vx + vy * vy) / max(1e-3, dt_s)
    if speed_xy < motion_deadband_mps and state.get("heading") is not None:
        raw_heading = state["heading"]
    else:
        n = math.sqrt(vx * vx + vy * vy)
        if n < 1e-3:
            raw_heading = state.get("heading", 0.0)
        else:
            raw_heading = math.atan2(vy / n, vx / n)

    prev_heading = state.get("heading")
    if prev_heading is None:
        heading = raw_heading
    else:
        d = wrap_pi(raw_heading - prev_heading)
        max_step = math.radians(max(1.0, yaw_rate_limit_deg_s)) * max(1e-3, dt_s)
        d = max(-max_step, min(max_step, d))
        heading = wrap_pi(prev_heading + d * max(0.0, min(1.0, dir_smooth_alpha)))
    ux, uy = math.cos(heading), math.sin(heading)

    az = math.radians(view_azimuth_deg)
    # 이동방향 벡터 (ux,uy)를 기준으로 카메라 방향을 회전.
    # az=180이면 정후방 추적, 135면 후좌측 대각, 90이면 측면.
    rx = ux * math.cos(az) - uy * math.sin(az)
    ry = ux * math.sin(az) + uy * math.cos(az)
    rn = math.sqrt(rx * rx + ry * ry)
    if rn < 1e-3:
        rx, ry = -ux, -uy
    else:
        rx, ry = rx / rn, ry / rn

    tx = scenter[0] + ux * lookahead_m
    ty = scenter[1] + uy * lookahead_m
    tz = scenter[2]
    cam_x = scenter[0] + rx * distance_m
    cam_y = scenter[1] + ry * distance_m
    cam_z = scenter[2] - height_m + view_elevation_bias_m
    q = calc_look_quaternion((cam_x, cam_y, cam_z), (tx, ty, tz))
    world_pose = Pose(Vector3r(cam_x, cam_y, cam_z), q)
    client.simSetCameraPose(camera_name, world_pose)
    state["prev_center"] = scenter
    state["prev_scenter"] = scenter
    state["heading"] = heading


def update_center_follow_camera(
    client: airsim.MultirotorClient,
    drone_names: list[str],
    camera_name: str,
    back_distance_m: float,
    up_height_m: float,
    lookahead_m: float,
    state: dict,
    center_smooth_alpha: float,
    dir_smooth_alpha: float,
    yaw_rate_limit_deg_s: float,
    motion_deadband_mps: float,
    dt_s: float,
) -> None:
    center = get_swarm_center(client, drone_names)
    if center is None:
        return
    prev_scenter = state.get("prev_scenter")
    if prev_scenter is None:
        scenter = center
    else:
        a = max(0.0, min(1.0, center_smooth_alpha))
        scenter = (
            lerp(prev_scenter[0], center[0], a),
            lerp(prev_scenter[1], center[1], a),
            lerp(prev_scenter[2], center[2], a),
        )

    prev_center = state.get("prev_center")
    if prev_center is None:
        vx, vy = 1.0, 0.0
    else:
        vx = scenter[0] - prev_center[0]
        vy = scenter[1] - prev_center[1]

    speed_xy = math.sqrt(vx * vx + vy * vy) / max(1e-3, dt_s)
    if speed_xy < motion_deadband_mps and state.get("heading") is not None:
        raw_heading = state["heading"]
    else:
        n = math.sqrt(vx * vx + vy * vy)
        if n < 1e-3:
            raw_heading = state.get("heading", 0.0)
        else:
            raw_heading = math.atan2(vy / n, vx / n)

    prev_heading = state.get("heading")
    if prev_heading is None:
        heading = raw_heading
    else:
        d = wrap_pi(raw_heading - prev_heading)
        max_step = math.radians(max(1.0, yaw_rate_limit_deg_s)) * max(1e-3, dt_s)
        d = max(-max_step, min(max_step, d))
        heading = wrap_pi(prev_heading + d * max(0.0, min(1.0, dir_smooth_alpha)))
    ux, uy = math.cos(heading), math.sin(heading)

    cam_x = scenter[0] - ux * back_distance_m
    cam_y = scenter[1] - uy * back_distance_m
    cam_z = scenter[2] - up_height_m
    tx = scenter[0] + ux * lookahead_m
    ty = scenter[1] + uy * lookahead_m
    tz = scenter[2]
    q = calc_look_quaternion((cam_x, cam_y, cam_z), (tx, ty, tz))
    client.simSetCameraPose(camera_name, Pose(Vector3r(cam_x, cam_y, cam_z), q))
    state["prev_center"] = scenter
    state["prev_scenter"] = scenter
    state["heading"] = heading


def update_observer_camera(
    client: airsim.MultirotorClient,
    drone_names: list[str],
    camera_name: str,
    observer_xyz: tuple[float, float, float],
) -> None:
    center = get_swarm_center(client, drone_names)
    if center is None:
        return
    ox, oy, oz = observer_xyz
    q = calc_look_quaternion((ox, oy, oz), center)
    client.simSetCameraPose(camera_name, Pose(Vector3r(ox, oy, oz), q))


# NOTE:
# 이 Colosseum 빌드의 multi-vehicle 에서는 moveToPositionAsync 가 vehicle local NED 로
# 해석되는 케이스가 재현됨. 따라서 world target 을 직접 넣지 않고 move_world_ned()
# 를 통해 spawn 기준 local 로 변환해 호출한다.


def main() -> int:
    p = argparse.ArgumentParser(description='AERION Phase 4-Δ — clean AirSim formation runner.')
    p.add_argument('--drones', type=int, default=3)
    p.add_argument('--ip', default='127.0.0.1')
    p.add_argument('--port', type=int, default=41451)
    p.add_argument('--settings-json', default='',
                   help='AirSim settings.json 경로. 지정 시 Vehicles spawn(X/Y/Z)로 SPAWN_NED 갱신')
    p.add_argument('--patterns', default='TRIANGLE,LINE_H,ECHELON_R,CIRCLE,DIAMOND3,ECHELON_L',
                   help='쉼표 구분 패턴 순환 시퀀스')
    p.add_argument('--hold-sec', type=float, default=8.0,
                   help='패턴 도달 후 hover hold 시간 (s)')
    p.add_argument('--leader-x', type=float, default=None,
                   help='leader x (NED, m). 미지정 시 drone1 spawn x 사용')
    p.add_argument('--leader-y', type=float, default=None,
                   help='leader y (NED, m). 미지정 시 drone1 spawn y 사용')
    p.add_argument('--velocity', type=float, default=3.0,
                   help='moveToPosition 속도 (m/s)')
    p.add_argument('--prefix', default='drone',
                   help='vehicle key prefix (settings.json 의 vehicle key 와 일치)')
    p.add_argument('--cycles', type=int, default=0,
                   help='패턴 순환 횟수. 0 = 무한 (Ctrl+C 종료)')
    p.add_argument('--control-mode', choices=['move', 'teleport', 'showcase'], default='move',
                   help='move=step moveToPositionAsync, teleport=simSetVehiclePose, showcase=연속 이동+포메이션 변환')
    p.add_argument('--tol-m', type=float, default=0.35,
                   help='목표 도달 판정 오차 허용치 (3D 거리, m)')
    p.add_argument('--tol-xy-m', type=float, default=1.10,
                   help='목표 도달 판정 수평(xy) 오차 허용치 (m)')
    p.add_argument('--tol-z-m', type=float, default=0.75,
                   help='목표 도달 판정 수직(z) 오차 허용치 (m)')
    p.add_argument('--teleport-retries', type=int, default=3,
                   help='teleport 모드에서 목표 도달 검증 실패 시 재시도 횟수')
    p.add_argument('--fail-on-miss', action='store_true',
                   help='도달 오차 허용치를 벗어나면 즉시 종료')
    p.add_argument('--fallback-teleport-on-miss', action='store_true',
                   help='move 모드에서 미도달 시 해당 드론만 teleport fallback 수행')
    p.add_argument('--segment-sec', type=float, default=8.0,
                   help='showcase 모드에서 세그먼트(포메이션 1회 전환) 시간')
    p.add_argument('--transition-while-moving', action='store_true',
                   help='포메이션 전환과 리더 이동을 동시에 수행 (기본은 분리: 전환 후 액션)')
    p.add_argument('--tick-hz', type=float, default=4.5,
                   help='showcase 모드 setpoint 발행 주기 (Hz)')
    p.add_argument('--command-stagger-ms', type=int, default=25,
                   help='showcase 모드에서 드론별 명령 발행 간격 (ms)')
    p.add_argument('--capture-sec', type=float, default=3.5,
                   help='showcase 세그먼트 후 최종 포메이션 수렴 대기 시간 (s)')
    p.add_argument('--pre-settle', action='store_true',
                   help='showcase 세그먼트 시작 전 현재 패턴을 완전히 맞춘 뒤 전환')
    p.add_argument('--pre-settle-retries', type=int, default=3,
                   help='pre-settle 정렬 재시도 횟수')
    p.add_argument('--formation-hold-sec', type=float, default=6.0,
                   help='showcase 모드에서 일반 포메이션 유지 시간 (s)')
    p.add_argument('--circle-hold-sec', type=float, default=8.5,
                   help='showcase 모드에서 CIRCLE 포메이션 유지 시간 (s)')
    p.add_argument('--min-separation-m', type=float, default=4.0,
                   help='showcase 전환 중 드론 간 최소 xy 간격 (m)')
    p.add_argument('--transition-z-lift', type=float, default=0.9,
                   help='showcase 전환 중 인접 드론 충돌 회피용 고도 레이어 진폭 (m)')
    p.add_argument('--z-lift-gain', type=float, default=0.45,
                   help='전환 z 분리 적용 강도(0~1). 낮출수록 상하 출렁임 감소')
    p.add_argument('--z-align-sec', type=float, default=1.2,
                   help='포메이션 완성 직후 목표 z 재정렬(충돌회피 z 분리 해제) 유지 시간 (s)')
    p.add_argument('--z-command-deadband-m', type=float, default=0.22,
                   help='z 목표 오차가 이 값 이하면 z 재명령을 생략 (출렁임 감소)')
    p.add_argument('--z-command-max-step-m', type=float, default=0.30,
                   help='한 번의 명령에서 허용할 z 목표 변화량 최대치 (m)')
    p.add_argument('--z-command-hold-band-m', type=float, default=0.12,
                   help='이전 z command 주변에서는 재명령을 억제하는 히스테리시스 밴드 (m)')
    p.add_argument('--z-command-cooldown-sec', type=float, default=0.55,
                   help='z command 재발행 최소 간격 (s)')
    p.add_argument('--z-collision-xy-thresh-m', type=float, default=4.6,
                   help='전환 중 z 수직 분리 강제를 적용할 XY 거리 임계값 (m)')
    p.add_argument('--z-collision-min-dz-m', type=float, default=1.25,
                   help='전환 중 XY 근접쌍 최소 z 분리 거리 (m)')
    p.add_argument('--z-collision-dz-gain-per-xy-m', type=float, default=0.35,
                   help='XY가 더 가까울수록 추가로 요구하는 z 분리 기울기 (m/m)')
    p.add_argument('--collision-hard-min-m', type=float, default=2.6,
                   help='전환 경로 사전검사 최소 허용 거리(m). 미만이면 자동 완화 재시도')
    p.add_argument('--collision-mitigation-rounds', type=int, default=3,
                   help='충돌 위험 시 완화(감속/고도분리) 자동 재시도 횟수')
    p.add_argument('--ca-crossing-penalty', type=float, default=20.0,
                   help='충돌회피 슬롯할당 시 경로 교차 패널티 가중치')
    p.add_argument('--ca-continuity-penalty', type=float, default=6.0,
                   help='이전 세그먼트 대비 슬롯 변경 패널티(역할 뒤바뀜 억제)')
    p.add_argument('--role-assignment', choices=['fixed', 'optimized'], default='fixed',
                   help='fixed=드론 역할 고정(충돌안전 우선), optimized=이동량 최소 슬롯 재할당')
    p.add_argument('--ca-near-dist-m', type=float, default=6.0,
                   help='가까운 상태로 판단하는 pairwise 거리 임계값 (m)')
    p.add_argument('--ca-slowdown-factor', type=float, default=0.72,
                   help='근접 시 세그먼트 속도 배율(0~1)')
    p.add_argument('--reanchor-gain', type=float, default=0.85,
                   help='포메이션 종료 후 리더 재보정 반영 비율(0~1)')
    p.add_argument('--safety-floor-z', type=float, default=-0.20,
                   help='월드 NED 안전 바닥. z가 이 값보다 커지면(지면 아래) 즉시 복구')
    p.add_argument('--safety-recovery-z', type=float, default=-3.50,
                   help='safety-floor 발동 시 복구할 월드 NED z')
    p.add_argument('--third-person-follow', action='store_true',
                   help='3인칭 자동 추적 카메라 활성화 (follow vehicle 카메라를 포메이션 중심으로 이동)')
    p.add_argument('--third-person-camera-name', default='front_center',
                   help='3인칭 추적에 사용할 camera name')
    p.add_argument('--third-person-mode', choices=['vehicle', 'external', 'center_follow', 'observer'], default='external',
                   help='vehicle=드론 탑재, external=외부 추적, center_follow=중심점 추적, observer=고정 관찰')
    p.add_argument('--third-person-follow-vehicle', default='drone1',
                   help='3인칭 카메라를 탑재할 vehicle name')
    p.add_argument('--third-person-distance-m', type=float, default=14.0,
                   help='포메이션 중심 뒤쪽 거리 (m)')
    p.add_argument('--third-person-height-m', type=float, default=6.0,
                   help='포메이션 중심 위쪽 높이 (m, NED 기준 z 감소)')
    p.add_argument('--third-person-lookahead-m', type=float, default=2.0,
                   help='포메이션 진행 방향 전방 바라보기 거리 (m)')
    p.add_argument('--third-person-update-hz', type=float, default=8.0,
                   help='3인칭 추적 카메라 업데이트 주기 (Hz)')
    p.add_argument('--third-person-view-azimuth-deg', type=float, default=135.0,
                   help='external 모드 카메라 시점 각도(이동방향 기준, 도). 180=후방, 135=후좌측, 90=측면')
    p.add_argument('--third-person-view-elevation-bias-m', type=float, default=0.0,
                   help='external 모드 카메라 높이 보정 (m)')
    p.add_argument('--cam-center-smooth-alpha', type=float, default=0.22,
                   help='center/external 추적 중심점 저역통과 계수(0~1). 낮을수록 안정적')
    p.add_argument('--cam-dir-smooth-alpha', type=float, default=0.35,
                   help='카메라 진행방향 스무딩 계수(0~1). 낮을수록 회전이 완만')
    p.add_argument('--cam-yaw-rate-limit-deg-s', type=float, default=40.0,
                   help='카메라 yaw 최대 회전 속도 제한(도/초)')
    p.add_argument('--cam-motion-deadband-mps', type=float, default=0.35,
                   help='중심 이동 속도가 이 값 미만이면 heading 고정 (m/s)')
    p.add_argument('--observer-x', type=float, default=0.0,
                   help='observer 모드 카메라 월드 NED X')
    p.add_argument('--observer-y', type=float, default=0.0,
                   help='observer 모드 카메라 월드 NED Y')
    p.add_argument('--observer-z', type=float, default=-20.0,
                   help='observer 모드 카메라 월드 NED Z (음수일수록 높음)')
    p.add_argument('--post-action-enabled', action='store_true',
                   help='각 포메이션 도달 후 동적 액션(형상 유지 이동/회전) 실행')
    p.add_argument('--post-action-style', choices=['translate_only', 'mixed'], default='mixed',
                   help='translate_only=회전 없이 형상 유지 직진만, mixed=직진+패턴별 회전')
    p.add_argument('--post-action-segment-sec', type=float, default=4.5,
                   help='포메이션 후속 액션 1회 길이 (s)')
    p.add_argument('--post-line-translate-m', type=float, default=18.0,
                   help='후속: 형상 유지 상태로 전체 병진 이동 거리 (m)')
    p.add_argument('--post-rotate-deg-list', default='90',
                   help='TRIANGLE/DIAMOND3/CIRCLE 후속 회전 각도 리스트(도, 쉼표구분)')
    p.add_argument('--post-action-velocity-scale', type=float, default=1.08,
                   help='후속 액션 속도 배율 (기본 속도 대비)')
    p.add_argument('--post-action-pause-sec', type=float, default=2.5,
                   help='후속 액션 완료 후 정지 연출 시간 (s)')
    args = p.parse_args()

    drone_names = [f'{args.prefix}{i}' for i in range(1, args.drones + 1)]
    formations = get_formations_for_count(args.drones)
    if args.drones >= 6:
        # 6대 이상에서는 포메이션 정의 자체가 6m급 간격을 갖도록 강제한다.
        args.min_separation_m = max(args.min_separation_m, 6.2)
        args.transition_z_lift = min(args.transition_z_lift, 0.85)
        args.collision_hard_min_m = max(args.collision_hard_min_m, 3.6)
        args.ca_near_dist_m = max(args.ca_near_dist_m, 8.0)
        args.ca_slowdown_factor = min(args.ca_slowdown_factor, 0.70)
        args.command_stagger_ms = max(args.command_stagger_ms, 16)
        args.z_collision_min_dz_m = max(args.z_collision_min_dz_m, 1.35)
    if args.settings_json:
        ok_spawn = update_spawn_from_settings(args.settings_json, drone_names)
        if ok_spawn:
            print(f'[spawn] loaded from settings: {args.settings_json}')
            print(f'  SPAWN_NED={SPAWN_NED}')
        else:
            print(f'[spawn-warn] settings load failed, fallback SPAWN_NED={SPAWN_NED}')
    pattern_seq = [s.strip().upper() for s in args.patterns.split(',') if s.strip()]
    pattern_seq = [s for s in pattern_seq if s in formations]
    if not pattern_seq:
        print('ERROR: --patterns 에 유효한 패턴 없음. 가능: TRIANGLE,LINE_H,ECHELON_R,DIAMOND3,ECHELON_L,CIRCLE',
              file=sys.stderr)
        return 1
    post_rotate_deg_list = []
    for tok in args.post_rotate_deg_list.split(','):
        tok = tok.strip()
        if not tok:
            continue
        try:
            deg = abs(float(tok))
            if deg > 0.1:
                post_rotate_deg_list.append(deg)
        except ValueError:
            pass
    if not post_rotate_deg_list:
        post_rotate_deg_list = [90.0]
    # 회전은 한 방향 단일 액션만 수행 (왕복/다중 회전 방지)
    post_rotate_deg_list = [post_rotate_deg_list[0]]

    print('=' * 60)
    print('  AERION Phase 4-Δ Simple Runner (AirSim 공식 패턴)')
    print(f'    drones={drone_names}')
    print(f'    patterns={pattern_seq}')
    leader_x_cfg = args.leader_x
    leader_y_cfg = args.leader_y
    if leader_x_cfg is None or leader_y_cfg is None:
        s1 = SPAWN_NED.get(f'{args.prefix}1', (0.0, 0.0, 0.0))
        if leader_x_cfg is None:
            leader_x_cfg = s1[0]
        if leader_y_cfg is None:
            leader_y_cfg = s1[1]
    print(f'    leader NED=({leader_x_cfg}, {leader_y_cfg}, {-ALTITUDE_M})')
    print(f'    velocity={args.velocity} m/s, hold={args.hold_sec}s')
    print(f'    control_mode={args.control_mode}')
    print(f'    tol3d={args.tol_m}m, tol_xy={args.tol_xy_m}m, tol_z={args.tol_z_m}m')
    print(f'    teleport_retries={args.teleport_retries}, fail_on_miss={args.fail_on_miss}')
    print(f'    fallback_teleport_on_miss={args.fallback_teleport_on_miss}')
    print(f'    segment_sec={args.segment_sec}, tick_hz={args.tick_hz}, stagger_ms={args.command_stagger_ms}')
    print(f'    transition_while_moving={args.transition_while_moving}')
    print(f'    capture_sec={args.capture_sec}')
    print(f'    pre_settle={args.pre_settle}, pre_settle_retries={args.pre_settle_retries}')
    print(f'    formation_hold_sec={args.formation_hold_sec}, circle_hold_sec={args.circle_hold_sec}')
    print(
        f'    min_separation_m={args.min_separation_m}, transition_z_lift={args.transition_z_lift}, z_lift_gain={args.z_lift_gain}, '
        f'z_align_sec={args.z_align_sec}, z_command_deadband_m={args.z_command_deadband_m}, '
        f'z_command_max_step_m={args.z_command_max_step_m}, z_command_hold_band_m={args.z_command_hold_band_m}, '
        f'z_command_cooldown_sec={args.z_command_cooldown_sec}, z_collision_xy_thresh_m={args.z_collision_xy_thresh_m}, '
        f'z_collision_min_dz_m={args.z_collision_min_dz_m}, z_collision_dz_gain_per_xy_m={args.z_collision_dz_gain_per_xy_m}'
    )
    print(
        f'    collision_hard_min_m={args.collision_hard_min_m}, '
        f'collision_mitigation_rounds={args.collision_mitigation_rounds}'
    )
    print(
        f'    ca_crossing_penalty={args.ca_crossing_penalty}, ca_continuity_penalty={args.ca_continuity_penalty}, '
        f'ca_near_dist_m={args.ca_near_dist_m}, '
        f'ca_slowdown_factor={args.ca_slowdown_factor}'
    )
    print(f'    role_assignment={args.role_assignment}')
    print(f'    reanchor_gain={args.reanchor_gain}')
    print(f'    safety_floor_z={args.safety_floor_z}, safety_recovery_z={args.safety_recovery_z}')
    print(
        f'    post_action_enabled={args.post_action_enabled}, post_action_style={args.post_action_style}, '
        f'post_action_segment_sec={args.post_action_segment_sec}, '
        f'post_line_translate_m={args.post_line_translate_m}, post_rotate_deg_list={args.post_rotate_deg_list}, '
        f'post_action_velocity_scale={args.post_action_velocity_scale}, post_action_pause_sec={args.post_action_pause_sec}'
    )
    print(
        f'    third_person_follow={args.third_person_follow}, camera={args.third_person_camera_name}, '
        f'mode={args.third_person_mode}, follow_vehicle={args.third_person_follow_vehicle}, '
        f'distance={args.third_person_distance_m}, '
        f'height={args.third_person_height_m}, lookahead={args.third_person_lookahead_m}, '
        f'update_hz={args.third_person_update_hz}, view_azimuth_deg={args.third_person_view_azimuth_deg}, '
        f'view_elevation_bias_m={args.third_person_view_elevation_bias_m}, '
        f'cam_center_smooth_alpha={args.cam_center_smooth_alpha}, cam_dir_smooth_alpha={args.cam_dir_smooth_alpha}, '
        f'cam_yaw_rate_limit_deg_s={args.cam_yaw_rate_limit_deg_s}, cam_motion_deadband_mps={args.cam_motion_deadband_mps}, '
        f'observer=({args.observer_x}, {args.observer_y}, {args.observer_z})'
    )
    print('=' * 60)

    # ---- AirSim connect ----
    c = airsim.MultirotorClient(ip=args.ip, port=args.port)
    c.confirmConnection()
    listed = c.listVehicles()
    print(f'\n[connect] ping OK. vehicles in scene = {listed}')
    if not all(v in listed for v in drone_names):
        missing = [v for v in drone_names if v not in listed]
        print(f'WARN: {missing} 가 scene 에 없음. UE Stop/Play 다시 시도하거나 --prefix 확인.')
    if args.third_person_follow and args.third_person_mode == 'vehicle' and args.third_person_follow_vehicle not in listed:
        print(
            f'WARN: follow vehicle({args.third_person_follow_vehicle}) 가 scene 에 없음. '
            f'기본 {drone_names[0]} 으로 대체'
        )
        args.third_person_follow_vehicle = drone_names[0]

    cam_dt = 1.0 / max(1.0, args.third_person_update_hz)
    last_cam_t = 0.0
    cam_state = {"prev_center": None, "prev_scenter": None, "heading": None}

    def maybe_update_camera(force: bool = False):
        nonlocal last_cam_t
        if not args.third_person_follow:
            return
        now = time.time()
        if not force and now - last_cam_t < cam_dt:
            return
        try:
            if args.third_person_mode == 'external':
                update_external_follow_camera(
                    c,
                    drone_names,
                    args.third_person_camera_name,
                    args.third_person_distance_m,
                    args.third_person_height_m,
                    args.third_person_lookahead_m,
                    args.third_person_view_azimuth_deg,
                    args.third_person_view_elevation_bias_m,
                    cam_state,
                    args.cam_center_smooth_alpha,
                    args.cam_dir_smooth_alpha,
                    args.cam_yaw_rate_limit_deg_s,
                    args.cam_motion_deadband_mps,
                    cam_dt,
                )
            elif args.third_person_mode == 'center_follow':
                update_center_follow_camera(
                    c,
                    drone_names,
                    args.third_person_camera_name,
                    args.third_person_distance_m,
                    args.third_person_height_m,
                    args.third_person_lookahead_m,
                    cam_state,
                    args.cam_center_smooth_alpha,
                    args.cam_dir_smooth_alpha,
                    args.cam_yaw_rate_limit_deg_s,
                    args.cam_motion_deadband_mps,
                    cam_dt,
                )
            elif args.third_person_mode == 'observer':
                update_observer_camera(
                    c,
                    drone_names,
                    args.third_person_camera_name,
                    (args.observer_x, args.observer_y, args.observer_z),
                )
            else:
                update_third_person_follow_camera(
                    c,
                    drone_names,
                    args.third_person_follow_vehicle,
                    args.third_person_camera_name,
                    args.third_person_distance_m,
                    args.third_person_height_m,
                    args.third_person_lookahead_m,
                )
            last_cam_t = now
        except Exception as e:
            print(f'    [camera-warn] {e}')

    # ---- Cleanup ----
    def cleanup(_sig=None, _frame=None):
        print('\n[cleanup] land + disarm...')
        try:
            fs = [c.landAsync(vehicle_name=v) for v in drone_names]
            for f in fs:
                try: f.join()
                except: pass
        except Exception as e:
            print(f'  landAsync error: {e}')
        for v in drone_names:
            try:
                c.armDisarm(False, vehicle_name=v)
                c.enableApiControl(False, vehicle_name=v)
            except: pass
        print('[cleanup] 완료')
        sys.exit(0)
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ---- Step 1: enable + arm + takeoff (AirSim 공식 패턴) ----
    print(f'\n[Step 1] enableApiControl + armDisarm + takeoffAsync × {len(drone_names)}')
    for v in drone_names:
        c.enableApiControl(True, vehicle_name=v)
        c.armDisarm(True, vehicle_name=v)
        print(f'  {v} enableApi + arm OK')
    takeoff_fs = [(v, c.takeoffAsync(vehicle_name=v)) for v in drone_names]
    for v, f in takeoff_fs:
        f.join()
        state = c.getMultirotorState(vehicle_name=v)
        p = state.kinematics_estimated.position
        print(f'  {v} takeoff done. world NED = ({p.x_val:+.2f}, {p.y_val:+.2f}, {p.z_val:+.2f})')

    # ---- Step 2: 각 vehicle 의 world spawn 위 ALTITUDE_M 으로 안정 hover ----
    # moveToPositionAsync 는 이 빌드에서 local NED 로 해석되어 world->local 변환 필요.
    print(f'\n[Step 2] 각 vehicle 의 world spawn 위 {ALTITUDE_M}m 로 hover')
    if args.control_mode == 'teleport':
        for v in drone_names:
            sx, sy, _ = SPAWN_NED.get(v, (0.0, 0.0, 0.0))
            ok = False
            for attempt in range(1, args.teleport_retries + 1):
                teleport_world_ned(c, v, sx, sy, -ALTITUDE_M)
                time.sleep(0.05)
                state = c.getMultirotorState(vehicle_name=v)
                wx, wy, wz = world_pos(state, v)
                err = dist3((wx, wy, wz), (sx, sy, -ALTITUDE_M))
                if err <= args.tol_m:
                    ok = True
                    print(f'  {v} hover teleport ok. world NED = ({wx:+.2f}, {wy:+.2f}, {wz:+.2f}), err={err:.3f}m')
                    break
            if not ok:
                print(f'  [MISS] {v} hover teleport failed after {args.teleport_retries} tries')
                if args.fail_on_miss:
                    return 2
    else:
        hover_fs = []
        for v in drone_names:
            sx, sy, _ = SPAWN_NED.get(v, (0.0, 0.0, 0.0))
            f = move_world_ned(c, v, sx, sy, -ALTITUDE_M, args.velocity)
            hover_fs.append((v, f))
        for v, f in hover_fs:
            f.join()
            state = c.getMultirotorState(vehicle_name=v)
            wx, wy, wz = world_pos(state, v)
            print(f'  {v} hover. world NED = ({wx:+.2f}, {wy:+.2f}, {wz:+.2f})')
    time.sleep(3.0)
    maybe_update_camera(force=True)
    print('  안정 hover 3초 완료')

    # ---- Step 3: 패턴 순환 ----
    print(f'\n[Step 3] 패턴 순환 시작 (Ctrl+C 종료)')
    leader_x = leader_x_cfg
    leader_y = leader_y_cfg
    leader_z = -ALTITUDE_M
    idx = 0
    cycle = 0
    prev_slot_map: dict[str, int] | None = None
    z_cmd_state: dict[str, dict[str, float]] = {}
    stats: list[tuple[int, str, str, float, str]] = []
    while args.cycles == 0 or cycle < args.cycles:
        pat = pattern_seq[idx % len(pattern_seq)]
        offsets = formations[pat]
        print(f'\n  [pattern {idx+1} → {pat}]')
        enforce_safety_floor(
            c, drone_names, args.safety_floor_z, args.safety_recovery_z, args.velocity
        )
        maybe_update_camera()

        targets = []
        for i, v in enumerate(drone_names):
            if i >= len(offsets):
                continue
            ox, oy, oz = offsets[i]
            # world NED target (leader 기준). moveToPositionAsync 가 world frame 이므로
            # 변환 없이 그대로 사용.
            tx = leader_x + ox
            ty = leader_y + oy
            tz = leader_z + oz
            targets.append((v, tx, ty, tz))
            print(f'    {v} world target ({tx:+.2f}, {ty:+.2f}, {tz:+.2f})')

        # AirSim Issue #991: SimpleFlight 의 hover() 가 버그 — 호출 후 drone 추락.
        # AirSim Issue #4421: moveToPositionAsync 완료 후 drone shaky behavior.
        # → 도달 후 명령 stream 끊기면 SimpleFlight 가 hover 못함 → 추락.
        # 또한 enableApiControl(True) 매번 호출이 vehicle controller 를 disrupt →
        # 그 vehicle 만 추락 ("하나씩 추락" 패턴).
        #
        # Fix: enableApiControl 재호출 제거 + hold 동안 1Hz 로 같은 target 으로
        # moveToPositionAsync 를 .join() 없이 재발행 → 명령 stream 유지 → drone 안정.
        #
        # 명령 stream 이 유지되면 API control 도 release 안 됨 → drone2/3 freeze 도 회피.
        if args.control_mode == 'teleport':
            for v, tx, ty, tz in targets:
                ok = False
                last_err = -1.0
                last_pos = (0.0, 0.0, 0.0)
                for attempt in range(1, args.teleport_retries + 1):
                    teleport_world_ned(c, v, tx, ty, tz)
                    time.sleep(0.05)
                    reached, pos, last_err, err_xy, err_z = verify_target(
                        c, v, (tx, ty, tz), args.tol_m, args.tol_xy_m, args.tol_z_m
                    )
                    wx, wy, wz = pos
                    last_pos = pos
                    if reached:
                        ok = True
                        print(
                            f'    [teleport-ok] {v} world=({wx:+.2f}, {wy:+.2f}, {wz:+.2f}) '
                            f'target=({tx:+.2f}, {ty:+.2f}, {tz:+.2f}) err={last_err:.3f}m '
                            f'attempt={attempt}/{args.teleport_retries}'
                        )
                        stats.append((idx + 1, pat, v, last_err, 'teleport'))
                        break
                if not ok:
                    print(
                        f'    [teleport-miss] {v} target=({tx:+.2f}, {ty:+.2f}, {tz:+.2f}) '
                        f'err={last_err:.3f}m after {args.teleport_retries} tries'
                    )
                    stats.append((idx + 1, pat, v, last_err, 'teleport_miss'))
                    if args.fail_on_miss:
                        return 2
            print(f'    [hold] {args.hold_sec}s 유지')
            time.sleep(args.hold_sec)
            maybe_update_camera(force=True)
        elif args.control_mode == 'move':
            send_fs = []
            for v, tx, ty, tz in targets:
                f = move_world_ned(c, v, tx, ty, tz, args.velocity)
                send_fs.append((v, f, (tx, ty, tz)))
            print(f'    [send] {len(send_fs)} 드론 명령 발행. .join() 대기...')

            for v, f, _ in send_fs:
                f.join()
                state = c.getMultirotorState(vehicle_name=v)
                wx, wy, wz = world_pos(state, v)
                print(f'    [arrived] {v} world = ({wx:+.2f}, {wy:+.2f}, {wz:+.2f})')

            misses = []
            for v, _, (tx, ty, tz) in send_fs:
                reached, pos, err, err_xy, err_z = verify_target(
                    c, v, (tx, ty, tz), args.tol_m, args.tol_xy_m, args.tol_z_m
                )
                wx, wy, wz = pos
                if reached:
                    print(
                        f'    [verify-ok] {v} world=({wx:+.2f}, {wy:+.2f}, {wz:+.2f}) '
                        f'target=({tx:+.2f}, {ty:+.2f}, {tz:+.2f}) err={err:.3f}m xy={err_xy:.3f} z={err_z:.3f}'
                    )
                    stats.append((idx + 1, pat, v, err, 'move'))
                else:
                    print(
                        f'    [verify-miss] {v} world=({wx:+.2f}, {wy:+.2f}, {wz:+.2f}) '
                        f'target=({tx:+.2f}, {ty:+.2f}, {tz:+.2f}) err={err:.3f}m xy={err_xy:.3f} z={err_z:.3f}'
                    )
                    stats.append((idx + 1, pat, v, err, 'move_miss'))
                    misses.append((v, tx, ty, tz, err))

            if misses and args.fallback_teleport_on_miss:
                print(f'    [fallback] move 미도달 {len(misses)}대에 teleport fallback 적용')
                for v, tx, ty, tz, _ in misses:
                    ok = False
                    last_err = -1.0
                    for attempt in range(1, args.teleport_retries + 1):
                        teleport_world_ned(c, v, tx, ty, tz)
                        time.sleep(0.05)
                        reached, pos, last_err, err_xy, err_z = verify_target(
                            c, v, (tx, ty, tz), args.tol_m, args.tol_xy_m, args.tol_z_m
                        )
                        wx, wy, wz = pos
                        if reached:
                            ok = True
                            print(
                                f'    [fallback-ok] {v} world=({wx:+.2f}, {wy:+.2f}, {wz:+.2f}) '
                                f'err={last_err:.3f}m attempt={attempt}/{args.teleport_retries}'
                            )
                            stats.append((idx + 1, pat, v, last_err, 'fallback_teleport'))
                            break
                    if not ok:
                        print(f'    [fallback-miss] {v} err={last_err:.3f}m')
                        stats.append((idx + 1, pat, v, last_err, 'fallback_miss'))
                        if args.fail_on_miss:
                            return 2
            elif misses and args.fail_on_miss:
                print('    [fail] move 미도달 발생, fail_on_miss=true 로 종료')
                return 2

            # Hold 동안 명령 stream 유지 — 1Hz 로 같은 target 재발행 (.join() 안 함).
            # SimpleFlight 의 buggy hover 우회.
            hold_start = time.time()
            hold_tick = 0
            while time.time() - hold_start < args.hold_sec:
                for v, _, (tx, ty, tz) in send_fs:
                    # join 안 함 → 명령만 발행. 같은 target 이므로 drone 은 그 위치에서 유지.
                    move_world_ned(c, v, tx, ty, tz, args.velocity * 0.85)
                hold_tick += 1
                # 첫 tick 만 상세 출력 (이후 spam 방지)
                if hold_tick == 1:
                    print(f'    [hold-stream] {args.hold_sec}s 동안 1Hz target 재발행...')
                maybe_update_camera()
                time.sleep(1.0)
        else:
            # showcase: 이동 중 연속적으로 setpoint 를 스트리밍하면서
            # 포메이션을 다음 패턴으로 부드럽게 변환한다.
            curr_pat = pattern_seq[idx % len(pattern_seq)]
            next_pat = pattern_seq[(idx + 1) % len(pattern_seq)]
            curr_offsets = formations[curr_pat]
            next_offsets = formations[next_pat]

            if args.pre_settle:
                pre_ok, _pre_targets = settle_to_pattern(
                    c,
                    drone_names,
                    curr_pat,
                    (leader_x, leader_y, leader_z),
                    max(1.8, args.velocity * 0.75),
                    args.tol_m,
                    args.tol_xy_m,
                    args.tol_z_m,
                    args.pre_settle_retries,
                    z_lift_for_settle=args.transition_z_lift * args.z_lift_gain,
                    min_separation_m=args.min_separation_m,
                    crossing_penalty=args.ca_crossing_penalty,
                )
                if not pre_ok:
                    print('    [pre-settle-warn] current pattern alignment not perfect, continue with caution')

            waypoints = [
                (leader_x, leader_y),
                (leader_x + 6.0, leader_y + 0.0),
                (leader_x + 9.0, leader_y + 5.0),
                (leader_x + 3.0, leader_y + 9.0),
                (leader_x - 5.0, leader_y + 6.0),
                (leader_x - 8.0, leader_y - 1.0),
                (leader_x - 2.0, leader_y - 6.0),
                (leader_x + 4.0, leader_y - 4.0),
            ]
            a = waypoints[idx % len(waypoints)]
            b_wp = waypoints[(idx + 1) % len(waypoints)]
            # 쇼케이스 가독성: 기본은 "포메이션 완성 후 액션".
            # 동시에 바꾸고 싶을 때만 --transition-while-moving 사용.
            b = b_wp if args.transition_while_moving else a
            steps = max(2, int(args.segment_sec * args.tick_hz))
            dt = 1.0 / max(args.tick_hz, 1.0)
            print(f'    [showcase] {curr_pat} -> {next_pat}, leader {a} -> {b}, steps={steps}')
            curr_world: dict[str, tuple[float, float, float]] = {}
            for v in drone_names:
                st = c.getMultirotorState(vehicle_name=v)
                curr_world[v] = world_pos(st, v)
            if args.role_assignment == 'fixed':
                slot_map = {}
                for i, v in enumerate(drone_names):
                    slot_map[v] = i
            else:
                slot_map = build_collision_aware_slot_assignment(
                    drone_names,
                    curr_world,
                    b,
                    next_offsets,
                    args.min_separation_m,
                    args.ca_crossing_penalty,
                    prev_slot_map=prev_slot_map,
                    continuity_penalty=args.ca_continuity_penalty,
                )
            print(f'    [deconflict] slot_map={slot_map}')
            prev_slot_map = slot_map.copy()
            per_drone_world_path: dict[str, list[tuple[float, float, float]]] = {v: [] for v in drone_names}
            path_vel_scale = 1.0
            z_lift_scale = 1.0
            for mitigate_round in range(args.collision_mitigation_rounds + 1):
                for v in per_drone_world_path:
                    per_drone_world_path[v].clear()
                worst_min_d = 9999.0
                for s in range(steps + 1):
                    t = s / steps
                    et = smoothstep(t)
                    lx = lerp(a[0], b[0], et)
                    ly = lerp(a[1], b[1], et)
                    step_targets: list[tuple[str, float, float, float]] = []
                    for i, v in enumerate(drone_names):
                        if v not in curr_world:
                            continue
                        sidx = slot_map.get(v, i)
                        if sidx >= len(next_offsets):
                            continue
                        if args.pre_settle:
                            if i >= len(curr_offsets):
                                continue
                            cox, coy, coz = curr_offsets[i]
                        else:
                            cox = curr_world[v][0] - a[0]
                            coy = curr_world[v][1] - a[1]
                            coz = curr_world[v][2] - leader_z
                        nox, noy, noz = next_offsets[sidx]
                        if len(drone_names) >= 6:
                            lane_gap = max(0.85, args.transition_z_lift * max(0.4, args.z_lift_gain))
                            lane_z = leader_z + centered_layer_index(i, len(drone_names)) * lane_gap
                            start_z = leader_z + coz
                            end_z = leader_z + noz
                            if t < 0.18:
                                pu = smoothstep(t / 0.18)
                                ox = cox
                                oy = coy
                                tz = lerp(start_z, lane_z, pu)
                            elif t < 0.86:
                                pu = smoothstep((t - 0.18) / 0.68)
                                ox = lerp(cox, nox, pu)
                                oy = lerp(coy, noy, pu)
                                tz = lane_z
                            else:
                                pu = smoothstep((t - 0.86) / 0.14)
                                ox = nox
                                oy = noy
                                tz = lerp(lane_z, end_z, pu)
                            tx = lx + ox
                            ty = ly + oy
                            step_targets.append((v, tx, ty, tz))
                            continue
                        ox = lerp(cox, nox, et)
                        oy = lerp(coy, noy, et)
                        oz = lerp(coz, noz, et)
                        bias = symmetric_layer_bias(i, len(drone_names))
                        oz += bias * args.transition_z_lift * z_lift_scale * args.z_lift_gain
                        tx = lx + ox
                        ty = ly + oy
                        tz = leader_z + oz
                        step_targets.append((v, tx, ty, tz))

                    step_targets = enforce_min_separation_points(step_targets, args.min_separation_m)
                    if len(drone_names) < 6:
                        step_targets = enforce_vertical_separation_for_close_xy(
                            step_targets,
                            args.z_collision_xy_thresh_m,
                            args.z_collision_min_dz_m,
                            dz_gain_per_xy_m=args.z_collision_dz_gain_per_xy_m,
                        )
                    step_xyz = [(tx, ty, tz) for _v, tx, ty, tz in step_targets]
                    dmin = min_pairwise_distance(step_xyz)
                    if dmin < worst_min_d:
                        worst_min_d = dmin
                    for v, tx, ty, tz in step_targets:
                        per_drone_world_path[v].append((tx, ty, tz))
                    maybe_update_camera()

                if worst_min_d >= args.collision_hard_min_m:
                    if mitigate_round > 0:
                        print(
                            f'    [collision-mitigated] round={mitigate_round}, '
                            f'min_step_dist={worst_min_d:.2f}m'
                        )
                    break
                if mitigate_round < args.collision_mitigation_rounds:
                    path_vel_scale *= 0.84
                    z_lift_scale *= 1.35
                    print(
                        f'    [collision-risk] min_step_dist={worst_min_d:.2f}m < {args.collision_hard_min_m:.2f}m '
                        f'-> retry(round={mitigate_round+1}) with slower+more_z_separation'
                    )

            # 고빈도 moveToPosition 재발행 대신 moveOnPathAsync 1회로 경로추종.
            path_fs = []
            near_pairs = 0
            pos_now = [curr_world[v] for v in drone_names if v in curr_world]
            nearest_now = min_pairwise_distance(pos_now)
            for i in range(len(pos_now)):
                for j in range(i + 1, len(pos_now)):
                    dx = pos_now[i][0] - pos_now[j][0]
                    dy = pos_now[i][1] - pos_now[j][1]
                    dz = pos_now[i][2] - pos_now[j][2]
                    if math.sqrt(dx * dx + dy * dy + dz * dz) < args.ca_near_dist_m:
                        near_pairs += 1
            speed_scale = args.ca_slowdown_factor if near_pairs > 0 else 1.0
            path_vel = max(1.2, args.velocity * 0.75 * speed_scale * path_vel_scale)
            if near_pairs > 0:
                print(f'    [deconflict] near_pairs={near_pairs}, nearest_now={nearest_now:.2f}m, path_vel={path_vel:.2f}m/s')
            for v in drone_names:
                pts_world = per_drone_world_path.get(v, [])
                if len(pts_world) < 2:
                    continue
                local_pts = world_path_to_local_points(v, pts_world)
                f = c.moveOnPathAsync(
                    local_pts,
                    path_vel,
                    timeout_sec=max(8.0, args.segment_sec * 2.0),
                    drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=YawMode(is_rate=False, yaw_or_rate=0.0),
                    lookahead=-1.0,
                    adaptive_lookahead=1.0,
                    vehicle_name=v,
                )
                path_fs.append((v, f))
                if args.command_stagger_ms > 0:
                    time.sleep(args.command_stagger_ms / 1000.0)
            for _v, f in path_fs:
                f.join()
            enforce_safety_floor(
                c, drone_names, args.safety_floor_z, args.safety_recovery_z, args.velocity
            )

            # 스트리밍 직후 바로 검증하면 동역학 지연으로 miss가 크게 나온다.
            # 최종 포메이션 target을 다시 발행하고 join으로 수렴을 보장한다.
            final_fs = []
            now_t = time.time()
            for i, v in enumerate(drone_names):
                sidx = slot_map.get(v, i)
                if sidx >= len(next_offsets):
                    continue
                nox, noy, noz = next_offsets[sidx]
                tx = b[0] + nox
                ty = b[1] + noy
                tz = leader_z + noz
                if len(drone_names) >= 6:
                    st = c.getMultirotorState(vehicle_name=v)
                    wz = world_pos(st, v)[2]
                    tz = stabilize_target_z_with_state(
                        v,
                        wz,
                        tz,
                        args.z_command_deadband_m,
                        args.z_command_max_step_m,
                        args.z_command_hold_band_m,
                        args.z_command_cooldown_sec,
                        now_t,
                        z_cmd_state,
                    )
                f = move_world_ned(c, v, tx, ty, tz, max(1.6, args.velocity * 0.62))
                final_fs.append((v, f, tx, ty, tz))
            for v, f, tx, ty, tz in final_fs:
                f.join()
            if args.capture_sec > 0.0:
                cap_start = time.time()
                while time.time() - cap_start < args.capture_sec:
                    now_t = time.time()
                    for v, _, tx, ty, tz in final_fs:
                        st = c.getMultirotorState(vehicle_name=v)
                        wz = world_pos(st, v)[2]
                        tz_cmd = stabilize_target_z_with_state(
                            v, wz, tz,
                            args.z_command_deadband_m, args.z_command_max_step_m,
                            args.z_command_hold_band_m, args.z_command_cooldown_sec,
                            now_t, z_cmd_state
                        )
                        move_world_ned(c, v, tx, ty, tz_cmd, max(1.8, args.velocity * 0.7))
                    enforce_safety_floor(
                        c, drone_names, args.safety_floor_z, args.safety_recovery_z, args.velocity
                    )
                    maybe_update_camera()
                    time.sleep(0.2)

            # 전환 중 충돌회피용 z 분리를 사용한 뒤, 완성 포메이션에서는 목표 z로 재정렬.
            if args.z_align_sec > 0.0:
                print(f'    [z-align] formation z normalize {args.z_align_sec:.1f}s')
                zt_start = time.time()
                while time.time() - zt_start < args.z_align_sec:
                    now_t = time.time()
                    zfs = []
                    for i, v in enumerate(drone_names):
                        sidx = slot_map.get(v, i)
                        if sidx >= len(next_offsets):
                            continue
                        nox, noy, noz = next_offsets[sidx]
                        tx = b[0] + nox
                        ty = b[1] + noy
                        tz = leader_z + noz
                        st = c.getMultirotorState(vehicle_name=v)
                        wz = world_pos(st, v)[2]
                        tz_cmd = stabilize_target_z_with_state(
                            v, wz, tz,
                            args.z_command_deadband_m, args.z_command_max_step_m,
                            args.z_command_hold_band_m, args.z_command_cooldown_sec,
                            now_t, z_cmd_state
                        )
                        zfs.append((v, move_world_ned(c, v, tx, ty, tz_cmd, max(1.5, args.velocity * 0.62))))
                    for _v, f in zfs:
                        f.join()
                    maybe_update_camera()
                    time.sleep(0.25)

            misses = 0
            miss_list = []
            for i, v in enumerate(drone_names):
                sidx = slot_map.get(v, i)
                if sidx >= len(next_offsets):
                    continue
                nox, noy, noz = next_offsets[sidx]
                tx = b[0] + nox
                ty = b[1] + noy
                tz = leader_z + noz
                reached, pos, err, err_xy, err_z = verify_target(
                    c, v, (tx, ty, tz), args.tol_m, args.tol_xy_m, args.tol_z_m
                )
                wx, wy, wz = pos
                if reached:
                    print(
                        f'    [showcase-ok] {v} world=({wx:+.2f}, {wy:+.2f}, {wz:+.2f}) '
                        f'target=({tx:+.2f}, {ty:+.2f}, {tz:+.2f}) err={err:.3f}m xy={err_xy:.3f} z={err_z:.3f}'
                    )
                    stats.append((idx + 1, f'{curr_pat}->{next_pat}', v, err, 'showcase'))
                else:
                    print(
                        f'    [showcase-miss] {v} world=({wx:+.2f}, {wy:+.2f}, {wz:+.2f}) '
                        f'target=({tx:+.2f}, {ty:+.2f}, {tz:+.2f}) err={err:.3f}m xy={err_xy:.3f} z={err_z:.3f}'
                    )
                    stats.append((idx + 1, f'{curr_pat}->{next_pat}', v, err, 'showcase_miss'))
                    misses += 1
                    miss_list.append((v, tx, ty, tz))

            # move 기반 복구: 미도달 드론에 대해 재-arm 없이 저속 재시도.
            # (재-arm/re-acquire 는 흔들림을 키울 수 있어 제외)
            if miss_list:
                print(f'    [recover] miss {len(miss_list)}대, 저속 move 재시도')
                for v, tx, ty, tz in miss_list:
                    rec_ok = False
                    rec_err = -1.0
                    for attempt in range(1, 4):
                        f = move_world_ned(c, v, tx, ty, tz, max(1.6, args.velocity * 0.6))
                        f.join()
                        time.sleep(0.25)
                        reached, pos, rec_err, rec_xy, rec_z = verify_target(
                            c, v, (tx, ty, tz), args.tol_m, args.tol_xy_m, args.tol_z_m
                        )
                        wx, wy, wz = pos
                        if reached:
                            rec_ok = True
                            print(
                                f'    [recover-ok] {v} world=({wx:+.2f}, {wy:+.2f}, {wz:+.2f}) '
                                f'err={rec_err:.3f}m xy={rec_xy:.3f} z={rec_z:.3f} attempt={attempt}/3'
                            )
                            stats.append((idx + 1, f'{curr_pat}->{next_pat}', v, rec_err, 'showcase_recover'))
                            misses -= 1
                            break
                    if not rec_ok:
                        print(f'    [recover-miss] {v} err={rec_err:.3f}m')
                        stats.append((idx + 1, f'{curr_pat}->{next_pat}', v, rec_err, 'showcase_recover_miss'))

            # 각 포메이션을 지정 시간 유지 (기본 10초)
            hold_sec_local = args.circle_hold_sec if next_pat == 'CIRCLE' else args.formation_hold_sec
            if hold_sec_local > 0.0:
                print(f'    [hold-formation] {next_pat} 유지 {hold_sec_local:.1f}s')
                hold_start = time.time()
                while time.time() - hold_start < hold_sec_local:
                    now_t = time.time()
                    hold_fs = []
                    for i, v in enumerate(drone_names):
                        sidx = slot_map.get(v, i)
                        if sidx >= len(next_offsets):
                            continue
                        nox, noy, noz = next_offsets[sidx]
                        tx = b[0] + nox
                        ty = b[1] + noy
                        tz = leader_z + noz
                        st = c.getMultirotorState(vehicle_name=v)
                        wz = world_pos(st, v)[2]
                        tz_cmd = stabilize_target_z_with_state(
                            v, wz, tz,
                            args.z_command_deadband_m, args.z_command_max_step_m,
                            args.z_command_hold_band_m, args.z_command_cooldown_sec,
                            now_t, z_cmd_state
                        )
                        hold_fs.append((v, move_world_ned(c, v, tx, ty, tz_cmd, max(1.4, args.velocity * 0.55))))
                    for _v, f in hold_fs:
                        f.join()
                    enforce_safety_floor(
                        c, drone_names, args.safety_floor_z, args.safety_recovery_z, args.velocity
                    )
                    maybe_update_camera()
                    time.sleep(0.6)
            # 포메이션 도달 후 동적 액션:
            #  - LINE_H: 형상 유지 병진 이동
            #  - TRIANGLE/DIAMOND3/CIRCLE: 중심 기준 회전(90/180 등)
            if args.post_action_enabled:
                action_vel = max(1.4, args.velocity * max(0.5, args.post_action_velocity_scale))
                action_start = b
                heading_x = b_wp[0] - a[0]
                heading_y = b_wp[1] - a[1]
                hn = math.sqrt(heading_x * heading_x + heading_y * heading_y)
                if hn < 1e-3:
                    ux, uy = 1.0, 0.0
                else:
                    ux, uy = heading_x / hn, heading_y / hn
                if next_pat == 'LINE_H' and args.post_line_translate_m > 0.1:
                    action_end = (
                        action_start[0] + ux * args.post_line_translate_m,
                        action_start[1] + uy * args.post_line_translate_m,
                    )
                    print(
                        f'    [post-action] LINE_H translate {args.post_line_translate_m:.1f}m '
                        f'({action_start[0]:+.2f},{action_start[1]:+.2f}) -> ({action_end[0]:+.2f},{action_end[1]:+.2f})'
                    )
                    ok_action, min_d = execute_rigid_formation_action(
                        c,
                        drone_names,
                        slot_map,
                        next_offsets,
                        action_start,
                        action_end,
                        leader_z,
                        args.post_action_segment_sec,
                        args.tick_hz,
                        action_vel,
                        args.command_stagger_ms,
                        args.min_separation_m,
                        args.collision_hard_min_m,
                        args.collision_mitigation_rounds,
                        args.transition_z_lift,
                        args.z_lift_gain,
                        args.safety_floor_z,
                        args.safety_recovery_z,
                        action_name='LINE_TRANSLATE',
                        rotate_deg=0.0,
                        z_collision_xy_thresh_m=args.z_collision_xy_thresh_m,
                        z_collision_min_dz_m=args.z_collision_min_dz_m,
                        z_collision_dz_gain_per_xy_m=args.z_collision_dz_gain_per_xy_m,
                    )
                    print(f'    [post-action-done] LINE_TRANSLATE ok={ok_action} min_step_dist={min_d:.2f}m')
                    b = action_end
                elif args.post_action_style == 'mixed' and next_pat in ('TRIANGLE', 'DIAMOND3', 'CIRCLE'):
                    for deg in post_rotate_deg_list:
                        print(f'    [post-action] {next_pat} center-rotate {deg:.0f}deg around leader')
                        ok_action, min_d = execute_rigid_formation_action(
                            c,
                            drone_names,
                            slot_map,
                            next_offsets,
                            action_start,
                            action_start,
                            leader_z,
                            args.post_action_segment_sec,
                            args.tick_hz,
                            action_vel,
                            args.command_stagger_ms,
                            args.min_separation_m,
                            args.collision_hard_min_m,
                            args.collision_mitigation_rounds,
                            args.transition_z_lift,
                            args.z_lift_gain,
                            args.safety_floor_z,
                            args.safety_recovery_z,
                            action_name=f'{next_pat}_ROTATE_{int(deg)}',
                            rotate_deg=deg,
                            z_collision_xy_thresh_m=args.z_collision_xy_thresh_m,
                            z_collision_min_dz_m=args.z_collision_min_dz_m,
                            z_collision_dz_gain_per_xy_m=args.z_collision_dz_gain_per_xy_m,
                        )
                        print(
                            f'    [post-action-done] {next_pat}_ROTATE_{int(deg)} '
                            f'ok={ok_action} min_step_dist={min_d:.2f}m'
                        )
                        # 회전 완료 상태를 다음 회전의 기준 offset으로 갱신
                        rotated = []
                        for ox, oy, oz in next_offsets:
                            rx, ry = rotate_offset_xy(ox, oy, deg)
                            rotated.append((rx, ry, oz))
                        next_offsets = rotated
                if args.post_action_pause_sec > 0.0:
                    print(f'    [post-action-pause] {args.post_action_pause_sec:.1f}s')
                    time.sleep(args.post_action_pause_sec)

            # 누적 꼬임 방지: 실제 드론 위치로 leader 기준 재보정
            est_leader = estimate_leader_from_actual(
                c, drone_names, slot_map, next_offsets, leader_z
            )
            if est_leader is not None:
                ex, ey, _ez = est_leader
                gx = max(0.0, min(1.0, args.reanchor_gain))
                leader_x = lerp(b[0], ex, gx)
                leader_y = lerp(b[1], ey, gx)
                print(
                    f'    [reanchor] planned=({b[0]:+.2f},{b[1]:+.2f}) '
                    f'est=({ex:+.2f},{ey:+.2f}) -> next_leader=({leader_x:+.2f},{leader_y:+.2f})'
                )
            else:
                leader_x, leader_y = b[0], b[1]
            if misses > 0 and args.fail_on_miss:
                return 2

        idx += 1
        if idx % len(pattern_seq) == 0:
            cycle += 1
            if args.cycles > 0:
                print(f'  cycle {cycle}/{args.cycles} 완료')

    print('\n[Step 4] cycle 완료. land + disarm')
    if stats:
        print('\n[Summary] pattern/drone 별 최종 오차')
        print('  step  pattern    drone    err(m)   mode')
        for step, pattern, drone, err, mode in stats:
            print(f'  {step:>4}  {pattern:<9} {drone:<7} {err:>7.3f}  {mode}')
    cleanup()
    return 0


if __name__ == '__main__':
    sys.exit(main())
