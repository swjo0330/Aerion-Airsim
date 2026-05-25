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
}


def get_formations_for_count(drone_count: int) -> dict[str, list[tuple[float, float, float]]]:
    if drone_count >= 5:
        return FORMATIONS_NED_5
    return FORMATIONS_NED

ALTITUDE_M = 5.0   # 이륙 고도 (ENU). AirSim NED z = -ALTITUDE_M


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
) -> tuple[bool, dict[str, tuple[float, float, float]]]:
    formations = get_formations_for_count(len(drone_names))
    offsets = formations.get(pattern_name, [])
    targets: dict[str, tuple[float, float, float]] = {}
    lx, ly, lz = leader_xyz
    for i, v in enumerate(drone_names):
        if i >= len(offsets):
            continue
        ox, oy, oz = offsets[i]
        targets[v] = (lx + ox, ly + oy, lz + oz)

    for attempt in range(1, retries + 1):
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

    prev_center = state.get("prev_center")
    if prev_center is None:
        vx, vy = 1.0, 0.0
    else:
        vx = center[0] - prev_center[0]
        vy = center[1] - prev_center[1]
    n = math.sqrt(vx * vx + vy * vy)
    if n < 1e-3:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = vx / n, vy / n

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

    tx = center[0] + ux * lookahead_m
    ty = center[1] + uy * lookahead_m
    tz = center[2]
    cam_x = center[0] + rx * distance_m
    cam_y = center[1] + ry * distance_m
    cam_z = center[2] - height_m + view_elevation_bias_m
    q = calc_look_quaternion((cam_x, cam_y, cam_z), (tx, ty, tz))
    world_pose = Pose(Vector3r(cam_x, cam_y, cam_z), q)
    client.simSetCameraPose(camera_name, world_pose)
    state["prev_center"] = center


def update_center_follow_camera(
    client: airsim.MultirotorClient,
    drone_names: list[str],
    camera_name: str,
    back_distance_m: float,
    up_height_m: float,
    lookahead_m: float,
    state: dict,
) -> None:
    center = get_swarm_center(client, drone_names)
    if center is None:
        return
    prev_center = state.get("prev_center")
    if prev_center is None:
        ux, uy = 1.0, 0.0
    else:
        vx = center[0] - prev_center[0]
        vy = center[1] - prev_center[1]
        n = math.sqrt(vx * vx + vy * vy)
        if n < 1e-3:
            ux, uy = 1.0, 0.0
        else:
            ux, uy = vx / n, vy / n

    cam_x = center[0] - ux * back_distance_m
    cam_y = center[1] - uy * back_distance_m
    cam_z = center[2] - up_height_m
    tx = center[0] + ux * lookahead_m
    ty = center[1] + uy * lookahead_m
    tz = center[2]
    q = calc_look_quaternion((cam_x, cam_y, cam_z), (tx, ty, tz))
    client.simSetCameraPose(camera_name, Pose(Vector3r(cam_x, cam_y, cam_z), q))
    state["prev_center"] = center


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
    p.add_argument('--patterns', default='TRIANGLE,LINE_H,ECHELON_R,DIAMOND3,ECHELON_L',
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
    p.add_argument('--formation-hold-sec', type=float, default=10.0,
                   help='showcase 모드에서 각 포메이션 유지 시간 (s)')
    p.add_argument('--min-separation-m', type=float, default=4.0,
                   help='showcase 전환 중 드론 간 최소 xy 간격 (m)')
    p.add_argument('--transition-z-lift', type=float, default=0.9,
                   help='showcase 전환 중 인접 드론 충돌 회피용 고도 레이어 진폭 (m)')
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
    p.add_argument('--observer-x', type=float, default=0.0,
                   help='observer 모드 카메라 월드 NED X')
    p.add_argument('--observer-y', type=float, default=0.0,
                   help='observer 모드 카메라 월드 NED Y')
    p.add_argument('--observer-z', type=float, default=-20.0,
                   help='observer 모드 카메라 월드 NED Z (음수일수록 높음)')
    args = p.parse_args()

    drone_names = [f'{args.prefix}{i}' for i in range(1, args.drones + 1)]
    formations = get_formations_for_count(args.drones)
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
        print('ERROR: --patterns 에 유효한 패턴 없음. 가능: TRIANGLE,LINE_H,ECHELON_R,DIAMOND3,ECHELON_L',
              file=sys.stderr)
        return 1

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
    print(f'    capture_sec={args.capture_sec}')
    print(f'    pre_settle={args.pre_settle}, pre_settle_retries={args.pre_settle_retries}')
    print(f'    formation_hold_sec={args.formation_hold_sec}')
    print(f'    min_separation_m={args.min_separation_m}, transition_z_lift={args.transition_z_lift}')
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
        f'    third_person_follow={args.third_person_follow}, camera={args.third_person_camera_name}, '
        f'mode={args.third_person_mode}, follow_vehicle={args.third_person_follow_vehicle}, '
        f'distance={args.third_person_distance_m}, '
        f'height={args.third_person_height_m}, lookahead={args.third_person_lookahead_m}, '
        f'update_hz={args.third_person_update_hz}, view_azimuth_deg={args.third_person_view_azimuth_deg}, '
        f'view_elevation_bias_m={args.third_person_view_elevation_bias_m}, '
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
    cam_state = {"prev_center": None}

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
            b = waypoints[(idx + 1) % len(waypoints)]
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
                        ox = lerp(cox, nox, et)
                        oy = lerp(coy, noy, et)
                        oz = lerp(coz, noz, et)
                        oz += (i - 1) * args.transition_z_lift * z_lift_scale * math.sin(math.pi * et)
                        tx = lx + ox
                        ty = ly + oy
                        tz = leader_z + oz
                        step_targets.append((v, tx, ty, tz))

                    step_targets = enforce_min_separation_points(step_targets, args.min_separation_m)
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
                print(f'    [deconflict] near_pairs={near_pairs}, path_vel={path_vel:.2f}m/s')
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
            for i, v in enumerate(drone_names):
                sidx = slot_map.get(v, i)
                if sidx >= len(next_offsets):
                    continue
                nox, noy, noz = next_offsets[sidx]
                tx = b[0] + nox
                ty = b[1] + noy
                tz = leader_z + noz
                f = move_world_ned(c, v, tx, ty, tz, max(1.8, args.velocity * 0.75))
                final_fs.append((v, f, tx, ty, tz))
            for v, f, tx, ty, tz in final_fs:
                f.join()
            if args.capture_sec > 0.0:
                cap_start = time.time()
                while time.time() - cap_start < args.capture_sec:
                    for v, _, tx, ty, tz in final_fs:
                        move_world_ned(c, v, tx, ty, tz, max(1.8, args.velocity * 0.7))
                    enforce_safety_floor(
                        c, drone_names, args.safety_floor_z, args.safety_recovery_z, args.velocity
                    )
                    maybe_update_camera()
                    time.sleep(0.2)

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
            if args.formation_hold_sec > 0.0:
                print(f'    [hold-formation] {next_pat} 유지 {args.formation_hold_sec:.1f}s')
                hold_start = time.time()
                while time.time() - hold_start < args.formation_hold_sec:
                    hold_fs = []
                    for i, v in enumerate(drone_names):
                        sidx = slot_map.get(v, i)
                        if sidx >= len(next_offsets):
                            continue
                        nox, noy, noz = next_offsets[sidx]
                        tx = b[0] + nox
                        ty = b[1] + noy
                        tz = leader_z + noz
                        hold_fs.append((v, move_world_ned(c, v, tx, ty, tz, max(1.4, args.velocity * 0.55))))
                    for _v, f in hold_fs:
                        f.join()
                    enforce_safety_floor(
                        c, drone_names, args.safety_floor_z, args.safety_recovery_z, args.velocity
                    )
                    maybe_update_camera()
                    time.sleep(0.6)
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
