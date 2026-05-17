#!/usr/bin/env python3
"""AERION Phase 3 — 멀티드론 토픽 인터페이스 자동 검증.

목적:
  N대 드론(기본 5)이 spawn된 상태에서 각 드론마다 표준 토픽이 모두 발행되는지,
  발행 Hz가 최소 기준을 충족하는지 자동 PASS/FAIL 판정.

검증 대상 토픽 (드론별):
  - /drone{n}/camera/image                    (BEST_EFFORT)
  - /drone{n}/camera/camera_info              (RELIABLE)
  - /drone{n}/range/{front,left,right}        (sensor data)
  - /drone{n}/mavros/state                    (RELIABLE)
  - /drone{n}/mavros/imu/data                 (sensor data)
  - /drone{n}/mavros/local_position/pose      (RELIABLE)
  - /drone{n}/mavros/local_position/odom      (RELIABLE)
  - /drone{n}/mavros/setpoint_velocity/cmd_vel (RELIABLE)

PASS 기준:
  - 모든 토픽이 ros2 topic list에 존재
  - 각 토픽 Hz가 임계값 이상 (camera ≥ 5Hz, range ≥ 15Hz, mavros pose ≥ 5Hz)
  - 측정 시간: 토픽당 5초 sliding window

사용:
  ros2 환경 source 후 (rmw_cyclonedds + cyclonedds.xml export 상태):
    python3 ~/airsim/airsim_ros2_bridge/scripts/verify_topics_phase3.py --drones 5
    python3 ~/airsim/airsim_ros2_bridge/scripts/verify_topics_phase3.py --drones 2 --hz-camera 3

의존:
  - bridge가 띄워져 있어야 함 (drone_count대만큼 프로세스 실행 중)
  - UE Play 상태 (AirSim RPC + 카메라 캡처 활성)
"""

import argparse
import json
import subprocess
import sys
import time
from typing import Dict, List, Tuple

# 토픽 패턴 (드론 번호 미포함, 검증 시 prefix 추가)
TOPIC_PATTERNS = [
    ('camera/image',                       'camera',  5.0),
    ('camera/camera_info',                 'meta',    0.5),  # latched 가능, low rate 허용
    ('range/front',                        'range',  15.0),
    ('range/left',                         'range',  15.0),
    ('range/right',                        'range',  15.0),
    ('mavros/state',                       'mavros', 0.5),
    ('mavros/imu/data',                    'mavros', 5.0),
    ('mavros/local_position/pose',         'mavros', 2.0),
    ('mavros/local_position/odom',         'mavros', 2.0),
    ('mavros/setpoint_velocity/cmd_vel',   'mavros', 0.0),  # 명령 토픽 — subscriber만 있으면 OK
]


def list_topics(timeout: float = 5.0) -> List[str]:
    try:
        out = subprocess.check_output(['ros2', 'topic', 'list'], timeout=timeout, text=True)
        return [t.strip() for t in out.splitlines() if t.strip()]
    except subprocess.CalledProcessError as e:
        print(f'[ERROR] ros2 topic list 실패: {e}', file=sys.stderr)
        return []


def measure_hz(topic: str, duration: float = 5.0) -> float:
    """ros2 topic hz를 duration초 동안 sampling. 평균 Hz 반환 (없으면 0.0)."""
    try:
        proc = subprocess.Popen(
            ['ros2', 'topic', 'hz', topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        )
        import time
        time.sleep(duration)
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
        # 출력에서 'average rate: X.XXX' 라인 추출
        for line in out.splitlines():
            line = line.strip()
            if line.startswith('average rate:'):
                try:
                    return float(line.split(':')[1].strip())
                except (IndexError, ValueError):
                    continue
        return 0.0
    except Exception as e:
        print(f'[WARN] hz 측정 실패 {topic}: {e}', file=sys.stderr)
        return 0.0


def verify_drone(drone_idx: int, all_topics: List[str], hz_overrides: dict,
                 measure_duration: float = 5.0) -> Tuple[int, int, List[str], List[Dict]]:
    """한 드론의 모든 표준 토픽 검증.

    Returns:
        (pass_count, total, fail_messages, per_topic_results)
        per_topic_results: 각 토픽별 결과 dict (JSON 출력용)
    """
    prefix = f'/drone{drone_idx}'
    passed, total = 0, len(TOPIC_PATTERNS)
    fails = []
    results = []
    for suffix, category, default_min_hz in TOPIC_PATTERNS:
        topic = f'{prefix}/{suffix}'
        min_hz = hz_overrides.get(category, default_min_hz)
        result = {
            'drone': drone_idx,
            'topic': topic,
            'category': category,
            'min_hz': min_hz,
            'measured_hz': None,
            'status': None,
        }
        if topic not in all_topics:
            fails.append(f'    [MISS]   {topic}')
            result['status'] = 'MISSING'
            results.append(result)
            continue
        if min_hz <= 0.001:
            passed += 1
            result['status'] = 'PRESENT'
            results.append(result)
            continue
        hz = measure_hz(topic, duration=measure_duration)
        result['measured_hz'] = hz
        if hz >= min_hz:
            passed += 1
            result['status'] = 'OK'
            print(f'    [OK {hz:6.2f}Hz ≥ {min_hz:.2f}]  {topic}')
        else:
            result['status'] = 'LOW_HZ'
            fails.append(f'    [LOW {hz:6.2f}Hz < {min_hz:.2f}]  {topic}')
        results.append(result)
    return passed, total, fails, results


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--drones', type=int, default=5, help='검증할 드론 수 (1~5)')
    p.add_argument('--hz-camera', type=float, default=5.0)
    p.add_argument('--hz-range',  type=float, default=15.0)
    p.add_argument('--hz-mavros', type=float, default=5.0)
    p.add_argument('--json-out',  type=str, default=None,
                   help='측정 결과를 JSON으로 저장 (없으면 stdout만)')
    p.add_argument('--measure-duration', type=float, default=5.0,
                   help='토픽당 hz 측정 시간 (초). 길수록 정확하나 전체 검증 시간 증가')
    args = p.parse_args()

    if args.drones < 1 or args.drones > 5:
        print('[ERROR] --drones 1~5만 지원', file=sys.stderr)
        sys.exit(2)

    hz_overrides = {
        'camera': args.hz_camera,
        'range':  args.hz_range,
        'mavros': args.hz_mavros,
        'meta':   0.5,
    }

    print(f'[verify] 드론 수={args.drones}  threshold camera={args.hz_camera} range={args.hz_range} mavros={args.hz_mavros}')
    print('[verify] ros2 topic list 수집 중...')
    all_topics = list_topics()
    if not all_topics:
        print('[FAIL] ros2 토픽 목록 비어있음. bridge 안 떠있거나 cyclonedds env 미설정.', file=sys.stderr)
        sys.exit(1)

    total_pass, total_count = 0, 0
    all_fails = []
    all_results = []
    t_start = time.time()
    for n in range(1, args.drones + 1):
        print(f'\n[verify] drone{n} 검증:')
        p_, t, fails, per = verify_drone(n, all_topics, hz_overrides, args.measure_duration)
        total_pass += p_
        total_count += t
        all_fails.extend([f'drone{n}: {f}' for f in fails])
        all_results.extend(per)
    elapsed = time.time() - t_start

    summary = {
        'pass': total_pass,
        'total': total_count,
        'pass_rate': total_pass / total_count if total_count else 0.0,
        'elapsed_sec': elapsed,
        'drones': args.drones,
        'thresholds': hz_overrides,
        'fails': all_fails,
        'topics': all_results,
    }

    print('\n' + '=' * 60)
    print(f'[RESULT] PASS {total_pass}/{total_count} ({100*total_pass/total_count:.1f}%)  '
          f'elapsed={elapsed:.1f}s')

    if args.json_out:
        with open(args.json_out, 'w') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        print(f'[verify] JSON 결과 저장 → {args.json_out}')

    if all_fails:
        print('[FAIL 항목]')
        for f in all_fails:
            print(f)
        sys.exit(1)
    print('[verify] All topics OK.')


if __name__ == '__main__':
    main()
