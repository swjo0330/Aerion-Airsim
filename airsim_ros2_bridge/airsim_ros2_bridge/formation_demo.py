"""AERION Phase 4-Δ — 포메이션 데모 CLI (`aerion_formation_demo` entry point).

기본 형식: `ros2 run airsim_ros2_bridge aerion_formation_demo --demo morphing-cycle [...]`.
formation_node 본체는 그대로 두고, 외부에서 `/aerion/formation/pattern` 만 발행.

데모 모드:
  - `morphing-cycle` : TRIANGLE → V3 → COLUMN → DIAMOND3 순환 (hold-sec 마다 전환).

변경 이력:
  2026-05-22 v1: 최초 작성 (Phase 4-Δ 검증 자동화)
"""

import argparse
import time


DEFAULT_PATTERNS = 'TRIANGLE,V3,COLUMN,DIAMOND3'
DEFAULT_HOLD_SEC = 10.0


def demo_morphing_cycle(args: argparse.Namespace) -> None:
    """TRIANGLE → V3 → COLUMN → DIAMOND3 순환 publish.

    각 패턴을 `hold-sec` 초간 유지하고 다음 패턴으로 전환.
    formation_node 가 morphing 을 자동 처리하므로 본 함수는 String 메시지 publish 만.
    """
    # rclpy/std_msgs 는 실제 데모 실행 시에만 임포트 (--help 가 ROS2 없이도 작동하도록).
    import rclpy  # noqa: PLC0415
    from rclpy.node import Node  # noqa: PLC0415
    from std_msgs.msg import String  # noqa: PLC0415

    patterns = [p.strip() for p in args.patterns.split(',') if p.strip()]
    if not patterns:
        raise SystemExit('--patterns must contain at least one pattern name')
    hold_sec = float(args.hold_sec)

    rclpy.init()
    node = Node('aerion_formation_morphing_cycle')
    pub = node.create_publisher(String, '/aerion/formation/pattern', 10)
    # publisher 가 graph 에 보일 때까지 잠시 대기 (discovery latency)
    time.sleep(1.0)

    try:
        idx = 0
        while rclpy.ok():
            name = patterns[idx % len(patterns)]
            msg = String()
            msg.data = name
            pub.publish(msg)
            node.get_logger().info(f'Pattern → {name} (hold {hold_sec:.1f}s)')
            t_end = time.time() + hold_sec
            while time.time() < t_end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
            idx += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(
        prog='aerion_formation_demo',
        description='AERION Phase 4-Δ 포메이션 데모 CLI.',
    )
    parser.add_argument(
        '--demo',
        choices=['morphing-cycle'],
        default='morphing-cycle',
        help='Pre-built demo sequence. morphing-cycle = TRIANGLE → V3 → COLUMN → DIAMOND3 순환.',
    )
    parser.add_argument(
        '--patterns',
        default=DEFAULT_PATTERNS,
        help=f'Comma-separated pattern names for --demo morphing-cycle (default: {DEFAULT_PATTERNS}).',
    )
    parser.add_argument(
        '--hold-sec',
        default=DEFAULT_HOLD_SEC,
        type=float,
        help=f'Seconds to hold each pattern before next transition (default: {DEFAULT_HOLD_SEC}).',
    )
    # ros2 run 이 --ros-args 등을 함께 전달하므로 unknown args 무시.
    parsed_args, _ = parser.parse_known_args(args)

    if parsed_args.demo == 'morphing-cycle':
        demo_morphing_cycle(parsed_args)
    else:
        raise SystemExit(f'Unknown --demo: {parsed_args.demo}')


if __name__ == '__main__':
    main()
