#!/usr/bin/env python3
"""AERION Phase 4 포메이션 노드 진입 스크립트.

본 스크립트는 entry_points 등록을 통해 `ros2 run airsim_ros2_bridge aerion_formation`로도 호출 가능.
실제 노드 구현은 `airsim_ros2_bridge.formation_node.FormationNode` (재사용 가능 모듈).

Usage:
  ros2 run airsim_ros2_bridge aerion_formation
  # 또는 launch에서 호출 (향후 aerion_phase4.launch.py 추가 예정)

변경 이력:
  2026-05-18 v1: 최초 작성 (Aerostack2 대신 자체 구현 결정 직후)
  2026-05-18 v2: formation_node.py 모듈로 이전, 본 파일은 진입점만
"""

from airsim_ros2_bridge.formation_node import main


if __name__ == '__main__':
    main()
