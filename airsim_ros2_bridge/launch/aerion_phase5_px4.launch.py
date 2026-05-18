"""AERION Phase 5 — PX4 SITL 5대 + MAVROS 5대 + bridge 5대 + formation 통합 launch.

전제:
  - Phase 4 (SimpleFlight 자체 포메이션) PASS
  - PX4-Autopilot 빌드 완료 (~/airsim/PX4-Autopilot/build/px4_sitl_default/bin/px4)
  - settings.json = sf_5drones_phase3 또는 px4_5drones_phase5 (Phase 5에서 후자 사용)

흐름:
  1. (외부) UE Play + px4_5drones_phase5.json deploy
  2. (외부) PX4 SITL 5개 인스턴스 기동 (~/airsim/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i {0..4})
  3. (본 launch) MAVROS 5대 + bridge×5 + formation_node + leader + tf_publisher 일괄 기동
  4. (외부) ros2 service call /drone{N}/mavros/cmd/arming + set_mode OFFBOARD (또는 별도 mavros_arm_all.py 헬퍼)
  5. (외부) /aerion/formation/pattern 명령

차이점 (Phase 4 launch와 비교):
  - bridge: control_backend='px4_mavros' (setpoint를 mavros 토픽으로)
  - MAVROS 5대 추가 (별도 노드, drone{N}/mavros namespace)
  - SimpleFlight 직접 제어 없음 (PX4가 모터 명령 생성)

변경 이력:
  2026-05-18 v1: 최초 작성 (Phase 5 사전 자산, 미검증)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_bridges_and_mavros(context, *args, **kwargs):
    drone_count = int(LaunchConfiguration('drone_count').perform(context))
    if not (1 <= drone_count <= 5):
        raise ValueError(f'drone_count 1~5만 지원: {drone_count}')

    nodes = []
    # PX4 SITL fcu_url 매핑 (px4_5drones_phase5.json 기준):
    # drone1: udp://:14555@127.0.0.1:14540 ... drone5: udp://:14559@127.0.0.1:14544
    for n in range(1, drone_count + 1):
        vehicle = f'drone{n}'
        fcu_recv_port = 14554 + n     # 14555..14559
        fcu_send_port = 14539 + n     # 14540..14544

        # MAVROS (per drone)
        nodes.append(Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace=vehicle,
            output='screen',
            parameters=[{
                'fcu_url': f'udp://:{fcu_recv_port}@127.0.0.1:{fcu_send_port}',
                'gcs_url': '',
                'target_system_id': n,
                'target_component_id': 1,
                'system_id': 255,
                'component_id': 240,
            }],
        ))

        # Bridge (per drone, control_backend='px4_mavros')
        nodes.append(Node(
            package='airsim_ros2_bridge',
            executable='bridge_node',
            name='airsim_bridge',
            namespace=vehicle,
            output='screen',
            parameters=[{
                'vehicle_name': vehicle,
                'airsim_ip': '127.0.0.1',
                'airsim_port': 41451,
                'enable_camera': True,
                'enable_range': True,
                'camera_fps': 10.0,
                'range_publish_rate': 20.0,
                'enable_ardu_compat': False,
                'control_backend': 'px4_mavros',
                'mavros_instance_namespace': f'mavros{n - 1}',  # mavros0..mavros4
            }],
        ))
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument('drone_count', default_value='5'),
        DeclareLaunchArgument('default_pattern', default_value='LINE'),
        DeclareLaunchArgument('default_altitude', default_value='5.0'),
        DeclareLaunchArgument('obstacle_stop_dist', default_value='1.0'),
        DeclareLaunchArgument('enable_leader_publisher', default_value='true'),
        DeclareLaunchArgument('leader_mode', default_value='static'),
    ]

    formation = Node(
        package='airsim_ros2_bridge',
        executable='aerion_formation',
        name='aerion_formation',
        output='screen',
        parameters=[{
            'drone_count': LaunchConfiguration('drone_count'),
            'default_pattern': LaunchConfiguration('default_pattern'),
            'default_altitude': LaunchConfiguration('default_altitude'),
            'obstacle_stop_dist': LaunchConfiguration('obstacle_stop_dist'),
            'publish_rate': 20.0,
            'enable_arrival_check': True,
        }],
    )

    leader = Node(
        package='airsim_ros2_bridge',
        executable='aerion_leader',
        name='aerion_leader',
        output='screen',
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(
            LaunchConfiguration('enable_leader_publisher')),
        parameters=[{
            'mode': LaunchConfiguration('leader_mode'),
            'init_z': 5.0,
        }],
    )

    tf_pub = Node(
        package='airsim_ros2_bridge',
        executable='aerion_tf',
        name='aerion_tf_publisher',
        output='screen',
        parameters=[{
            'drone_count': LaunchConfiguration('drone_count'),
        }],
    )

    return LaunchDescription(args + [
        OpaqueFunction(function=_build_bridges_and_mavros),
        formation,
        leader,
        tf_pub,
    ])
