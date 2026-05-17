"""AERION Phase 4 — 5대 드론 + 포메이션 노드 통합 launch.

흐름:
  1. (사전 외부) UE Play + sf_5drones_phase3.json deploy + airsim_arm_all.py 실행 (모든 드론 takeoff 완료 상태)
  2. (본 launch) bridge_node × N + formation_node 동시 기동
  3. (외부) /aerion/formation/{pattern, leader_pose} 토픽으로 명령

Usage:
    ros2 launch airsim_ros2_bridge aerion_phase4_formation.launch.py drone_count:=5 default_pattern:=LINE

사전:
    - 5대 settings.json deploy됨 (vehicle key drone1..drone5)
    - UE Play + airsim_arm_all.py로 모든 드론 takeoff 완료
    - cyclonedds.xml + RMW env 설정됨

종료 후:
    - airsim_land_all.py로 일괄 착륙 + disarm

변경 이력:
    2026-05-18 v1: 최초 작성 (Phase 4 통합 launch)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_bridges(context, *args, **kwargs):
    """drone_count대 만큼 bridge_node를 1 드론 = 1 프로세스 패턴으로 spawn."""
    drone_count = int(LaunchConfiguration('drone_count').perform(context))
    enable_camera = LaunchConfiguration('enable_camera').perform(context).lower() == 'true'
    enable_range = LaunchConfiguration('enable_range').perform(context).lower() == 'true'
    camera_fps = float(LaunchConfiguration('camera_fps').perform(context))
    range_publish_rate = float(LaunchConfiguration('range_publish_rate').perform(context))

    bridges = []
    for n in range(1, drone_count + 1):
        vehicle = f'drone{n}'
        bridges.append(Node(
            package='airsim_ros2_bridge',
            executable='bridge_node',
            name='airsim_bridge',
            namespace=vehicle,
            output='screen',
            parameters=[{
                'vehicle_name': vehicle,
                'airsim_ip': '127.0.0.1',
                'airsim_port': 41451,
                'enable_camera': enable_camera,
                'enable_range': enable_range,
                'camera_fps': camera_fps,
                'range_publish_rate': range_publish_rate,
                'enable_ardu_compat': False,
                'control_backend': 'airsim_direct',
            }],
        ))
    return bridges


def generate_launch_description():
    args = [
        DeclareLaunchArgument('drone_count', default_value='5'),
        DeclareLaunchArgument('default_pattern', default_value='LINE',
                              description='LINE | DIAMOND | ARROW | V | ECHELON'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_range', default_value='true'),
        DeclareLaunchArgument('camera_fps', default_value='10.0'),
        DeclareLaunchArgument('range_publish_rate', default_value='20.0'),
        DeclareLaunchArgument('default_altitude', default_value='5.0',
                              description='ENU z (m). leader_pose 미수신 시 기본 고도'),
        DeclareLaunchArgument('obstacle_stop_dist', default_value='1.0'),
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

    return LaunchDescription(args + [OpaqueFunction(function=_build_bridges), formation])
