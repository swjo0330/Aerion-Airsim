"""AERION Phase 2.3 — SimpleFlight 단일 드론 launch.

설계 의도:
  - bridge_node 1 프로세스 = vehicle 1개 (멀티드론 확장의 기본 단위).
  - AirSim settings.json의 vehicle key를 ROS2 namespace와 **동일 이름**(소문자 `drone1`)으로
    통일해 toplevel publisher가 자동으로 /drone1/camera/... 형태로 발행됨.
    (drone_controller.py 등이 토픽 prefix를 `f'/{vehicle_name}'` 절대 경로로 쓰는 구조라
     vehicle_name 자체를 표준 namespace 이름으로 두면 namespace remap 없이 자연 정합.)
  - control_backend='airsim_direct'는 SimpleFlight 직접 제어 (Phase 2~4).
    Phase 5 진입 시 'px4_mavros'로 바꿔서 PX4 SITL + MAVROS로 전환.

Usage:
    ros2 launch airsim_ros2_bridge aerion_single_drone.launch.py
    ros2 launch airsim_ros2_bridge aerion_single_drone.launch.py vehicle_name:=drone1 camera_fps:=10.0

사전 조건:
    UE Play 진행 중 + sf_1drone_phase2.json (vehicle key 'drone1') 적용 + CycloneDDS 환경변수 export.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('vehicle_name', default_value='drone1',
                              description='AirSim settings.json의 vehicle key (소문자 표준)'),
        DeclareLaunchArgument('namespace', default_value='drone1',
                              description='ROS2 namespace (vehicle_name과 동일 권장)'),
        DeclareLaunchArgument('airsim_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('airsim_port', default_value='41451'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_range', default_value='true'),
        DeclareLaunchArgument('camera_fps', default_value='10.0'),
        DeclareLaunchArgument('range_publish_rate', default_value='20.0'),
        DeclareLaunchArgument('control_backend', default_value='airsim_direct',
                              description='airsim_direct (SimpleFlight) | px4_mavros (Phase 5)'),
    ]

    bridge_node = Node(
        package='airsim_ros2_bridge',
        executable='bridge_node',
        name='airsim_bridge',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'airsim_ip': LaunchConfiguration('airsim_ip'),
            'airsim_port': LaunchConfiguration('airsim_port'),
            'enable_camera': LaunchConfiguration('enable_camera'),
            'enable_range': LaunchConfiguration('enable_range'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'range_publish_rate': LaunchConfiguration('range_publish_rate'),
            'enable_ardu_compat': False,
            'control_backend': LaunchConfiguration('control_backend'),
        }],
    )

    return LaunchDescription(args + [bridge_node])
