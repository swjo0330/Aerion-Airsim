"""AERION Phase 3 — 1~5대 멀티드론 launch (1 드론 = 1 브릿지 프로세스 패턴).

각 드론마다 ros2 Node를 별도 프로세스(spawn)로 띄움. 이유:
  - AirSim RPC IOLoop 재진입 방지 (microsoft/AirSim#2607).
  - 한 드론 RPC 타임아웃이 다른 드론에 0 영향 (격리).
  - 노드별 CPU/메모리/로그 분리 → 디버깅 효율.

매핑 규칙 (settings.json ↔ launch ↔ ROS 토픽):
    AirSim vehicle key = "drone{N}" (소문자, 1..5)  ← settings.json `Vehicles` 키와 동일
    ROS2 namespace     = "/drone{N}"                ← 토픽이 자동으로 /drone1/camera/... 형태로 발행됨
    bridge 프로세스 1개 = vehicle 1개

5대 모두 enable_camera=true는 GPU/대역폭 큰 부담 (CarlaAir 벤치 RTX 3050 기준 1280x720 1.5Hz).
1차 검증은 320x240 또는 일부 드론만 카메라 활성. enable_range는 거의 부담 없음.

Usage:
    ros2 launch airsim_ros2_bridge aerion_multi_drone.launch.py drone_count:=5
    ros2 launch airsim_ros2_bridge aerion_multi_drone.launch.py drone_count:=3 enable_camera:=false

사전 조건:
    1) UE 에디터에 sf_5drones_phase3.json (또는 drone_count에 맞는 settings) 적용된 상태
    2) UE Play 진행 중 (AirSim RPC :41451 살아 있음)
    3) cyclonedds.xml의 SocketReceiveBufferSize min=10MB ≤ sysctl net.core.rmem_max (영구화 필요)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context, *args, **kwargs):
    drone_count = int(LaunchConfiguration('drone_count').perform(context))
    airsim_ip = LaunchConfiguration('airsim_ip').perform(context)
    airsim_port = int(LaunchConfiguration('airsim_port').perform(context))
    enable_camera = LaunchConfiguration('enable_camera').perform(context).lower() == 'true'
    enable_range = LaunchConfiguration('enable_range').perform(context).lower() == 'true'
    camera_fps = float(LaunchConfiguration('camera_fps').perform(context))
    range_publish_rate = float(LaunchConfiguration('range_publish_rate').perform(context))
    control_backend = LaunchConfiguration('control_backend').perform(context)

    if drone_count < 1 or drone_count > 5:
        raise ValueError(f'drone_count 1~5만 지원: {drone_count}')

    nodes = []
    for n in range(1, drone_count + 1):
        vehicle_name = f'drone{n}'       # AirSim settings.json key = ROS2 namespace 통일
        namespace = f'drone{n}'
        # 주의: vehicle_names (복수형 STRING_ARRAY)는 의도적으로 전달하지 않음.
        #   ros2 launch가 [LaunchConfiguration(...)]를 STRING으로 평가하면서 STRING_ARRAY 타입 에러
        #   유발함. bridge_node 측 _resolve_vehicle_specs가 vehicle_name(단수)이 비어있지 않으면
        #   단일 vehicle 모드로 진입하므로 vehicle_names 불필요.
        # RPC stagger: bridge_node가 vehicle별로 자체 timer 시작 시간을 가지므로 자연스럽게 분산.
        #   추가적인 명시적 offset은 향후 RangePublisher/CameraPublisher의 timer 첫 호출 sleep으로 구현 검토.
        nodes.append(Node(
            package='airsim_ros2_bridge',
            executable='bridge_node',
            name='airsim_bridge',
            namespace=namespace,
            output='screen',
            parameters=[{
                'vehicle_name': vehicle_name,
                'airsim_ip': airsim_ip,
                'airsim_port': airsim_port,
                'enable_camera': enable_camera,
                'enable_range': enable_range,
                'camera_fps': camera_fps,
                'range_publish_rate': range_publish_rate,
                'enable_ardu_compat': False,
                'control_backend': control_backend,
            }],
        ))
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument('drone_count', default_value='5',
                              description='1~5대 (settings.json의 Drone0..Drone{N-1}와 일치해야 함)'),
        DeclareLaunchArgument('airsim_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('airsim_port', default_value='41451'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_range', default_value='true'),
        DeclareLaunchArgument('camera_fps', default_value='10.0'),
        DeclareLaunchArgument('range_publish_rate', default_value='20.0'),
        DeclareLaunchArgument('control_backend', default_value='airsim_direct'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_build_nodes)])
