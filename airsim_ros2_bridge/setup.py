import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'airsim_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'bridge_node = airsim_ros2_bridge.bridge_node:main',
            'ap_pose_cmdvel_probe = airsim_ros2_bridge.ap_pose_cmdvel_probe:main',
            # AERION Phase 4: 자체 구현 포메이션 노드 진입점.
            'aerion_formation = airsim_ros2_bridge.formation_node:main',
            # AERION Phase 4: dummy leader_pose 자동 발행 (외부 mission planner 없는 단일 머신 시연).
            'aerion_leader = airsim_ros2_bridge.leader_publisher:main',
            # AERION Phase 3+: TF tree publisher (map → droneN/odom → droneN/base_link → ...).
            'aerion_tf = airsim_ros2_bridge.tf_publisher:main',
            # AERION Phase 4-Δ: 포메이션 데모 CLI (--demo morphing-cycle).
            'aerion_formation_demo = airsim_ros2_bridge.formation_demo:main',
            # AERION PX4/MAVROS: keyboard velocity teleop for lag validation.
            'aerion_manual_mavros_control = airsim_ros2_bridge.manual_mavros_control:main',
        ],
    },
)
