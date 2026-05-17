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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'bridge_node = airsim_ros2_bridge.bridge_node:main',
            'ap_pose_cmdvel_probe = airsim_ros2_bridge.ap_pose_cmdvel_probe:main',
            # AERION Phase 4: 자체 구현 포메이션 노드 진입점.
            'aerion_formation = airsim_ros2_bridge.formation_node:main',
        ],
    },
)
