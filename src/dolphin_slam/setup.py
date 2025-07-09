from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dolphin_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cetc3',
    maintainer_email='cetc3@example.com',
    description='Dolphin SLAM package for underwater environments',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 核心节点
            'image_processing_node = dolphin_slam.image_processing_node:main',
            'local_view_node = dolphin_slam.local_view_node:main',
            'place_cell_node = dolphin_slam.place_cell_node:main',
            'experience_map_node = dolphin_slam.experience_map_node:main',
            'robot_state_node = dolphin_slam.robot_state_node:main',
            'dataset_player_node = dolphin_slam.dataset_player_node:main',
            
            # 仿真相关节点
            'simple_odom_publisher_node = nodes.simple_odom_publisher_node:main',
            'simple_waypoint_controller_node = nodes.simple_waypoint_controller_node:main',
            'trajectory_evaluator_node = nodes.trajectory_evaluator_node:main',
            
            # 如果将来要添加增强版控制器
            'enhanced_waypoint_controller_node = nodes.enhanced_waypoint_controller_node:main',
        ],
    },
)
