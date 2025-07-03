from setuptools import setup, find_packages
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
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        # Include rviz config files
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Bio-inspired SLAM for underwater vehicles',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processing_node = dolphin_slam.image_processing_node:main',
            'local_view_node = dolphin_slam.local_view_node:main',
            'place_cell_node = dolphin_slam.place_cell_node:main',
            'experience_map_node = dolphin_slam.experience_map_node:main',
            'robot_state_node = dolphin_slam.robot_state_node:main',
            'dataset_player_node = dolphin_slam.dataset_player_node:main',
        ],
    },
)
