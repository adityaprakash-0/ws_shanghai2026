from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skill23_case_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ADEV',
    maintainer_email='adityaprakash.main@gmail.com',
    description='Case-based lightweight navigation (no Nav2) — pure rclpy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cn_map_server = skill23_case_nav.map_server_node:main',
            'cn_localization = skill23_case_nav.localization_node:main',
            'cn_planner = skill23_case_nav.planner_node:main',
            'cn_watchdog = skill23_case_nav.watchdog_node:main',
            'cn_motion_fsm = skill23_case_nav.motion_fsm_node:main',
            'cn_lidar_gate = skill23_case_nav.lidar_gate_node:main',
            'cn_bridge = skill23_case_nav.bridge_node:main',
        ],
    },
)
