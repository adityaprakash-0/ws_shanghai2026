from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skill23_custom_nav'

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
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ADEV',
    maintainer_email='adityaprakash.main@gmail.com',
    description='Lightweight custom navigation stack (no Nav2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cn_world_model = skill23_custom_nav.world_model_node:main',
            'cn_obstacle_detector = skill23_custom_nav.obstacle_detector_node:main',
            'cn_collision_checker = skill23_custom_nav.collision_checker_node:main',
            'cn_planner = skill23_custom_nav.planner_node:main',
            'cn_path_smoother = skill23_custom_nav.path_smoother_node:main',
            'cn_controller = skill23_custom_nav.controller_node:main',
            'cn_goal_checker = skill23_custom_nav.goal_checker_node:main',
            'cn_recovery = skill23_custom_nav.recovery_manager_node:main',
            'cn_mission = skill23_custom_nav.mission_manager_node:main',
            'cn_bridge = skill23_custom_nav.bridge_node:main',
        ],
    },
)
