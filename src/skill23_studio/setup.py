from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skill23_studio'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), [
            f for f in glob('config/*')
            if os.path.isfile(f)
        ]),
        (os.path.join('share', package_name, 'config', 'nav2_params'),
            glob('config/nav2_params/*')),
        (os.path.join('share', package_name, 'config', 'hw_params'),
            glob('config/hw_params/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ADEV',
    maintainer_email='adityaprakash.main@gmail.com',
    description='WorldSkills Studio backend',
    license='MIT',
    entry_points={
        'console_scripts': [
            'studio_bridge = skill23_studio.studio_bridge_node:main',
            # Vendored custom_nav nodes (same names as original, different package)
            'cn_world_model = skill23_studio.nav.world_model_node:main',
            'cn_obstacle_detector = skill23_studio.nav.obstacle_detector_node:main',
            'cn_collision_checker = skill23_studio.nav.collision_checker_node:main',
            'cn_planner = skill23_studio.nav.planner_node:main',
            'cn_path_smoother = skill23_studio.nav.path_smoother_node:main',
            'cn_controller = skill23_studio.nav.controller_node:main',
            'cn_goal_checker = skill23_studio.nav.goal_checker_node:main',
            'cn_recovery = skill23_studio.nav.recovery_manager_node:main',
            'cn_mission = skill23_studio.nav.mission_manager_node:main',
            'cn_align_us = skill23_studio.nav.align_us_node:main',
            'cn_align_cobra = skill23_studio.nav.align_cobra_node:main',
            'cn_bridge = skill23_studio.nav.bridge_node:main',
            'studio_stop_stack = skill23_studio.process_manager:main_stop',
        ],
    },
)
