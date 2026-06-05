from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skill23_pick_place'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ADEV',
    maintainer_email='adityaprakash.main@gmail.com',
    description='Standalone pick-and-place module (separate from Nav2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pp_odom_fusion = skill23_pick_place.odom_fusion_node:main',
            'pp_sensor_fusion = skill23_pick_place.sensor_fusion_node:main',
            'pp_go_to_goal = skill23_pick_place.go_to_goal_node:main',
            'pp_relative_move = skill23_pick_place.relative_move_node:main',
            'pp_align_cobra = skill23_pick_place.align_cobra_node:main',
            'pp_align_camera = skill23_pick_place.align_camera_node:main',
            'pp_manipulator = skill23_pick_place.manipulator_node:main',
            'pp_mission_manager = skill23_pick_place.mission_manager_node:main',
            'pp_bridge = skill23_pick_place.bridge_node:main',
        ],
    },
)
