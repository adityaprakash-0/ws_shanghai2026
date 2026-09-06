"""Commissioning module — Nav2 + Gemini E + Detect+Align + OMS.

Does not start Titan a second time if you already ran OMS Software
standalone: Studio blocks that in the process manager.

  ros2 launch skill23_studio studio_commission.launch.py robot:=allterrain \\
    map:=/home/vmx/skill23_ws/studio_maps/foo/map.yaml
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('skill23_studio')
    nav2 = os.path.join(share, 'launch', 'studio_nav2.launch.py')
    default_map = os.path.expanduser('~/skill23_ws/test_map10.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='allterrain',
            description='Robot platform: allterrain | mecanum | xdrive | grip'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Absolute path to map.yaml'),
        DeclareLaunchArgument(
            'enable_lidar', default_value='true',
            description='Start dual YDLidar + merger'),
        DeclareLaunchArgument(
            'camera_stabilize_sec', default_value='12.0'),
        DeclareLaunchArgument('color_width', default_value='640'),
        DeclareLaunchArgument('color_height', default_value='360'),
        DeclareLaunchArgument('color_fps', default_value='5'),
        DeclareLaunchArgument(
            'with_detect_align', default_value='true',
            description='Start Detect+Align with the commissioning stack'),
        LogInfo(msg='[studio_commission] Nav2 + camera + yaw-align + OMS '
                    '(Detect+Align optional via with_detect_align)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2),
            launch_arguments={
                'robot': LaunchConfiguration('robot'),
                'map': LaunchConfiguration('map'),
                'enable_lidar': LaunchConfiguration('enable_lidar'),
                'with_camera': 'true',
                'with_align': 'true',
                'with_detect_align': LaunchConfiguration('with_detect_align'),
                'camera_stabilize_sec': LaunchConfiguration('camera_stabilize_sec'),
                'color_width': LaunchConfiguration('color_width'),
                'color_height': LaunchConfiguration('color_height'),
                'color_fps': LaunchConfiguration('color_fps'),
            }.items(),
        ),
    ])
