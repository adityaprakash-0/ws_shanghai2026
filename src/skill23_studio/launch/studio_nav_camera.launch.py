"""Camera deploy entry point for the Skill23 Studio navigation stack.

Same nav nodes as ``studio_nav.launch.py``. Selecting this file is how the
bridge records that Deploy asked for the camera: it launches the Orbbec node
after the basic nav startup delay, and Orbbec publishes on its own ``/camera/*``
topics that consumers subscribe to directly.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('skill23_studio')
    base_launch = os.path.join(share, 'launch', 'studio_nav.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='grip'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/skill23_ws/test_map11.yaml')),
        DeclareLaunchArgument('http_port', default_value='8084'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_bridge', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'robot': LaunchConfiguration('robot'),
                'map': LaunchConfiguration('map'),
                'http_port': LaunchConfiguration('http_port'),
                'enable_lidar': LaunchConfiguration('enable_lidar'),
                'enable_bridge': LaunchConfiguration('enable_bridge'),
            }.items(),
        ),
    ])
