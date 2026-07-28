"""Bring up Studica HW (params_mecanum) + light_tower_play for manual testing.

  ros2 launch mypackage light_tower_test.launch.py

Manual commands while running:
  ros2 service call /light_tower/set studica_control/srv/SetData "{params: 'green'}"
  ros2 service call /light_tower/set studica_control/srv/SetData "{params: 'red:blink_hw'}"
  ros2 topic echo /light_tower/state
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_prefix = get_package_prefix('studica_control')
    pkg_share = get_package_share_directory('studica_control')
    exec_path = os.path.join(pkg_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(pkg_share, 'config', 'params_mecanum.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='params.yaml with light_tower.enabled and pins',
    )
    once_arg = DeclareLaunchArgument(
        'once_cmd',
        default_value='',
        description='If set, send this once instead of cycling (e.g. green)',
    )

    manual_composition = ExecuteProcess(
        cmd=[
            'sudo', '-E', exec_path,
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', LaunchConfiguration('params_file'),
        ],
        output='screen',
    )

    play = Node(
        package='mypackage',
        executable='light_tower_play',
        name='light_tower_play',
        output='screen',
        parameters=[{
            'once_cmd': LaunchConfiguration('once_cmd'),
            'period_sec': 3.0,
        }],
    )

    return LaunchDescription([
        params_arg,
        once_arg,
        manual_composition,
        play,
    ])
