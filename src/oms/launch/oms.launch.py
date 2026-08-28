"""OMS motions. Rotation (Titan1 M2) + vertical servo.

  ros2 launch oms oms.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _oms_node(executable, name, yaml_path, extra_params=None):
    params = [yaml_path]
    if extra_params:
        params.append(extra_params)
    return Node(
        package='oms',
        executable=executable,
        name=name,
        parameters=params,
        output='screen',
        emulate_tty=True,
        additional_env={'PYTHONUNBUFFERED': '1'},
    )


def generate_launch_description():
    share = get_package_share_directory('oms')
    yaml_path = os.path.join(share, 'config', 'oms.yaml')
    default_log = os.path.expanduser(
        '~/skill23_ws/studio_maps/_logs/oms_motion.csv')
    return LaunchDescription([
        DeclareLaunchArgument(
            'motion_log',
            default_value=default_log,
            description='CSV: time, cmd, encoder ticks, computed angle, duty'),
        LogInfo(msg=['[oms] motion log → ', LaunchConfiguration('motion_log')]),
        _oms_node('oms_rotation', 'oms_rotation', yaml_path, {
            'motion_log': LaunchConfiguration('motion_log'),
            'motion_log_hz': 10.0,
        }),
        _oms_node('oms_vertical', 'oms_vertical', yaml_path),
    ])
