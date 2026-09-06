"""OMS — single node: rotation, lift, linear, servos, boot homing.

  ros2 launch oms oms.launch.py
  ros2 launch oms oms.launch.py auto_home:=false
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('oms')
    yaml_path = os.path.join(share, 'config', 'oms.yaml')
    rot_log = os.path.expanduser('~/skill23_ws/studio_maps/_logs/oms_motion.csv')
    lift_log = os.path.expanduser('~/skill23_ws/studio_maps/_logs/oms_lift_motion.csv')
    linear_log = os.path.expanduser(
        '~/skill23_ws/studio_maps/_logs/oms_linear_motion.csv')
    oms = Node(
        package='oms',
        executable='oms_all',
        name='oms',
        parameters=[{
            'config_file': yaml_path,
            'motion_log': LaunchConfiguration('motion_log'),
            'lift_motion_log': LaunchConfiguration('lift_motion_log'),
            'linear_motion_log': LaunchConfiguration('linear_motion_log'),
            'auto_home': ParameterValue(
                LaunchConfiguration('auto_home'), value_type=bool),
        }],
        output='screen',
        emulate_tty=True,
        additional_env={'PYTHONUNBUFFERED': '1'},
    )
    return LaunchDescription([
        DeclareLaunchArgument('motion_log', default_value=rot_log),
        DeclareLaunchArgument('lift_motion_log', default_value=lift_log),
        DeclareLaunchArgument('linear_motion_log', default_value=linear_log),
        DeclareLaunchArgument(
            'auto_home', default_value='false',
            description='if true, run boot home; default false — Home only from Studio'),
        LogInfo(msg=[f'[oms] config → {yaml_path}']),
        LogInfo(msg=['[oms] rotation log → ', LaunchConfiguration('motion_log')]),
        LogInfo(msg=['[oms] lift log → ', LaunchConfiguration('lift_motion_log')]),
        LogInfo(msg=['[oms] linear log → ', LaunchConfiguration('linear_motion_log')]),
        oms,
    ])
