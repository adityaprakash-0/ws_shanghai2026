"""OMS bring-up: Studica control_server (allterrain HW) + keyboard OMS teleop.

  ros2 launch mypackage oms.launch.py
  ros2 launch mypackage oms.launch.py enable_servos:=true   # actually drive PWM servos

Uses skill23_studio hw_params_allterrain.yaml.
Servos stay disabled unless enable_servos:=true.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    enable_servos = LaunchConfiguration('enable_servos').perform(context).lower() in (
        '1', 'true', 'yes', 'on')
    params_file = os.path.expanduser(LaunchConfiguration('params_file').perform(context))
    if not os.path.isfile(params_file):
        raise RuntimeError(f'HW params not found: {params_file}')

    exec_path = os.path.join(
        get_package_prefix('studica_control'),
        'lib', 'studica_control', 'manual_composition')

    cmd = [
        'sudo', '-E', exec_path,
        '--ros-args',
        '-r', '__node:=control_server',
        '--params-file', params_file,
        '-p', 'gamepad.enabled:=false',
    ]
    if enable_servos:
        cmd += ['-p', 'servo.enabled:=true']

    control_server = ExecuteProcess(cmd=cmd, output='screen')

    oms = Node(
        package='mypackage',
        executable='oms_node',
        name='oms_node',
        output='screen',
        emulate_tty=True,
    )

    return [control_server, oms]


def generate_launch_description():
    studio_share = get_package_share_directory('skill23_studio')
    default_params = os.path.join(
        studio_share, 'config', 'hw_params', 'hw_params_allterrain.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Studica control_server params (default: studio allterrain HW)',
        ),
        DeclareLaunchArgument(
            'enable_servos',
            default_value='false',
            description='Override servo.enabled so OMS PWM servos actually start',
        ),
        OpaqueFunction(function=_setup),
    ])
