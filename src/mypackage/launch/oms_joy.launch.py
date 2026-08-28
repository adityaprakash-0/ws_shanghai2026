"""OMS gamepad teleop + allterrain HW. Robot drive gamepad is left ON.

Do NOT wrap this launch in sudo — control_server is already sudo -E inside.

  ros2 launch mypackage oms_joy.launch.py enable_servos:=true
  # HW already running:
  ros2 launch mypackage oms_joy.launch.py start_hw:=false
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
    start_hw = LaunchConfiguration('start_hw').perform(context).lower() in (
        '1', 'true', 'yes', 'on')
    start_joy = LaunchConfiguration('start_joy').perform(context).lower() in (
        '1', 'true', 'yes', 'on')
    params_file = os.path.expanduser(LaunchConfiguration('params_file').perform(context))
    if start_hw and not os.path.isfile(params_file):
        raise RuntimeError(f'HW params not found: {params_file}')

    actions = []

    if start_hw:
        exec_path = os.path.join(
            get_package_prefix('studica_control'),
            'lib', 'studica_control', 'manual_composition')
        cmd = [
            'sudo', '-E', exec_path,
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', params_file,
        ]
        if enable_servos:
            cmd += ['-p', 'servo.enabled:=true']
        actions.append(ExecuteProcess(cmd=cmd, output='screen'))
        actions.append(Node(
            package='allterrain',
            executable='allterrain_node',
            name='allterrain_node',
            output='screen',
            emulate_tty=True,
        ))

    if start_joy:
        actions.append(Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'autorepeat_rate': 20.0,
                'deadzone': 0.05,
            }],
        ))

    actions.append(Node(
        package='mypackage',
        executable='oms_joy_node',
        name='oms_joy_node',
        output='screen',
        emulate_tty=True,
    ))
    return actions


def generate_launch_description():
    studio_share = get_package_share_directory('skill23_studio')
    default_params = os.path.join(
        studio_share, 'config', 'hw_params', 'hw_params_allterrain.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Studica control_server params (gamepad left enabled)',
        ),
        DeclareLaunchArgument(
            'enable_servos',
            default_value='false',
            description='Override servo.enabled so OMS PWM servos actually start',
        ),
        DeclareLaunchArgument(
            'start_hw',
            default_value='true',
            description='Start control_server (set false if HW is already running)',
        ),
        DeclareLaunchArgument(
            'start_joy',
            default_value='true',
            description='Start joy_node (set false if /joy already exists)',
        ),
        OpaqueFunction(function=_setup),
    ])
