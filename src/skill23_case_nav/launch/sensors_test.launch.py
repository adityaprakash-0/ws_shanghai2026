"""Minimal HW launch — Studica only, so you can echo US ranges.

No drivetrain, no lidars, no case-nav stack. Sharp IR is disabled in hw_params.

  ros2 launch skill23_case_nav sensors_test.launch.py robot:=mecanum

Then in other terminals:
  ros2 topic echo /ultrasonic_left/range --field range
  ros2 topic echo /ultrasonic_right/range --field range
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('allterrain', 'mecanum', 'xdrive'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive")

    share = get_package_share_directory('skill23_case_nav')
    studica_prefix = get_package_prefix('studica_control')
    hw_params = os.path.join(share, 'config', f'hw_params_{robot}.yaml')
    exec_path = os.path.join(
        studica_prefix, 'lib', 'studica_control', 'manual_composition')

    return [
        LogInfo(msg=f'[sensors_test] robot={robot} params={hw_params}'),
        LogInfo(msg='[sensors_test] Topics: '
                    '/ultrasonic_left/range /ultrasonic_right/range'),
        ExecuteProcess(
            cmd=[
                'sudo', '-E', exec_path,
                '--ros-args',
                '-r', '__node:=control_server',
                '--params-file', hw_params,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot', default_value='mecanum',
            description='Robot platform: allterrain | mecanum | xdrive'),
        OpaqueFunction(function=_setup),
    ])
