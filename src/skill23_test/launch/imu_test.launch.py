"""Launch hardware for raw IMU calibration (no EKF).

  # Terminal 1:
  ros2 launch skill23_test imu_test.launch.py robot:=xdrive

  # Terminal 2:
  ros2 run skill23_test imu_calibrator --ros-args -p robot:=xdrive
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _control_server_cmd(exec_path: str, hw_params: str, cpus: str = '2,3') -> list:
    cmd = ['sudo', '-E']
    if cpus:
        cmd += ['taskset', '-c', cpus]
    return cmd + [
        exec_path,
        '--ros-args',
        '-r', '__node:=control_server',
        '--params-file', hw_params,
    ]


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)

    if robot not in ('grip', 'mecanum', 'xdrive', 'allterrain'):
        raise RuntimeError(f"Unknown robot '{robot}'")

    studio_share = get_package_share_directory('skill23_studio')
    studica_prefix = get_package_prefix('studica_control')
    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')
    hw_params = os.path.join(studio_share, 'config', 'hw_params', f'hw_params_{robot}.yaml')

    drive = {
        'grip': ('grip', 'grip_node', 'grip_node'),
        'mecanum': ('mecanum', 'mecanum_node', 'mecanum_node'),
        'xdrive': ('xdrive', 'xdrive_node', 'xdrive_node'),
        'allterrain': ('allterrain', 'allterrain_node', 'allterrain_node'),
    }[robot]

    return [
        LogInfo(msg=f'[imu_test] robot={robot} hw={hw_params} (raw /imu, no EKF)'),
        ExecuteProcess(cmd=_control_server_cmd(exec_path, hw_params), output='screen'),
        TimerAction(period=2.0, actions=[
            Node(package=drive[0], executable=drive[1], name=drive[2], output='screen'),
        ]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='grip',
            description='grip | mecanum | xdrive | allterrain'),
        OpaqueFunction(function=_setup),
    ])
