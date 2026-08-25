"""Launch hardware for open-loop odometry calibration tests.

  # Terminal 1:
  ros2 launch skill23_test odom_test.launch.py robot:=xdrive

  # Terminal 2:
  ros2 run skill23_test odom_calibrator --ros-args -p robot:=xdrive

Uncomment exactly ONE drive node block below if you prefer manual selection
instead of the automatic ``robot`` argument.
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
    enable_ekf = LaunchConfiguration('enable_ekf').perform(context).lower() in (
        '1', 'true', 'yes', 'on')

    if robot not in ('grip', 'mecanum', 'xdrive', 'allterrain'):
        raise RuntimeError(f"Unknown robot '{robot}'")

    studio_share = get_package_share_directory('skill23_studio')
    studica_prefix = get_package_prefix('studica_control')
    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')
    hw_params = os.path.join(studio_share, 'config', 'hw_params', f'hw_params_{robot}.yaml')

    robot_pkg, ekf_file = {
        'grip': ('grip', 'ekf_grip.yaml'),
        'mecanum': ('mecanum', 'ekf_mecanum.yaml'),
        'xdrive': ('xdrive', 'ekf_xdrive.yaml'),
        'allterrain': ('allterrain', 'ekf_allterrain.yaml'),
    }[robot]
    ekf_yaml = os.path.join(get_package_share_directory(robot_pkg), 'config', ekf_file)

    drive = {
        'grip': ('grip', 'grip_node', 'grip_node'),
        'mecanum': ('mecanum', 'mecanum_node', 'mecanum_node'),
        'xdrive': ('xdrive', 'xdrive_node', 'xdrive_node'),
        'allterrain': ('allterrain', 'allterrain_node', 'allterrain_node'),
    }[robot]

    actions = [
        LogInfo(msg=f'[odom_test] robot={robot} hw={hw_params} ekf={ekf_yaml}'),
        ExecuteProcess(cmd=_control_server_cmd(exec_path, hw_params), output='screen'),
        TimerAction(period=2.0, actions=[
            Node(package=drive[0], executable=drive[1], name=drive[2], output='screen'),
        ]),
    ]

    # ── Manual override: comment automatic drive above and uncomment ONE ────
    # TimerAction(period=2.0, actions=[
    #     Node(package='grip', executable='grip_node', name='grip_node', output='screen'),
    # ]),
    # TimerAction(period=2.0, actions=[
    #     Node(package='mecanum', executable='mecanum_node', name='mecanum_node', output='screen'),
    # ]),
    # TimerAction(period=2.0, actions=[
    #     Node(package='xdrive', executable='xdrive_node', name='xdrive_node', output='screen'),
    # ]),
    # TimerAction(period=2.0, actions=[
    #     Node(package='allterrain', executable='allterrain_node', name='allterrain_node', output='screen'),
    # ]),

    if enable_ekf:
        actions.append(TimerAction(period=3.0, actions=[
            Node(
                package='robot_localization', executable='ekf_node',
                name='ekf_filter_node', parameters=[ekf_yaml], output='screen'),
        ]))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='grip',
            description='grip | mecanum | xdrive | allterrain'),
        DeclareLaunchArgument(
            'enable_ekf', default_value='false',
            description='Optional EKF from robot package ekf_*.yaml (odom test uses /wheel/odom)'),
        OpaqueFunction(function=_setup),
    ])
