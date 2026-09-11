"""Hardware bench — Titan control_server + dual lidars. No Nav2 / AMCL / OMS.

Used by Skill23 Studio Hardware Testing Software.

  ros2 launch skill23_studio test_hw.launch.py robot:=allterrain

Do NOT wrap in sudo. Stop Nav2 / localize / OMS first (duplicate Titan).
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
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from skill23_studio.ros_dds import studio_dds_launch_actions


def _node(package, executable, name, **kwargs):
    kwargs.setdefault('output', 'screen')
    return Node(package=package, executable=executable, name=name, **kwargs)


def _control_server_cmd(exec_path, hw_params, cpus='2,3'):
    cmd = ['sudo', '-E']
    if cpus:
        cmd += ['taskset', '-c', cpus]
    return cmd + [
        exec_path,
        '--ros-args',
        '-r', '__node:=control_server',
        '--params-file', hw_params,
    ]


def _hw_params(robot: str) -> str:
    try:
        share = get_package_share_directory('skill23_studio')
        path = os.path.join(share, 'config', 'hw_params', f'hw_params_{robot}.yaml')
        if os.path.isfile(path):
            return path
    except Exception:
        pass
    path = os.path.expanduser(
        f'~/skill23_ws/src/skill23_studio/config/hw_params/hw_params_{robot}.yaml')
    if os.path.isfile(path):
        return path
    raise RuntimeError(f'Missing hw_params_{robot}.yaml')


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    enable_lidar = LaunchConfiguration('enable_lidar').perform(context).lower() in (
        '1', 'true', 'yes', 'on')
    if robot not in ('allterrain', 'mecanum', 'xdrive', 'grip'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive | grip")

    hw_params = _hw_params(robot)
    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')
    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')
    if not os.path.isfile(exec_path):
        raise RuntimeError(f'manual_composition not found: {exec_path}')

    actions = [
        SetEnvironmentVariable(name='RCUTILS_LOGGING_BUFFERED_STREAM', value='1'),
        LogInfo(msg=f'[test_hw] robot={robot} params={hw_params} lidar={enable_lidar}'),
        ExecuteProcess(
            cmd=_control_server_cmd(exec_path, hw_params),
            output='screen',
            shell=False,
        ),
    ]

    if enable_lidar:
        y_front = os.path.join(studica_share, 'config', 'front_lidar_nav.yaml')
        y_back = os.path.join(studica_share, 'config', 'back_lidar_nav.yaml')
        if not os.path.isfile(y_front):
            y_front = os.path.join(studica_share, 'config', 'front_lidar.yaml')
        if not os.path.isfile(y_back):
            y_back = os.path.join(studica_share, 'config', 'back_lidar.yaml')
        actions.append(TimerAction(period=2.0, actions=[
            _node(
                'ydlidar_ros2_driver', 'ydlidar_ros2_driver_node', 'ydlidar_front',
                parameters=[y_front],
                remappings=[('/scan', '/scan/front')],
            ),
        ]))
        actions.append(TimerAction(period=4.5, actions=[
            _node(
                'ydlidar_ros2_driver', 'ydlidar_ros2_driver_node', 'ydlidar_back',
                parameters=[y_back],
                remappings=[('/scan', '/scan/back')],
            ),
        ]))
        actions.append(TimerAction(period=5.5, actions=[
            _node('allterrain', 'lidar_merger_node', 'lidar_merger_node'),
        ]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        *studio_dds_launch_actions(),
        DeclareLaunchArgument(
            'robot', default_value='allterrain',
            description='Robot platform: allterrain | mecanum | xdrive | grip'),
        DeclareLaunchArgument(
            'enable_lidar', default_value='true',
            description='Start dual YDLidar + merger'),
        OpaqueFunction(function=_setup),
    ])
