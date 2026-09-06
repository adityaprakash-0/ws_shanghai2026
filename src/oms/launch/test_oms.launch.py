"""Bench OMS — Titan control_server + OMS nodes. No Nav2, no camera.

Starts manual_composition (sudo -E inside) then all OMS nodes.
Boot homing runs via oms_home (rotation → lift → linear → vertical →
rotation_servo → gripper).

  ros2 run oms oms_test
  oms> 30
  oms> up
  oms> down


Do NOT wrap this launch in sudo. Stop Nav2 / localize / Detect+Align first
(duplicate Titan).

  # Terminal 1
  ros2 launch oms test_oms.launch.py
  ros2 launch oms test_oms.launch.py robot:=allterrain

  # Terminal 2 — type an angle, home, or stop:
  ros2 run oms oms_test
  ros2 run oms oms_test -- 30
  ros2 run oms oms_test -- home

  # Or from any terminal (absolute deg from home, −90…+90):
  ros2 topic pub --once /oms/rotation/set_angle std_msgs/msg/Float64 '{data: 30.0}'
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _control_server_cmd(exec_path, hw_params, cpus='2,3'):
    cmd = ['sudo', '-E']
    if cpus:
        cmd += ['taskset', '-c', cpus]
    return cmd + [
        exec_path,
        '--ros-args',
        '-r', '__node:=control_server',
        '--params-file', hw_params,
        '-p', 'servo.enabled:=true',
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
    if robot not in ('allterrain', 'mecanum', 'xdrive', 'grip'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive | grip")
    hw_params = _hw_params(robot)
    exec_path = os.path.join(
        get_package_prefix('studica_control'),
        'lib', 'studica_control', 'manual_composition')
    oms_launch = os.path.join(
        get_package_share_directory('oms'), 'launch', 'oms.launch.py')
    motion_log = os.path.expanduser(
        '~/skill23_ws/studio_maps/_logs/oms_motion.csv')
    auto_home = LaunchConfiguration('auto_home').perform(context)
    return [
        LogInfo(msg=(
            f'[test_oms] robot={robot}  hw={hw_params}  '
            f'auto_home={auto_home}  log→studio_maps/_logs/oms.log')),
        ExecuteProcess(cmd=_control_server_cmd(exec_path, hw_params), output='screen'),
        TimerAction(period=3.0, actions=[
            LogInfo(msg=f'[test_oms] OMS node — CSV {motion_log}'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(oms_launch),
                launch_arguments={
                    'motion_log': motion_log,
                    'auto_home': auto_home,
                }.items(),
            ),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot', default_value='allterrain',
            description='hw_params robot: allterrain | mecanum | xdrive | grip'),
        DeclareLaunchArgument(
            'auto_home', default_value='false',
            description='boot home sequence (false = Home only from Studio)'),
        OpaqueFunction(function=_setup),
    ])
