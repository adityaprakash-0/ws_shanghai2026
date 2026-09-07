"""Bench Cobra align — hardware + cobra channels, no Nav2/AMCL.

Two skill23_studio executables:

  studio_cobra       ← nav/cobra_align_node.py  (centers on reflectance)
  studio_cobra_test  ← nav/cobra_test_trigger_node.py  (sends one cmd)

Also starts: control_server (sudo inside), drive node.
Enable cobra in config/hw_params/hw_params_<robot>.yaml before running.

  # Holonomic (strafe):
  ros2 launch skill23_studio test_wp_cobra.launch.py robot:=mecanum

  # Diff / no strafe (yaw centering):
  ros2 launch skill23_studio test_wp_cobra.launch.py robot:=allterrain

  # HW + cobra node only; send cmd later:
  ros2 launch skill23_studio test_wp_cobra.launch.py robot:=mecanum send:=false
  ros2 run skill23_studio studio_cobra_test --ros-args -p auto_send:=true -p delay_sec:=1.0

Do NOT sudo this launch. Stop Nav2 / localize first.
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
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from skill23_studio.cobra_paths import cobra_yaml_for_robot
from skill23_studio.ros_dds import studio_dds_launch_actions


def _node(package, executable, name, **kwargs):
    kwargs.setdefault('output', 'screen')
    kwargs.setdefault('emulate_tty', True)
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


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('allterrain', 'mecanum', 'xdrive', 'grip'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive | grip")

    share = get_package_share_directory('skill23_studio')
    studica_prefix = get_package_prefix('studica_control')
    hw_params = os.path.join(share, 'config', 'hw_params', f'hw_params_{robot}.yaml')
    yaml_path = cobra_yaml_for_robot(robot, share)
    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')

    drive = {
        'allterrain': ('allterrain', 'allterrain_node', 'allterrain_node'),
        'mecanum': ('mecanum', 'mecanum_node', 'mecanum_node'),
        'xdrive': ('xdrive', 'xdrive_node', 'xdrive_node'),
        'grip': ('grip', 'grip_node', 'grip_node'),
    }[robot]

    send = str(LaunchConfiguration('send').perform(context)).lower() in (
        '1', 'true', 'yes', 'on')
    delay = float(LaunchConfiguration('delay_sec').perform(context))
    if delay <= 0.0:
        delay = 5.0

    trigger = {
        'delay_sec': 0.5,
        'auto_send': send,
        'timeout_sec': float(LaunchConfiguration('timeout_sec').perform(context)),
    }

    actions = [
        LogInfo(msg=(
            f'[test_wp_cobra] robot={robot} yaml={yaml_path}  '
            'Enable cobra in hw_params. Nodes: studio_cobra + studio_cobra_test.')),
        ExecuteProcess(cmd=_control_server_cmd(exec_path, hw_params), output='screen'),
        _node(drive[0], drive[1], drive[2]),
        TimerAction(period=3.0, actions=[
            LogInfo(msg='[test_wp_cobra] studio_cobra ← cobra_align_node.py'),
            _node(
                'skill23_studio', 'studio_cobra', 'cobra_align',
                parameters=[yaml_path]),
        ]),
    ]
    if send:
        actions.append(TimerAction(
            period=delay,
            actions=[
                LogInfo(msg='[test_wp_cobra] studio_cobra_test ← cobra_test_trigger_node.py'),
                _node(
                    'skill23_studio', 'studio_cobra_test', 'cobra_test_trigger',
                    parameters=[trigger]),
            ],
        ))
    return actions


def generate_launch_description():
    def _arg(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    return LaunchDescription([
        *studio_dds_launch_actions(),
        _arg('robot', 'mecanum', 'allterrain | mecanum | xdrive | grip'),
        _arg('delay_sec', '0', 'When to send (0 = auto 5s)'),
        _arg('send', 'true', 'Auto-send the test command'),
        _arg('timeout_sec', '20.0', 'Cobra align timeout'),
        OpaqueFunction(function=_setup),
    ])
