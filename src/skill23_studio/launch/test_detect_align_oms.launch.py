"""Bench Detect+Align OMS pick — camera + Titan + OMS stack.

  ros2 launch skill23_studio test_detect_align_oms.launch.py robot:=allterrain
  ros2 run skill23_studio studio_detect_align_oms_test --ros-args \\
      --params-file ~/skill23_ws/src/skill23_studio/config/detect_align_oms_pick.yaml
  # Terminal 2: type start + Enter  (or publish Empty on start_topic)
  rqt_image_view /studio/detect_align/debug_image

Vision-only bench (no OMS): test_detect_align.launch.py + studio_detect_align_test
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
from launch_ros.actions import Node

from skill23_studio.ros_dds import studio_dds_launch_actions
from skill23_studio.orbbec_dabai import (
    clear_orbbec_device_lock,
    dabai_launch_args,
    orbbec_launch_path,
    prefer_workspace_orbbec,
    wake_orbbec_usb,
)


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
        '-p', 'servo.enabled:=true',
    ]


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('allterrain', 'mecanum', 'xdrive', 'grip'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive | grip")

    share = get_package_share_directory('skill23_studio')
    studica_prefix = get_package_prefix('studica_control')
    hw_params = os.path.join(share, 'config', 'hw_params', f'hw_params_{robot}.yaml')
    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')
    overlay = prefer_workspace_orbbec()
    dabai_launch = orbbec_launch_path()

    drive = {
        'allterrain': ('allterrain', 'allterrain_node', 'allterrain_node'),
        'mecanum': ('mecanum', 'mecanum_node', 'mecanum_node'),
        'xdrive': ('xdrive', 'xdrive_node', 'xdrive_node'),
        'grip': ('grip', 'grip_node', 'grip_node'),
    }[robot]
    urdf_name = {
        'mecanum': 'mecanum_robot.urdf',
        'xdrive': 'xdrive_robot.urdf',
        'allterrain': 'allterrain_robot.urdf',
        'grip': 'grip_robot.urdf',
    }[robot]
    urdf_file = os.path.join(get_package_share_directory(robot), 'urdf', urdf_name)
    robot_desc = ''
    if os.path.isfile(urdf_file):
        with open(urdf_file, 'r', encoding='utf-8') as handle:
            robot_desc = handle.read()

    color_width = str(LaunchConfiguration('color_width').perform(context) or '640')
    color_height = str(LaunchConfiguration('color_height').perform(context) or '360')
    color_fps = str(LaunchConfiguration('color_fps').perform(context) or '5')
    cam_args = dabai_launch_args(
        color_width=color_width,
        color_height=color_height,
        color_fps=color_fps,
    )

    shm_note = clear_orbbec_device_lock()
    usb_note = wake_orbbec_usb()

    actions = [
        LogInfo(msg=(
            f'[test_detect_align_oms] robot={robot}  color={color_width}x{color_height}@{color_fps}fps  '
            f'orbbec={overlay}  {dabai_launch}  '
            'Terminal 2: ros2 run skill23_studio studio_detect_align_oms_test --ros-args '
            '--params-file ~/skill23_ws/src/skill23_studio/config/detect_align_oms_pick.yaml  '
            'START: type start + Enter  '
            'debug: rqt_image_view /studio/detect_align/debug_image')),
        LogInfo(msg=(
            f'[test_detect_align_oms] orbbec shm lock: {shm_note or "clean"}  usb: {usb_note}')),
        ExecuteProcess(cmd=_control_server_cmd(exec_path, hw_params), output='screen'),
        _node(drive[0], drive[1], drive[2]),
        LogInfo(msg=(
            f'[test_detect_align_oms] Gemini E {color_width}x{color_height} @ {color_fps}fps')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dabai_launch),
            launch_arguments=cam_args.items(),
        ),
    ]
    if robot_desc:
        actions.append(_node(
            'robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ))

    actions.append(TimerAction(period=7.0, actions=[
        LogInfo(msg='[test_detect_align_oms] OMS nodes (after Titan ready)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('oms'),
                'launch', 'oms.launch.py')),
        ),
    ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        *studio_dds_launch_actions(),
        DeclareLaunchArgument('robot', default_value='allterrain',
                              description='allterrain | mecanum | xdrive | grip'),
        DeclareLaunchArgument('color_width', default_value='640', description='Gemini E color width'),
        DeclareLaunchArgument('color_height', default_value='360',
                              description='Gemini E color height (native 360)'),
        DeclareLaunchArgument('color_fps', default_value='5',
                              description='Color + depth fps (Gemini E min ~5)'),
        OpaqueFunction(function=_setup),
    ])
