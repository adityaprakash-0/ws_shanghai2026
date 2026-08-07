"""Standalone pick-and-place launch (no Nav2).

Starts Studica hardware with PP params (titan0+titan1, servos, US/IR/Cobra),
drivetrain node, and all PP software nodes. Bridge listens on :8081.

  ros2 launch skill23_pick_place pick_place.launch.py robot:=mecanum
  ros2 launch skill23_pick_place pick_place.launch.py robot:=xdrive enable_vision:=true
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    enable_vision = LaunchConfiguration('enable_vision').perform(context).lower() in (
        '1', 'true', 'yes', 'on')
    http_port = int(LaunchConfiguration('http_port').perform(context))

    if robot not in ('allterrain', 'mecanum', 'xdrive', 'grip'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive | grip")

    pp_share = get_package_share_directory('skill23_pick_place')
    studica_prefix = get_package_prefix('studica_control')
    hw_params = os.path.join(pp_share, 'config', f'params_{robot}_pp.yaml')
    sw_params = os.path.join(pp_share, 'config', f'pp_{robot}.yaml')
    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')

    actions = []

    # 1) Studica hardware (root) — PP-only param file
    actions.append(ExecuteProcess(
        cmd=[
            'sudo', '-E', exec_path,
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', hw_params,
        ],
        output='screen',
    ))

    # 2) Drivetrain → /wheel/odom ; listens on /cmd_vel (relayed from /pp/cmd_vel)
    drive = {
        'allterrain': ('allterrain', 'allterrain_node', 'allterrain_node'),
        'mecanum': ('mecanum', 'mecanum_node', 'mecanum_node'),
        'xdrive': ('xdrive', 'xdrive_node', 'xdrive_node'),
        'grip': ('grip', 'grip_node', 'grip_node'),
    }[robot]
    actions.append(Node(
        package=drive[0],
        executable=drive[1],
        name=drive[2],
        output='screen',
    ))

    # 3) PP software stack
    common = {'parameters': [sw_params], 'output': 'screen'}

    actions.extend([
        Node(package='skill23_pick_place', executable='pp_odom_fusion',
             name='pp_odom_fusion', **common),
        Node(package='skill23_pick_place', executable='pp_sensor_fusion',
             name='pp_sensor_fusion', output='screen'),
        Node(package='skill23_pick_place', executable='pp_go_to_goal',
             name='pp_go_to_goal', **common),
        Node(package='skill23_pick_place', executable='pp_relative_move',
             name='pp_relative_move', **common),
        Node(package='skill23_pick_place', executable='pp_align_cobra',
             name='pp_align_cobra', **common),
        Node(package='skill23_pick_place', executable='pp_align_camera',
             name='pp_align_camera', **common),
        Node(package='skill23_pick_place', executable='pp_manipulator',
             name='pp_manipulator', **common),
        Node(package='skill23_pick_place', executable='pp_mission_manager',
             name='pp_mission', **common),
        Node(
            package='skill23_pick_place', executable='pp_bridge', name='pp_bridge',
            output='screen',
            parameters=[sw_params, {
                'robot': robot,
                'http_port': http_port,
                'maps_dir': os.path.expanduser('~/skill23_ws'),
            }],
        ),
    ])

    # 4) Optional vision (camera align / preview)
    if enable_vision:
        try:
            vision_share = get_package_share_directory('skill23_vision')
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vision_share, 'launch', 'vision.launch.py')),
                launch_arguments={
                    'enable_vision': 'true',
                    'launch_camera': LaunchConfiguration('launch_camera'),
                    'color_topic': LaunchConfiguration('color_topic'),
                }.items(),
            ))
        except Exception:
            pass

    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot', default_value='mecanum',
            description='Robot platform: allterrain | mecanum | xdrive | grip'),
        DeclareLaunchArgument('http_port', default_value='8081'),
        DeclareLaunchArgument(
            'enable_vision', default_value='false',
            description='Optional Orbbec + skill23_vision for camera align'),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument(
            'color_topic', default_value='/camera/color/image_raw'),
        OpaqueFunction(function=_setup),
    ])
