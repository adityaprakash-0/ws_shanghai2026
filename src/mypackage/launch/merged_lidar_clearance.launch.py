#!/usr/bin/env python3
"""Bring up dual YDLidar + merger + merged-scan clearance checker.

Needs hardware TF (control_server + drive + robot_state_publisher + EKF)
so the merger can transform scans into base_footprint.

  cd ~/skill23_ws && source install/setup.bash
  ros2 launch mypackage merged_lidar_clearance.launch.py robot:=grip

Watch:
  ros2 topic echo /merged_scan/clearance
  (also printed in the merged_scan_clearance log)

Optional FOV (degrees, half-width of each sector):
  ros2 launch mypackage merged_lidar_clearance.launch.py \\
    front_half_fov_deg:=30.0 left_half_fov_deg:=45.0
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('grip', 'mecanum', 'xdrive', 'allterrain'):
        raise RuntimeError(f"Unknown robot '{robot}'")

    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')
    cn_share = get_package_share_directory('skill23_custom_nav')
    robot_share = get_package_share_directory(robot)

    hw_params = os.path.join(cn_share, 'config', f'hw_params_{robot}.yaml')
    ekf_yaml = os.path.join(cn_share, 'config', 'ekf_custom_nav.yaml')
    y_front = os.path.join(studica_share, 'config', 'front_lidar_nav.yaml')
    y_back = os.path.join(studica_share, 'config', 'back_lidar_nav.yaml')
    exec_path = os.path.join(
        studica_prefix, 'lib', 'studica_control', 'manual_composition')

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
    urdf_file = os.path.join(robot_share, 'urdf', urdf_name)
    robot_desc = ''
    if os.path.isfile(urdf_file):
        with open(urdf_file, 'r', encoding='utf-8') as f:
            robot_desc = f.read()

    actions = []

    actions.append(ExecuteProcess(
        cmd=[
            'sudo', '-E', exec_path,
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', hw_params,
        ],
        output='screen',
    ))

    if robot_desc:
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ))

    actions.append(Node(
        package=drive[0],
        executable=drive[1],
        name=drive[2],
        output='screen',
    ))

    actions.append(TimerAction(period=1.5, actions=[
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml],
        ),
    ]))

    actions.append(TimerAction(period=2.0, actions=[
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_front',
            output='screen',
            parameters=[y_front],
            remappings=[('/scan', '/scan/front')],
        ),
    ]))
    actions.append(TimerAction(period=4.5, actions=[
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_back',
            output='screen',
            parameters=[y_back],
            remappings=[('/scan', '/scan/back')],
        ),
    ]))
    actions.append(TimerAction(period=5.5, actions=[
        Node(
            package='allterrain',
            executable='lidar_merger_node',
            name='lidar_merger_node',
            output='screen',
        ),
    ]))
    actions.append(TimerAction(period=6.5, actions=[
        Node(
            package='mypackage',
            executable='merged_scan_clearance',
            name='merged_scan_clearance',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'window_deg': float(
                    LaunchConfiguration('window_deg').perform(context)),
                'robot_length': 0.44 if robot == 'grip' else 0.44,
                'robot_width': 0.363 if robot == 'grip' else 0.40,
                'log_hz': 2.0,
            }],
        ),
    ]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='grip'),
        DeclareLaunchArgument(
            'window_deg', default_value='5.0',
            description='Half-window around each bearing (±deg) for min range'),
        OpaqueFunction(function=_setup),
    ])
