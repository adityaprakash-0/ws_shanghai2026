"""Case-based navigation launch (no Nav2).

  ros2 launch skill23_case_nav case_nav.launch.py robot:=mecanum \\
    map:=/home/vmx/skill23_ws/test_map11.yaml

Starts Studica HW (US L/R; Sharp disabled), drivetrain, dual lidar + merger (/scan),
EKF localization, and the case-nav Python (rclpy) stack. Bridge on :8082.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    map_yaml = os.path.expanduser(LaunchConfiguration('map').perform(context))
    http_port = int(LaunchConfiguration('http_port').perform(context))
    enable_lidar = LaunchConfiguration('enable_lidar').perform(context).lower() in (
        '1', 'true', 'yes', 'on')

    if robot not in ('allterrain', 'mecanum', 'xdrive'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive")

    share = get_package_share_directory('skill23_case_nav')
    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')
    hw_params = os.path.join(share, 'config', f'hw_params_{robot}.yaml')
    sw_params = os.path.join(share, 'config', f'case_nav_{robot}.yaml')
    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')

    drive = {
        'allterrain': ('allterrain', 'allterrain_node', 'allterrain_node'),
        'mecanum': ('mecanum', 'mecanum_node', 'mecanum_node'),
        'xdrive': ('xdrive', 'xdrive_node', 'xdrive_node'),
    }[robot]

    urdf_pkg = {
        'mecanum': 'mecanum',
        'xdrive': 'xdrive',
        'allterrain': 'allterrain',
    }[robot]
    urdf_name = {
        'mecanum': 'mecanum_robot.urdf',
        'xdrive': 'xdrive_robot.urdf',
        'allterrain': 'allterrain_robot.urdf',
    }[robot]
    urdf_file = os.path.join(
        get_package_share_directory(urdf_pkg), 'urdf', urdf_name)
    robot_desc = ''
    if os.path.isfile(urdf_file):
        with open(urdf_file, 'r') as f:
            robot_desc = f.read()

    actions = []

    # 1) Studica hardware with case-nav sensor layout
    actions.append(ExecuteProcess(
        cmd=[
            'sudo', '-E', exec_path,
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', hw_params,
        ],
        output='screen',
    ))

    # 2) Robot state (URDF owns imu_link + laser_front/back at ±0.21, back yaw=π)
    if robot_desc:
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ))
    # Note: do not publish duplicate static TFs for imu/lasers — URDF provides them.
    # base_footprint ← base_link also comes from URDF (do not invert with a static TF).

    # 3) Drivetrain (differential cmd_vel: vx + wz only from FSM)
    actions.append(Node(
        package=drive[0],
        executable=drive[1],
        name=drive[2],
        output='screen',
    ))

    # 3b) EKF: /wheel/odom + /imu → /odometry/filtered
    ekf_yaml = os.path.join(share, 'config', 'ekf_case_nav.yaml')
    actions.append(TimerAction(period=2.5, actions=[Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
    )]))

    # 4) Dual lidar + merger → /scan (no /scan_filtered)
    if enable_lidar:
        y_front = os.path.join(studica_share, 'config', 'front_lidar.yaml')
        y_back = os.path.join(studica_share, 'config', 'back_lidar.yaml')
        # laser_* frames come from URDF (robot_state_publisher)
        actions.append(TimerAction(period=2.0, actions=[Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_front',
            output='screen',
            parameters=[y_front],
            remappings=[('/scan', '/scan/front')],
        )]))
        actions.append(TimerAction(period=5.0, actions=[Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_back',
            output='screen',
            parameters=[y_back],
            remappings=[('/scan', '/scan/back')],
        )]))
        actions.append(TimerAction(period=6.0, actions=[Node(
            package='allterrain',
            executable='lidar_merger_node',
            name='lidar_merger_node',
            output='screen',
        )]))

    # 5) Case-nav software stack
    common = {'parameters': [sw_params], 'output': 'screen'}
    map_params = [sw_params, {'map': map_yaml}] if map_yaml else [sw_params]

    actions.extend([
        Node(package='skill23_case_nav', executable='cn_map_server',
             name='cn_map_server', parameters=map_params, output='screen'),
        Node(package='skill23_case_nav', executable='cn_localization',
             name='cn_localization', **common),
        Node(package='skill23_case_nav', executable='cn_watchdog',
             name='cn_watchdog', **common),
        Node(package='skill23_case_nav', executable='cn_planner',
             name='cn_planner', **common),
        Node(package='skill23_case_nav', executable='cn_motion_fsm',
             name='cn_motion_fsm', **common),
        Node(package='skill23_case_nav', executable='cn_lidar_gate',
             name='cn_lidar_gate', **common),
        Node(
            package='skill23_case_nav', executable='cn_bridge', name='cn_bridge',
            output='screen',
            parameters=[sw_params, {
                'robot': robot,
                'http_port': http_port,
                'maps_dir': os.path.expanduser('~/skill23_ws'),
            }],
        ),
    ])

    return actions


def generate_launch_description():
    default_map = os.path.expanduser('~/skill23_ws/test_map11.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot', default_value='mecanum',
            description='Robot platform: allterrain | mecanum | xdrive'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('http_port', default_value='8082'),
        DeclareLaunchArgument(
            'enable_lidar', default_value='true',
            description='Start dual YDLidar + merger publishing /scan'),
        OpaqueFunction(function=_setup),
    ])
