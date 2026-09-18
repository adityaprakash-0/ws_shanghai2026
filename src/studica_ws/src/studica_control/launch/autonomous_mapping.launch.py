"""Autonomous frontier mapping — hardware + SLAM + Nav2 SLAM mode + explore_lite.

  ros2 launch studica_control autonomous_mapping.launch.py robot:=xdrive

Starts Titan/control_server, drive node, dual lidars, EKF, slam_toolbox (sync),
Nav2 bringup in SLAM mode (no prior map), then explore_lite frontiers.

Requires packages: explore_lite, explore_lite_msgs (vendored under
src/m_explore_ros2), slam_toolbox, nav2_bringup.
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
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _control_server_cmd(exec_path: str, hw_params: str) -> list:
    return [
        'sudo', '-E', exec_path,
        '--ros-args', '-r', '__node:=control_server',
        '--params-file', hw_params,
    ]


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('grip', 'mecanum', 'xdrive', 'allterrain'):
        raise RuntimeError(f"Unknown robot '{robot}'")

    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')
    studio_share = get_package_share_directory('skill23_studio')
    robot_share = get_package_share_directory(robot)
    nav2_bringup = get_package_share_directory('nav2_bringup')
    explore_share = get_package_share_directory('explore_lite')

    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')
    hw_params = os.path.join(studio_share, 'config', 'hw_params', f'hw_params_{robot}.yaml')
    if not os.path.isfile(hw_params):
        hw_params = os.path.join(studica_share, 'config', f'params_{robot}.yaml')

    urdf_name = {
        'mecanum': 'mecanum_robot.urdf',
        'xdrive': 'xdrive_robot.urdf',
        'allterrain': 'allterrain_robot.urdf',
        'grip': 'grip_robot.urdf',
    }[robot]
    urdf_file = os.path.join(robot_share, 'urdf', urdf_name)
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    ekf_file = {
        'grip': 'ekf_grip.yaml',
        'mecanum': 'ekf_mecanum.yaml',
        'xdrive': 'ekf_xdrive.yaml',
        'allterrain': 'ekf_allterrain.yaml',
    }[robot]
    ekf_yaml = os.path.join(robot_share, 'config', ekf_file)
    nav2_params = os.path.join(robot_share, 'config', f'nav2_{robot}.yaml')
    if not os.path.isfile(nav2_params):
        nav2_params = os.path.join(studica_share, 'config', f'nav2_{robot}.yaml')

    slam_params = os.path.join(studica_share, 'config', 'slam_params.yaml')
    y_front = os.path.join(studica_share, 'config', 'front_lidar.yaml')
    y_back = os.path.join(studica_share, 'config', 'back_lidar.yaml')
    explore_params = os.path.join(explore_share, 'config', 'params.yaml')

    drive = {
        'grip': ('grip', 'grip_node', 'grip_node'),
        'mecanum': ('mecanum', 'mecanum_node', 'mecanum_node'),
        'xdrive': ('xdrive', 'xdrive_node', 'xdrive_node'),
        'allterrain': ('allterrain', 'allterrain_node', 'allterrain_node'),
    }[robot]

    nav2_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[explore_params, {
            'use_sim_time': False,
            'robot_base_frame': 'base_link',
            'costmap_topic': 'map',
            'costmap_updates_topic': 'map_updates',
            'visualize': True,
            'planner_frequency': 0.15,
            'progress_timeout': 30.0,
            'min_frontier_size': 0.5,
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    return [
        LogInfo(msg=f'[autonomous_mapping] robot={robot} hw={hw_params} '
                    f'slam={slam_params} explore={explore_params}'),
        ExecuteProcess(cmd=_control_server_cmd(exec_path, hw_params), output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_front',
            output='screen',
            parameters=[y_front],
            remappings=[('/scan', '/scan/front')],
        ),
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_back',
            output='screen',
            parameters=[y_back],
            remappings=[('/scan', '/scan/back')],
        ),
        Node(
            package='allterrain',
            executable='lidar_merger_node',
            name='lidar_merger_node',
            output='screen',
        ),
        TimerAction(period=2.0, actions=[
            Node(
                package=drive[0],
                executable=drive[1],
                name=drive[2],
                output='screen',
            ),
        ]),
        TimerAction(period=3.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_yaml],
            ),
        ]),
        TimerAction(period=5.0, actions=[
            LogInfo(msg='[autonomous_mapping] Starting SLAM Toolbox (sync)'),
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params],
            ),
        ]),
        TimerAction(period=10.0, actions=[
            LogInfo(msg='[autonomous_mapping] Starting Nav2 (navigation only, SLAM map)'),
            nav2_slam,
        ]),
        TimerAction(period=18.0, actions=[
            LogInfo(msg='[autonomous_mapping] Starting explore_lite frontier explorer'),
            explore_node,
        ]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='xdrive',
            description='grip | mecanum | xdrive | allterrain'),
        OpaqueFunction(function=_setup),
    ])
