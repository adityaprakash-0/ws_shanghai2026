"""Shared Pi-side robot bringup for split-stack navigation (Nav2 runs on laptop)."""

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _delayed_gate(gate_node, period=10.0):
    return TimerAction(
        period=period,
        actions=[
            LogInfo(
                msg='[INIT] Pi sensors ready. Launching Confidence Scan Gate '
                    '(start Nav2 on laptop when connected to Pi hotspot).'
            ),
            gate_node,
        ],
    )


def build_pi_bringup_launch_description(
    *,
    robot_name: str,
    default_params: str,
    urdf_file: str,
    ekf_config_file: str,
    gate_params_file: str,
    drive_package: str,
    drive_executable: str,
    drive_node_name: str,
    enable_lidar_shutdown_handlers: bool = False,
):
    """Hardware + perception on Pi; Nav2 is started separately on the laptop."""
    studica_pkg_prefix = get_package_prefix('studica_control')
    studica_pkg_share = get_package_share_directory('studica_control')

    exec_path = os.path.join(studica_pkg_prefix, 'lib', 'studica_control', 'manual_composition')
    ydlidar_front_config = os.path.join(studica_pkg_share, 'config', 'front_lidar_nav.yaml')
    ydlidar_back_config = os.path.join(studica_pkg_share, 'config', 'back_lidar_nav.yaml')

    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)

    manual_composition = ExecuteProcess(
        cmd=[
            'sudo', '-E', exec_path,
            '--ros-args', '-r', '__node:=control_server',
            '--params-file', LaunchConfiguration('params_file'),
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_imu',
        arguments=['0', '0', '0.20', '0', '0', '0', '1', 'base_link', 'imu_link'],
    )
    laser_front_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_laser_front',
        arguments=['0.172', '0', '0.15', '0', '0', '0', '1', 'base_link', 'laser_front'],
    )
    laser_back_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_laser_back',
        arguments=['-0.172', '0', '0.15', '3.14159', '0', '0', 'base_link', 'laser_back'],
    )

    ydlidar_front_node = Node(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node', name='ydlidar_front',
        output='screen', parameters=[ydlidar_front_config],
        remappings=[('/scan', '/scan/front')],
    )
    ydlidar_back_node = Node(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node', name='ydlidar_back',
        output='screen', parameters=[ydlidar_back_config],
        remappings=[('/scan', '/scan/back')],
    )

    lidar_merger_node = Node(
        package='allterrain', executable='lidar_merger_node',
        name='lidar_merger_node', output='screen',
    )

    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_config_file],
    )

    drive_node = Node(
        package=drive_package, executable=drive_executable,
        name=drive_node_name, output='screen',
    )

    confidence_gate_node = Node(
        package='allterrain', executable='confidence_scan_gate',
        name='confidence_scan_gate_node', output='screen',
        parameters=[gate_params_file],
    )

    delayed_ekf = TimerAction(period=3.0, actions=[ekf_node])
    delayed_gate = _delayed_gate(confidence_gate_node)

    actions = [
        LogInfo(
            msg=f'[PI BRINGUP] {robot_name}: hardware + sensors only — '
                'Nav2 must run on laptop (same ROS_DOMAIN_ID, Pi hotspot).'
        ),
        params_arg,
        manual_composition,
        robot_state_publisher,
        imu_tf,
        laser_front_tf,
        laser_back_tf,
        ydlidar_front_node,
        ydlidar_back_node,
        lidar_merger_node,
        drive_node,
        delayed_ekf,
        delayed_gate,
    ]

    if enable_lidar_shutdown_handlers:
        actions.extend([
            RegisterEventHandler(
                OnProcessExit(
                    target_action=ydlidar_front_node,
                    on_exit=[
                        LogInfo(
                            msg='YDLidar Front process exited. Triggering clean system shutdown...'
                        ),
                        EmitEvent(event=Shutdown(reason='YDLidar Front closed cleanly')),
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=ydlidar_back_node,
                    on_exit=[
                        LogInfo(
                            msg='YDLidar Back process exited. Triggering clean system shutdown...'
                        ),
                        EmitEvent(event=Shutdown(reason='YDLidar Back closed cleanly')),
                    ],
                )
            ),
        ])

    return LaunchDescription(actions)
