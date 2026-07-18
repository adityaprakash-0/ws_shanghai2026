"""Nav2 + wheel sensor safety guard — mecanum."""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    EmitEvent,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')
    guard_share = get_package_share_directory('skill23_nav_guard')
    allterrain_share = get_package_share_directory('allterrain')
    mecanum_share = get_package_share_directory('mecanum')

    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(guard_share, 'config', 'hardware', 'params_mecanum_guard.yaml')
    guard_params = os.path.join(guard_share, 'config', 'guard_mecanum.yaml')
    urdf_file = os.path.join(mecanum_share, 'urdf', 'mecanum_robot.urdf')
    ekf_config_file = os.path.join(mecanum_share, 'config', 'ekf_mecanum.yaml')
    default_nav2_params_path = os.path.join(mecanum_share, 'config', 'nav2_mecanum.yaml')
    default_map_path = os.path.expanduser('~/skill23_ws/test_map11.yaml')
    gate_params_file = os.path.join(allterrain_share, 'config', 'confidence_scan_gate.yaml')
    ydlidar_front_config = os.path.join(studica_share, 'config', 'front_lidar_nav.yaml')
    ydlidar_back_config = os.path.join(studica_share, 'config', 'back_lidar_nav.yaml')
    nav2_launch_file = os.path.join(guard_share, 'launch', 'nav2', 'bringup_launch.py')

    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    map_arg = DeclareLaunchArgument('map', default_value=default_map_path)
    nav2_params_arg = DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params_path)

    manual_composition = ExecuteProcess(
        cmd=['sudo', '-E', exec_path, '--ros-args', '-r', '__node:=control_server',
             '--params-file', LaunchConfiguration('params_file')],
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

    # Wheel-corner sensor mounts (adjust to your physical wiring)
    tf_us_fl = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_ultrasonic_fl',
        arguments=['0.17', '0.20', '0.05', '0', '0', '0', '1', 'base_link', 'ultrasonic_fl_link'],
    )
    tf_ir_fr = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_ir_fr',
        arguments=['0.17', '-0.20', '0.05', '0', '0', '0', '1', 'base_link', 'ir_fr_link'],
    )
    tf_us_rl = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_ultrasonic_rl',
        arguments=['-0.17', '0.20', '0.05', '0', '0', '0', '1', 'base_link', 'ultrasonic_rl_link'],
    )
    tf_ir_rr = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_ir_rr',
        arguments=['-0.17', '-0.20', '0.05', '0', '0', '0', '1', 'base_link', 'ir_rr_link'],
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
        package='allterrain', executable='lidar_merger_node', name='lidar_merger_node', output='screen',
    )

    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_config_file],
    )

    mecanum_node = Node(
        package='mecanum', executable='mecanum_node', name='mecanum_node', output='screen',
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': 'true',
        }.items(),
    )

    # --- DISABLED for Pi CPU testing (gate is already C++ in allterrain) ---
    # confidence_gate_node = Node(
    #     package='allterrain', executable='confidence_scan_gate',
    #     name='confidence_scan_gate_node', output='screen',
    #     parameters=[gate_params_file],
    # )
    scan_filtered_relay = Node(
        package='allterrain',
        executable='scan_relay',
        name='scan_filtered_relay',
        output='screen',
        parameters=[{
            'input_scan_topic': '/scan',
            'output_scan_topic': '/scan_filtered',
        }],
    )

    wheel_guard_node = Node(
        package='skill23_nav_guard',
        executable='wheel_safety_guard',
        name='wheel_safety_guard',
        output='screen',
        parameters=[guard_params],
    )

    delayed_ekf = TimerAction(period=3.0, actions=[ekf_node])
    delayed_nav2_cmd = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg='[nav_guard] Hardware & EKF ready — launching Nav2 (composed) + scan relay...'),
            scan_filtered_relay,
            nav2_bringup_cmd,
        ],
    )
    delayed_guard = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[nav_guard] Launching wheel safety guard (cmd_vel relay)...'),
            wheel_guard_node,
        ],
    )

    shutdown_handler_front = RegisterEventHandler(
        OnProcessExit(
            target_action=ydlidar_front_node,
            on_exit=[
                LogInfo(msg='YDLidar Front exited — shutting down.'),
                EmitEvent(event=Shutdown(reason='YDLidar Front closed cleanly')),
            ],
        )
    )
    shutdown_handler_back = RegisterEventHandler(
        OnProcessExit(
            target_action=ydlidar_back_node,
            on_exit=[
                LogInfo(msg='YDLidar Back exited — shutting down.'),
                EmitEvent(event=Shutdown(reason='YDLidar Back closed cleanly')),
            ],
        )
    )

    return LaunchDescription([
        params_arg,
        map_arg,
        nav2_params_arg,
        manual_composition,
        robot_state_publisher,
        imu_tf,
        laser_front_tf,
        laser_back_tf,
        tf_us_fl,
        tf_ir_fr,
        tf_us_rl,
        tf_ir_rr,
        ydlidar_front_node,
        ydlidar_back_node,
        lidar_merger_node,
        mecanum_node,
        delayed_ekf,
        delayed_nav2_cmd,
        delayed_guard,
        shutdown_handler_front,
        shutdown_handler_back,
    ])
