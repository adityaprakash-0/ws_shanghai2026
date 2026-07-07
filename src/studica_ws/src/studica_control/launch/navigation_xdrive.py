import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    studica_pkg_prefix = get_package_prefix('studica_control')
    studica_pkg_share = get_package_share_directory('studica_control')
    allterrain_pkg_share = get_package_share_directory('allterrain')
    xdrive_pkg_share = get_package_share_directory('xdrive')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    exec_path = os.path.join(studica_pkg_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(studica_pkg_share, 'config', 'params_xdrive.yaml')
    urdf_file = os.path.join(xdrive_pkg_share, 'urdf', 'xdrive_robot.urdf')
    ekf_config_file = os.path.join(xdrive_pkg_share, 'config', 'ekf_xdrive.yaml')
    default_nav2_params_path = os.path.join(xdrive_pkg_share, 'config', 'nav2_xdrive.yaml')
    default_map_path = os.path.expanduser('~/skill23_ws/test_map10.yaml')
    gate_params_file = os.path.join(allterrain_pkg_share, 'config', 'confidence_scan_gate.yaml')

    ydlidar_front_config = os.path.join(studica_pkg_share, 'config', 'front_lidar_nav.yaml')
    ydlidar_back_config = os.path.join(studica_pkg_share, 'config', 'back_lidar_nav.yaml')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    with open(urdf_file, 'r') as infp:
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
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
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
        package='allterrain', executable='lidar_merger_node', name='lidar_merger_node', output='screen',
    )

    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_config_file],
    )

    xdrive_node = Node(
        package='xdrive', executable='xdrive_node', name='xdrive_node', output='screen',
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

    confidence_gate_node = Node(
        package='allterrain', executable='confidence_scan_gate',
        name='confidence_scan_gate_node', output='screen',
        parameters=[gate_params_file],
    )

    delayed_ekf = TimerAction(period=3.0, actions=[ekf_node])
    delayed_nav2_cmd = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg='[INIT] Hardware & EKF warm-up complete. Launching Nav2 Stack...'),
            nav2_bringup_cmd,
        ],
    )
    delayed_gate = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[INIT] Map & Nav2 active. Launching Confidence Scan Gate...'),
            confidence_gate_node,
        ],
    )

    shutdown_handler_front = RegisterEventHandler(
        OnProcessExit(
            target_action=ydlidar_front_node,
            on_exit=[
                LogInfo(msg='YDLidar Front process exited. Triggering clean system shutdown...'),
                EmitEvent(event=Shutdown(reason='YDLidar Front closed cleanly')),
            ],
        )
    )
    shutdown_handler_back = RegisterEventHandler(
        OnProcessExit(
            target_action=ydlidar_back_node,
            on_exit=[
                LogInfo(msg='YDLidar Back process exited. Triggering clean system shutdown...'),
                EmitEvent(event=Shutdown(reason='YDLidar Back closed cleanly')),
            ],
        )
    )

    return LaunchDescription([
        params_arg, map_arg, nav2_params_arg,
        manual_composition, robot_state_publisher,
        imu_tf, laser_front_tf, laser_back_tf,
        ydlidar_front_node, ydlidar_back_node, lidar_merger_node,
        xdrive_node, delayed_ekf, delayed_nav2_cmd, delayed_gate,
        shutdown_handler_front, shutdown_handler_back,
    ])
