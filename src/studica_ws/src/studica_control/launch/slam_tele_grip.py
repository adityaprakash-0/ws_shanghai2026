import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    studica_pkg_prefix = get_package_prefix('studica_control')
    studica_pkg_share = get_package_share_directory('studica_control')
    grip_pkg_share = get_package_share_directory('grip')
    allterrain_pkg_share = get_package_share_directory('allterrain')

    exec_path = os.path.join(
        studica_pkg_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(studica_pkg_share, 'config', 'params_grip.yaml')
    urdf_file = os.path.join(grip_pkg_share, 'urdf', 'grip_robot.urdf')
    ekf_config_file = os.path.join(grip_pkg_share, 'config', 'ekf_grip.yaml')
    slam_config_file = os.path.join(studica_pkg_share, 'config', 'slam_params.yaml')
    gate_params_file = os.path.join(
        allterrain_pkg_share, 'config', 'confidence_scan_gate.yaml')

    ydlidar_front_config = os.path.join(studica_pkg_share, 'config', 'front_lidar.yaml')
    ydlidar_back_config = os.path.join(studica_pkg_share, 'config', 'back_lidar.yaml')

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

    ydlidar_front_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_front',
        output='screen',
        parameters=[ydlidar_front_config],
        remappings=[('/scan', '/scan/front')],
    )

    ydlidar_back_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_back',
        output='screen',
        parameters=[ydlidar_back_config],
        remappings=[('/scan', '/scan/back')],
    )

    lidar_merger_node = Node(
        package='allterrain',
        executable='lidar_merger_node',
        name='lidar_merger_node',
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
    )

    grip_node = Node(
        package='grip',
        executable='grip_node',
        name='grip_node',
        output='screen',
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file],
    )

    confidence_gate_node = Node(
        package='allterrain',
        executable='confidence_scan_gate',
        name='confidence_scan_gate_node',
        output='screen',
        parameters=[gate_params_file],
    )

    delayed_ekf = TimerAction(period=3.0, actions=[ekf_node])
    delayed_slam = TimerAction(period=5.0, actions=[slam_node])
    delayed_gate = TimerAction(period=12.0, actions=[confidence_gate_node])

    return LaunchDescription([
        params_arg,
        manual_composition,
        robot_state_publisher,
        ydlidar_front_node,
        ydlidar_back_node,
        grip_node,
        lidar_merger_node,
        delayed_ekf,
        delayed_slam,
      #  delayed_gate,
    ])
