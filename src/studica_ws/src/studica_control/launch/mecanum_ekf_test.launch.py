"""
mecanum_ekf_test.launch.py — hardware + mecanum + EKF + core test node.

Core tests only: 1 stationary | 2 straight 2m | 3 square | drive (free)

  source install/setup.bash
  ros2 launch studica_control mecanum_ekf_test.launch.py

  ros2 topic pub --once /ekf_test/cmd std_msgs/msg/String "{data: 'help'}"
  ros2 topic pub --once /ekf_test/cmd std_msgs/msg/String "{data: 'start 1'}"
  ros2 topic pub --once /ekf_test/cmd std_msgs/msg/String "{data: 'done'}"
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')
    mecanum_share = get_package_share_directory('mecanum')

    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(studica_share, 'config', 'params_mecanum.yaml')
    ekf_config = os.path.join(mecanum_share, 'config', 'ekf_mecanum.yaml')
    urdf_file = os.path.join(mecanum_share, 'urdf', 'mecanum_robot.urdf')
    test_script = os.path.join(studica_prefix, 'lib', 'studica_control', 'ekf_mecanum_test.py')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='studica_control params (IMU + gamepad + titan)',
    )

    # 1. VMX hardware (root)
    manual_composition = ExecuteProcess(
        cmd=[
            'sudo', '-E', 'env',
            f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}',
            f'RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")}',
            f'AMENT_PREFIX_PATH={os.environ.get("AMENT_PREFIX_PATH", "")}',
            f'LD_LIBRARY_PATH={os.environ.get("LD_LIBRARY_PATH", "")}',
            exec_path,
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', LaunchConfiguration('params_file'),
        ],
        output='screen',
    )

    # 2. TF tree for EKF — imu_link from URDF (do not duplicate static TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 3. Wheel odometry + motors
    mecanum_node = Node(
        package='mecanum',
        executable='mecanum_node',
        name='mecanum_node',
        output='screen',
    )

    # 4. EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    # 5. Interactive test monitor
    ekf_test = ExecuteProcess(
        cmd=['python3', test_script],
        output='screen',
    )

    delayed_ekf = TimerAction(period=3.0, actions=[ekf_node])
    delayed_test = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[EKF TEST] Starting ekf_mecanum_test — send: start 1'),
            ekf_test,
        ],
    )

    return LaunchDescription([
        params_arg,
        LogInfo(msg='[EKF TEST] Launching mecanum hardware + EKF + test node'),
        manual_composition,
        robot_state_publisher,
        mecanum_node,
        delayed_ekf,
        delayed_test,
    ])
