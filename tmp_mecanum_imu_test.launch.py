"""
TEMPORARY — mecanum teleop + IMU logging for EKF jitter test.
Delete after the drive test.

Usage:
  source /home/vmx/skill23_ws/install/setup.bash
  ros2 launch /home/vmx/skill23_ws/tmp_mecanum_imu_test.launch.py

Gamepad drives /cmd_vel. IMU logs to tmp_imu_ekf_drive.csv.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')

    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(studica_share, 'config', 'params_mecanum.yaml')
    logger_script = '/home/vmx/skill23_ws/tmp_imu_ekf_logger.py'

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='studica_control params (IMU + gamepad + titan)',
    )

    # VMX hardware (IMU, gamepad, motors) — needs root for pigpio
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

    mecanum_node = Node(
        package='mecanum',
        executable='mecanum_node',
        name='mecanum_node',
        output='screen',
    )

    imu_logger = ExecuteProcess(
        cmd=['python3', logger_script],
        output='screen',
    )

    return LaunchDescription([
        params_arg,
        manual_composition,
        mecanum_node,
        imu_logger,
    ])
