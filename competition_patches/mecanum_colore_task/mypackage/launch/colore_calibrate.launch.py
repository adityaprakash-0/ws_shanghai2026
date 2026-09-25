"""Interactive Colore calibration: start HW, collect samples, print reference xy."""

import os
import re

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _ask_color_name() -> str:
    print('\n========== COLORE CALIBRATION ==========', flush=True)
    while True:
        name = input(
            'Enter the color name to calibrate (example: red or yellow): '
        ).strip().lower()
        if re.fullmatch(r'[a-z][a-z0-9_-]*', name):
            break
        print('Use letters/numbers/_/- and start with a letter.', flush=True)
    input(
        f'\nPlace the "{name}" target under the sensor at competition height.\n'
        'Keep the lighting the same as the arena.\n'
        'Press ENTER when the target is steady: ')
    print('Starting sensor and collecting readings...', flush=True)
    return name


def generate_launch_description():
    color_name = _ask_color_name()
    control_prefix = get_package_prefix('studica_control')
    control_share = get_package_share_directory('studica_control')
    executable = os.path.join(
        control_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(
        control_share, 'config', 'params_mecanum_colore_task.yaml')
    if not os.path.isfile(default_params):
        default_params = os.path.expanduser(
            '~/skill23_ws/src/studica_ws/src/studica_control/config/'
            'params_mecanum_colore_task.yaml')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='HW params with colore.enabled=true')
    control_server = ExecuteProcess(
        cmd=[
            'sudo', '-E', 'env',
            f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}',
            f'RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")}',
            f'FASTDDS_BUILTIN_TRANSPORTS={os.environ.get("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")}',
            f'ROS_LOCALHOST_ONLY={os.environ.get("ROS_LOCALHOST_ONLY", "0")}',
            f'PATH={os.environ.get("PATH", "")}',
            f'AMENT_PREFIX_PATH={os.environ.get("AMENT_PREFIX_PATH", "")}',
            f'LD_LIBRARY_PATH={os.environ.get("LD_LIBRARY_PATH", "")}',
            executable, '--ros-args', '-r', '__node:=control_server',
            '--params-file', LaunchConfiguration('params_file'),
        ],
        output='screen',
    )
    calibrator = Node(
        package='mypackage', executable='colore_calibrate',
        name='colore_calibrate', output='screen', emulate_tty=True,
        parameters=[{
            'color_name': color_name,
            'sample_count': 40,
            'color_info_topic': '/colore0/color_info',
            'min_brightness': 0.001,
        }],
    )
    return LaunchDescription([
        params_arg,
        LogInfo(msg=(
            f'[colore_calibrate] Calibrating "{color_name}" with 40 readings')),
        control_server,
        calibrator,
    ])
