"""Mecanum colore mission HW — control_server + mecanum + PID + colore_detect.

  Terminal 1:
    ros2 launch studica_control mecanum_colore_task_m.py

  Terminal 2 (after HW is up — interactive color prompts):
    ros2 run mypackage task_manager_mecanum_colore

Starts:
  1. control_server (sudo) — Titan, IMU, light_tower, start_switch DIO 8, Colore USB
  2. mecanum_node
  3. pid_move + pid_turn  (pid_move supports /pid_move/cancel)
  4. colore_detect — watches /colore0/color_info, /colore_detect/watch → found

Does NOT start task_manager_mecanum_colore (needs interactive TTY for color names).

Params: config/params_mecanum_colore_task.yaml
  - dio.start_switch pin 8
  - light_tower enabled (override_hold_sec 6s for buzzer+green)
  - colore.enabled true
  - gamepad disabled
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_prefix = get_package_prefix('studica_control')
    pkg_share = get_package_share_directory('studica_control')

    exec_path = os.path.join(pkg_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(pkg_share, 'config', 'params_mecanum_colore_task.yaml')
    if not os.path.isfile(default_params):
        default_params = os.path.expanduser(
            '~/skill23_ws/src/studica_ws/src/studica_control/config/'
            'params_mecanum_colore_task.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='HW params (start_switch + light_tower + colore)',
    )

    manual_composition = ExecuteProcess(
        cmd=[
            'sudo', '-E', 'env',
            f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}',
            f'RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")}',
            f'FASTDDS_BUILTIN_TRANSPORTS={os.environ.get("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")}',
            f'ROS_LOCALHOST_ONLY={os.environ.get("ROS_LOCALHOST_ONLY", "0")}',
            f'PATH={os.environ.get("PATH", "")}',
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

    pid_move_node = Node(
        package='mecanum',
        executable='pid_move',
        name='pid_move',
        output='screen',
    )

    pid_turn_node = Node(
        package='mecanum',
        executable='pid_turn',
        name='pid_turn',
        output='screen',
    )

    colore_detect_node = Node(
        package='mypackage',
        executable='colore_detect',
        name='colore_detect',
        output='screen',
        parameters=[{
            'color_info_topic': '/colore0/color_info',
            'match_threshold': 0.08,
            'min_brightness': 0.02,
            'log_xy_hz': 2.0,   # live xy print while watching (0=off)
        }],
    )

    return LaunchDescription([
        params_arg,
        LogInfo(msg=(
            '[mecanum_colore_task] HW + PID + colore_detect ready. '
            'In another terminal: ros2 run mypackage task_manager_mecanum_colore'
        )),
        manual_composition,
        mecanum_node,
        pid_move_node,
        pid_turn_node,
        colore_detect_node,
    ])
