"""Mecanum PID task mission — HW + mecanum + PID + task_manager (start button).

  ros2 launch studica_control mecanum_task_m.py

Starts:
  1. control_server (sudo) — Titan, IMU, light_tower, start_switch DIO 8
  2. mecanum_node
  3. pid_move + pid_turn
  4. task_manager_mecanum — waits for START, then runs sequence
     (moving/stopped lights: light_tower follow_cmd_vel; finish: solid red)

Params: config/params_mecanum_task.yaml
  - dio.start_switch pin 8 (input)
  - light_tower enabled
  - gamepad disabled (so PID owns /cmd_vel)
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
    default_params = os.path.join(pkg_share, 'config', 'params_mecanum_task.yaml')
    # source-tree fallback
    if not os.path.isfile(default_params):
        default_params = os.path.expanduser(
            '~/skill23_ws/src/studica_ws/src/studica_control/config/'
            'params_mecanum_task.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='HW params (must enable dio start_switch + light_tower)',
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

    # light_tower (in control_server) already does moving/stopped via follow_cmd_vel
    # — do NOT also run mecanum light_status (would fight over /light_tower/set)

    task_manager_node = Node(
        package='mypackage',
        executable='task_manager_mecanum',
        name='task_manager_mecanum',
        output='screen',
        parameters=[{
            'start_topic': '/start_switch/state',
            'invert_start': True,
            'light_service': '/light_tower/set',
        }],
    )

    return LaunchDescription([
        params_arg,
        LogInfo(msg='[mecanum_task] HW + PID + task_manager — wait for START (DIO 8)'),
        manual_composition,
        mecanum_node,
        pid_move_node,
        pid_turn_node,
        task_manager_node,
    ])
