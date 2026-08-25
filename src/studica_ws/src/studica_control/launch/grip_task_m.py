import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():

    pkg_prefix = get_package_prefix('studica_control')
    pkg_share = get_package_share_directory('studica_control')

    exec_path = os.path.join(pkg_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(pkg_share, 'config', 'params_grip.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to params.yaml. Override to use a config from your own project: '
                    'params_file:=/path/to/your/params.yaml',
    )

    # 1. Core Hardware Driver (Runs as root)
    manual_composition = ExecuteProcess(
        cmd=[
            'sudo', '-E', exec_path,
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', LaunchConfiguration('params_file'),
        ],
        output='screen',
    )

    # 2. Kinematics & Odometry
    grip_node = Node(
        package='grip',
        executable='grip_node',
        name='grip_node',
        output='screen'
    )

    # 3. PID Controllers
    pid_move_node = Node(
        package='grip',
        executable='pid_move',
        name='pid_move',
        output='screen'
    )

    pid_turn_node = Node(
        package='grip',
        executable='pid_turn',
        name='pid_turn',
        output='screen'
    )

    # 4. Task Orchestrator
    task_manager_node = Node(
        package='mypackage',
        executable='task_manager_grip',
        name='task_manager_grip',
        output='screen'
    )

    return LaunchDescription([
        params_arg,
        manual_composition,
      #  grip_node,
       # pid_move_node,
       # pid_turn_node,
        # task_manager_node
    ])
