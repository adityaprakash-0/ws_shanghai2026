"""Start WorldSkills Studio bridge only (HTTP+SSE on :8083).

  ros2 launch skill23_studio studio.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    http_port = LaunchConfiguration('http_port')
    share = get_package_share_directory('skill23_studio')
    stop_sh = os.path.join(share, 'scripts', 'stop_studio_stack.sh')
    if not os.path.isfile(stop_sh):
        stop_sh = os.path.expanduser(
            '~/skill23_ws/src/skill23_studio/scripts/stop_studio_stack.sh')
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument('http_port', default_value='8083'),
        Node(
            package='skill23_studio',
            executable='studio_bridge',
            name='studio_bridge',
            output='screen',
            parameters=[{
                'http_port': http_port,
            }],
        ),
        # Waypoint wait tasks run inside cn_mission; no action helper is needed.
        # There is no camera relay node: Orbbec publishes on /camera/* and
        # consumers (laptop vision) subscribe to those topics directly.
        RegisterEventHandler(OnShutdown(on_shutdown=[
            LogInfo(msg='Studio shutdown — reaping nav/camera children'),
            ExecuteProcess(
                cmd=['bash', stop_sh],
                output='screen',
            ),
        ])),
    ])
