import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    guard_share = get_package_share_directory('skill23_nav_guard')
    robot = LaunchConfiguration('robot').perform(context)

    nav_files = {
        'mecanum': 'navigation_guard_mecanum.py',
        'xdrive': 'navigation_guard_xdrive.py',
    }

    if robot not in nav_files:
        raise RuntimeError(
            f"nav2_guard mode supports: mecanum | xdrive (got '{robot}')")

    nav_launch_args = {}
    map_explicit = LaunchConfiguration('map_file').perform(context).strip()
    if map_explicit:
        nav_launch_args['map'] = map_explicit

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(guard_share, 'launch', nav_files[robot])),
        launch_arguments=nav_launch_args.items(),
    )

    bridge = Node(
        package='adev_dashboard',
        executable='adev_bridge',
        name='adev_bridge',
        output='screen',
        parameters=[{
            'robot': robot,
            'nav_mode': 'nav2_guard',
            'http_port': int(LaunchConfiguration('http_port').perform(context)),
            'maps_dir': os.path.expanduser('~/skill23_ws'),
        }],
    )

    simulator = Node(
        package='adev_dashboard',
        executable='adev_simulator',
        name='adev_simulator',
        output='screen',
    )

    return [nav_launch, bridge, simulator]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot', default_value='mecanum',
            description='Robot platform: mecanum | xdrive'),
        DeclareLaunchArgument(
            'map_file', default_value='',
            description='Optional map yaml override.'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        OpaqueFunction(function=_launch_setup),
    ])
