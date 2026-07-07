import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    studica_share = get_package_share_directory('studica_control')
    robot = LaunchConfiguration('robot').perform(context)
    nav2_on_laptop = LaunchConfiguration('nav2_on_laptop').perform(context).lower() in (
        'true', '1', 'yes',
    )

    nav_files_on_pi = {
        'allterrain': 'navigation_allterrain.py',
        'mecanum': 'navigation_mecanum.py',
        'xdrive': 'navigation_xdrive.py',
    }
    nav_files_pi_only = {
        'allterrain': 'navigation_allterrain_pi.py',
        'mecanum': 'navigation_mecanum_pi.py',
        'xdrive': 'navigation_xdrive_pi.py',
    }

    nav_files = nav_files_pi_only if nav2_on_laptop else nav_files_on_pi

    if robot not in nav_files:
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive")

    nav_launch_args = {}
    if not nav2_on_laptop:
        # Use map_file (not map) so we do not shadow navigation_<robot>.py's map default.
        map_explicit = LaunchConfiguration('map_file').perform(context).strip()
        if map_explicit:
            nav_launch_args['map'] = map_explicit

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(studica_share, 'launch', nav_files[robot])),
        launch_arguments=nav_launch_args.items(),
    )

    bridge = Node(
        package='adev_dashboard',
        executable='adev_bridge',
        name='adev_bridge',
        output='screen',
        parameters=[{
            'robot': robot,
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
        # Root (sudo) manual_composition and user adev_bridge must share DDS discovery.
        # Without this, /battery_state and other VMX topics are invisible to the bridge.
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot', default_value='allterrain',
            description='Robot platform: allterrain | mecanum | xdrive'),
        DeclareLaunchArgument(
            'map_file', default_value='',
            description='Optional map yaml. If empty, navigation_<robot>.py default is used '
                        '(mecanum → test_map11.yaml).'),
        DeclareLaunchArgument(
            'nav2_on_laptop', default_value='false',
            description='If true, Pi runs hardware/sensors only; start Nav2 on laptop '
                        '(laptop_nav2_ws) over Pi hotspot with same ROS_DOMAIN_ID.'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        OpaqueFunction(function=_launch_setup),
    ])
