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

    nav_files = {
        'allterrain': 'navigation_allterrain.py',
        'mecanum': 'navigation_mecanum.py',
        'xdrive': 'navigation_xdrive.py',
    }

    if robot not in nav_files:
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive")

    nav_launch_args = {}
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
            'nav_mode': 'nav2',
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

    actions = [nav_launch, bridge, simulator]

    # Optional Orbbec + vision gate (default OFF — does not affect nav CPU)
    try:
        vision_share = get_package_share_directory('skill23_vision')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vision_share, 'launch', 'vision.launch.py')),
            launch_arguments={
                'enable_vision': LaunchConfiguration('enable_vision'),
                'launch_camera': LaunchConfiguration('launch_camera'),
                'color_topic': LaunchConfiguration('color_topic'),
            }.items(),
        ))
    except Exception:
        pass

    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot', default_value='allterrain',
            description='Robot platform: allterrain | mecanum | xdrive'),
        DeclareLaunchArgument(
            'map_file', default_value='',
            description='Optional map yaml override.'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument(
            'enable_vision', default_value='false',
            description='Optional Orbbec color vision (Task Builder camera blocks).'),
        DeclareLaunchArgument(
            'launch_camera', default_value='true',
            description='When enable_vision, also start Orbbec Gemini E.'),
        DeclareLaunchArgument(
            'color_topic', default_value='/camera/color/image_raw'),
        OpaqueFunction(function=_launch_setup),
    ])
