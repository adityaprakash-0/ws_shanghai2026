"""Launch skill23_amcl (vendored Nav2 Humble AMCL) with lifecycle autostart."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            description='YAML with amcl: ros__parameters (e.g. custom_nav2_param.yaml)'),
        Node(
            package='skill23_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['amcl'],
                'bond_timeout': 4.0,
            }],
        ),
    ])
