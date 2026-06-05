"""Optional vision + Orbbec for ADEV dashboard missions."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    enable = LaunchConfiguration('enable_vision').perform(context).lower() in (
        '1', 'true', 'yes', 'on')
    launch_camera = LaunchConfiguration('launch_camera').perform(context).lower() in (
        '1', 'true', 'yes', 'on')

    if not enable:
        return []

    actions = []
    ws = os.path.expanduser(os.environ.get('SKILL23_WS', '~/skill23_ws'))
    gemini = os.path.join(ws, 'scripts', 'launch_gemini_e.sh')

    if launch_camera and os.path.isfile(gemini):
        # Lightweight Orbbec profile for Pi — no IR / point cloud.
        actions.append(ExecuteProcess(
            cmd=[
                gemini,
                # Keep color configured so /camera/toggle_color exists.
                # vision_gate immediately stops the stream until it is needed.
                'enable_color:=true',
                'enable_ir:=false',
                'enable_point_cloud:=false',
                'enable_colored_point_cloud:=false',
                'color_fps:=15',
                'depth_fps:=15',
                'enable_depth:=false',
                'enable_ldp:=false',
                'publish_tf:=false',
            ],
            output='screen',
            name='orbbec_gemini_e',
        ))

    actions.append(Node(
        package='skill23_vision',
        executable='vision_gate',
        name='adev_vision_gate',
        output='screen',
        parameters=[{
            'color_topic': LaunchConfiguration('color_topic').perform(context),
            'min_pixel_ratio': 0.008,
            'publish_preview': True,
        }],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_vision', default_value='false',
            description='Start vision gate (and optionally Orbbec). Default OFF.'),
        DeclareLaunchArgument(
            'launch_camera', default_value='true',
            description='If enable_vision, also start Orbbec Gemini E (user, not sudo).'),
        DeclareLaunchArgument(
            'color_topic', default_value='/camera/color/image_raw'),
        OpaqueFunction(function=_setup),
    ])
