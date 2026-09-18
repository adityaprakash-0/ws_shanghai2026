"""Launch Skill23 lookup OCR/ArUco node (idle until /studio/lookup/cmd)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('confirm_frames', default_value='3'),
        DeclareLaunchArgument('retries', default_value='3'),
        DeclareLaunchArgument('timeout_sec', default_value='25.0'),
        DeclareLaunchArgument('warmup_ocr', default_value='true'),
        LogInfo(msg='[skill23_lookup] OCR/ArUco lookup node (idle until WP cmd)'),
        Node(
            package='skill23_lookup',
            executable='lookup_node',
            name='studio_lookup',
            output='screen',
            parameters=[{
                'color_topic': LaunchConfiguration('color_topic'),
                'confirm_frames': LaunchConfiguration('confirm_frames'),
                'retries': LaunchConfiguration('retries'),
                'timeout_sec': LaunchConfiguration('timeout_sec'),
                'warmup_ocr': LaunchConfiguration('warmup_ocr'),
            }],
        ),
    ])
