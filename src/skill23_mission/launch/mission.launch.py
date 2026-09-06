"""Mission runner — pick a sequence and send Nav2 goals.

  ros2 launch skill23_mission mission.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# Edit these
# ---------------------------------------------------------------------------
MAP_NAME = 'default'          # maps/<name> or studio_maps/<name>
DEFAULT_WAIT_SEC = 3.0
AUTO_SEQUENCE = ''            # e.g. '1' or 'short_loop' to skip terminal prompt
# ---------------------------------------------------------------------------


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='skill23_mission',
            executable='mission_runner',
            name='mission_runner',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'map': MAP_NAME},
                {'default_wait_sec': DEFAULT_WAIT_SEC},
                {'auto_sequence': AUTO_SEQUENCE},
            ],
        ),
    ])
