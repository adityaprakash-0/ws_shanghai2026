"""Teleop arena — HW (gamepad ON) + drive + simple pose + camera.

Easy usage (Pi):

  # Wi‑Fi DDS so laptop joy + camera share the same domain
  export ROS_LOCALHOST_ONLY=0
  export ROS_DOMAIN_ID=0

  ros2 launch skill23_studio teleop_arena.launch.py robot:=mecanum
  ros2 launch skill23_studio teleop_arena.launch.py robot:=xdrive
  ros2 launch skill23_studio teleop_arena.launch.py robot:=grip
  ros2 launch skill23_studio teleop_arena.launch.py robot:=allterrain

Or the thin wrappers:
  ros2 launch skill23_studio teleop_mecanum.launch.py
  ...

Laptop (separate terminals, same ROS_DOMAIN_ID):
  ros2 run joy joy_node
  # then skill23_vision teleop_capture (YOLO / AprilTag buttons)

Topics:
  /robot_pose         geometry_msgs/Pose2D   — x, y, yaw (starts 0,0,0)
  /robot_pose_xyyaw   Float64MultiArray      — [x, y, yaw]
  /camera/color/...   Orbbec color stream
  /joy                from laptop joy_node → Pi gamepad → /cmd_vel
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from skill23_studio.orbbec_dabai import (
    clear_orbbec_device_lock,
    dabai_cli_args,
    prefer_workspace_orbbec,
    wake_orbbec_usb,
)


# robot → (drive package, drive executable, drive_type for pose, strafe?)
ROBOTS = {
    'mecanum': ('mecanum', 'mecanum_node', 'mecanum', True),
    'xdrive': ('xdrive', 'xdrive_node', 'xdrive', True),
    'grip': ('grip', 'grip_node', 'diff', False),
    'allterrain': ('allterrain', 'allterrain_node', 'diff', False),
}


def _teleop_params(robot: str) -> str:
    share = get_package_share_directory('skill23_studio')
    path = os.path.join(share, 'config', 'hw_params', f'teleop_{robot}.yaml')
    if os.path.isfile(path):
        return path
    # source-tree fallback
    path = os.path.expanduser(
        f'~/skill23_ws/src/skill23_studio/config/hw_params/teleop_{robot}.yaml')
    if os.path.isfile(path):
        return path
    raise RuntimeError(f'Missing teleop_{robot}.yaml')


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context).strip().lower()
    if robot not in ROBOTS:
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: " + ' | '.join(ROBOTS))

    pkg, exe, drive_type, strafe = ROBOTS[robot]
    hw_params = _teleop_params(robot)
    studica_prefix = get_package_prefix('studica_control')
    exec_path = os.path.join(
        studica_prefix, 'lib', 'studica_control', 'manual_composition')
    if not os.path.isfile(exec_path):
        raise RuntimeError(f'manual_composition not found: {exec_path}')

    # Camera: color ON at start so laptop can grab frames on button press
    prefer_workspace_orbbec()
    clear_orbbec_device_lock()
    wake_orbbec_usb()
    cam_args = dabai_cli_args()
    # Override idle defaults for continuous teleop capture
    cam_args = [
        a for a in cam_args
        if not a.startswith('start_streams_on_init:=')
        and not a.startswith('enable_depth:=')
        and not a.startswith('enable_color:=')
    ] + [
        'enable_color:=true',
        'enable_depth:=false',
        'start_streams_on_init:=true',
    ]

    control = ExecuteProcess(
        cmd=[
            'sudo', '-E', 'env',
            f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}',
            f'RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")}',
            f'ROS_LOCALHOST_ONLY={os.environ.get("ROS_LOCALHOST_ONLY", "0")}',
            f'AMENT_PREFIX_PATH={os.environ.get("AMENT_PREFIX_PATH", "")}',
            f'LD_LIBRARY_PATH={os.environ.get("LD_LIBRARY_PATH", "")}',
            exec_path,
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', hw_params,
        ],
        output='screen',
    )

    drive = Node(
        package=pkg,
        executable=exe,
        name=exe,
        output='screen',
    )

    pose = Node(
        package='skill23_studio',
        executable='simple_pose',
        name='simple_pose',
        output='screen',
        parameters=[{
            'drive_type': drive_type,
            'odom_topic': '/wheel/odom',
            'pose_topic': '/robot_pose',
            'xyyaw_topic': '/robot_pose_xyyaw',
            'publish_hz': 20.0,
        }],
    )

    # Same pattern Studio uses — easy to read, reliable over Wi‑Fi DDS
    camera = ExecuteProcess(
        cmd=['ros2', 'launch', 'orbbec_camera', 'gemini_e.launch.py'] + cam_args,
        output='screen',
    )

    return [
        LogInfo(msg=(
            f'[teleop] robot={robot} drive={pkg}/{exe} '
            f'strafe={"yes" if strafe else "no"} '
            f'params={hw_params}'
        )),
        control,
        TimerAction(period=2.0, actions=[drive]),
        TimerAction(period=3.0, actions=[pose]),
        TimerAction(period=4.0, actions=[camera]),
    ]


def generate_launch_description():
    # Force Wi‑Fi DDS so laptop joy_node + vision see Pi topics
    return LaunchDescription([
        SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable(name='FASTDDS_BUILTIN_TRANSPORTS', value='UDPv4'),
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='0'),
        DeclareLaunchArgument(
            'robot', default_value='mecanum',
            description='mecanum | xdrive | grip | allterrain'),
        OpaqueFunction(function=_setup),
    ])
