"""Teleop arena — HW (gamepad ON) + drive + simple_pose + Orbbec camera.

  export ROS_DOMAIN_ID=0
  export ROS_LOCALHOST_ONLY=0
  ros2 launch skill23_studio teleop_mecanum.launch.py
  # or: teleop_xdrive / teleop_grip / teleop_allterrain
  # or: teleop_arena.launch.py robot:=mecanum

Starts (in order):
  1. control_server  — Titans + IMU + gamepad (/joy → /cmd_vel)
  2. <robot>_node    — motors + /wheel/odom (encoders + IMU yaw)
  3. simple_pose     — /robot_pose from odom, origin 0,0,0
  4. Orbbec Gemini E — color stream ON (/camera/color/...)
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
from skill23_studio.ros_dds import studio_dds_launch_actions


# robot → (package, executable, pose drive_type, has_strafe)
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
    path = os.path.expanduser(
        f'~/skill23_ws/src/skill23_studio/config/hw_params/teleop_{robot}.yaml')
    if os.path.isfile(path):
        return path
    raise RuntimeError(f'Missing teleop_{robot}.yaml')


def _sudo_control_cmd(exec_path: str, hw_params: str) -> list:
    """Root control_server with the SAME DDS env as non-root nodes.

    Shared-memory DDS breaks between sudo and vmx; force UDPv4 so /joy from
    the laptop reaches gamepad and /imu reaches the drive node.
    """
    return [
        'sudo', '-E', 'env',
        f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}',
        f'RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")}',
        f'FASTDDS_BUILTIN_TRANSPORTS={os.environ.get("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")}',
        # Always Wi‑Fi DDS for teleop (laptop joy_node + vision)
        'ROS_LOCALHOST_ONLY=0',
        f'PATH={os.environ.get("PATH", "")}',
        f'AMENT_PREFIX_PATH={os.environ.get("AMENT_PREFIX_PATH", "")}',
        f'LD_LIBRARY_PATH={os.environ.get("LD_LIBRARY_PATH", "")}',
        exec_path,
        '--ros-args',
        '-r', '__node:=control_server',
        '--params-file', hw_params,
    ]


def _camera_cli_args() -> list:
    """Studio Orbbec args, but streams start immediately (color for capture).

    Keep enable_color/depth true (driver creates topics/services the same way
    Studio does). start_streams_on_init true so laptop sees frames without
    a toggle call.
    """
    prefer_workspace_orbbec()
    clear_orbbec_device_lock()
    wake_orbbec_usb()
    args = [
        a for a in dabai_cli_args()
        if not a.startswith('start_streams_on_init:=')
        and not a.startswith('enable_color:=')
        and not a.startswith('enable_depth:=')
    ]
    args += [
        'enable_color:=true',
        'enable_depth:=true',
        'start_streams_on_init:=true',
    ]
    return args


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context).strip().lower()
    if robot not in ROBOTS:
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: " + ' | '.join(sorted(ROBOTS)))

    pkg, exe, drive_type, strafe = ROBOTS[robot]
    hw_params = _teleop_params(robot)

    studica_prefix = get_package_prefix('studica_control')
    exec_path = os.path.join(
        studica_prefix, 'lib', 'studica_control', 'manual_composition')
    if not os.path.isfile(exec_path):
        raise RuntimeError(f'manual_composition not found: {exec_path}')

    # Force Wi‑Fi discovery for this process tree (laptop joy + camera)
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

    cam_args = _camera_cli_args()

    control = ExecuteProcess(
        cmd=_sudo_control_cmd(exec_path, hw_params),
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

    # Same invocation Studio process_manager uses
    camera = ExecuteProcess(
        cmd=['ros2', 'launch', 'orbbec_camera', 'gemini_e.launch.py'] + cam_args,
        output='screen',
    )

    return [
        LogInfo(msg=(
            f'[teleop] robot={robot}  hw={hw_params}  '
            f'drive={pkg}/{exe}  strafe={"yes" if strafe else "no"}  '
            f'camera=gemini_e color+depth streams ON'
        )),
        control,
        # Wait for Titans/IMU before drive + pose
        TimerAction(period=2.0, actions=[drive]),
        TimerAction(period=3.0, actions=[pose]),
        TimerAction(period=4.0, actions=[camera]),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Clear stale DDS shm + UDPv4 (root ↔ user ↔ laptop)
        *studio_dds_launch_actions(),
        # Override Studio's default localhost-only so laptop sees Pi
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='0'),
        DeclareLaunchArgument(
            'robot', default_value='mecanum',
            description='mecanum | xdrive | grip | allterrain'),
        OpaqueFunction(function=_setup),
    ])
