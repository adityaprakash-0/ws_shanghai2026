"""Bench Align / Align+Move — hardware + IMU (+ front lidar), no Nav2/AMCL.

Two skill23_studio executables (both are real; rebuild if missing):

  studio_align       ← skill23_studio/nav/align_yaw_node.py
                       Does the yaw PID / Align+distance drive. Idle until a cmd.
  studio_align_test  ← skill23_studio/nav/align_test_trigger_node.py
                       One-shot: waits for IMU + align subscriber, then publishes
                       JSON on /studio/align/cmd (then exits after result).

Also starts: control_server (sudo inside), drive node, robot_state_publisher.
Align+Move also starts the front YDLidar → /scan/front.

Heading comes from /imu (NavX), not /amcl_pose. PID/duty come from
config/align/wp_align_<robot>.yaml (same file Deploy loads for that robot).

Do NOT wrap this launch in sudo — control_server is already sudo -E inside.
Stop localize / Nav2 first (duplicate Titan / lidar).

Usage (from workspace, after source install/setup.bash):

  # Align only — turn +90° relative to current IMU heading:
  ros2 launch skill23_studio test_wp_align.launch.py robot:=allterrain \\
    mode:=align yaw_deg:=90

  # Align + distance — yaw then front-lidar standoff (put a wall in front):
  ros2 launch skill23_studio test_wp_align.launch.py robot:=allterrain \\
    mode:=align_move yaw_deg:=0 distance_m:=0.10

  # Tune duty / PID without editing YAML:
  ros2 launch skill23_studio test_wp_align.launch.py robot:=allterrain \\
    mode:=align yaw_deg:=90 yaw_kp:=1.6 yaw_duty_min:=0.12 yaw_duty_max:=0.28

  # Start HW+align only; send the cmd yourself later:
  ros2 launch skill23_studio test_wp_align.launch.py robot:=allterrain \\
    mode:=align_move send:=false
  # then e.g.:
  #   ros2 run skill23_studio studio_align_test --ros-args \\
  #     -p mode:=align_move -p yaw_deg:=0.0 -p distance_m:=0.10 \\
  #     -p relative:=true -p auto_send:=true -p pose_source:=imu -p pose_topic:=/imu

If ros2 run cannot find studio_align / studio_align_test:
  cd ~/skill23_ws && colcon build --packages-select skill23_studio && source install/setup.bash
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
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from skill23_studio.align_paths import align_yaml_for_robot
from skill23_studio.ros_dds import studio_dds_launch_actions


def _node(package, executable, name, **kwargs):
    kwargs.setdefault('output', 'screen')
    kwargs.setdefault('emulate_tty', True)
    return Node(package=package, executable=executable, name=name, **kwargs)


def _control_server_cmd(exec_path, hw_params, cpus='2,3'):
    cmd = ['sudo', '-E']
    if cpus:
        cmd += ['taskset', '-c', cpus]
    return cmd + [
        exec_path,
        '--ros-args',
        '-r', '__node:=control_server',
        '--params-file', hw_params,
    ]


def _opt_float(context, name):
    raw = str(LaunchConfiguration(name).perform(context) or '').strip()
    if raw == '':
        return None
    return float(raw)


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    mode = str(LaunchConfiguration('mode').perform(context) or 'align').strip().lower()
    if mode in ('align+move', 'align_move', 'move'):
        mode = 'align_move'
    else:
        mode = 'align'
    if robot not in ('allterrain', 'mecanum', 'xdrive', 'grip'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive | grip")

    share = get_package_share_directory('skill23_studio')
    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')
    hw_params = os.path.join(share, 'config', 'hw_params', f'hw_params_{robot}.yaml')
    yaml_path = align_yaml_for_robot(robot, share)
    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')

    drive = {
        'allterrain': ('allterrain', 'allterrain_node', 'allterrain_node'),
        'mecanum': ('mecanum', 'mecanum_node', 'mecanum_node'),
        'xdrive': ('xdrive', 'xdrive_node', 'xdrive_node'),
        'grip': ('grip', 'grip_node', 'grip_node'),
    }[robot]
    urdf_name = {
        'mecanum': 'mecanum_robot.urdf',
        'xdrive': 'xdrive_robot.urdf',
        'allterrain': 'allterrain_robot.urdf',
        'grip': 'grip_robot.urdf',
    }[robot]
    urdf_file = os.path.join(get_package_share_directory(robot), 'urdf', urdf_name)
    robot_desc = ''
    if os.path.isfile(urdf_file):
        with open(urdf_file, 'r', encoding='utf-8') as handle:
            robot_desc = handle.read()

    imu_override = {
        'pose_source': 'imu',
        'pose_topic': '/imu',
        'log_hz': 4.0,
    }
    for key in (
        'yaw_kp', 'yaw_ki', 'yaw_kd', 'yaw_i_limit',
        'yaw_duty_min', 'yaw_duty_max', 'yaw_tolerance_deg',
        'angular_sign', 'timeout_sec', 'timeout_align_move_sec',
        'hold_after_done_sec', 'retries', 'retry_pause_sec',
    ):
        val = _opt_float(context, key)
        if val is not None:
            imu_override[key] = val
    log_hz = _opt_float(context, 'log_hz')
    if log_hz is not None:
        imu_override['log_hz'] = log_hz
    for key in (
        'lin_kp', 'lin_ki', 'lin_kd', 'lin_i_limit',
        'forward_duty_min', 'forward_duty_max',
        'backward_duty_min', 'backward_duty_max',
        'dist_tolerance_m', 'pause_after_align_sec', 'linear_sign',
    ):
        val = _opt_float(context, key)
        if val is not None:
            imu_override[key] = val

    align_params = [yaml_path, imu_override] if os.path.isfile(yaml_path) else [imu_override]
    send = str(LaunchConfiguration('send').perform(context)).lower() in (
        '1', 'true', 'yes', 'on')
    relative = str(LaunchConfiguration('relative').perform(context)).lower() in (
        '1', 'true', 'yes', 'on')
    delay = float(LaunchConfiguration('delay_sec').perform(context))
    if delay <= 0.0:
        delay = 7.5 if mode == 'align_move' else 4.5

    trigger = {
        'mode': mode,
        'yaw_deg': float(LaunchConfiguration('yaw_deg').perform(context)),
        'distance_m': float(LaunchConfiguration('distance_m').perform(context)),
        'delay_sec': 0.5,
        'auto_send': send,
        'relative': relative,
        'pose_source': 'imu',
        'pose_topic': '/imu',
    }

    actions = [
        LogInfo(msg=(
            f'[test_wp_align] robot={robot} mode={mode} yaw_deg='
            f'{trigger["yaw_deg"]:g} relative={relative}  yaml={yaml_path}  '
            'HW+IMU, no Nav2/AMCL. Do not sudo this launch. '
            'Nodes: studio_align (controller) + studio_align_test (cmd trigger).')),
        ExecuteProcess(cmd=_control_server_cmd(exec_path, hw_params), output='screen'),
        _node(drive[0], drive[1], drive[2]),
    ]
    if robot_desc:
        actions.append(_node(
            'robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ))

    if mode == 'align_move':
        y_front = os.path.join(studica_share, 'config', 'front_lidar_nav.yaml')
        if not os.path.isfile(y_front):
            y_front = os.path.join(studica_share, 'config', 'front_lidar.yaml')
        actions.append(TimerAction(period=2.0, actions=[
            LogInfo(msg='[test_wp_align] front lidar → /scan/front'),
            _node(
                'ydlidar_ros2_driver', 'ydlidar_ros2_driver_node', 'ydlidar_front',
                parameters=[y_front],
                remappings=[('/scan', '/scan/front')],
            ),
        ]))

    worker_delay = 3.0
    actions.append(TimerAction(period=worker_delay, actions=[
        LogInfo(msg='[test_wp_align] studio_align ← align_yaw_node.py'),
        _node('skill23_studio', 'studio_align', 'align', parameters=align_params),
    ]))

    if send:
        actions.append(TimerAction(
            period=delay,
            actions=[
                LogInfo(msg='[test_wp_align] studio_align_test ← align_test_trigger_node.py'),
                _node(
                    'skill23_studio', 'studio_align_test', 'align_test_trigger',
                    parameters=[trigger]),
            ],
        ))
    return actions


def generate_launch_description():
    def _arg(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    return LaunchDescription([
        *studio_dds_launch_actions(),
        _arg('robot', 'allterrain', 'allterrain | mecanum | xdrive | grip'),
        _arg('mode', 'align', 'align | align_move'),
        _arg('yaw_deg', '90.0', 'Turn this many degrees from current IMU (if relative)'),
        _arg('relative', 'true', 'true = yaw_deg is a turn from now; false = IMU heading'),
        _arg('distance_m', '0.10', 'Align+Move 0° front-lidar standoff (m)'),
        _arg('delay_sec', '0', 'When to send (0 = auto 4.5s align / 7.5s align_move)'),
        _arg('send', 'true', 'Auto-send the test command'),
        _arg('yaw_kp', '', 'Override yaml'),
        _arg('yaw_ki', '', ''),
        _arg('yaw_kd', '', ''),
        _arg('yaw_i_limit', '', ''),
        _arg('yaw_duty_min', '', 'Min Titan angular duty'),
        _arg('yaw_duty_max', '', 'Max Titan angular duty'),
        _arg('yaw_tolerance_deg', '', ''),
        _arg('angular_sign', '', '1 or -1 if turn direction is wrong'),
        _arg('timeout_sec', '', ''),
        _arg('timeout_align_move_sec', '', ''),
        _arg('hold_after_done_sec', '', 'Sit still after success before done'),
        _arg('retries', '', 'Extra tries after timeout (0-5)'),
        _arg('retry_pause_sec', '', ''),
        _arg('log_hz', '', ''),
        _arg('lin_kp', '', ''),
        _arg('lin_ki', '', ''),
        _arg('lin_kd', '', ''),
        _arg('lin_i_limit', '', ''),
        _arg('forward_duty_min', '', ''),
        _arg('forward_duty_max', '', ''),
        _arg('backward_duty_min', '', ''),
        _arg('backward_duty_max', '', ''),
        _arg('dist_tolerance_m', '', ''),
        _arg('pause_after_align_sec', '', ''),
        _arg('linear_sign', '', '1 or -1 if forward/back is swapped'),
        OpaqueFunction(function=_setup),
    ])
