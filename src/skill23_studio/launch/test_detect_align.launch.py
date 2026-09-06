"""Bench Detect+Align vision — camera + optional detect_align (OMS off).

  ros2 launch skill23_studio test_detect_align.launch.py robot:=allterrain
  ros2 run skill23_studio studio_detect_align_test
  ros2 topic pub --once /studio/detect_align_test/start std_msgs/msg/Empty {}
  rqt_image_view /studio/detect_align/debug_image
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from skill23_studio.detect_align_paths import detect_align_yaml_for_robot
from skill23_studio.ros_dds import studio_dds_launch_actions
from skill23_studio.orbbec_dabai import (
    clear_orbbec_device_lock,
    dabai_launch_args,
    orbbec_launch_path,
    prefer_workspace_orbbec,
    wake_orbbec_usb,
)


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
        '-p', 'servo.enabled:=true',
    ]


def _opt_float(context, name):
    raw = str(LaunchConfiguration(name).perform(context) or '').strip()
    if raw == '':
        return None
    return float(raw)


def _opt_bool(context, name):
    raw = str(LaunchConfiguration(name).perform(context) or '').strip().lower()
    if raw == '':
        return None
    if raw in ('1', 'true', 'yes', 'on'):
        return True
    if raw in ('0', 'false', 'no', 'off'):
        return False
    raise RuntimeError(f"Launch arg '{name}' must be true/false, got {raw!r}")


def _opt_str(context, name):
    raw = str(LaunchConfiguration(name).perform(context) or '').strip()
    return raw if raw else None


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('allterrain', 'mecanum', 'xdrive', 'grip'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive | grip")

    share = get_package_share_directory('skill23_studio')
    studica_prefix = get_package_prefix('studica_control')
    hw_params = os.path.join(share, 'config', 'hw_params', f'hw_params_{robot}.yaml')
    yaml_path = detect_align_yaml_for_robot(robot, share)
    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')
    overlay = prefer_workspace_orbbec()
    dabai_launch = orbbec_launch_path()

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

    override = {'log_hz': 4.0}
    for key in (
        'angular_sign', 'center_tol_norm', 'center_u_offset', 'confirm_frames',
        'detect_wait_sec', 'align_timeout_sec', 'depth_after_align_sec',
        'depth_samples', 'depth_sample_timeout_sec', 'depth_roi_frac',
        'camera_tilt_deg', 'hfov_deg', 'hold_after_done_sec', 'retries',
        'retry_pause_sec', 'conf_threshold', 'num_threads', 'min_box_area_frac',
        'target_class', 'skip_frames', 'verify_skip_frames', 'max_oms_moves',
        'stream_verify_frames',
    ):
        val = _opt_float(context, key)
        if val is not None:
            if key in ('confirm_frames', 'depth_samples', 'retries',
                       'num_threads', 'target_class', 'skip_frames',
                       'verify_skip_frames', 'max_oms_moves',
                       'stream_verify_frames'):
                override[key] = int(val)
            else:
                override[key] = val
    pub_dbg = _opt_bool(context, 'publish_debug_image')
    if pub_dbg is not None:
        override['publish_debug_image'] = pub_dbg
    log_hz = _opt_float(context, 'log_hz')
    if log_hz is not None:
        override['log_hz'] = log_hz
    target_name = _opt_str(context, 'target_name')
    if target_name is not None:
        override['target_name'] = target_name
    oms_rot = _opt_bool(context, 'oms_rotation_enabled')
    if oms_rot is not None:
        override['oms_rotation_enabled'] = oms_rot

    color_width = str(LaunchConfiguration('color_width').perform(context) or '640')
    color_height = str(LaunchConfiguration('color_height').perform(context) or '360')
    color_fps = str(LaunchConfiguration('color_fps').perform(context) or '5')
    cam_args = dabai_launch_args(
        color_width=color_width,
        color_height=color_height,
        color_fps=color_fps,
    )

    detect_params = [yaml_path, override] if os.path.isfile(yaml_path) else [override]
    shm_note = clear_orbbec_device_lock()
    usb_note = wake_orbbec_usb()

    actions = [
        LogInfo(msg=(
            f'[test_detect_align] robot={robot}  yaml={yaml_path}  '
            f'color={color_width}x{color_height}@{color_fps}fps '
            f'orbbec={overlay}  {dabai_launch}  '
            '(Gemini E is USB2-only; OpenNI enum often 30-50s). '
            'Terminal 2: ros2 run skill23_studio studio_detect_align_test  '
            'START: ros2 topic pub --once /studio/detect_align_test/start std_msgs/msg/Empty {}  '
            'debug: rqt_image_view /studio/detect_align/debug_image')),
        LogInfo(msg=(
            f'[test_detect_align] orbbec shm lock: {shm_note or "clean"}  usb: {usb_note}')),
        ExecuteProcess(cmd=_control_server_cmd(exec_path, hw_params), output='screen'),
        _node(drive[0], drive[1], drive[2]),
        LogInfo(msg=(
            f'[test_detect_align] Gemini E {color_width}x{color_height} @ {color_fps}fps '
            '(enable_color/depth=true, ir=false)')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dabai_launch),
            launch_arguments=cam_args.items(),
        ),
    ]
    if robot_desc:
        actions.append(_node(
            'robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ))

    actions.append(TimerAction(period=2.0, actions=[
        LogInfo(msg=(
            f'[test_detect_align] detect_align optional (oms off)  yaml={yaml_path}')),
        # Vision bench uses studio_detect_align_test only — detect_align not required.
        # _node(
        #     'skill23_studio', 'studio_detect_align', 'detect_align',
        #     parameters=detect_params),
    ]))
    # OMS rotation disabled for vision bench — uncomment to test full align.
    # actions.append(TimerAction(period=7.0, actions=[
    #     LogInfo(msg='[test_detect_align] OMS nodes (after Titan ready)'),
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(
    #             get_package_share_directory('oms'),
    #             'launch', 'oms.launch.py')),
    #     ),
    # ]))
    return actions


def generate_launch_description():
    def _arg(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    return LaunchDescription([
        *studio_dds_launch_actions(),
        _arg('robot', 'allterrain', 'allterrain | mecanum | xdrive | grip'),
        _arg('color_width', '640', 'Gemini E color width'),
        _arg('color_height', '360', 'Gemini E color height (native 360, not 480)'),
        _arg('color_fps', '5', 'Color + depth fps. Gemini E 640x360 has no 3fps — min is 5'),
        _arg('target_name', '', 'Class name: Cylinder | Rectangle | Semisphere (empty=any)'),
        _arg('target_class', '', 'Class id from metadata.yaml, -1 = any'),
        _arg('oms_rotation_enabled', 'false', 'Enable OMS in detect_align (bench: false)'),
        _arg('angular_sign', '', '1 or -1 if OMS turn direction is wrong'),
        _arg('center_tol_norm', '', 'Half-width fraction that counts as centered'),
        _arg('center_u_offset', '', ''),
        _arg('confirm_frames', '', 'Centered verify snapshots before aligned'),
        _arg('skip_frames', '', 'Frames to skip before first detect snapshot'),
        _arg('verify_skip_frames', '', 'Frames to skip before verify snapshots'),
        _arg('detect_wait_sec', '', 'How long to wait for the first box'),
        _arg('align_timeout_sec', '', 'Max time to center after first box'),
        _arg('depth_after_align_sec', '', 'Sit still after aligned, then depth'),
        _arg('depth_samples', '', ''),
        _arg('depth_sample_timeout_sec', '', ''),
        _arg('depth_roi_frac', '', ''),
        _arg('camera_tilt_deg', '', 'Downward pitch of the camera'),
        _arg('hfov_deg', '', 'Camera horizontal FOV for OMS angle calc'),
        _arg('max_oms_moves', '', 'Max OMS goto attempts per run'),
        _arg('stream_verify_frames', '', 'Color frames required before snapshot'),
        _arg('publish_debug_image', '', 'Publish /studio/detect_align/debug_image'),
        _arg('hold_after_done_sec', '', ''),
        _arg('retries', '', ''),
        _arg('retry_pause_sec', '', ''),
        _arg('conf_threshold', '', ''),
        _arg('num_threads', '', ''),
        _arg('min_box_area_frac', '', ''),
        _arg('log_hz', '', ''),
        OpaqueFunction(function=_setup),
    ])
