"""Custom navigation stack launch (no Nav2).

Startup order (dependencies):

  1. Hardware — control_server, robot_state_publisher, drive
  2. World model — loads map → /map + /custom_nav/static_edt + /custom_nav/world/ready
  3. EKF — odom TF (needed by AMCL)
  4. Lidars → merger → /scan
  5. AMCL — waits for world/ready (param), then localizes with EKF TF + /scan
  6. Perception — obstacle detector (/scan → local_grid), collision (local_grid)
  7. Nav stack — planner → path_smoother → mission/controller/recovery/bridge

  ros2 launch skill23_studio studio_nav.launch.py robot:=grip \\
    map:=/home/vmx/skill23_ws/test_map11.yaml
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from skill23_studio.detect_align_paths import detect_align_yaml_for_robot
from skill23_studio.ekf_paths import ekf_yaml_for_robot


def _node(package, executable, name, **kwargs):
    kwargs.setdefault('output', 'screen')
    return Node(package=package, executable=executable, name=name, **kwargs)


def _control_server_cmd(exec_path, hw_params, cpus='2,3'):
    """Same as slam_tele / custom_nav: sudo -E <binary> --ros-args …

    Do **not** wrap manual_composition in ``env PATH=… PYTHONPATH=…``.
    That drops ROS param overrides → InvalidParameterValueException and
    Titan never starts (cmd_vel published, motors dead).

    The closed-source VMX HAL busy-polls SPI from a SCHED_FIFO thread and
    burns a full core. Unpinned it preempts AMCL and the lidar merger on any
    core, which is what makes localization drift once the robot moves, so it
    is confined to the last two cores.
    """
    cmd = ['sudo', '-E']
    if cpus:
        cmd += ['taskset', '-c', cpus]
    return cmd + [
        exec_path,
        '--ros-args',
        '-r', '__node:=control_server',
        '--params-file', hw_params,
    ]


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    map_yaml = os.path.expanduser(LaunchConfiguration('map').perform(context))
    http_port = int(LaunchConfiguration('http_port').perform(context))
    enable_lidar = LaunchConfiguration('enable_lidar').perform(context).lower() in (
        '1', 'true', 'yes', 'on')
    enable_bridge = LaunchConfiguration('enable_bridge').perform(context).lower() in (
        '1', 'true', 'yes', 'on')

    if robot not in ('allterrain', 'mecanum', 'xdrive', 'grip'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive | grip")

    if map_yaml and not os.path.isfile(map_yaml):
        raise RuntimeError(f"Map file not found: {map_yaml}")

    share = get_package_share_directory('skill23_studio')
    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')
    hw_params = os.path.join(share, 'config', 'hw_params', f'hw_params_{robot}.yaml')
    amcl_params = os.path.join(share, 'config', 'amcl.yaml')
    detect_yaml = detect_align_yaml_for_robot(robot, share)
    if not os.path.isfile(amcl_params):
        raise RuntimeError(f'Missing {amcl_params}')

    robot_share = get_package_share_directory(robot)
    sw_params = os.path.join(robot_share, 'config', 'custom_nav2_param.yaml')
    if not os.path.isfile(sw_params):
        raise RuntimeError(
            f"Missing {robot}/config/custom_nav2_param.yaml "
            f"(expected at {sw_params})")


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
        with open(urdf_file, 'r') as f:
            robot_desc = f.read()

    common = {'parameters': [sw_params], 'output': 'screen'}
    map_params = [sw_params, {'map': map_yaml}] if map_yaml else [sw_params]
    ekf_yaml = ekf_yaml_for_robot(robot, navigation=True)

    actions = []

    # ── 1. Hardware (t≈0) ───────────────────────────────────────────────
    actions.append(ExecuteProcess(
        cmd=_control_server_cmd(exec_path, hw_params), output='screen'))

    if robot_desc:
        actions.append(_node(
            'robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ))

    actions.append(_node(drive[0], drive[1], drive[2]))

    # ── 2. World model first — owns /map, static EDT, world/ready ────────
    # Must start before AMCL. AMCL only activates after /custom_nav/world/ready.
    actions.append(_node(
        'skill23_studio', 'cn_world_model', 'cn_world_model',
        parameters=map_params,
    ))

    # ── 3. EKF (odom TF) — before AMCL tries to use map→odom ─────────────
    actions.append(TimerAction(period=1.5, actions=[
        _node(
            'robot_localization', 'ekf_node', 'ekf_filter_node',
            parameters=[ekf_yaml],
        ),
    ]))

    # ── 4. Lidars → merged /scan (AMCL + obstacle detector need this) ───
    if enable_lidar:
        y_front = os.path.join(studica_share, 'config', 'front_lidar_nav.yaml')
        y_back = os.path.join(studica_share, 'config', 'back_lidar_nav.yaml')
        actions.append(TimerAction(period=2.0, actions=[
            _node(
                'ydlidar_ros2_driver', 'ydlidar_ros2_driver_node', 'ydlidar_front',
                parameters=[y_front],
                remappings=[('/scan', '/scan/front')],
            ),
        ]))
        actions.append(TimerAction(period=4.5, actions=[
            _node(
                'ydlidar_ros2_driver', 'ydlidar_ros2_driver_node', 'ydlidar_back',
                parameters=[y_back],
                remappings=[('/scan', '/scan/back')],
            ),
        ]))
        actions.append(TimerAction(period=5.5, actions=[
            _node(
                'allterrain', 'lidar_merger_node', 'lidar_merger_node',
            ),
        ]))

    # ── 5. AMCL — simple timed startup, as in skill23_custom_nav ─────────
    # AMCL itself waits for /custom_nav/world/ready before localizing.
    actions.append(TimerAction(period=2.5, actions=[
        _node(
            'skill23_amcl', 'amcl', 'amcl',
            parameters=[amcl_params],
        ),
    ]))

    # ── 6. Perception — after the merged scan has started ────────────────
    perception_delay = 6.0 if enable_lidar else 3.0
    actions.append(TimerAction(period=perception_delay, actions=[
        GroupAction(actions=[
            _node(
                'skill23_studio', 'cn_obstacle_detector',
                'cn_obstacle_detector', **common),
            _node(
                'skill23_studio', 'cn_collision_checker',
                'cn_collision_checker', **common),
        ]),
    ]))

    # ── 7. Basic planning / mission / controller stack ──────────────────
    # No startup gate and no behaviour server. Native ROS topics connect
    # these nodes directly, matching skill23_custom_nav.
    nav_nodes = [
        _node(
            'skill23_studio', 'cn_planner', 'cn_planner',
            remappings=[('/custom_nav/path', '/custom_nav/planner_path')],
            **common),
        _node('skill23_studio', 'cn_path_smoother', 'cn_path_smoother', **common),
        _node('skill23_studio', 'cn_goal_checker', 'cn_goal_checker', **common),
        _node('skill23_studio', 'cn_recovery', 'cn_recovery', **common),
        _node('skill23_studio', 'cn_mission', 'cn_mission', **common),
        _node('skill23_studio', 'cn_controller', 'cn_controller', **common),
        _node(
            'skill23_studio', 'studio_detect_align', 'detect_align',
            parameters=[detect_yaml],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('oms'),
                'launch', 'oms.launch.py')),
        ),
    ]
    if enable_bridge:
        nav_nodes.append(
            _node(
                'skill23_studio', 'cn_bridge', 'cn_bridge',
                parameters=[sw_params, {
                    'robot': robot,
                    'http_port': http_port,
                    'maps_dir': os.path.expanduser('~/skill23_ws/studio_maps'),
                }],
            ))

    nav_delay = 6.5 if enable_lidar else 3.5
    actions.append(TimerAction(
        period=nav_delay,
        actions=[GroupAction(actions=nav_nodes)],
    ))

    return actions


def generate_launch_description():
    default_map = os.path.expanduser('~/skill23_ws/test_map11.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot', default_value='grip',
            description='Robot platform: allterrain | mecanum | xdrive | grip'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Absolute path to map.yaml (required by cn_world_model)'),
        DeclareLaunchArgument('http_port', default_value='8084'),
        DeclareLaunchArgument(
            'enable_lidar', default_value='true',
            description='Start dual YDLidar + merger publishing /scan'),
        DeclareLaunchArgument(
            'enable_bridge', default_value='false',
            description='Start vendored cn_bridge (studio uses studio_bridge on 8083)'),
        OpaqueFunction(function=_setup),
    ])
