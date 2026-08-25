"""Studio Nav2 stack — hardware + Nav2 (RPP + Theta*).

controller_server publishes /cmd_vel directly (no velocity_smoother).
Same map path as custom-nav deploy. Does not start cn_* nodes.

  ros2 launch skill23_studio studio_nav2.launch.py robot:=grip \\
    map:=/home/vmx/skill23_ws/studio_maps/grip1/map.yaml
"""

from __future__ import annotations

import os
import tempfile

import yaml
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

from skill23_studio.ekf_paths import ekf_yaml_for_robot


def _node(package, executable, name, **kwargs):
    kwargs.setdefault('output', 'screen')
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


def _prepare_nav2_params(robot: str, share: str) -> str:
    """Load robot YAML and resolve Studio BT XML paths under share/config/."""
    src = os.path.join(share, 'config', 'nav2_params', f'nav2_params_{robot}.yaml')
    with open(src, 'r', encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle)
    bt = cfg['bt_navigator']['ros__parameters']

    def _resolve_bt(key: str, default_name: str) -> str:
        raw = str(bt.get(key) or '').strip()
        name = raw if raw.endswith('.xml') else default_name
        if os.path.isabs(name) and os.path.isfile(name):
            return name
        # Bare filename or empty → skill23_studio/share/.../config/<file>
        base = os.path.basename(name) if name else default_name
        path = os.path.join(share, 'config', 'nav2_params', base)
        if not os.path.isfile(path):
            path = os.path.join(share, 'config', 'nav2_params', default_name)
        return path

    bt_xml = _resolve_bt('default_nav_to_pose_bt_xml', 'navigate_nav2.xml')
    through_xml = _resolve_bt(
        'default_nav_through_poses_bt_xml', 'navigate_through_poses_nav2.xml')
    bt['default_bt_xml_filename'] = bt_xml
    bt['default_nav_to_pose_bt_xml'] = bt_xml
    bt['default_nav_through_poses_bt_xml'] = through_xml

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix=f'_studio_nav2_{robot}.yaml', delete=False)
    yaml.dump(cfg, tmp, default_flow_style=False, sort_keys=False)
    tmp.close()
    return tmp.name


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    map_yaml = os.path.expanduser(LaunchConfiguration('map').perform(context))
    enable_lidar = LaunchConfiguration('enable_lidar').perform(context).lower() in (
        '1', 'true', 'yes', 'on')

    if robot not in ('allterrain', 'mecanum', 'xdrive', 'grip'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive | grip")
    if map_yaml and not os.path.isfile(map_yaml):
        raise RuntimeError(f"Map file not found: {map_yaml}")

    share = get_package_share_directory('skill23_studio')
    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')
    allterrain_share = get_package_share_directory('allterrain')
    gate_params_file = os.path.join(allterrain_share, 'config', 'confidence_scan_gate.yaml')
    hw_params = os.path.join(share, 'config', 'hw_params', f'hw_params_{robot}.yaml')
    exec_path = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')
    ekf_yaml = ekf_yaml_for_robot(robot, navigation=True)
    nav2_params = _prepare_nav2_params(robot, share)
    loc_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
    nav_launch = os.path.join(share, 'launch', 'studio_nav2_navigation.launch.py')

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
        with open(urdf_file, 'r') as handle:
            robot_desc = handle.read()

    actions = [
        LogInfo(msg=f'[studio_nav2] robot={robot} map={map_yaml}'),
        ExecuteProcess(cmd=_control_server_cmd(exec_path, hw_params), output='screen'),
    ]
    if robot_desc:
        actions.append(_node(
            'robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ))
    actions.append(_node(drive[0], drive[1], drive[2]))

    actions.append(TimerAction(period=1.5, actions=[
        _node(
            'robot_localization', 'ekf_node', 'ekf_filter_node',
            parameters=[ekf_yaml],
        ),
    ]))

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
            _node('allterrain', 'lidar_merger_node', 'lidar_merger_node'),
        ]))
        # Optional: confidence scan gate → /scan_filtered (costmaps only).
        # Studio nav2_params use /scan today; enable only if you switch costmaps
        # to /scan_filtered and uncomment the block below.
        # confidence_gate_node = _node(
        #     'allterrain', 'confidence_scan_gate', 'confidence_scan_gate_node',
        #     parameters=[gate_params_file],
        # )
        # actions.append(TimerAction(period=6.0, actions=[
        #     LogInfo(msg='[studio_nav2] Launching confidence scan gate → /scan_filtered'),
        #     confidence_gate_node,
        # ]))

    nav2_args = {
        'namespace': '',
        'use_sim_time': 'false',
        'autostart': 'true',
        'params_file': nav2_params,
        'use_composition': 'True',
        'use_respawn': 'False',
        'container_name': 'nav2_container',
        'log_level': 'info',
    }
    nav2 = [
        Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[nav2_params, {'autostart': True}],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(loc_launch),
            launch_arguments={
                **nav2_args,
                'map': map_yaml,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments=nav2_args.items(),
        ),
    ]
    actions.append(TimerAction(
        period=7.0 if enable_lidar else 3.0,
        actions=[
            LogInfo(msg='[studio_nav2] Launching Nav2 (Theta* + RPP, no velocity_smoother)'),
            *nav2,
        ],
    ))
    return actions


def generate_launch_description():
    default_map = os.path.expanduser('~/skill23_ws/test_map10.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot', default_value='grip',
            description='Robot platform: allterrain | mecanum | xdrive | grip'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Absolute path to map.yaml (same as custom-nav deploy)'),
        DeclareLaunchArgument(
            'enable_lidar', default_value='true',
            description='Start dual YDLidar + merger publishing /scan'),
        OpaqueFunction(function=_setup),
    ])
