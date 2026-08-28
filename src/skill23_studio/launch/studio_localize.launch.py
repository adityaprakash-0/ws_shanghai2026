"""Arena-live localization only (no planner / mission / world model).

Hardware + lidars + EKF, then Nav2 map_server + skill23_amcl using
skill23_studio/config/amcl.yaml (the same AMCL file as custom-nav).

Stock nav2_amcl + nav2_params hangs after "Creating" and never advertises
amcl/get_state (see localize.log). skill23_amcl is a plain node that
self-configures — do not put it under lifecycle_manager.

  ros2 launch skill23_studio studio_localize.launch.py \\
    robot:=grip map:=/home/vmx/skill23_ws/studio_maps/test1/map.yaml
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
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from skill23_studio.ekf_paths import ekf_yaml_for_robot


def _node(package, executable, name, **kwargs):
    kwargs.setdefault('output', 'screen')
    return Node(package=package, executable=executable, name=name, **kwargs)


def _control_server_cmd(exec_path, hw_params, cpus='2,3'):
    return [
        'sudo', '-E', 'taskset', '-c', cpus, exec_path,
        '--ros-args',
        '-r', '__node:=control_server',
        '--params-file', hw_params,
    ]


def _write_map_yaml(src_yaml: str) -> str:
    """Keep PGM 205 (unknown) as OccupancyGrid -1, not free/white.

    map_server occ = (255 - pixel) / 255. PGM unknown 205 → 0.196.
    free_thresh 0.25 therefore classifies unknown as free.
    """
    with open(src_yaml, 'r', encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle) or {}
    image = str(cfg.get('image') or 'map.pgm')
    if not os.path.isabs(image):
        cfg['image'] = os.path.join(os.path.dirname(os.path.abspath(src_yaml)), image)
    cfg['mode'] = 'trinary'
    cfg['negate'] = int(cfg.get('negate') or 0)
    cfg['free_thresh'] = 0.19
    cfg['occupied_thresh'] = float(cfg.get('occupied_thresh') or 0.65)
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='_studio_map.yaml', delete=False)
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
    if not map_yaml or not os.path.isfile(map_yaml):
        raise RuntimeError(f"Map file not found: {map_yaml}")

    share = get_package_share_directory('skill23_studio')
    studica_prefix = get_package_prefix('studica_control')
    studica_share = get_package_share_directory('studica_control')
    robot_share = get_package_share_directory(robot)

    hw_params = os.path.join(share, 'config', 'hw_params', f'hw_params_{robot}.yaml')
    if not os.path.isfile(hw_params):
        raise RuntimeError(f'Missing HW params: {hw_params}')
    amcl_params = os.path.join(share, 'config', 'amcl.yaml')
    if not os.path.isfile(amcl_params):
        raise RuntimeError(f'Missing AMCL params: {amcl_params}')
    loc_map_yaml = _write_map_yaml(map_yaml)

    ekf_yaml = ekf_yaml_for_robot(robot, navigation=False)

    exec_path = os.path.join(
        studica_prefix, 'lib', 'studica_control', 'manual_composition')

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
    urdf_file = os.path.join(robot_share, 'urdf', urdf_name)
    robot_desc = ''
    if os.path.isfile(urdf_file):
        with open(urdf_file, 'r', encoding='utf-8') as f:
            robot_desc = f.read()

    actions = [
        LogInfo(msg=(
            f'[studio_localize] robot={robot} map={map_yaml} '
            f'skill23_amcl params={amcl_params}')),
        ExecuteProcess(cmd=_control_server_cmd(exec_path, hw_params), output='screen'),
    ]

    if robot_desc:
        actions.append(_node(
            'robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ))

    actions.append(_node(drive[0], drive[1], drive[2]))

    # Studica gamepad reads /joy — without this the stick never moves the robot
    # and idle zeros are not the issue (component already latches one zero).
    actions.append(_node(
        'joy', 'joy_node', 'studio_joy_node',
        parameters=[{
            'autorepeat_rate': 20.0,
            'deadzone': 0.08,
        }],
    ))

    actions.append(TimerAction(period=1.5, actions=[
        _node(
            'robot_localization', 'ekf_node', 'ekf_filter_node',
            parameters=[ekf_yaml],
        ),
    ]))

    if enable_lidar:
        y_front = os.path.join(studica_share, 'config', 'front_lidar_nav.yaml')
        y_back = os.path.join(studica_share, 'config', 'back_lidar_nav.yaml')
        if not os.path.isfile(y_front):
            y_front = os.path.join(studica_share, 'config', 'front_lidar.yaml')
        if not os.path.isfile(y_back):
            y_back = os.path.join(studica_share, 'config', 'back_lidar.yaml')
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

    loc_delay = 6.5 if enable_lidar else 3.0
    actions.append(TimerAction(period=loc_delay, actions=[
        LogInfo(msg=f'[studio_localize] map_server yaml={loc_map_yaml}'),
        _node(
            'nav2_map_server', 'map_server', 'map_server',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': loc_map_yaml,
                'topic_name': 'map',
                'frame_id': 'map',
            }],
        ),
        _node(
            'nav2_lifecycle_manager', 'lifecycle_manager',
            'lifecycle_manager_localization',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 60.0,
            }],
        ),
    ]))
    # skill23_amcl is not a lifecycle node. Wait for map_server to latch /map.
    actions.append(TimerAction(period=loc_delay + 2.0, actions=[
        LogInfo(msg='[studio_localize] starting skill23_amcl (wait_for_world_ready=false)'),
        _node(
            'skill23_amcl', 'amcl', 'amcl',
            parameters=[
                amcl_params,
                {
                    'wait_for_world_ready': False,
                    'use_sim_time': False,
                },
            ],
        ),
    ]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot', default_value='grip',
            description='Robot platform: allterrain | mecanum | xdrive | grip'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Absolute path to map.yaml'),
        DeclareLaunchArgument(
            'enable_lidar', default_value='true',
            description='Start dual YDLidar + merger'),
        OpaqueFunction(function=_setup),
    ])
