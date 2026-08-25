"""Arena-live localization only (no planner / mission).

Starts hardware, lidars, EKF, world model (map), and AMCL so the robot
localizes on a saved studio map. Used by Skill23 Studio "Arena (live)".

  ros2 launch skill23_studio studio_localize.launch.py \\
    robot:=grip map:=/home/vmx/skill23_ws/studio_maps/test1/map.yaml
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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
    """Same as slam_tele: sudo -E <binary> --ros-args … (no env PATH= wrapper).

    Pinned like the nav launch: the HAL's SCHED_FIFO SPI poll would otherwise
    preempt AMCL on whichever core it lands on.
    """
    return [
        'sudo', '-E', 'taskset', '-c', cpus, exec_path,
        '--ros-args',
        '-r', '__node:=control_server',
        '--params-file', hw_params,
    ]


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

    hw_params = os.path.join(share, 'config', f'hw_params_{robot}.yaml')
    amcl_params = os.path.join(share, 'config', 'amcl.yaml')
    if not os.path.isfile(amcl_params):
        raise RuntimeError(f"Missing {amcl_params}")

    # Same EKF as mapping (slam_tele): full holonomic for xdrive/mecanum.
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

    map_params = [{'map': map_yaml}]
    actions = []

    actions.append(ExecuteProcess(
        cmd=_control_server_cmd(exec_path, hw_params), output='screen'))

    if robot_desc:
        actions.append(_node(
            'robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ))

    actions.append(_node(drive[0], drive[1], drive[2]))

    # World model → /map + /custom_nav/world/ready (AMCL waits on this)
    actions.append(_node(
        'skill23_studio', 'cn_world_model', 'cn_world_model',
        parameters=map_params,
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

    amcl_delay = 6.0 if enable_lidar else 3.0
    actions.append(TimerAction(period=amcl_delay, actions=[
        _node(
            'skill23_amcl', 'amcl', 'amcl',
            parameters=[amcl_params],
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
