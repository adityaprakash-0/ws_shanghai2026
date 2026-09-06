"""Lightweight planner preview — map_server + global costmap + planner_server only.

No robot hardware, no AMCL, no obstacle layer, no controller / BT.
Static map + inflation on the global costmap; planner_server exposes
compute_path_to_pose for Studio simulation.

RViz2 (on the Pi or laptop with ROS_DOMAIN_ID set):
  • Fixed Frame: map
  • Add → Map → Topic /map
  • Add → Map → Topic /global_costmap/costmap (or costmap_updates)

  ros2 launch skill23_studio studio_gc_planner.launch.py robot:=grip \\
    map:=/home/vmx/skill23_ws/studio_maps/myarena/map.yaml
"""

from __future__ import annotations

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from skill23_studio.ros_dds import studio_dds_launch_actions


def _node(package, executable, name, **kwargs):
    kwargs.setdefault('output', 'screen')
    return Node(package=package, executable=executable, name=name, **kwargs)


def _prepare_params(robot: str, share: str, map_yaml: str) -> str:
    src = os.path.join(
        share, 'config', 'nav2_params', 'nav2_params_gc_planner.yaml')
    with open(src, 'r', encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle)
    cfg['map_server']['ros__parameters']['yaml_filename'] = map_yaml
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix=f'_gc_planner_{robot}.yaml', delete=False)
    yaml.dump(cfg, tmp, default_flow_style=False, sort_keys=False)
    tmp.close()
    return tmp.name


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    map_yaml = os.path.expanduser(LaunchConfiguration('map').perform(context))
    if robot not in ('allterrain', 'mecanum', 'xdrive', 'grip'):
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive | grip")
    if not map_yaml or not os.path.isfile(map_yaml):
        raise RuntimeError(f'Map file not found: {map_yaml}')

    share = get_package_share_directory('skill23_studio')
    params_file = _prepare_params(robot, share, map_yaml)

    return [
        LogInfo(msg=f'[gc_planner] map={map_yaml} params={params_file}'),
        LogInfo(msg=(
            '[gc_planner] Open RViz2: Fixed Frame map, display /map and '
            '/global_costmap/costmap — then use Studio Simulate (2 points).'
        )),
        # Identity TF so global_costmap can track base_footprint in map frame.
        _node(
            'tf2_ros', 'static_transform_publisher', 'map_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint']),
        TimerAction(period=0.5, actions=[
            _node(
                'nav2_map_server', 'map_server', 'map_server',
                parameters=[params_file],
            ),
            _node(
                'nav2_planner', 'planner_server', 'planner_server',
                parameters=[params_file],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            ),
            _node(
                'nav2_lifecycle_manager', 'lifecycle_manager',
                'lifecycle_manager_gc_planner',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['map_server', 'planner_server'],
                    'bond_timeout': 60.0,
                }],
            ),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        *studio_dds_launch_actions(),
        DeclareLaunchArgument(
            'robot', default_value='grip',
            description='Robot platform (selects footprint radius in future)'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Absolute path to map.yaml'),
        OpaqueFunction(function=_setup),
    ])
