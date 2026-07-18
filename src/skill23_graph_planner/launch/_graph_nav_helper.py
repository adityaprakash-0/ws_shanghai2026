"""Helpers for skill23_graph_planner / nav2_secret_mode launch."""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

ROBOT_CONFIG = {
    'allterrain': {
        'nav_launch': 'navigation_allterrain.py',
        'default_map': '~/skill23_ws/test_map10.yaml',
        'secret_nav2_yaml': 'nav2_secret_allterrain.yaml',
    },
    'mecanum': {
        'nav_launch': 'navigation_mecanum.py',
        'default_map': '~/skill23_ws/test_map11.yaml',
        'secret_nav2_yaml': 'nav2_secret_mecanum.yaml',
    },
    'xdrive': {
        'nav_launch': 'navigation_xdrive.py',
        'default_map': '~/skill23_ws/test_map10.yaml',
        'secret_nav2_yaml': 'nav2_secret_xdrive.yaml',
    },
}


def get_robot_config(robot: str) -> dict:
    if robot not in ROBOT_CONFIG:
        raise RuntimeError(
            f"Unknown robot '{robot}'. Use: allterrain | mecanum | xdrive")
    return ROBOT_CONFIG[robot]


def default_graph_path(map_path: str) -> str:
    expanded = os.path.expanduser(map_path)
    stem = os.path.splitext(os.path.basename(expanded))[0]
    return os.path.join(os.path.dirname(expanded), f'{stem}_graph.geojson')


def prepare_secret_nav2_params(robot: str, graph_filepath: str) -> str:
    """Load copied secret nav2 yaml; only fill in graph_filepath at launch."""
    cfg = get_robot_config(robot)
    share = get_package_share_directory('skill23_graph_planner')
    base_yaml = os.path.join(share, 'config', cfg['secret_nav2_yaml'])

    with open(base_yaml, 'r', encoding='utf-8') as handle:
        nav_cfg = yaml.safe_load(handle)

    graph_filepath = os.path.expanduser(graph_filepath)
    if not os.path.isfile(graph_filepath):
        raise RuntimeError(f"Graph file not found: {graph_filepath}")

    grid = nav_cfg['planner_server']['ros__parameters']['GridBased']
    grid['graph_filepath'] = graph_filepath

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix=f'_secret_nav2_{robot}.yaml', delete=False)
    yaml.dump(nav_cfg, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name
