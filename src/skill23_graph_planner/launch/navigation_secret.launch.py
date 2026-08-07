"""nav2_secret_mode: GraphPlanner corridors + RPP follow."""

import os
import sys

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from _graph_nav_helper import (
    default_graph_path,
    get_robot_config,
    prepare_secret_nav2_params,
)


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    cfg = get_robot_config(robot)
    studica_share = get_package_share_directory('studica_control')

    map_path = LaunchConfiguration('map').perform(context).strip()
    if not map_path:
        map_path = os.path.expanduser(cfg['default_map'])
    else:
        map_path = os.path.expanduser(map_path)

    graph_path = LaunchConfiguration('graph').perform(context).strip()
    if not graph_path:
        graph_path = default_graph_path(map_path)
    else:
        graph_path = os.path.expanduser(graph_path)

    nav2_params = prepare_secret_nav2_params(robot, graph_path)

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(studica_share, 'launch', cfg['nav_launch'])),
        launch_arguments={
            'map': map_path,
            'nav2_params_file': nav2_params,
        }.items(),
    )

    return [
        LogInfo(msg=[
            f'[nav2_secret_mode] robot={robot} map={map_path} '
            f'graph={graph_path} params={nav2_params}'
        ]),
        nav_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='mecanum',
            description='allterrain | mecanum | xdrive | grip'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument(
            'graph', default_value='',
            description='GeoJSON graph; default <map_stem>_graph.geojson'),
        OpaqueFunction(function=_launch_setup),
    ])
