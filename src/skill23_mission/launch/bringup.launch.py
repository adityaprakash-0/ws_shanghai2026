"""skill23_mission bringup — hardware + Nav2 (flat launch style).

Edit ROBOT / MAP_NAME below, then:

  ros2 launch skill23_mission bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from skill23_studio.ekf_paths import ekf_yaml_for_robot
from skill23_studio.ros_dds import studio_dds_launch_actions

# ---------------------------------------------------------------------------
# Edit these
# ---------------------------------------------------------------------------
ROBOT = 'mecanum'       # mecanum | allterrain | grip | xdrive
MAP_NAME = 'default'    # folder under skill23_mission/maps/
ENABLE_LIDAR = True
# ---------------------------------------------------------------------------


def _share():
    return get_package_share_directory('skill23_mission')


def _map_yaml():
    path = os.path.join(_share(), 'maps', MAP_NAME, 'map.yaml')
    if not os.path.isfile(path):
        raise RuntimeError(
            f'Map not found: {path}\n'
            f'Copy map.yaml + map.pgm into skill23_mission/maps/{MAP_NAME}/')
    return path


def _hw_params():
    path = os.path.join(_share(), 'config', 'hw_params', f'hw_params_{ROBOT}.yaml')
    if not os.path.isfile(path):
        raise RuntimeError(f'Missing hw params: {path}')
    return path


def _prepare_nav2_params():
    src = os.path.join(_share(), 'config', 'nav2_params', f'nav2_params_{ROBOT}.yaml')
    with open(src, 'r', encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle)
    bt = cfg['bt_navigator']['ros__parameters']
    for key, default in (
        ('default_bt_xml_filename', 'navigate_nav2.xml'),
        ('default_nav_to_pose_bt_xml', 'navigate_nav2.xml'),
        ('default_nav_through_poses_bt_xml', 'navigate_through_poses_nav2.xml'),
    ):
        name = os.path.basename(str(bt.get(key) or default))
        path = os.path.join(_share(), 'config', 'nav2_params', name)
        if not os.path.isfile(path):
            path = os.path.join(_share(), 'config', 'nav2_params', default)
        bt[key] = path
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix=f'_mission_nav2_{ROBOT}.yaml', delete=False)
    yaml.dump(cfg, tmp, default_flow_style=False, sort_keys=False)
    tmp.close()
    return tmp.name


def _urdf_text():
    urdf_name = {
        'mecanum': 'mecanum_robot.urdf',
        'xdrive': 'xdrive_robot.urdf',
        'allterrain': 'allterrain_robot.urdf',
        'grip': 'grip_robot.urdf',
    }[ROBOT]
    path = os.path.join(get_package_share_directory(ROBOT), 'urdf', urdf_name)
    if not os.path.isfile(path):
        return ''
    with open(path, 'r', encoding='utf-8') as handle:
        return handle.read()


def generate_launch_description():
    if ROBOT not in ('mecanum', 'allterrain', 'grip', 'xdrive'):
        raise RuntimeError(f'Unknown ROBOT={ROBOT!r}')

    map_yaml = _map_yaml()
    hw_params = _hw_params()
    nav2_params = _prepare_nav2_params()
    ekf_yaml = ekf_yaml_for_robot(ROBOT, navigation=True)
    studica_share = get_package_share_directory('studica_control')
    control_exec = os.path.join(
        get_package_prefix('studica_control'),
        'lib', 'studica_control', 'manual_composition')
    loc_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'localization_launch.py')
    nav_launch = os.path.join(
        get_package_share_directory('skill23_studio'),
        'launch', 'studio_nav2_navigation.launch.py')

    drive_pkg, drive_exec, drive_name = {
        'allterrain': ('allterrain', 'allterrain_node', 'allterrain_node'),
        'mecanum': ('mecanum', 'mecanum_node', 'mecanum_node'),
        'xdrive': ('xdrive', 'xdrive_node', 'xdrive_node'),
        'grip': ('grip', 'grip_node', 'grip_node'),
    }[ROBOT]

    robot_desc = _urdf_text()
    nodes = [
        *studio_dds_launch_actions(),

        # Titan control server (needs sudo)
        ExecuteProcess(
            cmd=[
                'sudo', '-E', 'taskset', '-c', '2,3',
                control_exec,
                '--ros-args',
                '-r', '__node:=control_server',
                '--params-file', hw_params,
            ],
            output='screen',
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{'robot_description': robot_desc}],
        ),

        # Drive node
        Node(
            package=drive_pkg,
            executable=drive_exec,
            name=drive_name,
            output='screen',
            emulate_tty=True,
        ),

        # EKF (delay so odom starts first)
        TimerAction(
            period=1.5,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[ekf_yaml],
                ),
            ],
        ),
    ]

    if ENABLE_LIDAR:
        nodes.extend([
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package='ydlidar_ros2_driver',
                        executable='ydlidar_ros2_driver_node',
                        name='ydlidar_front',
                        output='screen',
                        emulate_tty=True,
                        parameters=[os.path.join(
                            studica_share, 'config', 'front_lidar_nav.yaml')],
                        remappings=[('/scan', '/scan/front')],
                    ),
                ],
            ),
            TimerAction(
                period=4.5,
                actions=[
                    Node(
                        package='ydlidar_ros2_driver',
                        executable='ydlidar_ros2_driver_node',
                        name='ydlidar_back',
                        output='screen',
                        emulate_tty=True,
                        parameters=[os.path.join(
                            studica_share, 'config', 'back_lidar_nav.yaml')],
                        remappings=[('/scan', '/scan/back')],
                    ),
                ],
            ),
            TimerAction(
                period=5.5,
                actions=[
                    Node(
                        package='allterrain',
                        executable='lidar_merger_node',
                        name='lidar_merger_node',
                        output='screen',
                        emulate_tty=True,
                    ),
                ],
            ),
        ])

    nav2_delay = 7.0 if ENABLE_LIDAR else 3.0
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
    nodes.append(
        TimerAction(
            period=nav2_delay,
            actions=[
                Node(
                    package='rclcpp_components',
                    executable='component_container_isolated',
                    name='nav2_container',
                    output='screen',
                    emulate_tty=True,
                    parameters=[nav2_params, {'autostart': True}],
                    arguments=['--ros-args', '--log-level', 'info'],
                    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
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
            ],
        ),
    )

    return LaunchDescription(nodes)
