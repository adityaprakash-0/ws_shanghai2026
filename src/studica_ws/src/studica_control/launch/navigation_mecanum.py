import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    # Package Directories
    studica_pkg_prefix = get_package_prefix('studica_control')
    studica_pkg_share  = get_package_share_directory('studica_control')
    allterrain_pkg_share = get_package_share_directory('allterrain')
    mecanum_pkg_share = get_package_share_directory('mecanum')
    gate_params_file = os.path.join(allterrain_pkg_share, 'config', 'confidence_scan_gate.yaml')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
 
    # File Paths
    exec_path = os.path.join(studica_pkg_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(studica_pkg_share, 'config', 'params_mecanum.yaml')
    urdf_file = os.path.join(mecanum_pkg_share, 'urdf', 'mecanum_robot.urdf')
    ekf_config_file = os.path.join(mecanum_pkg_share, 'config', 'ekf_mecanum.yaml')
    
    # Pathing for Nav2 and Maps
    default_nav2_params_path = os.path.join(mecanum_pkg_share, 'config', 'nav2_mecanum.yaml')
    # Hardcoded path to your active workspace map
    default_map_path = os.path.expanduser('~/skill23_ws/test_map11.yaml')

    ydlidar_front_config = os.path.join(studica_pkg_share, 'config', 'front_lidar_nav.yaml')
    ydlidar_back_config = os.path.join(studica_pkg_share, 'config', 'back_lidar_nav.yaml')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Declare Launch Arguments
    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    map_arg = DeclareLaunchArgument('map', default_value=default_map_path)
    nav2_params_arg = DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params_path)

    # 1. Core Hardware Driver (Runs as root)
    manual_composition = ExecuteProcess(
        cmd=['sudo', '-E', exec_path, '--ros-args', '-r', '__node:=control_server', '--params-file', LaunchConfiguration('params_file')],
        output='screen',
    )

    # 2. Robot State Publisher (Reads URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. Sensor frames from URDF (imu_link, laser_front ±0.192, laser_back yaw=π)
    
    # 4. Hardware LiDAR Drivers
    ydlidar_front_node = Node(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node', name='ydlidar_front',
        output='screen', parameters=[ydlidar_front_config],
        remappings=[('/scan', '/scan/front')]
    )
    
    ydlidar_back_node = Node(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node', name='ydlidar_back',
        output='screen', parameters=[ydlidar_back_config],
        remappings=[('/scan', '/scan/back')]
    )

    # 5. The New C++ LiDAR Merger
    lidar_merger_node = Node(
        package='allterrain',
        executable='lidar_merger_node',
        name='lidar_merger_node',
        output='screen'
    )

    # 6. RF2O Laser Odometry
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[{
            'laser_scan_topic': '/scan',        
            'odom_topic': '/laser/odom',        
            'publish_tf': False,                
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0                        
        }]
    )

    # 7. EKF Odometry Filter
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )

    # 8. Kinematics (Hardware Encoders)
    mecanum_node = Node(
        package='mecanum',
        executable='mecanum_node',
        name='mecanum_node',
        output='screen'
    )

    # 9. Nav2 Bringup Command
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false', # MUST be false for physical robot
            'params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': 'true'
        }.items()
    )

    # /scan → gate → /scan_filtered (costmaps only); AMCL uses raw /scan
    # --- DISABLED for Pi CPU testing (gate is already C++ in allterrain) ---
    # To re-enable: uncomment confidence_gate_node + delayed_gate, remove delayed_scan_relay.
    # confidence_gate_node = Node(
    #     package='allterrain',
    #     executable='confidence_scan_gate',
    #     name='confidence_scan_gate_node',
    #     output='screen',
    #     parameters=[gate_params_file],
    # )
    scan_filtered_relay = Node(
        package='allterrain',
        executable='scan_relay',
        name='scan_filtered_relay',
        output='screen',
        parameters=[{
            'input_scan_topic': '/scan',
            'output_scan_topic': '/scan_filtered',
        }],
    )

    # --- Staggered Timers to prevent race conditions & CPU choking ---
    delayed_rf2o = TimerAction(period=12.0, actions=[rf2o_node])
    delayed_ekf = TimerAction(period=3.0, actions=[ekf_node])

    # Wait for EKF + scan_filtered before dumping Nav2 on the Pi
    delayed_nav2_cmd = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="[INIT] Hardware & EKF warm-up complete. Launching Nav2 Stack..."),
            nav2_bringup_cmd
        ]
    )


    # delayed_gate = TimerAction(
    #     period=15.0,
    #     actions=[
    #         LogInfo(msg="[INIT] Map & Nav2 active. Launching Confidence Scan Gate..."),
    #         confidence_gate_node
    #     ]
    # )
    # Costmaps need /scan_filtered before Nav2 activates — start relay early
    delayed_scan_relay = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="[INIT] Gate disabled — relaying /scan -> /scan_filtered for costmaps"),
            scan_filtered_relay,
        ],
    )

    fake_map_odom_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='fake_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )
    
    fake_odom_base_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='fake_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_footprint']
    )

    # =========================================================================
    # HARDWARE BUG FIX: The YDLidar Anti-Zombie Shutdown Sequence
    # This forces the launch system to gracefully close the YDLidar serial port 
    # when you press Ctrl+C, preventing the 0x202 Internal Error loop.
    # =========================================================================
    shutdown_handler_front = RegisterEventHandler(
        OnProcessExit(
            target_action=ydlidar_front_node,
            on_exit=[
                LogInfo(msg='YDLidar Front process exited. Triggering clean system shutdown...'),
                EmitEvent(event=Shutdown(reason='YDLidar Front closed cleanly'))
            ]
        )
    )

    shutdown_handler_back = RegisterEventHandler(
        OnProcessExit(
            target_action=ydlidar_back_node,
            on_exit=[
                LogInfo(msg='YDLidar Back process exited. Triggering clean system shutdown...'),
                EmitEvent(event=Shutdown(reason='YDLidar Back closed cleanly'))
            ]
        )
    )

    return LaunchDescription([
        params_arg,
        map_arg,
        nav2_params_arg,
        manual_composition,
        robot_state_publisher,
    #  fake_map_odom_tf,   # <-- ADDED
    #  fake_odom_base_tf,  # <-- ADDED
        ydlidar_front_node,
        ydlidar_back_node,
        lidar_merger_node,
        mecanum_node,
        delayed_ekf,
        delayed_nav2_cmd,
        delayed_scan_relay,
        # shutdown_handler_front,
        # shutdown_handler_back,
    ])


# import os
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, LogInfo, RegisterEventHandler, EmitEvent
# from launch.events import Shutdown
# from launch.event_handlers import OnProcessExit
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory, get_package_prefix

# def generate_launch_description():
#     studica_pkg_prefix = get_package_prefix('studica_control')
#     studica_pkg_share  = get_package_share_directory('studica_control')
#     allterrain_pkg_share = get_package_share_directory('allterrain')
#     mecanum_pkg_share = get_package_share_directory('mecanum')
 
#     exec_path = os.path.join(studica_pkg_prefix, 'lib', 'studica_control', 'manual_composition')
#     default_params = os.path.join(studica_pkg_share, 'config', 'params_mecanum.yaml')
#     urdf_file = os.path.join(mecanum_pkg_share, 'urdf', 'mecanum_robot.urdf')
#     ekf_config_file = os.path.join(mecanum_pkg_share, 'config', 'ekf_mecanum.yaml')
#     ydlidar_front_config = os.path.join(studica_pkg_share, 'config', 'front_lidar_nav.yaml')
#     ydlidar_back_config = os.path.join(studica_pkg_share, 'config', 'back_lidar_nav.yaml')

#     with open(urdf_file, 'r') as infp:
#         robot_desc = infp.read()

#     params_arg = DeclareLaunchArgument('params_file', default_value=default_params)

#     manual_composition = ExecuteProcess(
#         cmd=['sudo', '-E', exec_path, '--ros-args', '-r', '__node:=control_server', '--params-file', LaunchConfiguration('params_file')],
#         output='screen',
#     )

#     robot_state_publisher = Node(
#         package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
#         output='screen', parameters=[{'robot_description': robot_desc}]
#     )

#     imu_tf = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_imu', arguments=['0', '0', '0.20', '0', '0', '0', '1', 'base_link', 'imu_link'])
#     laser_front_tf = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_laser_front', arguments=['0.172', '0', '0.15', '0', '0', '0', '1', 'base_link', 'laser_front'])
#     laser_back_tf = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_laser_back', arguments=['-0.172', '0', '0.15', '3.14159', '0', '0', 'base_link', 'laser_back'])

#     ydlidar_front_node = Node(
#         package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node', name='ydlidar_front',
#         output='screen', parameters=[ydlidar_front_config], remappings=[('/scan', '/scan/front')]
#     )

#     ydlidar_back_node = Node(
#         package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node', name='ydlidar_back',
#         output='screen', parameters=[ydlidar_back_config], remappings=[('/scan', '/scan/back')]
#     )

#     lidar_merger_node = Node(package='allterrain', executable='lidar_merger_node', name='lidar_merger_node', output='screen')

#     rf2o_node = Node(
#         package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node', name='rf2o_laser_odometry',
#         output='screen', arguments=['--ros-args', '--log-level', 'WARN'],
#         parameters=[{'laser_scan_topic': '/scan', 'odom_topic': '/laser/odom', 'publish_tf': False, 'base_frame_id': 'base_footprint', 'odom_frame_id': 'odom', 'init_pose_from_topic': '', 'freq': 20.0}]
#     )

#     ekf_node = Node(
#         package='robot_localization', executable='ekf_node', name='ekf_filter_node',
#         output='screen', parameters=[ekf_config_file]
#     )

#     mecanum_node = Node(package='mecanum', executable='mecanum_node', name='amecanum_node', output='screen')

#     delayed_rf2o = TimerAction(period=12.0, actions=[rf2o_node])
#     delayed_ekf = TimerAction(period=3.0, actions=[ekf_node])

#     shutdown_handler_front = RegisterEventHandler(OnProcessExit(target_action=ydlidar_front_node, on_exit=[LogInfo(msg='YDLidar Front closed.'), EmitEvent(event=Shutdown(reason='YDLidar closed'))]))
#     shutdown_handler_back = RegisterEventHandler(OnProcessExit(target_action=ydlidar_back_node, on_exit=[LogInfo(msg='YDLidar Back closed.'), EmitEvent(event=Shutdown(reason='YDLidar closed'))]))

#     return LaunchDescription([
#         params_arg, manual_composition, robot_state_publisher,
#         imu_tf, laser_front_tf, laser_back_tf,
#         ydlidar_front_node, ydlidar_back_node, lidar_merger_node, mecanum_node,
#         #delayed_rf2o, 
#         delayed_ekf,
#         shutdown_handler_front, shutdown_handler_back
#     ])