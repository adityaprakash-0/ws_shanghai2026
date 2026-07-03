import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    # Package Directories
    studica_pkg_prefix = get_package_prefix('studica_control')
    studica_pkg_share  = get_package_share_directory('studica_control')
    allterrain_pkg_share = get_package_share_directory('allterrain')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
 
    # File Paths
    exec_path = os.path.join(studica_pkg_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(studica_pkg_share, 'config', 'params_allterrain.yaml')
    urdf_file = os.path.join(allterrain_pkg_share, 'urdf', 'allterrain_robot.urdf')
    ekf_config_file = os.path.join(allterrain_pkg_share, 'config', 'ekf_allterrain.yaml')
    
    # Pathing for Nav2 and Maps
    default_nav2_params_path = os.path.join(allterrain_pkg_share, 'config', 'nav2_allterrain.yaml')
    # Hardcoded path to your active workspace map
    default_map_path = os.path.expanduser('~/skill23_ws/test_map10.yaml')

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

    # 3. Static Transforms (IMU and Dual LiDARs)
    imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_imu',
        arguments=['0', '0', '0.20', '0', '0', '0', '1', 'base_link', 'imu_link']
    )
    
    laser_front_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_laser_front',
        arguments=['0.172', '0', '0.15', '0', '0', '0', '1', 'base_link', 'laser_front']
    )
    
    laser_back_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_laser_back',
        arguments=['-0.172', '0', '0.15', '3.14159', '0', '0', 'base_link', 'laser_back']
    )

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
    allterrain_node = Node(
        package='allterrain',
        executable='allterrain_node',
        name='allterrain_node',
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
    
    confidence_gate_node = Node(
        package='allterrain',
        executable='confidence_scan_gate',
        name='confidence_scan_gate_node',
        output='screen',
        parameters=[{
            'input_scan_topic': '/scan',
            'output_scan_topic': '/scan_filtered',
            'map_topic': '/map',
            'odom_topic': '/odometry/filtered', # EKF default output
        }]
    )
        

    # --- Staggered Timers to prevent race conditions & CPU choking ---
    delayed_rf2o = TimerAction(period=12.0, actions=[rf2o_node])
    delayed_ekf = TimerAction(period=3.0, actions=[ekf_node])

    # Wait 18 seconds total (8 seconds after EKF starts) so the TF tree is perfectly established
    delayed_nav2_cmd = TimerAction(
        period=7.0, 
        actions=[
            LogInfo(msg="[INIT] Hardware & EKF warm-up complete. Launching Nav2 Stack..."),
            nav2_bringup_cmd
        ]
    )

    delayed_gate = TimerAction(
        period=15.0, 
        actions=[
            LogInfo(msg="[INIT] Map & Nav2 active. Launching Confidence Scan Gate..."),
            confidence_gate_node
        ]
    )

    fake_map_odom_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='fake_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )
    
    fake_odom_base_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='fake_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_footprint']
    )

    return LaunchDescription([
        params_arg,
        map_arg,
        nav2_params_arg,
        manual_composition,
        robot_state_publisher,
        imu_tf,
        laser_front_tf,
        laser_back_tf,
    #  fake_map_odom_tf,   # <-- ADDED
    #  fake_odom_base_tf,  # <-- ADDED
        ydlidar_front_node,
        ydlidar_back_node,
        lidar_merger_node,
        allterrain_node,
       #delayed_rf2o,         # T+8s: Starts Laser Odometry
        delayed_ekf,          # T+10s: Starts Sensor Fusion (IMU + Laser)
       confidence_gate_node
       #delayed_nav2_cmd,      # T+18s: Starts Nav2 Path Planning & AMCL
       #delayed_gate

    ])