import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    # Package Directories
    studica_pkg_prefix = get_package_prefix('studica_control')
    studica_pkg_share  = get_package_share_directory('studica_control')
    allterrain_pkg_share = get_package_share_directory('allterrain')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')

    # File Paths
    exec_path = os.path.join(studica_pkg_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(studica_pkg_share, 'config', 'params.yaml')
    urdf_file = os.path.join(allterrain_pkg_share, 'urdf', 'allterrain_robot.urdf')
    ekf_config_file = os.path.join(allterrain_pkg_share, 'config', 'ekf_allterrain.yaml')
    slam_config_file = os.path.join(studica_pkg_share, 'config', 'slam_params.yaml')
    
    ydlidar_front_config = os.path.join(studica_pkg_share, 'config', 'front_lidar.yaml')
    ydlidar_back_config = os.path.join(studica_pkg_share, 'config', 'back_lidar.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)

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
        parameters=[{
            'laser_scan_topic': '/scan',        
            'odom_topic': '/laser/odom',        # Changed to custom topic
            'publish_tf': False,                # Disabled so EKF can take over TF
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 7.0                        
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

    # 9. SLAM Toolbox (Mapping Engine)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file]
    )


    delayed_ydf = TimerAction(period=2.0, actions=[ydlidar_front_node])
    delayed_ydb = TimerAction(period=5.0, actions=[ydlidar_back_node])

   
    # --- Staggered Timers to prevent race conditions ---
    delayed_rf2o = TimerAction(period=8.0, actions=[rf2o_node])
    delayed_ekf = TimerAction(period=7.0, actions=[ekf_node])
    delayed_slam = TimerAction(period=9.0, actions=[slam_node])

    return LaunchDescription([
        params_arg,
        manual_composition,
        robot_state_publisher,
        imu_tf,
        laser_front_tf,
        laser_back_tf,
       # ydlidar_front_node,
      #  ydlidar_back_node,
        allterrain_node,
        delayed_ydf,
        delayed_ydb,
       # delayed_rf2o,         # T+8s: Starts Laser Odometry
        delayed_ekf,          # T+10s: Starts Sensor Fusion (IMU + Laser)
      #  delayed_slam          # T+15s: Starts Mapping
    ])