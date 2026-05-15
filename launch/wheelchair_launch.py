#!/usr/bin/env python3
"""
ROS 2 Launch File: Autonomous Smart Wheelchair System
Launches complete navigation stack: LiDAR drivers, SLAM, Nav2, DQN controller
"""
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, TimerAction, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate complete wheelchair autonomy launch description"""
    
    # Package directories
    pkg_dir = get_package_share_directory('smart_wheelchair_autonomy')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    cartographer_pkg = get_package_share_directory('cartographer_ros')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_mode = LaunchConfiguration('slam_mode', default='mapping')  # mapping | localization
    map_file = LaunchConfiguration('map_file', default='')
    dqn_enabled = LaunchConfiguration('dqn_enabled', default='true')
    rviz_enabled = LaunchConfiguration('rviz_enabled', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time if true'
    )
    declare_slam_mode = DeclareLaunchArgument(
        'slam_mode', default_value='mapping',
        description='SLAM mode: mapping or localization'
    )
    declare_map_file = DeclareLaunchArgument(
        'map_file', default_value='',
        description='Map YAML file for localization mode'
    )
    declare_dqn = DeclareLaunchArgument(
        'dqn_enabled', default_value='true',
        description='Enable DQN obstacle avoidance layer'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz_enabled', default_value='true',
        description='Launch RViz2 visualization'
    )
    
    # ============================================================
    # LIDAR Drivers (2x VLP-16 LiDARs)
    # ============================================================
    lidar_front = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='lidar_front',
        parameters=[{
            'device_ip': '192.168.1.201',
            'port': 2368,
            'model': 'VLP16',
            'rpm': 600.0,
            'frame_id': 'lidar_front_link',
            'use_sim_time': use_sim_time,
        }],
        remappings=[('velodyne_packets', 'lidar_front/packets')],
    )
    
    lidar_rear = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='lidar_rear',
        parameters=[{
            'device_ip': '192.168.1.202',
            'port': 2369,
            'model': 'VLP16',
            'rpm': 600.0,
            'frame_id': 'lidar_rear_link',
            'use_sim_time': use_sim_time,
        }],
        remappings=[('velodyne_packets', 'lidar_rear/packets')],
    )
    
    # PointCloud2 converters
    lidar_front_convert = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        name='lidar_front_convert',
        parameters=[{
            'calibration': os.path.join(pkg_dir, 'config', 'VLP16db.yaml'),
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('velodyne_packets', 'lidar_front/packets'),
            ('velodyne_points', 'lidar_front/points'),
        ],
    )
    
    lidar_rear_convert = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        name='lidar_rear_convert',
        parameters=[{
            'calibration': os.path.join(pkg_dir, 'config', 'VLP16db.yaml'),
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('velodyne_packets', 'lidar_rear/packets'),
            ('velodyne_points', 'lidar_rear/points'),
        ],
    )
    
    # Point cloud merger (front + rear -> combined)
    point_cloud_merger = Node(
        package='point_cloud_merger',
        executable='point_cloud_merger_node',
        name='point_cloud_merger',
        parameters=[{
            'input_topics': ['/lidar_front/points', '/lidar_rear/points'],
            'output_frame': 'base_link',
            'use_sim_time': use_sim_time,
        }],
        remappings=[('merged_cloud', '/points_merged')],
    )
    
    # ============================================================
    # Cartographer SLAM
    # ============================================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            os.path.join(pkg_dir, 'config', 'cartographer'),
            '-configuration_basename',
            'wheelchair_2d.lua'
        ],
        remappings=[
            ('scan', 'scan_merged'),
            ('points2', 'points_merged'),
        ],
    )
    
    cartographer_occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.05,
        }],
    )
    
    # ============================================================
    # EKF State Estimation
    # ============================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ekf_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('odometry/filtered', 'odom')],
    )
    
    # ============================================================
    # Nav2 Navigation Stack
    # ============================================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        }.items(),
    )
    
    # ============================================================
    # DQN Obstacle Avoidance (optional layer)
    # ============================================================
    dqn_controller = Node(
        package='smart_wheelchair_autonomy',
        executable='dqn_controller_node',
        name='dqn_obstacle_avoidance',
        output='screen',
        condition=IfCondition(dqn_enabled),
        parameters=[{
            'model_path': os.path.join(pkg_dir, 'models', 'dqn_wheelchair.pt'),
            'confidence_threshold': 0.45,
            'min_obstacle_dist': 0.3,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('scan', '/scan_merged'),
            ('odom', '/odom'),
            ('cmd_vel_in', '/nav2/cmd_vel'),
            ('cmd_vel_out', '/cmd_vel'),
        ],
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(
                os.path.join(pkg_dir, 'urdf', 'wheelchair.urdf')
            ).read(),
            'use_sim_time': use_sim_time,
        }],
    )
    
    # RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(rviz_enabled),
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'wheelchair_nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Startup sequence with delays
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_slam_mode,
        declare_map_file,
        declare_dqn,
        declare_rviz,
        
        # Hardware drivers (immediate)
        lidar_front,
        lidar_rear,
        
        # Converters (after drivers)
        TimerAction(period=1.0, actions=[lidar_front_convert, lidar_rear_convert]),
        
        # Merger (after converters)
        TimerAction(period=2.0, actions=[point_cloud_merger]),
        
        # SLAM (after sensor stack)
        TimerAction(period=3.0, actions=[
            cartographer_node, 
            cartographer_occupancy_grid
        ]),
        
        # State estimation
        TimerAction(period=2.0, actions=[ekf_node]),
        
        # Robot model
        robot_state_publisher,
        
        # Navigation stack (after SLAM)
        TimerAction(period=5.0, actions=[nav2_bringup]),
        
        # DQN controller (after Nav2)
        TimerAction(period=7.0, actions=[dqn_controller]),
        
        # Visualization (after everything)
        TimerAction(period=8.0, actions=[rviz_node]),
        
        # Log startup complete
        TimerAction(
            period=9.0,
            actions=[LogInfo(msg='Wheelchair autonomy stack fully launched')]
        ),
    ])
