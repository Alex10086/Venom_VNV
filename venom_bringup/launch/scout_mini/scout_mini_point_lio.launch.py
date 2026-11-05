"""Scout Mini + Point-LIO bringup launch file.

Integrates:
  1. Scout Mini base driver (can0, wheel odom isolated to scout_odom)
  2. Livox MID360 driver (/livox/lidar, /livox/imu)
  3. Point-LIO (3D LiDAR-inertial odometry -> /odom, /cloud_registered, TF: odom -> base_link)
  4. Pointcloud to laserscan (/cloud_registered -> /scan)
  5. Static TF transforms (venom_robot_description)
  6. Optional RViz2 visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scout_base_dir = get_package_share_directory('scout_base')
    venom_bringup_dir = get_package_share_directory('venom_bringup')
    robot_description_dir = get_package_share_directory('venom_robot_description')

    headless = LaunchConfiguration('headless')
    port_name = LaunchConfiguration('port_name')
    point_lio_cfg = LaunchConfiguration('point_lio_cfg')
    livox_cfg = LaunchConfiguration('livox_cfg')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Do not launch RViz if true'
    )
    declare_port_name = DeclareLaunchArgument(
        'port_name',
        default_value='can0',
        description='CAN bus interface for Scout Mini'
    )
    declare_point_lio_cfg = DeclareLaunchArgument(
        'point_lio_cfg',
        default_value=os.path.join(venom_bringup_dir, 'config', 'scout_mini', 'point_lio_mapping.yaml'),
        description='Path to Point-LIO parameter YAML'
    )
    declare_livox_cfg = DeclareLaunchArgument(
        'livox_cfg',
        default_value=os.path.join(venom_bringup_dir, 'config', 'scout_mini', 'MID360_config.json'),
        description='Path to Livox MID360 config JSON'
    )

    # 1. Scout Mini Base Driver
    scout_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scout_base_dir, 'launch', 'scout_mini_base.launch.py')
        ),
        launch_arguments={
            'port_name': port_name,
            'is_scout_mini': 'true',
            'is_omni_wheel': 'false',
            'odom_frame': 'scout_odom',
            'odom_topic_name': 'scout_odom',
            'base_frame': 'scout_base_link',
        }.items()
    )

    # 2. Livox MID360 Driver
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 1},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'body'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': livox_cfg},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ]
    )

    # 3. Point-LIO Node
    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        output='screen',
        parameters=[point_lio_cfg, {'frame.base_frame_id': 'body', 'frame.odom_frame_id': 'odom'}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # 4. Pointcloud to laserscan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.2,
            'min_height': 0.05,
            'max_height': 0.60,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.005,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 30.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/cloud_registered'),
            ('scan', '/scan'),
        ]
    )

    # 5. Static TF transforms
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_dir, 'launch', 'scout_mini_description.launch.py')
        )
    )

    # 6. RViz2 (optional)
    rviz_node = Node(
        condition=UnlessCondition(headless),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(venom_bringup_dir, 'rviz_cfg', 'mapping.rviz')],
        output='screen',
    )

    return LaunchDescription([
        declare_headless,
        declare_port_name,
        declare_point_lio_cfg,
        declare_livox_cfg,
        scout_base_launch,
        livox_driver_node,
        point_lio_node,
        pointcloud_to_laserscan_node,
        robot_description_launch,
        rviz_node,
    ])
