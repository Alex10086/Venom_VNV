"""Mapless, omnidirectional Sentry navigation with opt-in hardware."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_stack(context):
    config_dir = LaunchConfiguration('config_dir').perform(context)
    robot_file = os.path.join(config_dir, 'robot.yaml')
    with open(robot_file, encoding='utf-8') as config_file:
        robot = yaml.safe_load(config_file)

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    clock = {'use_sim_time': use_sim_time}
    nav_condition = IfCondition(LaunchConfiguration('nav_enabled'))
    description_dir = get_package_share_directory('venom_robot_description')

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                description_dir, 'launch', 'sentry_description.launch.py')),
            launch_arguments={'config_file': robot_file}.items(),
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('lidar_enabled')),
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[robot['lidar'], clock, {
                'user_config_path': os.path.join(config_dir, 'mid360.json'),
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('lio_enabled')),
            package='point_lio',
            executable='pointlio_mapping',
            name='point_lio',
            output='screen',
            parameters=[os.path.join(config_dir, 'point_lio.yaml'), clock],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[robot['scan'], clock],
            remappings=[('cloud_in', '/cloud_registered'), ('scan', '/scan')],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('serial_enabled')),
            package='venom_serial_driver',
            executable='serial_node',
            name='serial_node',
            output='screen',
            parameters=[robot['serial'], clock, {
                'port_name': LaunchConfiguration('port_name'),
            }],
        ),
    ]

    # Keep the node set independent of nav2_bringup's optional route/docking nodes.
    nav_nodes = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]
    nav_file = os.path.join(config_dir, 'nav2.yaml')
    for package, executable in nav_nodes:
        remappings = []
        if executable in ('controller_server', 'behavior_server', 'velocity_smoother'):
            remappings.append(('cmd_vel', '/cmd_vel_nav'))
        if executable == 'velocity_smoother':
            remappings.append(('cmd_vel_smoothed', '/cmd_vel'))
        actions.append(Node(
            condition=nav_condition,
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=[nav_file, clock],
            remappings=remappings,
        ))
    actions.append(Node(
        condition=nav_condition,
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[clock, {
            'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
            'node_names': [executable for _, executable in nav_nodes],
        }],
    ))
    actions.append(Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[clock],
        arguments=['-f', 'odom'],
    ))
    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('venom_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_dir',
            default_value=os.path.join(bringup_dir, 'config', 'sentry_v1'),
            description='Directory containing mid360.json, point_lio.yaml, nav2.yaml and robot.yaml.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'lidar_enabled', default_value='false',
            description='Start MID360; disabled for hardware-free bringup.',
        ),
        DeclareLaunchArgument(
            'lio_enabled', default_value='false',
            description='Start Point-LIO; enable for live lidar or raw bag input.',
        ),
        DeclareLaunchArgument(
            'serial_enabled', default_value='false',
            description='Enable C-board commands only after physical calibration and safety checks.',
        ),
        DeclareLaunchArgument('nav_enabled', default_value='true'),
        DeclareLaunchArgument(
            'autostart', default_value='false',
            description='Activate Nav2 automatically only when odom, TF and cloud input are ready.',
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'port_name', default_value='/dev/ttyACM0',
            description='C-board serial device; used only when serial_enabled is true.',
        ),
        OpaqueFunction(function=_launch_stack),
    ])
