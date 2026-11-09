"""BIEVR online mapping or saved-pose-graph localization with manual Nav2 goals."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from venom_bringup.scout_mount import load_mount


def launch_setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    bringup = Path(get_package_share_directory('venom_bringup'))
    scout = Path(get_package_share_directory('scout_base'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    sim = value('use_sim_time').lower() == 'true'
    mount = load_mount(value('mount_config'))
    geometry = mount.get('vehicle', {})
    lidar = next(t for t in mount['transforms'] if t['child_frame'] == 'scan_link')
    height = float(lidar['translation'][2])
    minimum = float(geometry.get('ground_clearance_filter', .06))
    maximum = float(geometry.get('obstacle_height_max', .65))
    mode = value('slam_mode')
    slam = {'use_sim_time': sim, 'mode': mode, 'transform_timeout': .3,
            'tf_buffer_duration': 30., 'minimum_time_interval': .2,
            'use_map_saver': True, 'max_laser_range': 20.}
    if mode == 'localization':
        prefix = Path(value('map_file')).expanduser()
        if not value('map_file') or not Path(str(prefix)+'.posegraph').is_file():
            raise ValueError('Localization needs map_file:=/path/prefix with prefix.posegraph and prefix.data')
        if not Path(str(prefix)+'.data').is_file():
            raise ValueError('Missing serialized scan data: '+str(prefix)+'.data')
        saved_mount = Path(str(prefix)+'.mount.yaml')
        if saved_mount.is_file() and load_mount(saved_mount)['transforms'] != mount['transforms']:
            raise ValueError('Saved map calibration differs from mount_config; use matching calibration')
        slam.update(map_file_name=str(prefix), map_start_at_dock=False,
                    map_start_pose=[0., 0., 0.])
        if value('initial_pose'):
            pose = json.loads(value('initial_pose'))
            if len(pose) != 3:
                raise ValueError('initial_pose must be [x, y, yaw_radians]')
            slam['map_start_pose'] = [float(x) for x in pose]

    def include(path, arguments=None, condition=None):
        options = {'launch_arguments': (arguments or {}).items()}
        if condition is not None:
            options['condition'] = condition
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), **options)

    return [
        include(bringup/'launch/scout_mini_bievr.launch.py', {
            'start_driver': value('start_driver'), 'rviz': value('rviz'),
            'use_sim_time': value('use_sim_time'), 'rviz_fixed_frame': 'map',
            'mount_config': value('mount_config'),
        }),
        include(scout/'launch/scout_mini_base.launch.py', {
            'port_name': 'can0', 'is_scout_mini': 'true', 'is_omni_wheel': 'false',
            'odom_frame': 'scout_odom', 'odom_topic_name': 'scout_odom',
            'base_frame': 'scout_base_link', 'use_sim_time': value('use_sim_time'),
        }, IfCondition(LaunchConfiguration('start_base'))),
        Node(package='venom_bringup', executable='bievr_scan_filter', output='screen',
             parameters=[{'use_sim_time': sim, 'vehicle_length': float(geometry.get('length', .60)),
                          'vehicle_width': float(geometry.get('width', .57)),
                          'self_padding': float(geometry.get('footprint_padding', .03)),
                          'min_height': minimum, 'max_height': maximum}]),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan', output='screen', parameters=[{
                 'use_sim_time': sim, 'target_frame': 'scan_link', 'transform_tolerance': .3,
                 'min_height': minimum-height-.001, 'max_height': maximum-height+.001,
                 'angle_min': -3.14159, 'angle_max': 3.14159,
                 'angle_increment': .005, 'scan_time': .1,
                 'range_min': .3, 'range_max': 20., 'use_inf': True,
             }], remappings=[('cloud_in', '/bievr_lio/points/navigation'), ('scan', '/scan')]),
        TimerAction(period=8., actions=[
            Node(package='slam_toolbox',
                 executable='localization_slam_toolbox_node' if mode == 'localization' else 'async_slam_toolbox_node',
                 name='slam_toolbox', output='screen', parameters=[
                     str(bringup/'config/scout_mini/slam_toolbox_mapping.yaml'), slam]),
        ]),
        TimerAction(period=12., actions=[
            include(nav2/'launch/navigation_launch.py', {
                'use_sim_time': value('use_sim_time'), 'params_file': value('navigation_params'),
                'autostart': value('navigation_autostart'), 'use_composition': 'False',
            }, IfCondition(LaunchConfiguration('navigation'))),
        ]),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('venom_bringup'))/'config/scout_mini/bievr'
    return LaunchDescription([
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('navigation_autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mount_config', default_value=str(config/'mount.yaml')),
        DeclareLaunchArgument('navigation_params', default_value=str(config/'nav2.yaml')),
        DeclareLaunchArgument('slam_mode', default_value='mapping', choices=['mapping', 'localization']),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('initial_pose', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
