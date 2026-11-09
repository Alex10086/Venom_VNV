"""BIEVR odometry only; no chassis driver or navigation is started."""
from datetime import datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from venom_bringup.scout_mount import load_mount, mount_nodes


def launch_setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    bringup = Path(get_package_share_directory('venom_bringup'))
    runtime = Path(value('output_dir')).expanduser().resolve()
    runtime.mkdir(parents=True, exist_ok=True)
    params = yaml.safe_load(Path(value('params_file')).read_text())
    params['debug']['trajectory_path'] = str(runtime / 'trajectory.tum')
    effective = runtime / 'bievr_params.yaml'
    effective.write_text(yaml.safe_dump(params, sort_keys=False))
    # Store calibration alongside each run for reproducible comparisons.
    sensor = runtime / 'sensor.yaml'
    sensor.write_text(Path(value('sensor_config')).read_text())
    rviz = yaml.safe_load((bringup / 'rviz_cfg/bievr_mapping.rviz').read_text())
    rviz['Visualization Manager']['Global Options']['Fixed Frame'] = value('rviz_fixed_frame')
    rviz_file = runtime / 'mapping.rviz'
    rviz_file.write_text(yaml.safe_dump(rviz, sort_keys=False))
    use_sim_time = value('use_sim_time').lower() == 'true'
    mount = load_mount(value('mount_config'))
    (runtime / 'mount.yaml').write_text(yaml.safe_dump(mount, sort_keys=False))
    return mount_nodes(mount, use_sim_time,
                       IfCondition(LaunchConfiguration('publish_description'))) + [
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen',
             condition=IfCondition(LaunchConfiguration('start_driver')),
             parameters=[{
                 'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                 'publish_freq': 10.0, 'output_data_type': 0, 'frame_id': 'body',
                 'lvx_file_path': '/home/livox/livox_test.lvx',
                 'user_config_path': str(bringup / 'config/scout_mini/MID360_config.json'),
                 'cmdline_input_bd_code': '47MDLAS0020103',
             }]),
        Node(package='bievr_lio_ros2', executable='process_topics',
             name='bievr_lio', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['--sensor_config_file', str(sensor), '--params_file', str(effective)]),
        Node(package='venom_bringup', executable='bievr_map_recorder',
             name='bievr_map_recorder', output='screen',
             condition=IfCondition(LaunchConfiguration('record_map')),
             parameters=[{'use_sim_time': use_sim_time,
                          'save_path': str(runtime / 'maps/bievr.pcd')}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(rviz_file)],
             parameters=[{'use_sim_time': use_sim_time}],
             additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('venom_bringup'))
    config = share / 'config/scout_mini/bievr'
    # A new folder on every launch avoids replacing an earlier comparison.
    output = Path.home() / 'venom_ws/runtime/bievr' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    return LaunchDescription([
        DeclareLaunchArgument('start_driver', default_value='false'),
        DeclareLaunchArgument('publish_description', default_value='true'),
        DeclareLaunchArgument('record_map', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_fixed_frame', default_value='odom'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=str(output)),
        DeclareLaunchArgument('sensor_config', default_value=str(config / 'mid360.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(config / 'params.yaml')),
        DeclareLaunchArgument('mount_config', default_value=str(config / 'mount.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
