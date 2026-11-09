"""TARE Humble exploration; owns control separately from Nav2/SLAM Toolbox."""
import json
import math
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from venom_bringup.scout_mount import load_mount, mount_nodes


def launch_setup(context):
    val = lambda name: LaunchConfiguration(name).perform(context)
    flag = lambda name: val(name).lower() == 'true'
    bringup = Path(get_package_share_directory('venom_bringup'))
    description = Path(get_package_share_directory('venom_robot_description'))
    config = bringup/'config/scout_mini/tare'
    lio = val('lio')
    if lio not in ('bievr', 'fastlio'):
        raise ValueError('lio must be bievr or fastlio')
    sim = flag('use_sim_time')
    actions = []

    def include(path, arguments=None, condition=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                launch_arguments=(arguments or {}).items(), **({'condition': condition} if condition else {}))

    def critical(node):
        # Any essential process dying tears down the chain, including chassis.
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node,
                       on_exit=[EmitEvent(event=Shutdown(reason='TARE essential process exited'))])))
        actions.append(node)

    # The native CMU stack uses map numerically. This exploration-only map is the
    # continuous odom origin; do not run SLAM Toolbox's map->odom alongside it.
    critical(Node(package='tf2_ros', executable='static_transform_publisher',
        name='tare_map_origin', arguments=['--frame-id','map','--child-frame-id','odom'],
        parameters=[{'use_sim_time': sim}]))
    if lio == 'bievr':
        mount_config = load_mount(val('mount_config'))
        for node in mount_nodes(mount_config, sim):
            critical(node)
    else:
        mount_config = yaml.safe_load((description/'config/scout_mini.yaml').read_text())
        actions.append(include(description/'launch/scout_mini_description.launch.py'))
    if flag('start_lio'):
        if lio == 'bievr':
            actions.append(include(bringup/'launch/scout_mini_bievr.launch.py', {
                'start_driver': val('start_driver'), 'publish_description':'false',
                'rviz':'false', 'use_sim_time': val('use_sim_time'),
                'mount_config': val('mount_config'),
            }))
        else:
            fastlio = Path(get_package_share_directory('fast_lio'))
            alignment = json.loads(Path(val('fastlio_alignment_file')).read_text())
            x,y,z=alignment['translation']; qx,qy,qz,qw=alignment['quaternion']
            critical(Node(package='tf2_ros', executable='static_transform_publisher',
                name='tare_fastlio_alignment', parameters=[{'use_sim_time':sim}],
                arguments=['--x',str(x),'--y',str(y),'--z',str(z),'--qx',str(qx),'--qy',str(qy),
                           '--qz',str(qz),'--qw',str(qw),'--frame-id','odom','--child-frame-id','lio_init']))
            critical(Node(package='fast_lio', executable='fastlio_mapping',name='fast_lio',output='screen',
                parameters=[str(fastlio/'config/mid360.yaml'),{
                    'use_sim_time':sim,'frame.odom_frame_id':'lio_init','frame.base_frame_id':'body',
                    'frame.cloud_registered_frame_id':'lio_init','frame.map_frame_id':'lio_init',
                    'mapping.extrinsic_est_en':False,'publish.odom_topic':'/lio_odom',
                    'pcd_save.pcd_save_en':False,
                }]))
            if flag('start_driver'):
                critical(Node(package='livox_ros_driver2',executable='livox_ros_driver2_node',
                    name='livox_lidar_publisher',output='screen',parameters=[{
                        'xfer_format':1,'multi_topic':0,'data_src':0,'publish_freq':10.0,
                        'output_data_type':0,'frame_id':'body','lvx_file_path':'/home/livox/livox_test.lvx',
                        'user_config_path':str(bringup/'config/scout_mini/MID360_config.json'),
                        'cmdline_input_bd_code':'47MDLAS0020103',
                    }]))
    elif flag('start_driver'):
        raise ValueError('start_driver requires start_lio=true')
    if flag('start_base'):
        scout=Path(get_package_share_directory('scout_base'))
        actions.append(include(scout/'launch/scout_mini_base.launch.py',{
            'port_name':'can0','is_scout_mini':'true','is_omni_wheel':'false',
            'odom_frame':'scout_odom','odom_topic_name':'scout_odom','base_frame':'scout_base_link',
        }))

    calibration=mount_config['transforms']
    mount=next(t for t in calibration if t['parent_frame']=='body' and t['child_frame']=='base_link')
    roll,pitch,yaw=mount['rotation']
    cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
    r=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    offset=-r.T@np.array(mount['translation'])
    stack=yaml.safe_load((config/'ground_stack.yaml').read_text())
    for name in ('localPlanner','pathFollower'):
        stack[name]['ros__parameters'].update(sensorOffsetX=float(offset[0]),sensorOffsetY=float(offset[1]))
    stack['terrainAnalysisExt']['ros__parameters']['terrainUnderVehicle']=-float(offset[2])
    planner=yaml.safe_load(Path(val('planner_params')).read_text())['tare_planner_node']['ros__parameters']
    planner['kViewPointHeightFromTerrain']=float(offset[2])
    # TARE viewpoints are at the IMU; cover the furthest chassis corner from it.
    chassis_radius=float(np.linalg.norm(np.abs(offset[:2])+np.array([.30,.285])))+.03
    planner['kViewPointCollisionMargin']=max(float(planner['kViewPointCollisionMargin']),chassis_radius)
    critical(Node(package='tf2_ros',executable='static_transform_publisher',name='tare_vehicle_frame',
        parameters=[{'use_sim_time':sim}],arguments=['--x',str(-offset[0]),'--y',str(-offset[1]),
        '--frame-id','tare_sensor','--child-frame-id','vehicle']))
    critical(Node(package='venom_bringup',executable='tare_lio_adapter',output='screen',parameters=[{
        'use_sim_time':sim,
        'odom_topic':'/bievr_lio/odom' if lio=='bievr' else '/lio_odom',
        'cloud_topic':'/bievr_lio/points/registered' if lio=='bievr' else '/cloud_registered',
        'vehicle_length':.60, 'vehicle_width':.57, 'self_padding':.03,
    }]))
    topics=('state_estimation','registered_scan','terrain_map','terrain_map_ext','joy','way_point','speed',
            'navigation_boundary','added_obstacles','check_obstacle','stop','path','free_paths','map_clearing')
    remaps=[('/'+t,'/tare/'+t) for t in topics]+[('/cmd_vel','/tare/cmd_vel_raw')]
    for package,executable in (('terrain_analysis','terrainAnalysis'),('terrain_analysis_ext','terrainAnalysisExt'),
                               ('local_planner','localPlanner'),('local_planner','pathFollower')):
        params=stack[executable]['ros__parameters']
        if executable=='localPlanner':
            params['pathFolder']=str(Path(get_package_share_directory('local_planner'))/'paths')
        critical(Node(package=package,executable=executable,name=executable,namespace='tare',
            output='screen',parameters=[params,{'use_sim_time':sim}],remappings=remaps))
    critical(Node(package='tare_planner',executable='tare_planner_node',name='tare_planner_node',namespace='tare',
        output='screen',parameters=[planner,{'use_sim_time':sim}]))
    critical(Node(package='venom_bringup',executable='tare_command_gate',output='screen',parameters=[{
        'use_sim_time':sim,'output_topic':val('cmd_vel_topic'),'max_speed':.3,'max_yaw_rate':.8,
    }]))
    actions.append(Node(package='rviz2',executable='rviz2',name='tare_rviz',output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d',str(bringup/'rviz_cfg/tare_exploration.rviz')],
        parameters=[{'use_sim_time':sim}],additional_env={'LIBGL_ALWAYS_SOFTWARE':'1'}))
    return actions


def generate_launch_description():
    config=Path(get_package_share_directory('venom_bringup'))/'config/scout_mini/tare'
    return LaunchDescription([
        DeclareLaunchArgument('lio',default_value='bievr',choices=['bievr','fastlio']),
        DeclareLaunchArgument('start_lio',default_value='true'),
        DeclareLaunchArgument('start_driver',default_value='false'),
        DeclareLaunchArgument('start_base',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic',default_value='/cmd_vel'),
        DeclareLaunchArgument('fastlio_alignment_file',default_value=str(config/'fastlio_alignment.json')),
        DeclareLaunchArgument('planner_params',default_value=str(config/'planner.yaml')),
        DeclareLaunchArgument('mount_config',default_value=str(config.parent/'bievr/mount.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
