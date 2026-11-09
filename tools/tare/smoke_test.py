#!/usr/bin/env python3
"""Hardware-free TARE integration: actual BIEVR fed synthetic tilted MID360 data."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
os.environ['ROS_DOMAIN_ID']='178'
os.environ['ROS_LOCALHOST_ONLY']='1'
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from livox_ros_driver2.msg import CustomMsg, CustomPoint


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--lio',choices=['bievr','fastlio'],default='bievr')
    parser.add_argument('--domain',type=int,default=178)
    options=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']=str(options.domain)
    root=Path.home()/'venom_ws/runtime/tare_smoke'/(options.lio+'_'+time.strftime('%Y%m%d_%H%M%S'))
    root.mkdir(parents=True)
    synthetic_mount = root/'synthetic_mount.yaml'
    synthetic_mount.write_text(yaml.safe_dump({'transforms': [
        {'parent_frame': 'body', 'child_frame': 'base_link',
         'translation': [0., 0., -.34], 'rotation': [0., -.9352, 0.]},
        {'parent_frame': 'base_link', 'child_frame': 'scan_link',
         'translation': [.27360406073952004, 0., .20184354819226957], 'rotation': [0., 0., 0.]},
    ]}))
    print(root,flush=True)
    log=(root/'launch.log').open('w')
    alignment=root/'synthetic_fastlio_alignment.json'
    alignment.write_text(json.dumps({'translation':[0.,0.,0.],
        'quaternion':[0.,math.sin(.9352/2),0.,math.cos(.9352/2)]}))
    proc=subprocess.Popen(['ros2','launch','venom_bringup','scout_mini_tare.launch.py',
        'start_driver:=false','start_base:=false','rviz:=false',
        'cmd_vel_topic:=/tare/test_cmd_vel',f'lio:={options.lio}',
        f'fastlio_alignment_file:={alignment}',f'mount_config:={synthetic_mount}'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(); node=Node('tare_synthetic_probe')
    counts={}; latest={}; commands=[]; statuses=[]; paths=[]; waypoints=[]
    def observe(key):
        def cb(msg):
            counts[key]=counts.get(key,0)+1; latest[key]=msg
            if key=='cmd': commands.append((time.monotonic()-begin,msg.linear.x,msg.angular.z))
            if key=='status':
                statuses.append((time.monotonic()-begin,json.loads(msg.data)))
                (root/'progress.json').write_text(json.dumps({'counts':counts,'status':statuses[-1]},indent=2))
            if key=='path': paths.append(len(msg.poses))
            if key=='waypoint': waypoints.append([msg.point.x,msg.point.y,msg.point.z])
        return cb
    subs=[]
    for typ,topic,key in [(PointCloud2,'/tare/registered_scan','scan'),(PointCloud2,'/tare/terrain_map','terrain'),
        (PointCloud2,'/tare/terrain_map_ext','terrain_ext'),(Odometry,'/tare/state_estimation','odom'),
        (PointStamped,'/tare/way_point','waypoint'),(NavPath,'/tare/path','path'),
        (NavPath,'/tare/local_path','tour'),(Twist,'/tare/test_cmd_vel','cmd'),(String,'/tare/status','status')]:
        subs.append(node.create_subscription(typ,topic,observe(key),5))
    lidar=node.create_publisher(CustomMsg,'/livox/lidar',20)
    imu_pub=node.create_publisher(Imu,'/livox/imu',20)
    client=node.create_client(SetBool,'/tare/set_enabled')
    angle=.9352; c,s=math.cos(angle),math.sin(angle)
    rotation=np.array([[c,0,s],[0,1,0],[-s,0,c]])
    accel=rotation.T@np.array([0,0,9.80665])
    rng=np.random.default_rng(42)
    # Ground + room surfaces beyond the initially visited region; test all
    # production terrain/collision settings, not a fake waypoint publisher.
    room=[]
    for axis,limit in ((1,-2.),(1,2.),(2,-.34*c),(2,2.8)):
        points=rng.uniform([-10,-2,-.34*c],[10,2,2.8],size=(4000,3))
        points[:,axis]=limit; room.append(points[np.linalg.norm(points[:, :2],axis=1)<6.0])
    points_lidar=np.concatenate(room)@rotation-np.array([-.011,-.02329,.04412])
    points=[CustomPoint(x=float(p[0]),y=float(p[1]),z=float(p[2]),
        offset_time=int(i*99000000/(len(points_lidar)-1)),reflectivity=80,tag=0x10,line=i%4)
        for i,p in enumerate(points_lidar)]
    cloud=CustomMsg(points=points,point_num=len(points))
    imu=Imu(); imu.header.frame_id='body'
    imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=map(float,accel)
    requests={}; actions=set(); conflict=None; result={}
    def enable(key,value):
        request=SetBool.Request(); request.data=value
        requests[key]=client.call_async(request)
    begin=time.monotonic(); next_cloud=begin+1.
    try:
        while time.monotonic()-begin<68.:
            t=time.monotonic()-begin
            if proc.poll() is not None: raise RuntimeError('Launch exited: '+str(proc.returncode))
            now_ns=node.get_clock().now().nanoseconds
            if not 48.<t<51.:
                imu.header.stamp=Time(nanoseconds=now_ns).to_msg(); imu_pub.publish(imu)
                if time.monotonic()>=next_cloud:
                    cloud.timebase=now_ns-100000000
                    cloud.header.stamp=Time(nanoseconds=cloud.timebase).to_msg(); cloud.header.frame_id='body'
                    lidar.publish(cloud); next_cloud=time.monotonic()+.1
            else: next_cloud=time.monotonic()+.1
            for at,key,value in ((12.,'start',True),(32.,'pause',False),(35.,'resume',True),(54.,'recover',True),(64.,'conflict_refused',True)):
                if t>at and key not in actions and client.service_is_ready():
                    enable(key,value); actions.add(key)
            if t>60. and conflict is None:
                conflict=node.create_publisher(Twist,'/tare/test_cmd_vel',1)
            rclpy.spin_once(node,timeout_sec=.003)
        replies={k:{'success':v.result().success,'message':v.result().message} for k,v in requests.items() if v.done()}
        def max_command(start,end):
            return max([max(abs(x),abs(y)) for t,x,y in commands if start<t<end] or [0.])
        q=latest['odom'].pose.pose.orientation
        pitch=math.degrees(math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))))
        checks={'data_chain':all(counts.get(k,0)>10 for k in ('scan','terrain','terrain_ext','odom','path')),
            'real_tare_waypoints':counts.get('waypoint',0)>5 and counts.get('tour',0)>3,
            'collision_free_path':max(paths or [0])>5,
            'tilt_corrected':abs(pitch)<2.,
            'moving_command':max_command(15,31)>.01,
            'bounded_speed':all(abs(x)<=.300001 and abs(y)<=.800001 for t,x,y in commands),
            'initial_paused':max_command(0,11)==0.,
            'pause_stops':max_command(33,34.8)==0.,
            'data_timeout_stops':max_command(49.5,53.8)==0.,
            'no_auto_resume':any(51.5<t<54 and not s['enabled'] for t,s in statuses),
            'conflict_disarms':any(t>61 and not s['enabled'] and s['conflict_publishers'] for t,s in statuses),
            'conflict_enable_refused':not replies.get('conflict_refused',{}).get('success',True),
            'enable_resume_success':all(replies.get(k,{}).get('success',False) for k in ('start','pause','resume','recover'))}
        result={'lio':options.lio,'checks':checks,'passed':all(checks.values()),'counts':counts,'service_replies':replies,
            'max_path_points':max(paths or [0]),'leveled_pitch_degrees':pitch,
            'motion_peak':max_command(15,31),'sample_waypoints':waypoints[:8]}
    except Exception as error:
        result={'passed':False,'error':str(error),'counts':counts}
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try: proc.wait(timeout=15)
        except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGTERM); proc.wait(timeout=10)
        (root/'commands.json').write_text(json.dumps(commands))
        (root/'statuses.json').write_text(json.dumps(statuses))
        (root/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)
        node.destroy_node(); rclpy.shutdown(); log.close()
    return 0 if result.get('passed') else 1


if __name__=='__main__': raise SystemExit(main())
