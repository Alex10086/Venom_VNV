#!/usr/bin/env python3
"""Hardware-free integration check: synthetic tilted MID360 in a static room.
Run after sourcing ROS Humble and venom_ws. Always uses a private ROS domain.
This checks interfaces/initialization, not real-world accuracy or performance.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, PointCloud2, LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--navigation', action='store_true', help='Also check Nav2 lifecycle; chassis stays disabled')
    options = parser.parse_args()
    root = Path.home() / 'venom_ws/runtime/bievr_smoke' / time.strftime('%Y%m%d_%H%M%S')
    root.mkdir(parents=True)
    # Fixture geometry is explicit and independent of the current hardware mount.
    synthetic_mount = root/'synthetic_mount.yaml'
    synthetic_mount.write_text(yaml.safe_dump({'transforms': [
        {'parent_frame': 'body', 'child_frame': 'base_link',
         'translation': [0., 0., -.34], 'rotation': [0., -.9352, 0.]},
        {'parent_frame': 'base_link', 'child_frame': 'scan_link',
         'translation': [.27360406073952004, 0., .20184354819226957], 'rotation': [0., 0., 0.]},
    ]}))
    log = (root / 'launch.log').open('w')
    proc = subprocess.Popen([
        'ros2', 'launch', 'venom_bringup', 'scout_mini_bievr_mapping.launch.py',
        'start_driver:=false', 'start_base:=false',
        f'navigation:={str(options.navigation).lower()}',
        'rviz:=false', f'output_dir:={root}',
        f'mount_config:={synthetic_mount}',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('bievr_synthetic_probe')
    counts = {'odom': 0, 'cloud': 0, 'scan': 0, 'map': 0, 'grid': 0}
    latest = {}

    def observe(key):
        def callback(msg):
            counts[key] += 1
            latest[key] = msg
        return callback

    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(Odometry, '/bievr_lio/odom', observe('odom'), transient),
        node.create_subscription(PointCloud2, '/bievr_lio/points/registered', observe('cloud'), transient),
        node.create_subscription(PointCloud2, '/bievr_lio/map', observe('map'), transient),
        node.create_subscription(LaserScan, '/scan', observe('scan'), qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid, '/map', observe('grid'), transient),
    ]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    lidar = node.create_publisher(CustomMsg, '/livox/lidar', qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, '/livox/imu', qos_profile_sensor_data)
    angle = 0.9352
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    accel = rotation.T @ np.array([0, 0, 9.80665])
    rng = np.random.default_rng(42)
    room = []
    for axis, limit in ((0, -4.), (0, 4.), (1, -4.), (1, 4.), (2, -0.34*c), (2, 2.8)):
        points = rng.uniform([-4, -4, -0.34*c], [4, 4, 2.8], size=(1200, 3))
        points[:, axis] = limit
        room.append(points)
    # BIEVR applies T_IMU_LIDAR, then the initialized IMU -> odom rotation.
    points_lidar = np.concatenate(room) @ rotation - np.array([-0.011, -0.02329, 0.04412])
    points = [CustomPoint(x=float(p[0]), y=float(p[1]), z=float(p[2]),
                          offset_time=int(i * 99000000 / (len(points_lidar)-1)),
                          reflectivity=80, tag=0x10, line=i % 4)
              for i, p in enumerate(points_lidar)]
    cloud = CustomMsg(points=points, point_num=len(points))
    imu = Imu()
    imu.header.frame_id = 'body'
    imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, accel)
    result = {}
    try:
        start = time.monotonic()
        next_cloud = start + 1.0
        while time.monotonic() - start < 30.0:
            if proc.poll() is not None:
                raise RuntimeError(f'Launch exited with {proc.returncode}; inspect {root / "launch.log"}')
            now_ns = node.get_clock().now().nanoseconds
            imu.header.stamp = Time(nanoseconds=now_ns).to_msg()
            imu_pub.publish(imu)
            if time.monotonic() >= next_cloud:
                cloud.timebase = now_ns - 100000000
                cloud.header.stamp = Time(nanoseconds=cloud.timebase).to_msg()
                cloud.header.frame_id = 'body'
                lidar.publish(cloud)
                next_cloud += .1
            rclpy.spin_once(node, timeout_sec=.004)
        odom = latest['odom']
        pose = odom.pose.pose
        drift = math.sqrt(pose.position.x**2 + pose.position.y**2 + pose.position.z**2)
        transform = buffer.lookup_transform('odom', 'base_link', Time())
        q = transform.transform.rotation
        base_pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        result = {'counts': counts, 'static_drift_m': drift, 'base_pitch_degrees': math.degrees(base_pitch),
                  'odom_frame': odom.header.frame_id, 'body_frame': odom.child_frame_id,
                  'cloud_frame': latest['cloud'].header.frame_id}
        if options.navigation:
            states = {}
            for name in ('controller_server', 'smoother_server', 'planner_server',
                         'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother'):
                client = node.create_client(GetState, f'/{name}/get_state')
                assert client.wait_for_service(timeout_sec=2), f'{name} lifecycle service unavailable'
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2)
                assert future.done(), f'{name} lifecycle request timed out'
                states[name] = future.result().current_state.label
                node.destroy_client(client)
            result['nav2_lifecycle'] = states
            assert all(state == 'active' for state in states.values()), states
        assert all(counts[key] >= 10 for key in ('odom', 'cloud', 'scan')), result
        assert counts['map'] >= 1 and counts['grid'] >= 1, result
        assert drift < .1 and abs(base_pitch) < .05, result
        assert (odom.header.frame_id, odom.child_frame_id, latest['cloud'].header.frame_id) == ('odom', 'body', 'odom'), result
    finally:
        proc.send_signal(signal.SIGINT) if proc.poll() is None else None
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    pcd = root / 'maps/bievr.pcd'
    assert pcd.exists() and pcd.stat().st_size > 200, f'Missing PCD: {pcd}'
    assert (root / 'trajectory.tum').stat().st_size > 100
    result.update(pcd_bytes=pcd.stat().st_size, output_dir=str(root), passed=True)
    (root / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
