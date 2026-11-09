#!/usr/bin/env python3
"""Inspect live mapping and compute paths without commanding robot motion."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathToPose
from sensor_msgs.msg import LaserScan
from scout_msgs.msg import ScoutStatus
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState
from action_msgs.msg import GoalStatusArray
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    args = parser.parse_args()
    rclpy.init()
    node = Node('bievr_read_only_validation')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    received, counts, positions = {}, {}, []
    subscriptions = []
    def callback(key, msg):
        received[key] = msg
        counts[key] = counts.get(key, 0)+1
        if key == 'odom':
            p = msg.pose.pose.position
            positions.append([p.x, p.y, p.z])
    for key, topic, kind, qos in [
        ('global', '/global_costmap/costmap', OccupancyGrid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ('local', '/local_costmap/costmap', OccupancyGrid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ('map', '/map', OccupancyGrid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ('scan', '/scan', LaserScan, qos_profile_sensor_data),
        ('filter', '/bievr_lio/scan_filter_status', String, qos_profile_sensor_data),
        ('scout', '/scout_status', ScoutStatus, qos_profile_sensor_data),
        ('odom', '/bievr_lio/odom', Odometry, qos_profile_sensor_data),
        ('navigation_goals', '/navigate_to_pose/_action/status', GoalStatusArray,
         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]:
        subscriptions.append(node.create_subscription(kind, topic, lambda msg, k=key: callback(k, msg), qos))
    deadline = time.monotonic()+12
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    report = {'counts': counts, 'lifecycle': {}, 'paths': []}
    report['tare_nodes'] = [name for name, namespace in node.get_node_names_and_namespaces()
                            if 'tare' in name.lower()]
    report['active_navigation_goals'] = [item.status for item in received.get(
        'navigation_goals', GoalStatusArray()).status_list if item.status in (1, 2, 3)]
    for name in ['controller_server', 'planner_server', 'smoother_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']:
        client = node.create_client(GetState, '/'+name+'/get_state')
        if client.wait_for_service(timeout_sec=1):
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            if future.done() and future.result():
                report['lifecycle'][name] = future.result().current_state.label
    for key in ['map', 'global', 'local']:
        if key in received:
            m = received[key]
            values, number = np.unique(m.data, return_counts=True)
            report[key] = {'width': m.info.width, 'height': m.info.height, 'resolution': m.info.resolution,
                           'origin': [m.info.origin.position.x, m.info.origin.position.y],
                           'cells': dict(zip(map(str, values), map(int, number)))}
    if 'scan' in received:
        report['scan_finite'] = int(np.isfinite(received['scan'].ranges).sum())
    if 'filter' in received:
        report['filter'] = json.loads(received['filter'].data)
    if 'scout' in received:
        report['scout'] = message_to_ordereddict(received['scout'])
    if positions:
        report['odom_peak_to_peak_m'] = np.ptp(positions, axis=0).tolist()
    tf = buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
    x, y = tf.translation.x, tf.translation.y
    report['base_in_map'] = [x, y, tf.translation.z]
    q = tf.rotation
    report['base_rpy_degrees'] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True).tolist()
    m = received['global']
    grid = np.asarray(m.data).reshape(m.info.height, m.info.width)
    res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
    start = (int((y-oy)/res), int((x-ox)/res))
    report['robot_cell'] = list(start)
    queue, seen, choices = deque([start]), {start}, {}
    while queue and len(choices) < 2:
        row, col = queue.popleft()
        gx, gy = ox+(col+.5)*res, oy+(row+.5)*res
        distance = math.hypot(gx-x, gy-y)
        value = int(grid[row, col])
        if .7 < distance < 1.6 and value in (-1, 0):
            choices.setdefault('unknown' if value == -1 else 'known_free', (gx, gy))
        if distance > 2.5:
            continue
        for rr, cc in [(row-1,col), (row+1,col), (row,col-1), (row,col+1)]:
            if 0 <= rr < grid.shape[0] and 0 <= cc < grid.shape[1] and (rr,cc) not in seen and grid[rr,cc] < 99:
                seen.add((rr,cc))
                queue.append((rr,cc))
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    if planner.wait_for_server(timeout_sec=5):
        for label, (gx,gy) in choices.items():
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = 'map'
            goal.goal.header.stamp = node.get_clock().now().to_msg()
            goal.goal.pose.position.x, goal.goal.pose.position.y = gx, gy
            goal.goal.pose.orientation.w = 1.
            goal.planner_id = 'GridBased'
            goal.use_start = False
            future = planner.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future, timeout_sec=8)
            item = {'kind': label, 'goal': [gx,gy]}
            if future.done() and future.result() and future.result().accepted:
                result = future.result().get_result_async()
                rclpy.spin_until_future_complete(node, result, timeout_sec=8)
                if result.done() and result.result():
                    item.update(status=result.result().status, points=len(result.result().result.path.poses))
            report['paths'].append(item)
    report['passed'] = (len(report['lifecycle']) == 7 and all(v == 'active' for v in report['lifecycle'].values())
                        and report.get('scan_finite', 0) > 20 and len(report['paths']) == 2
                        and report.get('odom_peak_to_peak_m') is not None
                        and int(report.get('local', {}).get('cells', {}).get('100', 0)) > 0
                        and not report['tare_nodes'] and not report['active_navigation_goals']
                        and all(p.get('status') == 4 and p.get('points', 0) > 1 for p in report['paths']))
    Path(args.output).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
