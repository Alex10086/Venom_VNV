"""Bounded voxel snapshot of registered BIEVR scans, with atomic PCD saves."""
from collections import OrderedDict
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class MapRecorder(Node):
    def __init__(self):
        super().__init__('bievr_map_recorder')
        for key, default in (
            ('save_path', str(Path.home() / 'venom_ws/runtime/bievr/maps/bievr.pcd')),
            ('voxel_size', 0.1), ('max_points', 300000),
            ('sample_period', 1.0), ('publish_period', 5.0), ('save_period', 60.0),
        ):
            self.declare_parameter(key, default)
        self.path = Path(self.get_parameter('save_path').value).expanduser()
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.sample_period = float(self.get_parameter('sample_period').value)
        publish_period = float(self.get_parameter('publish_period').value)
        save_period = float(self.get_parameter('save_period').value)
        if min(self.voxel_size, self.max_points, self.sample_period, publish_period, save_period) <= 0:
            raise ValueError('Map recorder resolutions, limits and periods must be positive')
        self.voxels = OrderedDict()
        self.header = None
        self.last_sample = float('-inf')
        self.dirty = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PointCloud2, '/bievr_lio/map', qos)
        self.subscription = self.create_subscription(
            PointCloud2, '/bievr_lio/points/registered', self.add_scan, qos_profile_sensor_data)
        self.create_timer(publish_period, self.publish_map)
        self.create_timer(save_period, self.save_map)
        self.get_logger().info(f'PCD snapshot: {self.path}; voxel={self.voxel_size}m, cap={self.max_points}')

    def add_scan(self, msg):
        now = time.monotonic()
        if now - self.last_sample < self.sample_period:
            return
        # Drop mixed coordinate frames instead of silently corrupting the map.
        if self.header is not None and msg.header.frame_id != self.header.frame_id:
            self.get_logger().error('Registered cloud frame changed; ignoring cloud')
            return
        rows = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not rows:
            return
        points = np.array([tuple(row) for row in rows], dtype=np.float32)
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return
        keys = np.floor(points / self.voxel_size).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        for index in indices:
            key = tuple(keys[index])
            self.voxels[key] = tuple(points[index])
            self.voxels.move_to_end(key)
        while len(self.voxels) > self.max_points:
            self.voxels.popitem(last=False)
        self.header = msg.header
        self.last_sample = now
        self.dirty = True

    def publish_map(self):
        if self.header is not None:
            self.publisher.publish(point_cloud2.create_cloud_xyz32(self.header, list(self.voxels.values())))

    def save_map(self):
        if not self.dirty:
            return
        count = len(self.voxels)
        header = (
            '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
            'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            f'WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n'
        )
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with temporary.open('wb') as stream:
                stream.write(header.encode('ascii'))
                stream.write(np.asarray(list(self.voxels.values()), dtype='<f4').tobytes())
            temporary.replace(self.path)
            self.dirty = False
            if rclpy.ok():
                self.get_logger().info(f'Saved {count} points to {self.path}')
        except OSError as exc:
            self.get_logger().error(f'Cannot save PCD: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = MapRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_map()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
