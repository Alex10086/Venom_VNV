"""Filter registered points in chassis coordinates before 2D scan projection."""
from collections import deque
import json
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class ScanFilter(Node):
    def __init__(self):
        super().__init__('bievr_scan_filter')
        for key, value in [('vehicle_length', .60), ('vehicle_width', .57),
                           ('self_padding', .03), ('min_height', .06),
                           ('max_height', .65), ('max_age', .6)]:
            self.declare_parameter(key, value)
        self.buffer = Buffer(cache_time=Duration(seconds=10.))
        self.listener = TransformListener(self.buffer, self)
        self.pending = deque(maxlen=4)
        self.warning_time = 0.
        self.status_time = 0.
        self.pub = self.create_publisher(PointCloud2, '/bievr_lio/points/navigation', 2)
        self.status = self.create_publisher(String, '/bievr_lio/scan_filter_status', 2)
        self.create_subscription(PointCloud2, '/bievr_lio/points/registered',
                                 self.receive, qos_profile_sensor_data)
        self.create_timer(.02, self.flush)

    def receive(self, msg):
        self.pending.append(msg)
        self.flush()

    def flush(self):
        while self.pending:
            msg = self.pending[0]
            age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds*1e-9
            if age > self.get_parameter('max_age').value or age < -.15:
                self.pending.popleft()
                continue
            try:
                tf = self.buffer.lookup_transform('base_link', msg.header.frame_id,
                                                   Time.from_msg(msg.header.stamp)).transform
                self.publish(msg, tf)
            except (TransformException, ValueError) as exc:
                if time.monotonic()-self.warning_time > 5.:
                    self.get_logger().warning(str(exc))
                    self.warning_time = time.monotonic()
                return
            self.pending.popleft()

    def publish(self, msg, tf):
        fields = {f.name: f for f in msg.fields}
        if not all(name in fields and fields[name].datatype == PointField.FLOAT32
                   for name in ('x', 'y', 'z')):
            raise ValueError('Registered cloud must contain FLOAT32 x/y/z')
        endian = '>' if msg.is_bigendian else '<'
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': [endian+'f4']*3,
                          'offsets': [fields[n].offset for n in ('x', 'y', 'z')],
                          'itemsize': msg.point_step})
        data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                          strides=(msg.row_step, msg.point_step)).reshape(-1)
        xyz = np.column_stack([data[n] for n in ('x', 'y', 'z')])
        q = tf.rotation
        xyz = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply(xyz)
        xyz += [tf.translation.x, tf.translation.y, tf.translation.z]
        param = lambda name: self.get_parameter(name).value
        own_body = ((np.abs(xyz[:, 0]) <= param('vehicle_length')/2+param('self_padding'))
                    & (np.abs(xyz[:, 1]) <= param('vehicle_width')/2+param('self_padding')))
        finite = np.isfinite(xyz).all(axis=1)
        height = (xyz[:, 2] >= param('min_height')) & (xyz[:, 2] <= param('max_height'))
        output = np.asarray(xyz[finite & height & ~own_body], dtype='<f4')
        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = 'base_link'
        cloud.height, cloud.width = 1, len(output)
        cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.point_step, cloud.row_step, cloud.is_dense = 12, 12*len(output), True
        cloud.data = output.tobytes()
        self.pub.publish(cloud)
        if time.monotonic()-self.status_time > 1.:
            self.status.publish(String(data=json.dumps({
                'input_points': len(xyz), 'output_points': len(output),
                'below_ground_filter': int((finite & (xyz[:, 2] < param('min_height'))).sum()),
                'self_points': int((finite & own_body).sum()),
                'min_height': param('min_height'), 'max_height': param('max_height')})))
            self.status_time = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
