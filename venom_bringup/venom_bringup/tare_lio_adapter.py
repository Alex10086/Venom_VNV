"""Adapt gravity-aligned LIO and the tilted MID360 to the CMU ground stack."""
from collections import deque
from copy import deepcopy
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException


def quat(q):
    return np.array([q.x, q.y, q.z, q.w], dtype=float)


def multiply(a, b):
    av, aw, bv, bw = a[:3], a[3], b[:3], b[3]
    return np.r_[aw*bv + bw*av + np.cross(av, bv), aw*bw - av.dot(bv)]


def rotation(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def assign(q, values):
    q.x, q.y, q.z, q.w = map(float, values)


class LioAdapter(Node):
    def __init__(self):
        super().__init__('tare_lio_adapter')
        self.declare_parameter('odom_topic', '/bievr_lio/odom')
        self.declare_parameter('cloud_topic', '/bievr_lio/points/registered')
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('max_age', 0.6)
        self.declare_parameter('vehicle_length', 0.60)
        self.declare_parameter('vehicle_width', 0.57)
        self.declare_parameter('self_padding', 0.03)
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.odometry = deque(maxlen=100)
        self.pending = deque(maxlen=5)
        self.last_warning = 0.0
        self.state_pub = self.create_publisher(Odometry, '/tare/state_estimation', 5)
        self.scan_state_pub = self.create_publisher(Odometry, '/tare/state_estimation_at_scan', 5)
        self.scan_pub = self.create_publisher(PointCloud2, '/tare/registered_scan', 5)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_timer(0.02, self.flush)

    def fresh(self, stamp):
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9
        return -0.1 <= age <= self.get_parameter('max_age').value

    def state(self, msg):
        if msg.child_frame_id != 'body':
            raise ValueError('LIO child frame must be body, got ' + msg.child_frame_id)
        stamp = Time.from_msg(msg.header.stamp)
        world = self.buffer.lookup_transform('map', msg.header.frame_id, stamp).transform
        chassis = self.buffer.lookup_transform('body', 'base_link', Time()).transform
        q_world = quat(world.rotation)
        q_body = multiply(q_world, quat(msg.pose.pose.orientation))
        q_base = multiply(q_body, quat(chassis.rotation))
        p = msg.pose.pose.position
        position = rotation(q_world) @ [p.x, p.y, p.z]
        position += [world.translation.x, world.translation.y, world.translation.z]
        out = deepcopy(msg)
        out.header.frame_id = 'map'
        out.child_frame_id = 'tare_sensor'
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, position)
        assign(out.pose.pose.orientation, q_base)
        # Angular/linear vectors are expressed in the leveled sensor axes.
        rotate = rotation(quat(chassis.rotation)).T
        for vector in (out.twist.twist.linear, out.twist.twist.angular):
            vector.x, vector.y, vector.z = map(float, rotate @ [vector.x, vector.y, vector.z])
        return out, chassis

    def on_odom(self, msg):
        if self.fresh(msg.header.stamp):
            self.odometry.append(msg)

    def on_cloud(self, msg):
        if self.fresh(msg.header.stamp):
            self.pending.append(msg)
        self.flush()

    def flush(self):
        while self.pending and self.odometry:
            cloud = self.pending[0]
            if not self.fresh(cloud.header.stamp):
                self.pending.popleft()
                continue
            ns = Time.from_msg(cloud.header.stamp).nanoseconds
            odom = min(self.odometry, key=lambda m: abs(Time.from_msg(m.header.stamp).nanoseconds-ns))
            if abs(Time.from_msg(odom.header.stamp).nanoseconds-ns) > 100000000:
                return
            try:
                state, chassis = self.state(odom)
                transform = self.buffer.lookup_transform('map', cloud.header.frame_id,
                                                         Time.from_msg(cloud.header.stamp)).transform
                self.publish(cloud, state, chassis, transform)
            except (TransformException, ValueError, KeyError) as error:
                if time.monotonic() - self.last_warning > 5.0:
                    self.get_logger().warning(str(error))
                    self.last_warning = time.monotonic()
                return
            self.pending.popleft()

    def publish(self, cloud, state, chassis, transform):
        fields = {f.name: f for f in cloud.fields}
        names = [name for name in ('x', 'y', 'z', 'intensity') if name in fields]
        if not all(name in names and fields[name].datatype == PointField.FLOAT32 for name in ('x', 'y', 'z')):
            raise ValueError('Registered scan needs FLOAT32 x/y/z')
        endian = '>' if cloud.is_bigendian else '<'
        dtype = np.dtype({'names': names, 'formats': [endian+'f4']*len(names),
                          'offsets': [fields[n].offset for n in names], 'itemsize': cloud.point_step})
        points = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                            strides=(cloud.row_step, cloud.point_step)).reshape(-1)
        xyz = np.column_stack([points[n] for n in ('x', 'y', 'z')])
        xyz = xyz @ rotation(quat(transform.rotation)).T
        xyz += [transform.translation.x, transform.translation.y, transform.translation.z]
        p = state.pose.pose.position
        sensor_xyz = np.array([p.x, p.y, p.z])
        local = (xyz-sensor_xyz) @ rotation(quat(state.pose.pose.orientation))
        # body -> base_link translation in chassis axes is the sensor-to-base vector.
        offset = rotation(quat(chassis.rotation)).T @ [chassis.translation.x, chassis.translation.y, chassis.translation.z]
        local_base = local - offset
        half_length = self.get_parameter('vehicle_length').value/2 + self.get_parameter('self_padding').value
        half_width = self.get_parameter('vehicle_width').value/2 + self.get_parameter('self_padding').value
        own_body = ((np.abs(local_base[:, 0]) < half_length) & (np.abs(local_base[:, 1]) < half_width)
                    & (local_base[:, 2] > -.12) & (local_base[:, 2] < .65))
        valid = (np.isfinite(xyz).all(axis=1) & ~own_body
                 & (np.linalg.norm(xyz[:, :2]-sensor_xyz[:2], axis=1) < self.get_parameter('max_range').value))
        output = np.zeros((int(valid.sum()), 4), dtype='<f4')
        output[:, :3] = xyz[valid]
        if 'intensity' in names:
            output[:, 3] = np.nan_to_num(points['intensity'][valid])
        scan = PointCloud2()
        scan.header.stamp = cloud.header.stamp
        scan.header.frame_id = 'map'
        scan.height, scan.width = 1, len(output)
        scan.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                       for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        scan.point_step, scan.row_step, scan.is_dense = 16, 16*len(output), True
        scan.data = output.tobytes()
        state.header.stamp = cloud.header.stamp
        self.state_pub.publish(state)
        self.scan_state_pub.publish(state)
        self.scan_pub.publish(scan)
        tf = TransformStamped()
        tf.header = state.header
        tf.child_frame_id = 'tare_sensor'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
        tf.transform.rotation = state.pose.pose.orientation
        self.broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = LioAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
