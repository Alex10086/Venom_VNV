"""Single Twist output with explicit arming and freshness watchdogs."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from rclpy.time import Time


class CommandGate(Node):
    def __init__(self):
        super().__init__('tare_command_gate')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('max_yaw_rate', 0.8)
        self.enabled = False
        self.reason = 'paused: call /tare/set_enabled'
        self.seen = {}
        self.stamps = {}
        self.latest = None
        self.finished = False
        self.home = None
        self.position = None
        self.conflict = False
        self.pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, 1)
        self.start = self.create_publisher(Bool, '/tare/start_exploration', 1)
        self.status = self.create_publisher(String, '/tare/status', 1)
        self.create_service(SetBool, '/tare/set_enabled', self.set_enabled)
        self.create_subscription(TwistStamped, '/tare/cmd_vel_raw', self.command, 1)
        for kind, msg_type, topic in (
            ('odom', Odometry, '/tare/state_estimation'),
            ('scan', PointCloud2, '/tare/registered_scan'),
            ('terrain', PointCloud2, '/tare/terrain_map'),
            ('path', Path, '/tare/path'),
            ('waypoint', PointStamped, '/tare/way_point'),
        ):
            self.create_subscription(msg_type, topic,
                                     lambda msg, key=kind: self.observe(key, msg), 1)
        self.create_subscription(Bool, '/tare/exploration_finish', self.finish, 1)
        self.create_timer(.05, self.tick)
        self.create_timer(.5, self.check_graph)

    def observe(self, key, msg):
        if key == "odom":
            p = msg.pose.pose.position
            self.position = (p.x, p.y, p.z)
            if self.home is None or all(abs(v) < .01 for v in self.home):
                self.home = self.position
        if key == 'path' and len(msg.poses) < 2:
            self.seen.pop('path', None)
            return
        if key in ('scan', 'terrain') and msg.width*msg.height == 0:
            self.seen.pop(key, None)
            return
        if key == 'waypoint' and not all(math.isfinite(v) for v in (msg.point.x, msg.point.y, msg.point.z)):
            self.seen.pop(key, None)
            return
        self.seen[key] = time.monotonic()
        self.stamps[key] = Time.from_msg(msg.header.stamp).nanoseconds*1e-9

    def command(self, msg):
        self.latest = msg.twist
        self.observe('command', msg)

    def finish(self, msg):
        self.finished = msg.data
        # Upstream true means coverage complete; the return-home leg follows.
        # Continue following until the configured home radius is reached.

    def check_graph(self):
        publishers = self.get_publishers_info_by_topic(self.get_parameter('output_topic').value)
        others = [p.node_name for p in publishers
                  if p.node_name != self.get_name() or p.node_namespace != self.get_namespace()]
        if len(publishers) > 1 and not others:
            others = ['duplicate tare_command_gate']
        self.conflict = bool(others)
        if self.conflict and self.enabled:
            self.disarm('another velocity publisher: ' + ', '.join(others))
        self.status.publish(String(data=json.dumps({'enabled': self.enabled, 'reason': self.reason,
                                                   'conflict_publishers': others})))

    def stale(self, key, timeout):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds*1e-9
        age = ros_now-self.stamps.get(key, 0.)
        return now-self.seen.get(key, -1e9) > timeout or age > timeout or age < -.15

    def disarm(self, reason):
        self.enabled = False
        self.reason = reason
        self.latest = None
        if not self.conflict:
            self.pub.publish(Twist())
        self.get_logger().warning(reason)

    def set_enabled(self, request, response):
        self.check_graph()
        if not request.data:
            self.disarm('paused by service')
            response.success = True
        elif self.conflict:
            response.success = False
            self.reason = 'refused: another velocity publisher exists'
        elif (self.finished and self.home is not None and self.position is not None
              and math.dist(self.home, self.position) < .5):
            response.success = False
            self.reason = 'refused: exploration finished; restart exploration to reset map'
        else:
            missing = [key for key in ('odom', 'scan', 'terrain') if self.stale(key, .8)]
            response.success = not missing
            if missing:
                self.reason = 'refused: missing or stale ' + ', '.join(missing)
            else:
                self.enabled = True
                if self.home is None:
                    self.home = self.position
                self.reason = 'enabled; waiting for fresh planner output'
                self.latest = None
                for key in ('path', 'waypoint', 'command'):
                    self.seen.pop(key, None)
                self.start.publish(Bool(data=True))
        response.message = self.reason
        return response

    def tick(self):
        out = Twist()
        if (self.enabled and self.finished and self.home is not None and self.position is not None
                and math.dist(self.home, self.position) < .5):
            self.disarm('coverage and return home completed')
        if self.enabled:
            missing = [key for key in ('odom', 'scan', 'terrain') if self.stale(key, .8)]
            if missing:
                self.disarm('data timeout: ' + ', '.join(missing) + '; enable again after recovery')
            elif (self.latest is not None and not self.stale('command', .4)
                  and not self.stale('path', .6) and not self.stale('waypoint', 3.0)):
                x, yaw = self.latest.linear.x, self.latest.angular.z
                if math.isfinite(x) and math.isfinite(yaw):
                    speed = self.get_parameter('max_speed').value
                    rate = self.get_parameter('max_yaw_rate').value
                    out.linear.x = max(-speed, min(speed, x))
                    out.angular.z = max(-rate, min(rate, yaw))
                    self.reason = 'running'
                else:
                    self.disarm('non-finite command')
            else:
                self.reason = 'holding: waiting for fresh path/waypoint/command'
        # Do not contend with another stack even when this stack is paused.
        if not self.conflict:
            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.conflict:
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
