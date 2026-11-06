#!/usr/bin/env python3
"""APF obstacle avoidance for the real follower robot.

Inputs:
  /follower/scan
  /follower/cmd_vel_raw
  /leader/odom
  /follower/odom_map

Output:
  /follower/cmd_vel

The leader is filtered out using shared-map coordinates so the follower does not
avoid the leader as if it were a normal obstacle.
"""

import math
import time

from follower_input_guard import fresh, valid_odom

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RealAPFAvoidance(Node):
    def __init__(self):
        super().__init__('real_apf_avoidance')

        self.declare_parameter('scan_topic', '/follower/scan')
        self.declare_parameter('raw_cmd_topic', '/follower/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/follower/cmd_vel')
        self.declare_parameter('leader_odom_topic', '/leader/odom')
        self.declare_parameter('follower_odom_topic', '/follower/odom_map')

        self.declare_parameter('influence_radius', 0.70)
        self.declare_parameter('repulsive_gain', 0.50)
        self.declare_parameter('safety_dist', 0.25)
        self.declare_parameter('leader_filter_radius', 0.50)
        self.declare_parameter('max_linear', 0.10)
        self.declare_parameter('max_angular', 0.20)
        self.declare_parameter('avoidance_speed_scale', 0.60)
        self.declare_parameter('control_rate', 20.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.raw_cmd_topic = str(self.get_parameter('raw_cmd_topic').value)
        self.output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)
        self.leader_odom_topic = str(self.get_parameter('leader_odom_topic').value)
        self.follower_odom_topic = str(self.get_parameter('follower_odom_topic').value)

        self.influence_radius = float(self.get_parameter('influence_radius').value)
        self.repulsive_gain = float(self.get_parameter('repulsive_gain').value)
        self.safety_dist = float(self.get_parameter('safety_dist').value)
        self.leader_filter_radius = float(
            self.get_parameter('leader_filter_radius').value
        )
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.avoidance_speed_scale = float(
            self.get_parameter('avoidance_speed_scale').value
        )
        control_rate = float(self.get_parameter('control_rate').value)

        self.data_timeout = 0.5
        self.input_stamps = {}
        self.last_scan_rx = None
        self.last_raw_cmd_rx = None
        self.last_leader_rx = None
        self.last_follower_rx = None
        self.scan = None
        self.raw_cmd = Twist()
        self.leader_pos = None
        self.follower_pos = None
        self.follower_yaw = 0.0

        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.create_subscription(Twist, self.raw_cmd_topic, self.raw_cmd_cb, 10)
        self.create_subscription(Odometry, self.leader_odom_topic, self.leader_cb, 10)
        self.create_subscription(Odometry, self.follower_odom_topic, self.follower_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.create_timer(1.0 / max(control_rate, 1.0), self.control_loop)

        self.get_logger().info(
            'Real APF started: '
            f'safety={self.safety_dist:.2f} m, '
            f'influence={self.influence_radius:.2f} m'
        )

    @staticmethod
    def yaw_from_quaternion(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def scan_cb(self, msg):
        self.last_scan_rx = None
        self.scan = None
        # Until scan-to-base TF support is implemented, reject sensor-frame scans.
        if (msg.header.frame_id != 'follower/base_link'
                or not fresh(msg.header.stamp, self.get_clock().now().nanoseconds, self.data_timeout)
                or not msg.ranges
                or not all(math.isfinite(v) for v in (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max))
                or msg.angle_increment == 0 or not 0 < msg.range_min < msg.range_max):
            return
        self.input_stamps['scan'] = msg.header.stamp
        self.last_scan_rx = time.monotonic()
        self.scan = msg

    def raw_cmd_cb(self, msg):
        self.last_raw_cmd_rx = None
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            self.raw_cmd = Twist()
            return
        self.last_raw_cmd_rx = time.monotonic()
        self.raw_cmd = msg

    def leader_cb(self, msg):
        self.last_leader_rx = None
        self.leader_pos = None
        if not valid_odom(msg, 'map', 'base_link') or not fresh(msg.header.stamp, self.get_clock().now().nanoseconds, self.data_timeout):
            return
        self.input_stamps['leader'] = msg.header.stamp
        self.last_leader_rx = time.monotonic()
        self.leader_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def follower_cb(self, msg):
        self.last_follower_rx = None
        self.follower_pos = None
        if not valid_odom(msg, 'map', 'follower/base_link') or not fresh(msg.header.stamp, self.get_clock().now().nanoseconds, self.data_timeout):
            return
        self.input_stamps['follower'] = msg.header.stamp
        self.last_follower_rx = time.monotonic()
        self.follower_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        self.follower_yaw = self.yaw_from_quaternion(
            msg.pose.pose.orientation
        )

    def is_leader_return(self, scan_angle, distance):
        if self.leader_pos is None or self.follower_pos is None:
            return False

        world_angle = scan_angle + self.follower_yaw
        ox = self.follower_pos[0] + distance * math.cos(world_angle)
        oy = self.follower_pos[1] + distance * math.sin(world_angle)

        return math.hypot(
            ox - self.leader_pos[0],
            oy - self.leader_pos[1],
        ) < self.leader_filter_radius

    def inputs_fresh(self):
        now = time.monotonic()
        timestamps = (
            self.last_scan_rx,
            self.last_raw_cmd_rx,
            self.last_leader_rx,
            self.last_follower_rx,
        )
        if not all(fresh(self.input_stamps[key], self.get_clock().now().nanoseconds, self.data_timeout)
                   for key in ('scan', 'leader', 'follower') if key in self.input_stamps):
            return False
        return len(self.input_stamps) == 3 and all(
            stamp is not None
            and 0 <= now - stamp <= self.data_timeout
            for stamp in timestamps
        )

    def control_loop(self):
        if not self.inputs_fresh():
            self.cmd_pub.publish(Twist())
            return

        # Fail-safe: no scan or no follower localization -> stop.
        if self.scan is None or self.follower_pos is None:
            self.cmd_pub.publish(Twist())
            return

        # A stop from the upstream controller must remain a stop.
        if self.raw_cmd.linear.x == 0.0 and self.raw_cmd.angular.z == 0.0:
            self.cmd_pub.publish(Twist())
            return
        if not any(math.isfinite(r) and self.scan.range_min <= r <= self.scan.range_max
                   for r in self.scan.ranges):
            self.cmd_pub.publish(Twist())
            return

        fx_rep = 0.0
        fy_rep = 0.0
        min_dist = float('inf')

        for i, r in enumerate(self.scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < self.scan.range_min or r > self.scan.range_max:
                continue

            angle = self.scan.angle_min + i * self.scan.angle_increment

            if self.is_leader_return(angle, r):
                continue

            min_dist = min(min_dist, r)

            if r > self.influence_radius:
                continue

            magnitude = (
                self.repulsive_gain
                * (1.0 / r - 1.0 / self.influence_radius)
                / (r * r)
            )
            fx_rep += -magnitude * math.cos(angle)
            fy_rep += -magnitude * math.sin(angle)

        if min_dist < self.safety_dist:
            self.get_logger().warning(
                f'APF STOP: obstacle {min_dist:.2f} m',
                throttle_duration_sec=1.0,
            )
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()
        rep_magnitude = math.hypot(fx_rep, fy_rep)

        if rep_magnitude > 0.1:
            rep_angle = math.atan2(fy_rep, fx_rep)
            avoidance_angular = rep_angle * min(1.0, rep_magnitude * 0.5)

            cmd.linear.x = max(
                0.0,
                min(
                    self.max_linear,
                    self.raw_cmd.linear.x * self.avoidance_speed_scale,
                ),
            )
            cmd.angular.z = max(
                -self.max_angular,
                min(
                    self.max_angular,
                    self.raw_cmd.angular.z + avoidance_angular,
                ),
            )
        else:
            cmd.linear.x = max(
                -self.max_linear,
                min(self.max_linear, self.raw_cmd.linear.x),
            )
            cmd.angular.z = max(
                -self.max_angular,
                min(self.max_angular, self.raw_cmd.angular.z),
            )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RealAPFAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
