#!/usr/bin/env python3
"""Pure Pursuit controller for a real follower robot.

All pose comparisons happen in the shared `map` frame:
  /leader/odom      : real leader pose in map
  /leader/path      : sampled real leader path in map
  /follower/odom_map: follower pose in map

Output is deliberately conservative for first hardware tests:
  /follower/cmd_vel_raw
"""

import math
import time

from follower_input_guard import fresh, valid_odom, valid_path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class RealPurePursuit(Node):
    def __init__(self):
        super().__init__('real_pure_pursuit')

        self.declare_parameter('leader_path_topic', '/leader/path')
        self.declare_parameter('leader_odom_topic', '/leader/odom')
        self.declare_parameter('follower_odom_topic', '/follower/odom_map')
        self.declare_parameter('cmd_vel_raw_topic', '/follower/cmd_vel_raw')

        self.declare_parameter('follow_dist', 1.5)
        self.declare_parameter('stop_dist', 1.0)
        self.declare_parameter('lookahead_dist', 0.8)
        self.declare_parameter('leader_stop_threshold', 0.03)

        # First real-hardware limits reuse the proven leader test limits.
        self.declare_parameter('max_linear', 0.10)
        self.declare_parameter('min_linear', 0.03)
        self.declare_parameter('max_angular', 0.20)
        self.declare_parameter('control_rate', 20.0)

        self.leader_path_topic = str(self.get_parameter('leader_path_topic').value)
        self.leader_odom_topic = str(self.get_parameter('leader_odom_topic').value)
        self.follower_odom_topic = str(self.get_parameter('follower_odom_topic').value)
        self.cmd_vel_raw_topic = str(self.get_parameter('cmd_vel_raw_topic').value)

        self.follow_dist = float(self.get_parameter('follow_dist').value)
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.leader_stop_th = float(
            self.get_parameter('leader_stop_threshold').value
        )
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.min_linear = float(self.get_parameter('min_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        control_rate = float(self.get_parameter('control_rate').value)

        self.path = []
        self.data_timeout = 0.5
        self.input_stamps = {}
        self.last_path_rx = None
        self.last_leader_rx = None
        self.last_follower_rx = None
        self.follower_ready = False
        self.leader_ready = False

        self.follower_x = 0.0
        self.follower_y = 0.0
        self.follower_yaw = 0.0
        self.leader_x = 0.0
        self.leader_y = 0.0
        self.leader_vx = 0.0
        self.leader_wz = 0.0
        self.leader_speed = 0.0

        self.create_subscription(
            Path, self.leader_path_topic, self.path_cb, 10
        )
        self.create_subscription(
            Odometry, self.follower_odom_topic, self.follower_cb, 10
        )
        self.create_subscription(
            Odometry, self.leader_odom_topic, self.leader_cb, 10
        )

        self.cmd_pub = self.create_publisher(
            Twist, self.cmd_vel_raw_topic, 10
        )
        self.create_timer(1.0 / max(control_rate, 1.0), self.control_loop)

        self.get_logger().info(
            'Real Pure Pursuit started in shared map frame; '
            f'max_linear={self.max_linear:.2f} m/s, '
            f'max_angular={self.max_angular:.2f} rad/s'
        )

    @staticmethod
    def yaw_from_quaternion(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def path_cb(self, msg):
        self.last_path_rx = None
        if not valid_path(msg) or not fresh(msg.header.stamp, self.get_clock().now().nanoseconds, self.data_timeout):
            self.path = []
            return
        self.input_stamps['path'] = msg.header.stamp
        self.last_path_rx = time.monotonic()
        self.path = [
            (p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ]

    def follower_cb(self, msg):
        self.last_follower_rx = None
        self.follower_ready = False
        if not valid_odom(msg, 'map', 'follower/base_link') or not fresh(msg.header.stamp, self.get_clock().now().nanoseconds, self.data_timeout):
            return
        self.input_stamps['follower'] = msg.header.stamp
        self.last_follower_rx = time.monotonic()
        self.follower_ready = True
        self.follower_x = msg.pose.pose.position.x
        self.follower_y = msg.pose.pose.position.y
        self.follower_yaw = self.yaw_from_quaternion(
            msg.pose.pose.orientation
        )

    def leader_cb(self, msg):
        self.last_leader_rx = None
        self.leader_ready = False
        if not valid_odom(msg, 'map', 'base_link') or not fresh(msg.header.stamp, self.get_clock().now().nanoseconds, self.data_timeout):
            return
        self.input_stamps['leader'] = msg.header.stamp
        self.last_leader_rx = time.monotonic()
        self.leader_ready = True
        self.leader_x = msg.pose.pose.position.x
        self.leader_y = msg.pose.pose.position.y
        self.leader_vx = msg.twist.twist.linear.x
        self.leader_wz = msg.twist.twist.angular.z
        self.leader_speed = abs(self.leader_vx)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def inputs_fresh(self):
        now = time.monotonic()
        timestamps = (
            self.last_path_rx,
            self.last_leader_rx,
            self.last_follower_rx,
        )
        if not all(fresh(self.input_stamps[key], self.get_clock().now().nanoseconds, self.data_timeout)
                   for key in ('path', 'leader', 'follower') if key in self.input_stamps):
            return False
        return len(self.input_stamps) == 3 and all(
            stamp is not None
            and 0 <= now - stamp <= self.data_timeout
            for stamp in timestamps
        )

    def control_loop(self):
        if not self.inputs_fresh():
            self.publish_stop()
            return

        if not self.leader_ready or not self.follower_ready:
            self.publish_stop()
            return

        cmd = Twist()
        dist_to_leader = math.hypot(
            self.leader_x - self.follower_x,
            self.leader_y - self.follower_y,
        )

        leader_stopped = self.leader_speed < self.leader_stop_th
        leader_reversing = self.leader_vx < -0.01
        leader_rotating = (
            abs(self.leader_wz) > 0.05 and self.leader_speed < 0.05
        )

        if dist_to_leader <= self.stop_dist:
            self.publish_stop()
            return

        if leader_rotating:
            cmd.angular.z = max(
                -self.max_angular,
                min(self.max_angular, self.leader_wz * 0.8),
            )
            self.cmd_pub.publish(cmd)
            return

        if leader_reversing:
            if dist_to_leader > self.follow_dist * 1.2:
                cmd.linear.x = -min(
                    self.max_linear,
                    (dist_to_leader - self.follow_dist) * 0.5,
                )
                cmd.angular.z = max(
                    -self.max_angular,
                    min(self.max_angular, self.leader_wz * 0.5),
                )
            self.cmd_pub.publish(cmd)
            return

        if leader_stopped and dist_to_leader <= self.follow_dist * 1.2:
            self.cmd_pub.publish(cmd)
            return

        if len(self.path) < 2:
            self.cmd_pub.publish(cmd)
            return

        target = self.find_target(self.lookahead_dist)
        if target is None:
            self.cmd_pub.publish(cmd)
            return

        tx, ty = target
        dx = tx - self.follower_x
        dy = ty - self.follower_y

        local_x = (
            math.cos(self.follower_yaw) * dx
            + math.sin(self.follower_yaw) * dy
        )
        local_y = (
            -math.sin(self.follower_yaw) * dx
            + math.cos(self.follower_yaw) * dy
        )

        ld2 = local_x * local_x + local_y * local_y
        if ld2 < 1e-6:
            self.cmd_pub.publish(cmd)
            return

        curvature = 2.0 * local_y / ld2
        dist_error = dist_to_leader - self.follow_dist

        if dist_error > 1.0:
            linear = self.max_linear
        elif dist_error > 0.3:
            ratio = max(0.0, min(1.0, dist_error))
            linear = self.min_linear + (
                self.max_linear - self.min_linear
            ) * ratio
        else:
            linear = max(0.0, dist_error * 0.5)
            if linear > 0.0:
                linear = max(self.min_linear, linear)

        linear = min(self.max_linear, linear)
        angular = max(
            -self.max_angular,
            min(self.max_angular, curvature * linear),
        )

        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def find_target(self, lookahead):
        closest_idx = 0
        min_dist = float('inf')

        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - self.follower_x, py - self.follower_y)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        for i in range(closest_idx, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - self.follower_x, py - self.follower_y) >= lookahead:
                return px, py

        return self.path[-1] if self.path else None


def main(args=None):
    rclpy.init(args=args)
    node = RealPurePursuit()
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
