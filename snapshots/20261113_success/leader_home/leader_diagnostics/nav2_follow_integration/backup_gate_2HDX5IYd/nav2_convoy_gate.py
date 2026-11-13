#!/usr/bin/env python3
"""Fail-closed Nav2 velocity gate for a separately controlled follower.

Nav2's smoothed output is input only. This node is the sole publisher to the
leader motor's /cmd_vel topic and stops the leader unless the follower reports
a fresh FOLLOW phase, healthy armed feedback, fresh poses, and adequate gap.
It never arms or commands the follower.
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from nav2_convoy_gate_core import Nav2ConvoyGateCore


def latest_qos(depth=1):
    return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


class Nav2ConvoyGate(Node):
    def __init__(self):
        super().__init__('nav2_convoy_gate')
        self.core = Nav2ConvoyGateCore()
        self.output = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.nav_cb, latest_qos())
        self.create_subscription(String, '/follower/local_avoidance/status',
                                 self.follower_cb, latest_qos())
        self.create_subscription(String, '/follower/motor_candidate/status',
                                 self.motor_cb, latest_qos())
        self.create_subscription(Odometry, '/leader/odom', self.leader_cb, latest_qos())
        self.create_subscription(Odometry, '/follower/odom_map', self.follower_pose_cb,
                                 latest_qos())
        self.create_timer(.05, self.tick)
        self.last_reason = None
        self.last_graph_check = 0.
        self.core.graph_ok = False
        self.get_logger().warning('FAIL-CLOSED: waiting for exclusive /cmd_vel graph')

    def _pose(self, msg):
        p = msg.pose.pose.position
        return p.x, p.y

    def nav_cb(self, msg):
        self.core.update_nav(msg.linear.x, msg.angular.z, time.monotonic())

    def follower_cb(self, msg):
        self.core.update_follower(msg.data, time.monotonic())

    def motor_cb(self, msg):
        self.core.update_motor(msg.data, time.monotonic())

    def leader_cb(self, msg):
        x, y = self._pose(msg)
        self.core.update_leader_pose(x, y, time.monotonic())

    def follower_pose_cb(self, msg):
        x, y = self._pose(msg)
        self.core.update_follower_pose(x, y, time.monotonic())

    def graph_check(self):
        # Never arbitrate two motor writers. Nav2's velocity smoother is the
        # sole upstream command source; this process is sole /cmd_vel writer.
        writers = self.get_publishers_info_by_topic('/cmd_vel')
        readers = self.get_subscriptions_info_by_topic('/cmd_vel')
        upstream = self.get_publishers_info_by_topic('/cmd_vel_smoothed')
        return (len(writers) == 1 and writers[0].node_name == self.get_name()
                and len(readers) == 1 and readers[0].node_name == 'md200t_controller'
                and len(upstream) == 1 and upstream[0].node_name == 'velocity_smoother')

    def tick(self):
        now = time.monotonic()
        if now - self.last_graph_check >= .5:
            self.core.graph_ok = self.graph_check()
            self.last_graph_check = now
        result = self.core.evaluate(now)
        msg = Twist()
        msg.linear.x = result.linear_x
        msg.angular.z = result.angular_z
        self.output.publish(msg)
        if result.reason != self.last_reason:
            self.get_logger().info(
                f'{result.reason}: v={result.linear_x:.3f}, w={result.angular_z:.3f}')
            self.last_reason = result.reason


def main(args=None):
    # Keep the ROS context alive when Ctrl+C reaches this process so the
    # fail-closed zero can be sent before the publisher is destroyed.
    rclpy.init(args=args, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Nav2ConvoyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            zero = Twist()
            try:
                for _ in range(3):
                    node.output.publish(zero)
                    time.sleep(.02)
            except Exception as exc:
                node.get_logger().error(f'Could not publish shutdown zero: {exc}')
            node.destroy_node()
            rclpy.shutdown()
        else:
            # Covers external context shutdown paths without attempting to
            # publish through an invalid ROS context.
            node.destroy_node()


if __name__ == '__main__':
    main()
