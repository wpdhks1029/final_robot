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
        self.scan = msg

    def raw_cmd_cb(self, msg):
        self.raw_cmd = msg

    def leader_cb(self, msg):
        self.leader_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def follower_cb(self, msg):
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

    def control_loop(self):
        # Fail-safe: no scan or no follower localization -> stop.
        if self.scan is None or self.follower_pos is None:
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
