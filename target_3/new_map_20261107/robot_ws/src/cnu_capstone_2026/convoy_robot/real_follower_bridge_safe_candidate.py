#!/usr/bin/env python3
"""Publish a real follower pose/velocity in the shared map frame.

Expected follower TF chain:
    map -> follower/odom -> follower/base_link

This bridge does not generate TF. It only exposes a convoy-facing odometry topic
whose pose is expressed in the common map frame.
"""

import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener


class RealFollowerBridge(Node):
    def __init__(self):
        super().__init__('real_follower_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'follower/base_link')
        self.declare_parameter('filtered_odom_topic', '/follower/odometry/filtered')
        self.declare_parameter('output_topic', '/follower/odom_map')
        self.declare_parameter('publish_rate', 10.0)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.filtered_odom_topic = str(
            self.get_parameter('filtered_odom_topic').value
        )
        self.output_topic = str(self.get_parameter('output_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_filtered_odom = None
        self.last_ekf_rx = None
        self.ekf_timeout = 0.5

        self.create_subscription(
            Odometry,
            self.filtered_odom_topic,
            self.filtered_odom_callback,
            20,
        )
        self.pub = self.create_publisher(Odometry, self.output_topic, 20)
        self.create_timer(
            1.0 / max(publish_rate, 1.0),
            self.publish_state,
        )

        self.get_logger().info(
            f'Real follower bridge: {self.map_frame}->{self.base_frame} '
            f'+ {self.filtered_odom_topic} -> {self.output_topic}'
        )

    def filtered_odom_callback(self, msg):
        self.latest_filtered_odom = msg
        self.last_ekf_rx = time.monotonic()

    def ekf_is_fresh(self):
        if self.latest_filtered_odom is None:
            return False
        if self.last_ekf_rx is None:
            return False

        age = time.monotonic() - self.last_ekf_rx
        return 0.0 <= age <= self.ekf_timeout

    def publish_state(self):
        # Do not publish map odometry using stale EKF data.
        if not self.ekf_is_fresh():
            self.get_logger().warning(
                'Follower Bridge: EKF missing or stale',
                throttle_duration_sec=2.0,
            )
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for TF {self.map_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = transform.transform.translation.x
        msg.pose.pose.position.y = transform.transform.translation.y
        msg.pose.pose.position.z = transform.transform.translation.z
        msg.pose.pose.orientation = transform.transform.rotation

        if self.latest_filtered_odom is not None:
            msg.twist = self.latest_filtered_odom.twist

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealFollowerBridge()
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
