#!/usr/bin/env python3
"""Publish a real follower pose/velocity in the shared map frame.

Expected follower TF chain:
    map -> follower/odom -> follower/base_link

This bridge does not generate TF. It only exposes a convoy-facing odometry topic
whose pose is expressed in the common map frame.
"""

import time

from follower_input_guard import valid_odom

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
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
        # 동적 TF는 최신 상태를 우선한다. static_qos는 지정하지 않아 기존
        # Reliable/Transient Local 장착 TF 수신 동작을 유지한다.
        dynamic_qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST)
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=dynamic_qos)
        self.latest_filtered_odom = None
        self.last_ekf_rx = None
        self.ekf_timeout = 0.5

        self.create_subscription(
            Odometry,
            self.filtered_odom_topic,
            self.filtered_odom_callback,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST),
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
        self.latest_filtered_odom = None
        self.last_ekf_rx = None
        if not valid_odom(msg, 'follower/odom', self.base_frame):
            return
        self.latest_filtered_odom = msg
        self.last_ekf_rx = time.monotonic()

    def ekf_is_fresh(self):
        if self.latest_filtered_odom is None:
            return False
        if self.last_ekf_rx is None:
            return False

        # Check elapsed time since actual reception.
        rx_age = time.monotonic() - self.last_ekf_rx
        if not (0.0 <= rx_age <= self.ekf_timeout):
            return False

        # Check original measurement timestamp.
        stamp = self.latest_filtered_odom.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return False

        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec

        age = (now_ns - stamp_ns) / 1e9
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
                # 단일 executor 안에서 TF 수신 callback을 기다리며 막지 않는다.
                # 없으면 현재 주기를 건너뛰고 다음 주기에 재시도한다.
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for TF {self.map_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        # Reject missing or stale dynamic TF.
        tf_stamp = transform.header.stamp
        tf_stamp_ns = (
            tf_stamp.sec * 1000000000
            + tf_stamp.nanosec
        )

        now_ns = self.get_clock().now().nanoseconds

        odom_stamp=self.latest_filtered_odom.header.stamp
        odom_ns=odom_stamp.sec*1000000000+odom_stamp.nanosec
        if max((now_ns-odom_ns)/1e9,(now_ns-tf_stamp_ns)/1e9)>.15:
            self.get_logger().warning(
                f'Bridge source delay: ekf_age={(now_ns-odom_ns)/1e9:.3f}s, '
                f'tf_age={(now_ns-tf_stamp_ns)/1e9:.3f}s',throttle_duration_sec=2.0)

        if tf_stamp_ns == 0:
            self.get_logger().warning(
                'Follower Bridge: TF timestamp missing',
                throttle_duration_sec=2.0,
            )
            return

        tf_age = (now_ns - tf_stamp_ns) / 1e9

        if not (0.0 <= tf_age <= self.ekf_timeout):
            self.get_logger().warning(
                f'Follower Bridge: TF stale, age={tf_age:.3f}s',
                throttle_duration_sec=2.0,
            )
            return

        msg = Odometry()
        # Both pose and twist must be fresh; preserve the older source stamp.
        odom_stamp = self.latest_filtered_odom.header.stamp
        odom_ns = odom_stamp.sec * 1000000000 + odom_stamp.nanosec
        msg.header.stamp = tf_stamp if tf_stamp_ns <= odom_ns else odom_stamp
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
