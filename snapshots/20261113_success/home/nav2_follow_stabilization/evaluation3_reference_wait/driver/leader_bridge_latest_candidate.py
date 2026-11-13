#!/usr/bin/env python3
"""
Real leader bridge for the hierarchical convoy robot.

The proven real-hardware stack keeps its native topics/TF:
  /odometry/filtered + map->odom->base_link

This node adds a convoy-facing interface without changing the motor, EKF,
LiDAR, IMU, AMCL, or Nav2 stack:
  /leader/odom  : leader pose in the shared map frame + filtered twist
  /leader/path  : sampled leader path in the shared map frame
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import Buffer, TransformException, TransformListener


def latest_state_qos(depth=1):
    return QoSProfile(depth=depth,history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


class RealLeaderBridge(Node):
    def __init__(self):
        super().__init__('real_leader_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('filtered_odom_topic', '/odometry/filtered')
        self.declare_parameter('leader_odom_topic', '/leader/odom')
        self.declare_parameter('leader_path_topic', '/leader/path')
        self.declare_parameter('path_min_distance', 0.10)
        self.declare_parameter('path_reset_distance', 0.50)
        # 10Hz 주기의 최대 100ms 대기를 20Hz의 최대 50ms로 줄인다.
        # 원본 stamp와 입력 유효기간은 그대로 유지한다.
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_path_points', 5000)
        self.declare_parameter('tf_timeout', 0.5)
        self.declare_parameter('odom_timeout', 0.5)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.filtered_odom_topic = str(
            self.get_parameter('filtered_odom_topic').value
        )
        self.leader_odom_topic = str(
            self.get_parameter('leader_odom_topic').value
        )
        self.leader_path_topic = str(
            self.get_parameter('leader_path_topic').value
        )
        self.path_min_distance = float(
            self.get_parameter('path_min_distance').value
        )
        self.path_reset_distance = float(self.get_parameter('path_reset_distance').value)
        if not math.isfinite(self.path_reset_distance) or self.path_reset_distance <= self.path_min_distance:
            raise ValueError('path_reset_distance must exceed path_min_distance')
        self.publish_rate = float(
            self.get_parameter('publish_rate').value
        )
        self.max_path_points = int(
            self.get_parameter('max_path_points').value
        )
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        if not math.isfinite(self.publish_rate) or not 1.0 <= self.publish_rate <= 30.0:
            raise ValueError('publish_rate must be finite and between 1 and 30 Hz')
        if any(not math.isfinite(value) or value <= 0.0
               for value in (self.tf_timeout, self.odom_timeout)):
            raise ValueError('tf_timeout and odom_timeout must be finite and positive')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=latest_state_qos(10))

        self.latest_filtered_odom = None

        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.last_path_x = None
        self.last_path_y = None
        self.last_observed_xy = None

        self.create_subscription(
            Odometry,
            self.filtered_odom_topic,
            self.filtered_odom_callback,
            latest_state_qos(),
        )

        self.leader_odom_pub = self.create_publisher(
            Odometry,
            self.leader_odom_topic,
            latest_state_qos(),
        )
        self.leader_path_pub = self.create_publisher(
            Path,
            self.leader_path_topic,
            latest_state_qos(),
        )

        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 1)

        self.create_timer(
            1.0 / self.publish_rate,
            self.publish_leader_state,
        )

        self.get_logger().info(
            'Real leader bridge started: '
            f'{self.map_frame}->{self.base_frame}, '
            f'{self.filtered_odom_topic} -> '
            f'{self.leader_odom_topic}, {self.leader_path_topic}; '
            f'latest-state BestEffort/1, nonblocking TF, oldest source stamp, {self.publish_rate:g}Hz'
        )

    def filtered_odom_callback(self, msg):
        self.latest_filtered_odom = msg

    def reset_path(self, reason):
        self.path.poses.clear()
        self.last_path_x = None
        self.last_path_y = None
        self.last_observed_xy = None
        self.get_logger().info('Leader path reset: ' + reason)

    def initial_pose_callback(self, msg):
        if msg.header.frame_id == self.map_frame:
            self.reset_path('initial pose received')

    @staticmethod
    def stamp_is_fresh(stamp, now, timeout):
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        age = (now.nanoseconds - stamp_ns) / 1e9
        # Reject missing stamps and clock jumps; allow 100 ms clock skew.
        return stamp_ns > 0 and -0.1 <= age <= timeout

    def publish_leader_state(self):
        if self.latest_filtered_odom is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for TF {self.map_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        # TF 조회는 기다리지 않는다. A cached TF or twist
        # must not be presented to the follower as a new measurement.
        now = self.get_clock().now()
        if not self.stamp_is_fresh(transform.header.stamp, now, self.tf_timeout):
            self.get_logger().warning(
                'Leader publication paused: stale or invalid TF stamp',
                throttle_duration_sec=2.0,
            )
            return
        if not self.stamp_is_fresh(
                self.latest_filtered_odom.header.stamp, now, self.odom_timeout):
            self.get_logger().warning(
                'Leader publication paused: stale or invalid filtered odometry stamp',
                throttle_duration_sec=2.0,
            )
            return

        leader_odom = Odometry()
        # 자세와 속도 중 더 오래된 원본 시각을 보존한다.
        tf_stamp=transform.header.stamp
        odom_stamp=self.latest_filtered_odom.header.stamp
        tf_ns=tf_stamp.sec*1000000000+tf_stamp.nanosec
        odom_ns=odom_stamp.sec*1000000000+odom_stamp.nanosec
        source_stamp=tf_stamp if tf_ns<=odom_ns else odom_stamp
        leader_odom.header.stamp = source_stamp
        leader_odom.header.frame_id = self.map_frame
        leader_odom.child_frame_id = self.base_frame

        leader_odom.pose.pose.position.x = transform.transform.translation.x
        leader_odom.pose.pose.position.y = transform.transform.translation.y
        leader_odom.pose.pose.position.z = transform.transform.translation.z
        leader_odom.pose.pose.orientation = transform.transform.rotation

        if self.latest_filtered_odom is not None:
            leader_odom.twist = self.latest_filtered_odom.twist

        self.leader_odom_pub.publish(leader_odom)

        x = leader_odom.pose.pose.position.x
        y = leader_odom.pose.pose.position.y
        if self.last_observed_xy is not None:
            px, py = self.last_observed_xy
            if math.hypot(x-px, y-py) > self.path_reset_distance:
                self.reset_path('map pose discontinuity')
        self.last_observed_xy = (x, y)
        moved_enough = (
            self.last_path_x is None
            or math.hypot(x - self.last_path_x, y - self.last_path_y)
            >= self.path_min_distance
        )

        if moved_enough:
            pose = PoseStamped()
            pose.header = leader_odom.header
            pose.pose = leader_odom.pose.pose
            self.path.poses.append(pose)

            if len(self.path.poses) > self.max_path_points:
                excess = len(self.path.poses) - self.max_path_points
                del self.path.poses[:excess]

            self.last_path_x = x
            self.last_path_y = y

        # Republish periodically so a follower that joins later receives the
        # current path even while the leader is stopped.
        self.path.header.stamp = source_stamp
        self.leader_path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = RealLeaderBridge()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
