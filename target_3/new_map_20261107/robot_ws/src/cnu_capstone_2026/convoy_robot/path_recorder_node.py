#!/usr/bin/env python3
"""
path_recorder_node.py
선두 로봇의 odom을 구독해서 경로를 기록하고
/leader/path 토픽으로 퍼블리시하는 노드
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        
        # 선두 로봇 odom 구독
        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 경로 퍼블리시
        self.pub = self.create_publisher(Path, '/leader/path', 10)
        
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        # 최소 이동 거리 (너무 잦은 기록 방지)
        self.min_dist = 0.1
        self.last_x = None
        self.last_y = None
        
        self.get_logger().info('Path Recorder 시작!')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 처음이거나 일정 거리 이상 이동했을 때만 기록
        if self.last_x is None or \
           ((x - self.last_x)**2 + (y - self.last_y)**2) ** 0.5 > self.min_dist:

            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = 'odom'
            pose.pose = msg.pose.pose
            self.path.poses.append(pose)
            self.path.header.stamp = self.get_clock().now().to_msg()

            self.last_x = x
            self.last_y = y

            # 경로 퍼블리시
            self.pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
