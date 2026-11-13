#!/usr/bin/env python3
"""
Pure Pursuit + 거리 유지 + 후진 + 회전 개선
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # ── 파라미터 ──────────────────────────────────────
        self.follow_dist    = 1.5
        self.stop_dist      = 1.0
        self.max_linear     = 0.4
        self.min_linear     = 0.05
        self.max_angular    = 1.2
        self.lookahead_dist = 0.8
        self.leader_stop_th = 0.03

        # ── 상태 변수 ─────────────────────────────────────
        self.path         = []
        self.follower_x   = 0.0
        self.follower_y   = 0.0
        self.follower_yaw = 0.0
        self.leader_x     = 0.0
        self.leader_y     = 0.0
        self.leader_vx    = 0.0  # 선두 직진 속도 (음수=후진)
        self.leader_wz    = 0.0  # 선두 회전 속도
        self.leader_speed = 0.0

        # ── 구독 ──────────────────────────────────────────
        self.create_subscription(Path, '/leader/path',
                                 self.path_cb, 10)
        self.create_subscription(Odometry, '/follower/odom',
                                 self.follower_cb, 10)
        self.create_subscription(Odometry, '/odom',
                                 self.leader_cb, 10)

        # ── 퍼블리시 ──────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            Twist, '/follower/cmd_vel_raw', 10)

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Pure Pursuit 노드 시작!')

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y)
                     for p in msg.poses]

    def follower_cb(self, msg):
        self.follower_x = msg.pose.pose.position.x
        self.follower_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.follower_yaw = math.atan2(siny, cosy)

    def leader_cb(self, msg):
        self.leader_x  = msg.pose.pose.position.x
        self.leader_y  = msg.pose.pose.position.y
        self.leader_vx = msg.twist.twist.linear.x   # 부호 포함
        self.leader_wz = msg.twist.twist.angular.z
        self.leader_speed = abs(self.leader_vx)

    def control_loop(self):
        cmd = Twist()

        dist_to_leader = math.hypot(
            self.leader_x - self.follower_x,
            self.leader_y - self.follower_y
        )

        leader_stopped   = self.leader_speed < self.leader_stop_th
        leader_reversing = self.leader_vx < -0.01  # 후진 중
        leader_rotating  = (abs(self.leader_wz) > 0.05 and
                            self.leader_speed < 0.05)  # 제자리 회전

        # ── 제자리 회전 모드 ──────────────────────────────
        # 선두가 제자리 회전 중이면 후발도 같은 방향으로 회전
        if leader_rotating:
            cmd.linear.x  = 0.0
            cmd.angular.z = self.leader_wz * 0.8
            self.cmd_pub.publish(cmd)
            return

        # ── 후진 모드 ─────────────────────────────────────
        if leader_reversing:
            # 거리가 유지 거리보다 크면 후발도 후진
            if dist_to_leader > self.follow_dist * 1.2:
                cmd.linear.x  = -min(self.max_linear,
                                     (dist_to_leader - self.follow_dist) * 0.5)
                cmd.angular.z = self.leader_wz * 0.5
            # 거리 유지되면 정지
            else:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # ── 정지 조건 ─────────────────────────────────────
        if dist_to_leader <= self.stop_dist:
            self.cmd_pub.publish(cmd)
            return

        if leader_stopped and dist_to_leader <= self.follow_dist * 1.2:
            self.cmd_pub.publish(cmd)
            return

        # ── 경로 없으면 정지 ──────────────────────────────
        if len(self.path) < 2:
            self.cmd_pub.publish(cmd)
            return

        # ── Pure Pursuit ──────────────────────────────────
        target = self.find_target(self.lookahead_dist)
        if target is None:
            self.cmd_pub.publish(cmd)
            return

        tx, ty = target
        dx = tx - self.follower_x
        dy = ty - self.follower_y
        local_x = ( math.cos(self.follower_yaw) * dx
                  + math.sin(self.follower_yaw) * dy)
        local_y = (-math.sin(self.follower_yaw) * dx
                  + math.cos(self.follower_yaw) * dy)

        ld2 = local_x**2 + local_y**2
        if ld2 < 1e-6:
            self.cmd_pub.publish(cmd)
            return

        curvature = 2.0 * local_y / ld2

        dist_error = dist_to_leader - self.follow_dist

        # 거리가 멀면 원래 속도로, 가까워지면 천천히
        if dist_error > 1.0:
            # 1m 이상 떨어져 있으면 최대 속도 (선두 멈춰도 따라잡기)
            linear = self.max_linear
        elif dist_error > 0.3:
            # 0.3~1m 사이 점진적 감속
            linear = self.min_linear + (self.max_linear - self.min_linear) * (dist_error / 1.0)
        else:
            # 가까우면 천천히 (정렬)
            linear = max(self.min_linear, dist_error * 0.5)

        angular = max(-self.max_angular,
                      min(self.max_angular, curvature * linear))

        cmd.linear.x  = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def find_target(self, lookahead):
        closest_idx = 0
        min_dist = float('inf')
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - self.follower_x,
                           py - self.follower_y)
            if d < min_dist:
                min_dist = d
                closest_idx = i
        for i in range(closest_idx, len(self.path)):
            px, py = self.path[i]
            d = math.hypot(px - self.follower_x,
                           py - self.follower_y)
            if d >= lookahead:
                return (px, py)
        return self.path[-1]


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
