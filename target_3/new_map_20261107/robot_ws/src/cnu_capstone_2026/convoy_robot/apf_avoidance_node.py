#!/usr/bin/env python3
"""
APF 장애물 회피 노드 (선두 필터링 + 벡터 합성 패치)
- 좌표 기반 선두 필터링 (팀킬 방지)
- 척력 벡터를 angular.z에 합성 (코너링 보존)
- 회피 중 속도 감속 (안전)
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


class APFAvoidance(Node):
    def __init__(self):
        super().__init__('apf_avoidance')

        # ── 파라미터 ─────────────────────────────────
        self.influence_radius = 0.7
        self.repulsive_gain   = 0.5
        self.max_linear       = 0.45
        self.max_angular      = 1.5
        self.safety_dist      = 0.25

        # ── 상태 변수 ────────────────────────────────
        self.scan         = None
        self.pure_cmd     = Twist()
        self.follower_pos = None
        self.follower_yaw = 0.0
        self.path         = []
        self.l_x, self.l_y = None, None

        # ── 구독 및 퍼블리시 ──────────────────────────
        self.create_subscription(LaserScan, '/follower/scan', self.scan_cb, 10)
        self.create_subscription(Twist, '/follower/cmd_vel_raw', self.pure_cmd_cb, 10)
        self.create_subscription(Odometry, '/follower/odom', self.f_odom_cb, 10)
        self.create_subscription(Odometry, '/odom', self.l_odom_cb, 10)
        self.create_subscription(Path, '/leader/path', self.path_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/follower/cmd_vel', 10)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('🚧 개선된 APF 장애물 회피 노드 시작!')

    def scan_cb(self, msg): self.scan = msg
    def pure_cmd_cb(self, msg): self.pure_cmd = msg
    def path_cb(self, msg): self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def l_odom_cb(self, msg):
        self.l_x = msg.pose.pose.position.x
        self.l_y = msg.pose.pose.position.y

    def f_odom_cb(self, msg):
        self.follower_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.follower_yaw = math.atan2(siny, cosy)

    def is_leader(self, angle, distance):
        """🌟 좌표 기반 선두 필터링"""
        if self.l_x is None or self.follower_pos is None:
            return False
        ox = self.follower_pos[0] + distance * math.cos(angle + self.follower_yaw)
        oy = self.follower_pos[1] + distance * math.sin(angle + self.follower_yaw)
        return math.hypot(ox - self.l_x, oy - self.l_y) < 0.5

    def calc_apf_repulsive(self):
        if self.scan is None:
            return 0.0, 0.0, float('inf')

        fx_rep, fy_rep = 0.0, 0.0
        min_dist = float('inf')

        angle_min = self.scan.angle_min
        inc = self.scan.angle_increment

        for i, r in enumerate(self.scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < self.scan.range_min or r > self.scan.range_max:
                continue

            angle = angle_min + i * inc

            # 🌟 선두 로봇이면 척력 무시!
            if self.is_leader(angle, r):
                continue

            if r < self.influence_radius:
                magnitude = self.repulsive_gain * (1.0/r - 1.0/self.influence_radius) / (r*r)
                fx_rep += -magnitude * math.cos(angle)
                fy_rep += -magnitude * math.sin(angle)

            if r < min_dist:
                min_dist = r

        return fx_rep, fy_rep, min_dist

    def control_loop(self):
        cmd = Twist()

        if self.scan is None or self.follower_pos is None:
            self.cmd_pub.publish(self.pure_cmd)
            return

        fx_rep, fy_rep, min_dist = self.calc_apf_repulsive()

        # 비상 정지
        if min_dist < self.safety_dist:
            self.get_logger().warn('🛑 비상 정지 (안전 거리 진입)')
            self.cmd_pub.publish(cmd)
            return

        rep_magnitude = math.hypot(fx_rep, fy_rep)

        if rep_magnitude > 0.1:
            # 회피 모드: 척력 벡터의 각도를 조향각에 추가
            rep_angle = math.atan2(fy_rep, fx_rep)

            # 척력 크기에 비례하여 가중치 적용
            avoidance_angular = rep_angle * (rep_magnitude * 0.5)

            angular = max(-self.max_angular,
                          min(self.max_angular,
                              self.pure_cmd.angular.z + avoidance_angular))

            # 회피 중 속도 절반 감속
            linear = max(0.0, self.pure_cmd.linear.x * 0.8)

            cmd.linear.x = linear
            cmd.angular.z = angular

            self.get_logger().info(
                f'🌪️ APF 회피 기동 (척력: {rep_magnitude:.2f})',
                throttle_duration_sec=0.5
            )
        else:
            # 장애물 없으면 Pure Pursuit 그대로
            cmd = self.pure_cmd

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(APFAvoidance())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
