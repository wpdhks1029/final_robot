#!/usr/bin/env python3
"""Forward-only calculation preview. Fixed test output; no motor/scan interfaces.

This is not a deployable controller: no obstacle gate, path session/progress
tracking, or validated timing/geometry margins. Large turns stop, not align.
"""
import math
import json
from pathlib import Path
import sys
import time

SOURCE = Path.home() / 'robot_ws/src/cnu_capstone_2026/convoy_robot'
sys.path.insert(0, str(SOURCE))
from follower_input_guard import fresh, valid_odom, valid_path

TEST_TOPIC = '/follower/cmd_vel_test'


def validate_output_topic(resolved):
    if resolved != TEST_TOPIC:
        raise RuntimeError('Preview output remap rejected: only /follower/cmd_vel_test is allowed')


class PreviewCore:
    def __init__(self):
        self.timeout = 0.5
        # 두 장치의 작은 시계 차이만 허용. 로컬 센서·모터 TTL은 변경하지 않는다.
        self.remote_clock_margin_ns = 50_000_000
        self.leader_start_speed=.03
        self.leader_stop_speed=.01
        self.leader_moving=False
        self.follow_distance = 1.5
        self.stop_distance = 1.0
        self.lookahead = 0.8
        self.max_linear = 0.10
        self.max_angular = 0.20
        self.heading_limit = math.radians(45.)
        self.inputs = {}
        self.last_input_rejection = None
        self.rejection_sequence = 0

    def input_fresh(self,key,stamp,now_ns):
        if fresh(stamp,now_ns,self.timeout):
            return True
        value=stamp.sec*1_000_000_000+stamp.nanosec
        return (key in ('leader','path') and value>0
                and 0<value-now_ns<=self.remote_clock_margin_ns)

    def reject_input(self, key, msg, now_ns, valid_geometry):
        source=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        self.rejection_sequence+=1
        self.last_input_rejection=dict(key=key,source_ns=source,received_ros_ns=now_ns,
            age_s=(now_ns-source)/1e9 if source>0 else None,
            reason='INVALID_GEOMETRY_OR_FRAME' if not valid_geometry else 'INVALID_SOURCE_TIME',
            path_points=len(msg.poses) if key=='path' else None,
            allowed_remote_future_s=self.remote_clock_margin_ns/1e9 if key in ('leader','path') else 0.)

    def path_callback(self, msg, now_ns, received):
        self.inputs.pop('path', None)
        geometry=valid_path(msg)
        if not geometry or not self.input_fresh('path',msg.header.stamp, now_ns):
            self.reject_input('path',msg,now_ns,geometry)
            return
        self.inputs['path'] = (
            msg.header.stamp, received,
            tuple((p.pose.position.x, p.pose.position.y) for p in msg.poses),
        )

    def odom_callback(self, key, msg, now_ns, received):
        previous=self.inputs.pop(key, None)
        if key=='leader' and (previous is None
                or not self.input_fresh('leader',previous[0],now_ns)
                or not 0<=received-previous[1]<=self.timeout):
            self.leader_moving=False
        child = 'base_link' if key == 'leader' else 'follower/base_link'
        geometry=key in ('leader','follower') and valid_odom(msg,'map',child)
        if (not geometry
                or not self.input_fresh(key,msg.header.stamp, now_ns)):
            if key=='leader':self.leader_moving=False
            self.reject_input(key,msg,now_ns,geometry)
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.inputs[key] = (msg.header.stamp, received,
                            (p.x, p.y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
        if key=='leader':
            v,w=msg.twist.twist.linear.x,msg.twist.twist.angular.z
            if v<=self.leader_stop_speed or (abs(w)>.05 and abs(v)<=self.leader_stop_speed):
                self.leader_moving=False
            elif v>self.leader_start_speed:
                self.leader_moving=True

    def speed_gate(self, now_ns, received):
        """Leader/input/distance gates shared by reference and detour steering."""
        leader=self.inputs.get('leader')
        if (leader is None or not self.input_fresh('leader',leader[0],now_ns)
                or not 0<=received-leader[1]<=self.timeout):
            self.leader_moving=False
        for key in ('path', 'leader', 'follower'):
            row = self.inputs.get(key)
            if row is None:
                return 0., 0., f'STOP_MISSING_OR_INVALID_{key.upper()}'
            stamp, rx, _ = row
            if (not self.input_fresh(key,stamp, now_ns)
                    or not 0. <= received-rx <= self.timeout):
                return 0., 0., f'STOP_STALE_{key.upper()}'
        lx, ly, _, lv, lw = self.inputs['leader'][2]
        fx, fy, yaw, _, _ = self.inputs['follower'][2]
        distance = math.hypot(lx-fx, ly-fy)
        if not math.isfinite(distance):
            return 0., 0., 'STOP_INVALID_GEOMETRY'
        if distance <= self.stop_distance:
            return 0., 0., 'STOP_NEAR_LEADER'
        if lv < -0.01:
            return 0., 0., 'STOP_LEADER_REVERSE'
        if abs(lw) > 0.05 and abs(lv) <= self.leader_stop_speed:
            return 0., 0., 'STOP_LEADER_ROTATING'
        if not self.leader_moving:
            return 0., 0., 'STOP_LEADER_STOPPED'
        if distance <= self.follow_distance:
            return 0., 0., 'STOP_FOLLOW_DISTANCE'

        return min(self.max_linear, max(0., .5*(distance-self.follow_distance))), 0., 'FORWARD_ALLOWED'

    def command(self, now_ns, received):
        linear, _, reason = self.speed_gate(now_ns, received)
        if linear <= 0.:return 0., 0., reason
        points = self.inputs['path'][2]
        fx, fy, yaw, _, _ = self.inputs['follower'][2]

        # Preview retains the reference nearest-point selection. Sessions,
        # intersections and monotonic progress must be solved before deployment.
        closest = min(range(len(points)), key=lambda i: math.hypot(points[i][0]-fx, points[i][1]-fy))
        target = next((p for p in points[closest:] if math.hypot(p[0]-fx, p[1]-fy) >= self.lookahead), points[-1])
        dx, dy = target[0]-fx, target[1]-fy
        x = math.cos(yaw)*dx + math.sin(yaw)*dy
        y = -math.sin(yaw)*dx + math.cos(yaw)*dy
        if not math.isfinite(x) or not math.isfinite(y):
            return 0., 0., 'STOP_INVALID_GEOMETRY'
        if x <= 0.:
            return 0., 0., 'STOP_TARGET_BEHIND'
        if abs(math.atan2(y, x)) > self.heading_limit:
            return 0., 0., 'STOP_ALIGN_REQUIRED'
        squared = x*x+y*y
        if not math.isfinite(squared):
            return 0., 0., 'STOP_INVALID_GEOMETRY'
        if squared < 1e-6:
            return 0., 0., 'STOP_TARGET_TOO_CLOSE'
        angular = max(-self.max_angular, min(self.max_angular, 2*y/squared*linear))
        if not all(math.isfinite(v) for v in (linear, angular)):
            return 0., 0., 'STOP_INVALID_COMMAND'
        return linear, angular, 'PREVIEW_TRACK'


def main():
    # ROS is imported only for explicit runtime use; core tests need no ROS context.
    import rclpy
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import Twist,TwistStamped
    from follower_stamped_preview import PURSUIT_TOPIC,FRAME,oldest_pursuit_source
    from nav_msgs.msg import Odometry, Path as RosPath
    from follower_remote_qos import latest_remote_qos,latest_state_qos

    class PreviewNode(Node):
        def __init__(self):
            super().__init__('follower_pursuit_preview')
            validate_output_topic(self.resolve_topic_name(TEST_TOPIC))
            if self.resolve_topic_name(PURSUIT_TOPIC)!=PURSUIT_TOPIC:raise RuntimeError('Stamped preview remap rejected')
            self.core = PreviewCore()
            self.last_source_ns=None
            self.pub = self.create_publisher(Twist, TEST_TOPIC, latest_state_qos())
            self.stamped_pub=self.create_publisher(TwistStamped,PURSUIT_TOPIC,latest_state_qos())
            self.last_report = 0.
            self.reported_rejection_sequence=0
            self.rejection_reports={}
            self.kept_subs = [
                self.create_subscription(RosPath, '/leader/path', self.path_cb, latest_remote_qos()),
                self.create_subscription(Odometry, '/leader/odom', lambda m: self.odom_cb('leader', m), latest_remote_qos()),
                self.create_subscription(Odometry, '/follower/odom_map', lambda m: self.odom_cb('follower', m), latest_remote_qos()),
            ]
            self.timer = self.create_timer(.025, self.tick)
            self.get_logger().info('TEST OUTPUT ONLY: /follower/cmd_vel_test; no motor/obstacle interface')
            self.get_logger().info('선두 속도 판정: 출발 >0.03m/s, 출발 후 정지 <=0.01m/s; 원본/수신 시간 검사 유지')

        def path_cb(self, msg):
            self.core.path_callback(msg, self.get_clock().now().nanoseconds, time.monotonic())
            self.report_input_rejection()
            self.publish_new_source()

        def odom_cb(self, key, msg):
            self.core.odom_callback(key, msg, self.get_clock().now().nanoseconds, time.monotonic())
            self.report_input_rejection()
            self.publish_new_source()

        def report_input_rejection(self):
            if self.reported_rejection_sequence==self.core.rejection_sequence:return
            self.reported_rejection_sequence=self.core.rejection_sequence
            d=self.core.last_input_rejection
            key=(d['key'],d['reason']);now=time.monotonic()
            if now-self.rejection_reports.get(key,float('-inf'))>=2.:
                self.get_logger().warning('INPUT_REJECTED '+json.dumps(d,ensure_ascii=False))
                self.rejection_reports[key]=now

        def publish_new_source(self):
            source_ns=oldest_pursuit_source(self.core.inputs,
                remote_clock_margin_ns=self.core.remote_clock_margin_ns)
            if source_ns!=self.last_source_ns:
                self.tick()

        def tick(self):
            now = time.monotonic()
            v, w, reason = self.core.command(self.get_clock().now().nanoseconds, now)
            msg = Twist()
            msg.linear.x, msg.angular.z = v, w
            stamped=TwistStamped();stamped.header.frame_id=FRAME
            source_ns=oldest_pursuit_source(self.core.inputs,
                remote_clock_margin_ns=self.core.remote_clock_margin_ns)
            self.last_source_ns=source_ns
            stamped.header.stamp.sec=source_ns//10**9;stamped.header.stamp.nanosec=source_ns%10**9
            stamped.twist=msg;self.stamped_pub.publish(stamped)
            self.pub.publish(msg)
            if now-self.last_report >= 2.:
                self.get_logger().info(f'{reason}: preview_v={v:.3f}, preview_w={w:.3f}')
                self.last_report = now

    rclpy.init()
    node = None
    try:
        node = PreviewNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        # ROS 종료 과정의 문맥 닫힘만 허용하고 실행 중 오류는 숨기지 않는다.
        if rclpy.ok():raise
    finally:
        if node is not None:
            if rclpy.ok():
                node.pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
