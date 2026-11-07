#!/usr/bin/env python3
"""명시적으로 시작하는 선두 직진 시험: 0.05m/s 명령을 최대 8초 발행."""
import argparse
import json
import math
import os
import select
import sys
import time


def wait_for_start(spin,stream=None,readable=None):
    """운전자 입력 대기 중에도 ROS 수신 처리를 계속한다."""
    stream=sys.stdin if stream is None else stream
    readable=(lambda: bool(select.select([stream],[],[],0)[0])) if readable is None else readable
    while True:
        spin()
        if readable():
            return stream.readline().strip()=='START'


def feedback_ok(sample, now_ns, mono):
    if sample is None:
        return False
    msg, received = sample
    stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
    return (msg.header.frame_id == 'map' and stamp > 0
            and 0 <= (now_ns-stamp)/1e9 <= .3
            and 0 <= mono-received <= .3
            and math.isfinite(msg.twist.twist.linear.x)
            and math.isfinite(msg.twist.twist.angular.z)
            and abs(msg.twist.twist.linear.x) <= .12
            and abs(msg.twist.twist.angular.z) <= .1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-one-forward', action='store_true')
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--ack-clear-path-power-cut-ready', action='store_true')
    args = parser.parse_args()
    if not args.run_one_forward and not args.check_only:
        parser.print_help()
        return 0
    if not args.check_only and not args.ack_clear_path_power_cut_ready:
        parser.error('빈 경로 및 전원 차단 준비 확인 옵션이 필요합니다.')
    if os.environ.get('ROS_DOMAIN_ID') != '10':
        parser.error('ROS_DOMAIN_ID=10이 필요합니다.')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('leader_short_forward_trial')
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    latest = None
    started = False
    reason = '시작 전 종료'

    def receive(msg):
        nonlocal latest
        latest = (msg, time.monotonic())

    sub = node.create_subscription(Odometry, '/leader/odom', receive, 1)

    def graph_ok():
        writers = node.get_publishers_info_by_topic('/cmd_vel')
        readers = node.get_subscriptions_info_by_topic('/cmd_vel')
        return (len(writers) == 1 and writers[0].node_name == node.get_name()
                and len(readers) == 1 and readers[0].node_name == 'md200t_controller')

    try:
        if not args.check_only:
            print('대기 중: 아직 명령을 보내지 않습니다. 후발 시험 도구 실행 후 START + Enter.', flush=True)
            if not wait_for_start(lambda:rclpy.spin_once(node,timeout_sec=.02)):
                return 1
        print('연결 발견과 최신 odom을 최대 12초 확인합니다. 이동 명령 없음.', flush=True)
        end = time.monotonic()+12
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if graph_ok() and feedback_ok(latest, node.get_clock().now().nanoseconds, time.monotonic()):
                break
        else:
            print('시작 거부: 최신 선두 odom 또는 단일 명령 연결을 확인하지 못했습니다.', flush=True)
            writers = node.get_publishers_info_by_topic('/cmd_vel')
            readers = node.get_subscriptions_info_by_topic('/cmd_vel')
            print(json.dumps({
                'cmd_vel_publishers': [e.node_namespace+'/'+e.node_name for e in writers],
                'cmd_vel_subscribers': [e.node_namespace+'/'+e.node_name for e in readers],
                'leader_odom_received': latest is not None,
                'leader_odom_rx_age_s': None if latest is None else time.monotonic()-latest[1],
                'leader_odom_header_age_s': None if latest is None else
                    (node.get_clock().now().nanoseconds-(latest[0].header.stamp.sec*10**9
                     +latest[0].header.stamp.nanosec))/1e9,
                'leader_odom_frame': None if latest is None else latest[0].header.frame_id,
                'leader_v': None if latest is None else latest[0].twist.twist.linear.x,
                'leader_w': None if latest is None else latest[0].twist.twist.angular.z,
            }, ensure_ascii=False, indent=2), flush=True)
            return 1
        if args.check_only:
            print('조회 정상: 최신 선두 odom 및 단일 명령 연결 확인. 이동 명령 없음.', flush=True)
            return 0
        started = True
        deadline = time.monotonic()+8
        print('선두 직진 시작: 명령 0.05m/s, 최대 8초. Ctrl+C로 중단.', flush=True)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if not graph_ok() or not feedback_ok(latest, node.get_clock().now().nanoseconds, time.monotonic()):
                reason = '입력 또는 명령 연결 이상'
                break
            msg = Twist()
            msg.linear.x = .05
            pub.publish(msg)
            time.sleep(.03)
        else:
            reason = '8초 종료'
        return 0
    except (KeyboardInterrupt, EOFError):
        reason = '사용자 중단'
        return 1
    finally:
        if started:
            # 시작 요청 이후 모든 정상/예외 종료 경로에서 영속도 발행을 시도합니다.
            end = time.monotonic()+1
            while time.monotonic() < end and rclpy.ok():
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.02)
                time.sleep(.03)
            print('종료 사유: '+reason+'; 영속도 명령 발행 완료. 실물 정지를 확인하세요.', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
