#!/usr/bin/env python3
"""Explicit opt-in, localhost Domain217 synthetic ROS integration; no hardware."""
from follower_remote_qos import latest_state_qos
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true', help='Run isolated synthetic ROS test')
    args = parser.parse_args()
    if not args.run:
        parser.print_help()
        return 0
    # Override only this process and its child; the caller's terminal is unchanged.
    os.environ['ROS_DOMAIN_ID'] = '217'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=[])
    node = Node('follower_synthetic_preview_check')
    child = None
    output = []
    failures = []
    topics = ['/leader/path', '/leader/odom', '/follower/odom_map']
    try:
        print('ISOLATED Domain217 localhost-only. Real Domain10 inputs/outputs are not used.', flush=True)
        # Let discovery settle before adding any synthetic publishers.
        end = time.monotonic()+2.
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.1)
        others = [name for name, ns in node.get_node_names_and_namespaces()
                  if name != node.get_name()]
        if others:
            raise RuntimeError(f'Domain217 already has nodes {others}; refusing synthetic publication')
        for topic in topics + ['/follower/cmd_vel_test']:
            if node.get_publishers_info_by_topic(topic):
                raise RuntimeError(f'Existing Domain217 publisher: {topic}')
        for topic in topics + ['/follower/cmd_vel_test', '/follower/cmd_vel', '/follower/cmd_vel_raw']:
            if node.get_subscriptions_info_by_topic(topic):
                raise RuntimeError(f'Existing Domain217 subscriber: {topic}')

        pubs = {
            'path': node.create_publisher(RosPath, topics[0], 10),
            'leader': node.create_publisher(Odometry, topics[1], 10),
            'follower': node.create_publisher(Odometry, topics[2], 10),
        }
        sub = node.create_subscription(Twist, '/follower/cmd_vel_test',
                                      lambda msg: output.append((time.monotonic(), msg.linear.x, msg.angular.z)), latest_state_qos())
        preview = Path(__file__).with_name('follower_pursuit_preview.py')
        child = subprocess.Popen([sys.executable, '-B', str(preview)],
                                 env=os.environ.copy(), stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True)
        end = time.monotonic()+6.
        while time.monotonic()<end and not output:
            if child.poll() is not None:
                raise RuntimeError('Preview child exited during startup')
            rclpy.spin_once(node, timeout_sec=.1)
        if not output:
            raise RuntimeError('No preview test output discovered')

        def publish(config):
            stamp = node.get_clock().now().to_msg()
            messages = {}
            for key, child_frame in [('leader', 'base_link'), ('follower', 'follower/base_link')]:
                msg = Odometry()
                msg.header.stamp = stamp
                msg.header.frame_id = 'map'
                msg.child_frame_id = child_frame
                msg.pose.pose.orientation.w = 1.
                if key == 'leader':
                    msg.pose.pose.position.x = 3.
                    msg.twist.twist.linear.x = config.get('vx', .1)
                    msg.twist.twist.angular.z = config.get('wz', 0.)
                    if config.get('bad_frame'):
                        msg.header.frame_id = 'wrong'
                messages[key] = msg
            path = RosPath()
            path.header.stamp = stamp
            path.header.frame_id = 'map'
            points = config.get('points', [(0., 0.), (3., 0.)])
            for x, y in points:
                point = PoseStamped()
                point.header.stamp = stamp
                point.header.frame_id = 'map'
                point.pose.position.x, point.pose.position.y = x, y
                point.pose.orientation.w = 1.
                path.poses.append(point)
            messages['path'] = path
            for key, msg in messages.items():
                if key != config.get('omit'):
                    pubs[key].publish(msg)

        stop = lambda v,w: abs(v)<1e-9 and abs(w)<1e-9
        cases = [
            ('straight', {}, lambda v,w: 0.<v<=.1 and abs(w)<1e-9),
            ('left', {'points':[(0.,0.),(3.,1.)]}, lambda v,w: 0.<v<=.1 and 0.<w<=.2),
            ('right', {'points':[(0.,0.),(3.,-1.)]}, lambda v,w: 0.<v<=.1 and -.2<=w<0.),
            ('behind', {'points':[(-3.,0.),(-2.,0.)]}, stop),
            ('large_angle', {'points':[(0.,0.),(1.,3.)]}, stop),
            ('leader_stopped', {'vx':0.}, stop),
            ('leader_reverse', {'vx':-.1}, stop),
            ('leader_rotating', {'vx':0.,'wz':.1}, stop),
            ('path_reset', {'points':[(0.,0.)]}, stop),
            ('wrong_leader_frame', {'bad_frame':True}, stop),
        ]
        # Each missing-input case starts with a fresh nonzero tracking phase.
        for key in ('leader','path','follower'):
            cases += [(f'recover_before_{key}_loss', {}, lambda v,w: v>0.),
                      (f'{key}_loss', {'omit':key}, stop)]
        cases.append(('final_recovery', {}, lambda v,w: v>0.))
        for name, config, check in cases:
            output.clear()
            start = time.monotonic()
            next_pub = start
            while time.monotonic()-start < 1.5:
                if child.poll() is not None:
                    raise RuntimeError('Preview child exited during test')
                now = time.monotonic()
                if now >= next_pub:
                    publish(config)
                    next_pub = now+.05
                rclpy.spin_once(node, timeout_sec=.01)
            # Allow previous commands, DDS transition and the 0.5s expiry to settle.
            samples = [(v,w) for t,v,w in output if t-start>=1.]
            passed = len(samples)>=3 and all(check(v,w) for v,w in samples)
            print(f'{"PASS" if passed else "FAIL"} {name}: evaluated={len(samples)}, '
                  f'last={samples[-1] if samples else None}', flush=True)
            if not passed:
                failures.append(name)
        print(f'RESULT: {len(cases)-len(failures)}/{len(cases)} passed. Failures={failures}',flush=True)
        print('This checks synthetic ROS wiring and late-phase outputs, not physical safety or exact stop latency.')
        return 1 if failures else 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f'Test stopped: {exc}', file=sys.stderr)
        return 1
    finally:
        if child is not None:
            child.terminate()
            try:
                logs, _ = child.communicate(timeout=4)
            except subprocess.TimeoutExpired:
                child.kill()
                logs, _ = child.communicate()
            if child.returncode not in (0, -15):
                print('Preview child exit/log:', child.returncode, logs[-2500:])
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
