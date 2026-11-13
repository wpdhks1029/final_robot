#!/usr/bin/env python3
"""Opt-in telemetry/status relay; no motor command publisher or service calls."""
import argparse
from collections import Counter
import socket
import time
import zlib

from unicast_wire import encode, decode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=('leader', 'follower'), required=True)
    parser.add_argument('--peer', required=True)
    parser.add_argument('--port', type=int, default=20180)
    args = parser.parse_args()
    socket.inet_aton(args.peer)
    import rclpy
    from rclpy.serialization import serialize_message, deserialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import String
    from action_msgs.msg import GoalStatusArray
    best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    channels = {
        1: ('leader', '/leader/odom', '/leader/odom_unicast', Odometry),
        2: ('leader', '/leader/path', '/leader/path_unicast', Path),
        3: ('leader', '/navigate_to_pose/_action/status', '/leader/nav2_goal_status_unicast', GoalStatusArray),
        4: ('follower', '/follower/motor_candidate/status', '/follower/motor_candidate/status_unicast', String),
        5: ('follower', '/follower/local_avoidance/status', '/follower/local_avoidance/status_unicast', String),
        6: ('follower', '/follower/odom_map', '/follower/odom_map_unicast', Odometry),
    }
    rclpy.init(args=[])
    node = rclpy.create_node(args.role + '_unicast_relay')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))
    sock.setblocking(False)
    pubs = {}; subs = []; latest = {}; received = {}; stats = Counter()
    sequence = time.time_ns()
    for channel, (role, source, target, kind) in channels.items():
        if role == args.role:
            def callback(msg, c=channel):
                latest[c] = (bytes(serialize_message(msg)), time.monotonic())
            subs.append(node.create_subscription(kind, source, callback, latched if channel == 3 else best))
        else:
            pubs[channel] = node.create_publisher(kind, target, best)

    def send_latest():
        nonlocal sequence
        for channel, (raw, receipt) in list(latest.items()):
            # Action status is retained like its original TRANSIENT_LOCAL topic.
            # All measurement/status payload timestamps stay byte-for-byte intact.
            if channel != 3 and time.monotonic() - receipt > 0.3:
                continue
            try:
                sequence += 1
                packet = encode(channel, sequence, node.get_clock().now().nanoseconds, raw)
                sock.sendto(packet, (args.peer, args.port))
                stats['sent'] += 1
            except (ValueError, OSError):
                stats['send_rejected'] += 1

    def receive():
        for _ in range(64):
            try:
                packet, address = sock.recvfrom(65535)
            except BlockingIOError:
                break
            if address[0] != args.peer or address[1] != args.port:
                stats['wrong_peer'] += 1
                continue
            try:
                channel, seq, raw = decode(packet, node.get_clock().now().nanoseconds)
                if channel not in pubs or seq <= received.get(channel, 0):
                    raise ValueError('WRONG_CHANNEL_OR_REORDERED')
                msg = deserialize_message(raw, channels[channel][3])
                received[channel] = seq
                pubs[channel].publish(msg)
                stats['received_' + str(channel)] += 1
            except (ValueError, RuntimeError, TypeError, zlib.error):
                stats['receive_rejected'] += 1

    node.create_timer(0.05, send_latest)
    node.create_timer(0.01, receive)
    node.create_timer(5., lambda: node.get_logger().info(str(dict(stats))))
    node.get_logger().info(f'Telemetry relay {args.role}: UDP {args.port}, peer={args.peer}; original source stamps retained; no arm/service/motor commands')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
