#!/usr/bin/env python3
"""Read-only convoy topic sampling. No files, motor ports or velocity publishers."""
import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


class Receiver(Node):
    def __init__(self):
        super().__init__('follower_convoy_rx_check')
        self.sample_start = time.monotonic()
        self.records = {}
        self.subscriptions_kept = []
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        for topic, kind in [('/leader/odom', Odometry), ('/leader/path', Path),
                            ('/follower/odom_map', Odometry)]:
            self.records[topic] = {'rx': [], 'stamps': [], 'ages': [], 'frames': set(),
                                   'latest': None}
            self.subscriptions_kept.append(self.create_subscription(
                kind, topic, lambda msg, name=topic: self.receive(name, msg), qos))

    def receive(self, topic, msg):
        row = self.records[topic]
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        row['rx'].append(time.monotonic())
        row['stamps'].append(stamp)
        row['ages'].append((self.get_clock().now().nanoseconds - stamp) / 1e9)
        row['frames'].add(msg.header.frame_id)
        row['latest'] = msg

    def report(self):
        for topic, row in self.records.items():
            times, stamps, ages = row['rx'], row['stamps'], row['ages']
            print(f'\n{topic}: received={len(times)}, frames={sorted(row["frames"])}')
            if not times:
                print('  NO DATA (not proof of a specific network fault)')
                continue
            gaps = [b-a for a, b in zip(times, times[1:])]
            rate = (len(times)-1)/(times[-1]-times[0]) if gaps and times[-1]>times[0] else 0.
            print(f'  observed_rate={rate:.3f}Hz, last_rx_age={time.monotonic()-times[-1]:.3f}s')
            if gaps:
                print(f'  rx_gap min/p95/max={min(gaps):.4f}/{percentile(gaps,.95):.4f}/{max(gaps):.4f}s')
                worst = gaps.index(max(gaps))
                print(f'  worst_gap elapsed={times[worst]-self.sample_start:.3f}'
                      f'..{times[worst+1]-self.sample_start:.3f}s after sampling start')
            print(f'  header_age min/p95/max={min(ages):.4f}/{percentile(ages,.95):.4f}/{max(ages):.4f}s')
            print(f'  zero_stamp={sum(s==0 for s in stamps)}, age>0.5s={sum(a>.5 for a in ages)}, '
                  f'future>0.1s={sum(a<-.1 for a in ages)}')
            print(f'  repeated_stamp={sum(b==a for a,b in zip(stamps,stamps[1:]))}, '
                  f'backward_stamp={sum(b<a for a,b in zip(stamps,stamps[1:]))}')
            stable = [(t,a) for t,a in zip(times,ages) if t-self.sample_start>=3.0]
            if len(stable)>1:
                stable_gaps = [b[0]-a[0] for a,b in zip(stable,stable[1:])]
                stable_ages = [a for _,a in stable]
                print(f'  AFTER FIRST 3s: received={len(stable)}, '
                      f'gap p95/max={percentile(stable_gaps,.95):.4f}/{max(stable_gaps):.4f}s, '
                      f'age p95/max={percentile(stable_ages,.95):.4f}/{max(stable_ages):.4f}s, '
                      f'age>0.5s={sum(a>.5 for a in stable_ages)}')
            msg = row['latest']
            if isinstance(msg, Path):
                print(f'  latest_path_points={len(msg.poses)}')
                if msg.poses:
                    for label, point in [('first',msg.poses[0]),('last',msg.poses[-1])]:
                        p=point.pose.position
                        print(f'  {label}_point=({p.x:.4f},{p.y:.4f}), frame={point.header.frame_id}')
            else:
                p,q=msg.pose.pose.position,msg.pose.pose.orientation
                yaw=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                print(f'  child={msg.child_frame_id}, latest_pose=({p.x:.4f},{p.y:.4f}), yaw={yaw:.3f}deg')
        print('\nAges require synchronized clocks. Thresholds are diagnostics, not validated stop deadlines.')
        print('No sequence IDs: counts alone do not establish packet loss. No motor commands sent.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=10.)
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 1<=args.seconds<=60:
        parser.error('--seconds must be between 1 and 60')
    rclpy.init(args=[])
    node=Receiver()
    try:
        node.sample_start = time.monotonic()
        print(f'Sampling for {args.seconds:g}s; keep both robots stationary.',flush=True)
        end=time.monotonic()+args.seconds
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=min(.1,max(0.,end-time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
