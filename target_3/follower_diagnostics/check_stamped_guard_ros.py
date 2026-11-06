#!/usr/bin/env python3
"""New stamped guard ROS test, Domain217 localhost, no motor or real inputs."""
from follower_remote_qos import latest_state_qos
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');a=p.parse_args()
    if not a.run:p.print_help();return 0
    os.environ['ROS_DOMAIN_ID']='217';os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    from follower_stamped_preview import PURSUIT_TOPIC,GUARD_TOPIC,FRAME
    rclpy.init(args=[]);node=rclpy.create_node('stamped_guard_contract_check')
    child=None;stream=None;samples=[];sent=set();failures=[]
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        if any(n!=node.get_name() for n,ns in node.get_node_names_and_namespaces()):raise RuntimeError('Domain217 in use')
        cmd=node.create_publisher(TwistStamped,PURSUIT_TOPIC,10)
        scan=node.create_publisher(LaserScan,'/follower/scan',qos_profile_sensor_data)
        def receive(m):
            samples.append((time.monotonic(),m.twist.linear.x,m.twist.angular.z,
                            m.header.stamp.sec*10**9+m.header.stamp.nanosec,m.header.frame_id))
        subscription=node.create_subscription(TwistStamped,GUARD_TOPIC,receive,latest_state_qos())
        log=Path(__file__).parent/'logs'/f'stamped_guard_ros_{time.time_ns()}.log'
        log.parent.mkdir(exist_ok=True);stream=log.open('x')
        child=subprocess.Popen([sys.executable,'-B',str(Path(__file__).with_name('follower_obstacle_preview.py'))],
                               env=os.environ.copy(),stdout=stream,stderr=subprocess.STDOUT)
        end=time.monotonic()+6
        while not samples and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        if not samples:raise RuntimeError('Stamped guard not discovered; see '+str(log))
        cases=[('fresh_source_preserved',-.1,-.02,2.,False,True),
               ('stale_source_republish_stops',-1.,-.02,2.,False,False),
               ('future_source_stops',.1,-.02,2.,False,False),
               ('stale_scan_stops',-.1,-1.,2.,False,False),
               ('obstacle_overrides_forward',-.1,-.02,.3,False,False),
               ('recover',-.1,-.02,2.,False,True),
               ('source_loss_stops',-.1,-.02,2.,True,False)]
        print('ISOLATED Domain217 localhost. Stamped sources/scan only; no motor node.',flush=True)
        for name,cmd_age,scan_age,distance,omit,moving in cases:
            samples.clear();sent.clear();start=time.monotonic();next_send=start
            while time.monotonic()-start<1.5:
                if child.poll() is not None:raise RuntimeError('Guard exited')
                if time.monotonic()>=next_send:
                    now=node.get_clock().now().nanoseconds
                    if not omit:
                        m=TwistStamped();m.header.frame_id=FRAME
                        stamp=now+round(cmd_age*1e9);sent.add(stamp)
                        m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9
                        m.twist.linear.x=.1;cmd.publish(m)
                    s=LaserScan();s.header.frame_id=FRAME;stamp=now+round(scan_age*1e9);sent.add(stamp)
                    s.header.stamp.sec=stamp//10**9;s.header.stamp.nanosec=stamp%10**9
                    s.angle_min=0.;s.angle_increment=.01;s.range_min=.28;s.range_max=16.;s.ranges=[distance]
                    scan.publish(s);next_send=time.monotonic()+.05
                rclpy.spin_once(node,timeout_sec=.01)
            late=[r for r in samples if r[0]-start>=.9]
            ok=len(late)>=3 and all(r[4]==FRAME and (r[1]>.09 if moving else abs(r[1])<1e-9) for r in late)
            if moving:ok=ok and all(r[3] in sent and r[3]>0 for r in late)
            if name in ('stale_source_republish_stops','future_source_stops','stale_scan_stops','source_loss_stops'):
                ok=ok and all(r[3]==0 for r in late)
            print(('PASS ' if ok else 'FAIL ')+name,flush=True)
            if not ok:failures.append(name)
        print(f'RESULT: {len(cases)-len(failures)}/{len(cases)} passed. Failures={failures}')
        print('No real inputs/motor commands. Source TTL checks only, not physical stop latency.')
        return 1 if failures else 0
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        if stream is not None:stream.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
