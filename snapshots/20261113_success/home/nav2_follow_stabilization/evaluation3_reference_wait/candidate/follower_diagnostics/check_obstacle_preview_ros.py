#!/usr/bin/env python3
"""Opt-in Domain217 localhost synthetic scan/TF/command gate test; no hardware."""
from follower_remote_qos import latest_state_qos
import argparse
import math
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true')
    args=parser.parse_args()
    if not args.run:
        parser.print_help();return 0
    os.environ['ROS_DOMAIN_ID']='217'
    os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist,TransformStamped
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster
    from builtin_interfaces.msg import Time as Stamp

    rclpy.init(args=[])
    node=Node('follower_synthetic_obstacle_check')
    child=None;outputs=[];failures=[]
    try:
        print('ISOLATED Domain217 localhost-only. No real scan, command or motor interfaces.',flush=True)
        end=time.monotonic()+2.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        others=[name for name,ns in node.get_node_names_and_namespaces() if name!=node.get_name()]
        if others:raise RuntimeError(f'Domain217 has other nodes {others}; refusing test')
        for topic in ['/follower/scan','/follower/cmd_vel_test','/follower/cmd_vel_guard_test',
                      '/follower/cmd_vel','/follower/cmd_vel_raw','/tf_static']:
            if node.get_publishers_info_by_topic(topic) or node.get_subscriptions_info_by_topic(topic):
                raise RuntimeError(f'Existing Domain217 endpoint: {topic}')
        scan_pub=node.create_publisher(LaserScan,'/follower/scan',qos_profile_sensor_data)
        cmd_pub=node.create_publisher(Twist,'/follower/cmd_vel_test',10)
        sub=node.create_subscription(Twist,'/follower/cmd_vel_guard_test',
            lambda msg:outputs.append((time.monotonic(),msg.linear.x,msg.angular.z)),latest_state_qos())
        broadcaster=StaticTransformBroadcaster(node)
        child=subprocess.Popen([sys.executable,'-B',str(Path(__file__).with_name('follower_obstacle_preview.py'))],
                               env=os.environ.copy(),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        end=time.monotonic()+6.
        while time.monotonic()<end and not outputs:
            if child.poll() is not None:raise RuntimeError('Preview exited during startup')
            rclpy.spin_once(node,timeout_sec=.1)
        if not outputs:raise RuntimeError('No guard output discovered')

        def send_tf():
            tf=TransformStamped();tf.header.stamp=node.get_clock().now().to_msg()
            tf.header.frame_id='follower/base_link';tf.child_frame_id='follower/laser'
            tf.transform.translation.x=.26;tf.transform.translation.z=.24
            tf.transform.rotation.w=1.
            broadcaster.sendTransform(tf)

        def publish(config):
            if config.get('omit')!='command':
                cmd=Twist();cmd.linear.x=config.get('v',.1);cmd.angular.z=config.get('w',0.)
                cmd_pub.publish(cmd)
            if config.get('omit')=='scan':return
            msg=LaserScan()
            ns=node.get_clock().now().nanoseconds+config.get('stamp_offset_ns',0)
            msg.header.stamp=Stamp(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
            msg.header.frame_id=config.get('frame','follower/laser')
            msg.angle_min=config.get('angle',0.);msg.angle_max=msg.angle_min
            msg.angle_increment=.01;msg.range_min=.28;msg.range_max=16.
            msg.ranges=[config.get('r',2.)]
            scan_pub.publish(msg)

        stop=lambda v,w:abs(v)<1e-9 and abs(w)<1e-9
        full=lambda v,w:abs(v-.1)<1e-6 and abs(w)<1e-9
        cases=[
            ('missing_TF',{},stop),
            ('clear_pass',{},full),
            ('front_slow',{'r':.5},lambda v,w:abs(v-.0633333333)<1e-5 and abs(w)<1e-9),
            ('front_stop',{'r':.30},stop),
            ('side_stop',{'r':.5,'angle':math.pi/2},stop),
            ('inside_footprint',{'r':.315,'angle':math.radians(-141)},stop),
            ('upstream_zero_near_obstacle',{'r':.30,'v':0.},stop),
            ('all_invalid_scan',{'r':float('inf')},stop),
            ('wrong_scan_frame',{'frame':'wrong'},stop),
            ('stale_scan_stamp',{'stamp_offset_ns':-1_000_000_000},stop),
            ('future_scan_stamp',{'stamp_offset_ns':1_000_000_000},stop),
            ('recover_before_command_loss',{},full),
            ('command_loss',{'omit':'command'},stop),
            ('recover_before_scan_loss',{},full),
            ('scan_loss',{'omit':'scan'},stop),
            ('final_recovery',{},full),
        ]
        for index,(name,config,check) in enumerate(cases):
            if index==1:send_tf()
            outputs.clear();start=time.monotonic();next_pub=start
            while time.monotonic()-start<1.5:
                if child.poll() is not None:raise RuntimeError('Preview exited during test')
                now=time.monotonic()
                if now>=next_pub:publish(config);next_pub=now+.05
                rclpy.spin_once(node,timeout_sec=.01)
            samples=[(v,w) for t,v,w in outputs if t-start>=1.]
            passed=len(samples)>=3 and all(check(v,w) for v,w in samples)
            print(f'{"PASS" if passed else "FAIL"} {name}: evaluated={len(samples)}, '
                  f'last={samples[-1] if samples else None}',flush=True)
            if not passed:failures.append(name)
        print(f'RESULT: {len(cases)-len(failures)}/{len(cases)} passed. Failures={failures}',flush=True)
        print('Synthetic ROS/QoS/TF integration only. Not physical braking, blind-area coverage or exact stop latency.')
        return 1 if failures else 0
    except (Exception,KeyboardInterrupt) as exc:
        print(f'Test stopped: {exc}',file=sys.stderr);return 1
    finally:
        if child is not None:
            child.terminate()
            try:logs,_=child.communicate(timeout=4)
            except subprocess.TimeoutExpired:
                child.kill();logs,_=child.communicate()
            if child.returncode not in (0,-15):print('Preview exit/log:',child.returncode,logs[-2500:])
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':sys.exit(main())
