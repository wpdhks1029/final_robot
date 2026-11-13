#!/usr/bin/env python3
"""Domain217 MEMORY end-to-end guard-envelope/gateway/drive check. No real motors."""
import argparse
import json
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
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from follower_guard_gateway import PREFIX,INPUT,OUTPUT
    rclpy.init(args=[]);node=rclpy.create_node('follower_guard_gateway_check')
    children=[];logs=[];status=[];wire=[];sent=set();passed=[]
    mode=dict(v=.05,age=.01)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if any(c.poll() is not None for c in children):raise RuntimeError('Child exited; inspect printed log paths')
            rclpy.spin_once(node,timeout_sec=.02)
    def wait(predicate,timeout=8.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():return
            spin(.02)
        raise RuntimeError('Condition timed out; latest='+str(status[-1] if status else None))
    def spawn(name,flags):
        directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
        path=directory/f'gateway_check_{name}_{time.time_ns()}.log';stream=path.open('x');logs.append(stream)
        children.append(subprocess.Popen([sys.executable,'-B',str(Path(__file__).with_name(name+'.py')),*flags],
                                         env=os.environ.copy(),stdout=stream,stderr=subprocess.STDOUT))
        print('CHILD_LOG='+str(path),flush=True)
    def expect(name,ok):
        if not ok:raise AssertionError(name)
        passed.append(name);print('PASS '+name,flush=True)
    def call(client):
        if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('Arm service unavailable')
        f=client.call_async(Trigger.Request());wait(lambda:f.done(),4.);return f.result()
    try:
        spin(2.)
        if any(n!=node.get_name() for n,ns in node.get_node_names_and_namespaces()):raise RuntimeError('Domain217 already in use')
        print('ISOLATED Domain217 localhost. Synthetic guard + gateway + MEMORY drive, no physical ports.',flush=True)
        source=node.create_publisher(TwistStamped,INPUT,10)
        def publish():
            stamp=node.get_clock().now().nanoseconds-round(mode['age']*1e9);sent.add(stamp)
            m=TwistStamped();m.header.frame_id='follower/base_link'
            m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9
            m.twist.linear.x=mode['v'];source.publish(m)
        timer=node.create_timer(.05,publish)
        status_sub=node.create_subscription(String,PREFIX+'/status',lambda m:status.append(json.loads(m.data)),20)
        wire_sub=node.create_subscription(String,OUTPUT,lambda m:wire.append(json.loads(m.data)),20)
        arm=node.create_client(Trigger,PREFIX+'/arm')
        spawn('follower_drive_node',['--run-memory','--enable-motion-input','--input-kind','guard'])
        wait(lambda:bool(status) and status[-1]['valid_feedback']>=6)
        spawn('follower_guard_gateway',['--run'])
        wait(lambda:len(node.get_publishers_info_by_topic(OUTPUT))==1)
        spin(.3)
        expect('gateway_does_not_arm_or_move',not status[-1]['armed'] and status[-1]['measured_rpm']==[0,0] and not wire)
        response=call(arm);expect('explicit_operator_arm_required',response.success)
        wait(lambda:status[-1]['measured_rpm']==[-3,3] and bool(wire))
        expect('guard_reaches_bounded_memory_drive',status[-1]['input_kind']=='guard')
        expect('source_and_transmission_stamps_retained',all(m['version']==2 and m['source_stamp_ns'] in sent
               and m['source_stamp_ns']<=m['stamp_ns'] for m in wire))
        mode['v']=0.
        wait(lambda:status[-1]['measured_rpm']==[0,0] and status[-1]['armed'])
        expect('fresh_zero_guard_stops_without_rearm',status[-1]['fault'] is None)
        mode['v']=.05;wait(lambda:status[-1]['measured_rpm']==[-3,3])
        expect('fresh_guard_can_resume_within_same_trial',status[-1]['armed'])
        mode['age']=1.
        wait(lambda:status[-1]['fault'] is not None and status[-1]['measured_rpm']==[0,0])
        expect('stale_source_republish_stops_and_latches',status[-1]['fault']=='COMMAND_TIMEOUT')
        mode['age']=.01;spin(.5)
        expect('fresh_source_cannot_auto_rearm',not status[-1]['armed'] and status[-1]['measured_rpm']==[0,0])
        expect('latched_fault_rearm_rejected',not call(arm).success)
        print(f'RESULT: {len(passed)}/{len(passed)} passed. MEMORY/ROS contract only; no physical stop validation.')
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.kill();child.wait()
        for stream in logs:stream.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
