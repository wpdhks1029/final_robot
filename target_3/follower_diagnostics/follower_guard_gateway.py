#!/usr/bin/env python3
"""Forward stamped guard TEST output only to an explicitly armed candidate session.

No arm/disarm services, serial access or real cmd_vel publishers. Default no node.
Both original measurement stamp and a new ordering/transmission stamp are retained.
"""
import argparse
import json
import math
import re
import time

from follower_motor_mock import parse_envelope,strict_pairs

PREFIX='/follower/motor_candidate'
INPUT='/follower/cmd_vel_guard_stamped_test'
OUTPUT=PREFIX+'/guard_command'


def wait_for_publishers(node,spin_once,timeout=12.,clock=time.monotonic):
    """Allow DDS discovery to converge, but never accept duplicate sources."""
    end=clock()+timeout;counts={}
    while clock()<end:
        spin_once()
        counts={topic:len(node.get_publishers_info_by_topic(topic)) for topic in [INPUT,PREFIX+'/status']}
        if any(n>1 for n in counts.values()):raise RuntimeError('Duplicate input publishers: '+str(counts))
        if all(n==1 for n in counts.values()):return counts
    raise RuntimeError('Publisher discovery timed out; inputs='+str(counts))


def parse_guard_envelope(text):
    if not isinstance(text,str) or len(text)>2048:raise ValueError('Invalid envelope size')
    obj=json.loads(text,object_pairs_hook=strict_pairs)
    keys={'version','epoch','frame','stamp_ns','source_stamp_ns','v','w'}
    if not isinstance(obj,dict) or set(obj)!=keys or type(obj['version']) is not int or obj['version']!=2:
        raise ValueError('Guard envelope version/fields mismatch')
    source=obj['source_stamp_ns']
    if type(source) is not int or source<=0:raise ValueError('Invalid original measurement stamp')
    legacy=obj.copy();del legacy['source_stamp_ns'];legacy['version']=1
    parse_envelope(json.dumps(legacy,allow_nan=False))
    return obj


class Gateway:
    def __init__(self):
        self.status=None;self.guard=None;self.last_tx=0;self.reason='WAIT_STATUS'

    def update_status(self,text,received):
        self.status=None
        try:
            s=json.loads(text,object_pairs_hook=strict_pairs)
            if not isinstance(s,dict) or not re.fullmatch('[0-9a-f]{16}',s.get('epoch','')):return
            self.status=(s,received)
        except (ValueError,TypeError,RecursionError):return

    def update_guard(self,msg,received):
        self.guard=None
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        t=msg.twist;v,w=t.linear.x,t.angular.z
        if msg.header.frame_id!='follower/base_link' or stamp<=0:return
        values=(v,w,t.linear.y,t.linear.z,t.angular.x,t.angular.y)
        if not all(math.isfinite(n) for n in values) or any(abs(n)>1e-9 for n in values[2:]):return
        if not 0<=v<=.1 or abs(w)>.2 or v<abs(w)*.560/2:return
        self.guard=(stamp,received,v,w)

    def envelope(self,now_ns,received):
        if self.status is None:self.reason='WAIT_STATUS';return None
        s,rx=self.status
        if not 0<=received-rx<=.3:self.reason='STATUS_STALE';return None
        if (s.get('mode') not in ('MEMORY_ONLY','HARDWARE_CANDIDATE') or s.get('input_kind')!='guard'
                or s.get('armed') is not True or s.get('enabled') is not True
                or s.get('fault') is not None or s.get('feedback_fresh') is not True):
            self.reason='NOT_ARMED_OR_NOT_READY';return None
        if self.guard is None:self.reason='GUARD_INVALID';return None
        stamp,rx,v,w=self.guard;arm=s.get('arm_ros_ns')
        if type(arm) is not int or not arm<=stamp<=now_ns or not 0<=(now_ns-stamp)/1e9<=.3:
            self.reason='SOURCE_STALE_FUTURE_OR_PREARM';return None
        if not 0<=received-rx<=.3:self.reason='GUARD_RX_STALE';return None
        if now_ns<=self.last_tx:self.reason='CLOCK_NOT_INCREASING';return None
        self.last_tx=now_ns;self.reason='FORWARD_TO_ARMED_CANDIDATE'
        return dict(version=2,epoch=s['epoch'],frame='follower/base_link',stamp_ns=now_ns,
                    source_stamp_ns=stamp,v=v,w=w)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');a=p.parse_args()
    if not a.run:p.print_help();return 0
    import rclpy
    from std_msgs.msg import String
    from geometry_msgs.msg import TwistStamped
    from rclpy.executors import ExternalShutdownException
    from follower_remote_qos import latest_state_qos
    rclpy.init(args=[]);node=rclpy.create_node('follower_guard_gateway');core=Gateway()
    try:
        status_sub=node.create_subscription(String,PREFIX+'/status',lambda m:core.update_status(m.data,time.monotonic()),1)
        guard_sub=node.create_subscription(TwistStamped,INPUT,lambda m:core.update_guard(m,time.monotonic()),latest_state_qos())
        node.get_logger().info('Waiting up to 12s for guard/status publishers; no commands sent while waiting')
        wait_for_publishers(node,lambda:rclpy.spin_once(node,timeout_sec=.1))
        if node.get_publishers_info_by_topic(OUTPUT):raise RuntimeError('Existing gateway publisher')
        pub=node.create_publisher(String,OUTPUT,10)
        last_log=0.
        def tick():
            nonlocal last_log
            now=time.monotonic();obj=core.envelope(node.get_clock().now().nanoseconds,now)
            if obj is not None:pub.publish(String(data=json.dumps(obj,allow_nan=False)))
            if now-last_log>2:
                node.get_logger().info(core.reason+'; gateway never arms or rearms');last_log=now
        timer=node.create_timer(.05,tick)
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0


if __name__=='__main__':raise SystemExit(main())
