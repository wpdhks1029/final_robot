#!/usr/bin/env python3
"""Read live feedback JSONL only; publish assumed-60-unit odom to TEST topic.

No serial, motor commands, TF or real odom output. Frames are preview-only.
No stationary mask: actual position changes remain visible, even at zero RPM.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

from follower_feedback_trace import delta32

TOPIC='/follower/odom_position_test'


def latest_trace(directory):
    paths=list(Path(directory).glob('measurement_feedback_*.jsonl'))
    if not paths:raise ValueError('No measurement feedback trace found')
    return max(paths,key=lambda p:p.stat().st_mtime_ns)


class PositionPreview:
    def __init__(self):
        self.previous=None;self.history=deque();self.left=self.right=0.
        self.x=self.y=self.yaw=0.;self.v=self.w=0.
        self.gaps=0;self.accepted=0

    def update(self,row):
        t=float(row['elapsed']);positions=(row['m1_position'],row['m2_position'])
        if (not math.isfinite(t) or any(type(p) is not int or not -2**31<=p<2**31 for p in positions)
                or row['m1_status']!=0 or row['m2_status']!=0):
            raise ValueError('Invalid position/time/device status')
        if self.previous is not None:
            old=self.previous
            if t<=old['elapsed'] or row['feedback_count']<=old['feedback_count']:
                raise ValueError('Repeated/backward source sample')
            dt=t-old['elapsed']
            if dt>.3:
                # Do not manufacture distance/velocity across unknown motion.
                self.gaps+=1;self.history.clear();self.previous=row.copy()
                self.v=self.w=0.
                return False
            dl=-delta32(positions[0],old['m1_position'])*math.pi*.130/60
            dr=delta32(positions[1],old['m2_position'])*math.pi*.130/60
            if abs(dl)>.1 or abs(dr)>.1:raise ValueError('Implausible per-sample displacement')
            self.left+=dl;self.right+=dr
            ds=(dl+dr)/2;dyaw=(dr-dl)/.560
            self.x+=ds*math.cos(self.yaw+dyaw/2)
            self.y+=ds*math.sin(self.yaw+dyaw/2)
            self.yaw=math.atan2(math.sin(self.yaw+dyaw),math.cos(self.yaw+dyaw))
        self.previous=row.copy();self.history.append((t,self.left,self.right))
        while len(self.history)>1 and self.history[1][0]<=t-.5:self.history.popleft()
        first=self.history[0];span=t-first[0]
        # Warmup is not interpreted as zero measured velocity; no output until ready.
        if span<.45:return False
        vl=(self.left-first[1])/span;vr=(self.right-first[2])/span
        self.v=(vl+vr)/2;self.w=(vr-vl)/.560
        self.accepted+=1
        return True


class PositionPublisher:
    """Attach TEST output to the existing serial owner without a second port."""
    topic=TOPIC
    odom_frame='follower_position_preview/odom'
    base_frame='follower_position_preview/base_link'

    def __init__(self,node):
        from nav_msgs.msg import Odometry
        if node.resolve_topic_name(self.topic)!=self.topic:raise RuntimeError('Position output remap rejected')
        if node.get_publishers_info_by_topic(self.topic):raise RuntimeError('Existing position output publisher')
        self.node=node;self.message=Odometry;self.pub=node.create_publisher(Odometry,self.topic,20)
        self.core=PositionPreview();self.previous=0;self.start=None

    def publish(self,source,stamp):
        if source.valid_count==self.previous:return False
        self.previous=source.valid_count
        if self.start is None:self.start=source.clock()
        row=dict(source.feedback,elapsed=source.clock()-self.start,feedback_count=source.valid_count)
        if not self.core.update(row):return False
        m=self.message();m.header.stamp=stamp
        m.header.frame_id=self.odom_frame;m.child_frame_id=self.base_frame
        m.pose.pose.position.x=self.core.x;m.pose.pose.position.y=self.core.y
        m.pose.pose.orientation.z=math.sin(self.core.yaw/2);m.pose.pose.orientation.w=math.cos(self.core.yaw/2)
        m.twist.twist.linear.x=self.core.v;m.twist.twist.angular.z=self.core.w
        m.pose.covariance[0]=m.pose.covariance[7]=.05;m.pose.covariance[35]=.1
        m.twist.covariance[0]=m.twist.covariance[7]=.02;m.twist.covariance[35]=.05
        self.pub.publish(m);return True


class PositionCandidatePublisher(PositionPublisher):
    """Opt-in native follower frames; candidate input, never replaces RPM raw output."""
    topic='/follower/odom_position_candidate'
    odom_frame='follower/odom'
    base_frame='follower/base_link'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    choice=p.add_mutually_exclusive_group(required=True)
    choice.add_argument('--trace',type=Path)
    choice.add_argument('--latest',action='store_true',help='Select newest measurement trace once at startup; never switch files')
    p.add_argument('--seconds',type=int,default=30)
    args=p.parse_args()
    if not 5<=args.seconds<=120:p.error('seconds must be 5..120')
    if args.latest:args.trace=latest_trace(Path(__file__).parent/'logs')
    print('TRACE='+str(args.trace),flush=True)
    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init(args=[]);node=rclpy.create_node('follower_position_preview')
    core=PositionPreview();stale=invalid=0;v_values=[];last_warning=0.
    try:
        if node.resolve_topic_name(TOPIC)!=TOPIC:raise RuntimeError('Preview topic remap rejected')
        pub=node.create_publisher(Odometry,TOPIC,20)
        # Start at end: old motion is never replayed as live measurements.
        with args.trace.open() as source:
            source.seek(0,2)
            print(f'TEST ONLY: fresh appended rows -> {TOPIC}; assumed 60 units/rev; no TF/motor access.',flush=True)
            until=time.monotonic()+args.seconds
            while time.monotonic()<until:
                for _ in range(50):
                    offset=source.tell();line=source.readline()
                    if not line:break
                    if not line.endswith('\n'):source.seek(offset);break
                    try:
                        row=json.loads(line)
                        age=(node.get_clock().now().nanoseconds-row['stamp_ns'])/1e9
                        if not -.1<=age<=.3:stale+=1;continue
                        if not core.update(row):continue
                        m=Odometry();stamp=row['stamp_ns']
                        m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9
                        m.header.frame_id='follower_position_preview/odom'
                        m.child_frame_id='follower_position_preview/base_link'
                        m.pose.pose.position.x=core.x;m.pose.pose.position.y=core.y
                        m.pose.pose.orientation.z=math.sin(core.yaw/2);m.pose.pose.orientation.w=math.cos(core.yaw/2)
                        m.twist.twist.linear.x=core.v;m.twist.twist.angular.z=core.w
                        # Trial values only; not calibrated uncertainty, never an EKF input.
                        m.pose.covariance[0]=m.pose.covariance[7]=.05;m.pose.covariance[35]=.1
                        m.twist.covariance[0]=m.twist.covariance[7]=.02;m.twist.covariance[35]=.05
                        pub.publish(m);v_values.append(core.v)
                    except (ValueError,KeyError,TypeError) as e:
                        invalid+=1
                        if time.monotonic()-last_warning>2:
                            print('Rejected sample: '+str(e),flush=True);last_warning=time.monotonic()
                rclpy.spin_once(node,timeout_sec=0.)
                time.sleep(.02)
    except KeyboardInterrupt:pass
    finally:
        print(f'published={core.accepted}, gaps={core.gaps}, stale={stale}, invalid={invalid}')
        print(f'position_delta_m=({core.x:.6f},{core.y:.6f}), yaw_rad={core.yaw:.6f}')
        if v_values:print(f'window_velocity_min/max={min(v_values):.6f}/{max(v_values):.6f}m/s')
        print('No fake zero/stationary suppression. New topic stops when source stops; no real odom changed.')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
