"""One-process comparison recorder and TEST position odom publisher.

No motor transport/commands. Real EKF remains subscribed to RPM raw odom.
"""
import csv
import math
from pathlib import Path
import time

from follower_pulse_odom import Observer
from follower_position_preview import PositionPreview,TOPIC


def summary(rows):
    if not rows:return 'NO DATA'
    result=f'n={len(rows)}'
    if len(rows)>1:
        result+=f', max_rx_gap={max(b["rx_s"]-a["rx_s"] for a,b in zip(rows,rows[1:])):.4f}s'
    if rows[0]['x'] is not None:
        result+=f', delta_xy=({rows[-1]["x"]-rows[0]["x"]:.6f},{rows[-1]["y"]-rows[0]["y"]:.6f})m'
    angles=[r['yaw'] for r in rows if r.get('yaw') is not None]
    if len(angles)>1:
        delta=sum(math.atan2(math.sin(b-a),math.cos(b-a)) for a,b in zip(angles,angles[1:]))
        result+=f', delta_yaw={math.degrees(delta):.3f}deg'
    if rows[0].get('wz') is not None and rows[0].get('stamp_ns') is not None:
        integral=0.;skips=0
        for a,b in zip(rows,rows[1:]):
            dt=(b['stamp_ns']-a['stamp_ns'])/1e9
            if 0<dt<=.25:integral+=(a['wz']+b['wz'])*.5*dt
            else:skips+=1
        result+=f', integrated_wz={math.degrees(integral):.3f}deg, integration_skips={skips}'
    return result


def quaternion_yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class ComparisonObserver(Observer):
    def __init__(self):
        super().__init__()
        self.stream=None;self.finished=False;self.rows={};self.subs=[]
        self.position=PositionPreview();self.source_start=None
        try:
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu
            from rclpy.qos import QoSProfile,ReliabilityPolicy
            if self.node.resolve_topic_name(TOPIC)!=TOPIC:raise RuntimeError('Preview remap rejected')
            if self.node.get_publishers_info_by_topic(TOPIC):
                raise RuntimeError('Existing position preview publisher; stop the separate preview first')
            self.shadow=self.node.create_publisher(Odometry,TOPIC,20)
            qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
            directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
            self.path=directory/f'comparison_{time.time_ns()}.csv'
            self.stream=self.path.open('x',newline='')
            self.writer=csv.DictWriter(self.stream,fieldnames=['topic','rx_s','stamp_ns','frame','child','x','y','yaw','vx','wz'])
            self.writer.writeheader();self.start=time.monotonic()
            topics=[('/follower/odom_raw',Odometry),('/follower/odometry/filtered',Odometry),
                    (TOPIC,Odometry),('/follower/imu/data',Imu)]
            for topic,kind in topics:
                self.rows[topic]=[]
                self.subs.append(self.node.create_subscription(kind,topic,lambda m,t=topic:self.receive(t,m),qos))
            print('COMPARISON_CSV='+str(self.path),flush=True)
        except BaseException:
            self.close();raise

    def receive(self,topic,m):
        if self.finished:return
        row=dict(topic=topic,rx_s=time.monotonic()-self.start,
                 stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                 frame=m.header.frame_id,child='',x=None,y=None,yaw=None,vx=None,wz=None)
        if hasattr(m,'twist'):
            row.update(child=m.child_frame_id,x=m.pose.pose.position.x,y=m.pose.pose.position.y,
                       yaw=quaternion_yaw(m.pose.pose.orientation),
                       vx=m.twist.twist.linear.x,wz=m.twist.twist.angular.z)
        else:row.update(wz=m.angular_velocity.z)
        self.rows[topic].append(row);self.writer.writerow(row);self.stream.flush()

    def publish(self,core):
        fresh=super().publish(core)
        if fresh:
            if self.source_start is None:self.source_start=core.clock()
            row=dict(core.feedback,elapsed=core.clock()-self.source_start,feedback_count=core.valid_count)
            if self.position.update(row):
                m=self.message_type();m.header.stamp=self.node.get_clock().now().to_msg()
                m.header.frame_id='follower_position_preview/odom';m.child_frame_id='follower_position_preview/base_link'
                m.pose.pose.position.x=self.position.x;m.pose.pose.position.y=self.position.y
                m.pose.pose.orientation.z=math.sin(self.position.yaw/2);m.pose.pose.orientation.w=math.cos(self.position.yaw/2)
                m.twist.twist.linear.x=self.position.v;m.twist.twist.angular.z=self.position.w
                m.pose.covariance[0]=m.pose.covariance[7]=.05;m.pose.covariance[35]=.1
                m.twist.covariance[0]=m.twist.covariance[7]=.02;m.twist.covariance[35]=.05
                self.shadow.publish(m)
        # Bounded, nonblocking dispatch; no subprocesses or second serial owners.
        for _ in range(4):self.ros.spin_once(self.node,timeout_sec=0.)
        return fresh

    def finish_capture(self):
        if self.finished:return
        self.finished=True
        print('COMPARISON_SUMMARY (motion disabled; feedback continues):',flush=True)
        for topic,rows in self.rows.items():print(topic+': '+summary(rows),flush=True)
        print('COMPARISON_CSV='+str(self.path),flush=True)
        if self.stream is not None:self.stream.close();self.stream=None

    def close(self):
        if getattr(self,'stream',None) is not None:self.stream.close();self.stream=None
        super().close()
