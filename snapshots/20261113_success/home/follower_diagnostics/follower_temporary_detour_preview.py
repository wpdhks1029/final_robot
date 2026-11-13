#!/usr/bin/env python3
"""Explicit bounded temporary-path candidate; existing Gateway/driver required."""
import argparse
import hashlib
import json
import math
import os
import time
from pathlib import Path

from apf_case3_candidate import Settings, clearance, wrap
from follower_pursuit_preview import PreviewCore
from follower_stamped_preview import stamp_ns, oldest_pursuit_source
from temporary_detour_candidate import Pose, pursue, to_reference
from watch_temporary_detour_shadow import PreparedPathReview

ROOT=Path(__file__).resolve().parent
PLAN=ROOT/'logs/current_scene_left_detour_plan.json'
KIND='TEMPORARY_DETOUR_BOUNDED_V2'
NODE='follower_temporary_detour_preview'
TOPICS=('/follower/cmd_vel_test','/follower/cmd_vel_stamped_test',
        '/follower/cmd_vel_guard_test','/follower/cmd_vel_guard_stamped_test')
STATUS='/follower/obstacle_preview/status'


def load_plan(path):
    raw=Path(path).read_bytes();data=json.loads(raw)
    if data.get('reason')!='DETOUR_PLANNED' or data.get('box_forward_m',0)!=0:
        raise ValueError('Unchanged-scene checked plan required')
    PreparedPathReview(data['path'])
    values=[*data['snapshot']['reference_origin'],data['snapshot']['reference_heading']]
    if len(values)!=3 or not all(math.isfinite(v) for v in values):raise ValueError('Invalid reference')
    return data,hashlib.sha256(raw).hexdigest()


def transform_history_qos():
    # /tf multiplexes many child frames; depth 1 can discard the required
    # odom->base update when another frame arrives during path computation.
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    return QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)


class AwaitTransform(Exception):
    """The scan is valid but its measured-time TF has not arrived yet."""


class DeferredScan:
    def __init__(self,core):
        self.core=core;self.pending=None;self.last_stamp=0;self.detail=None

    def invalidate(self,detail):
        self.pending=None;self.detail=detail
        self.core.scan=None;self.core.review=None;self.core.review_stamp=None

    def offer(self,stamp,rx,payload,now):
        if stamp<=self.last_stamp or not 0<=(now-stamp)/1e9<=.3:
            self.invalidate('STOP_SCAN_STALE_OR_REORDERED');return
        self.last_stamp=stamp;self.pending=(stamp,rx,payload)

    def retry(self,now,mono,resolve):
        if self.pending is None:return
        stamp,rx,payload=self.pending
        if not (0<=(now-stamp)/1e9<=.3 and 0<=mono-rx<=.3):
            self.invalidate('STOP_SCAN_TF_WAIT_EXPIRED');return
        try:
            points,body=resolve(payload)
            self.core.scan_update(stamp,rx,points,body)
            self.pending=None;self.detail=None
        except AwaitTransform as exc:
            # Keep only the previous checked scan, with its original source and RX
            # times. evaluate() still enforces the unchanged 0.3 s bound.
            self.detail='WAIT_MEASURED_TIME_TF: '+str(exc)
        except ValueError as exc:self.invalidate(str(exc))


class DetourCore:
    def __init__(self,data,plan_hash,trial_seconds=5):
        if type(trial_seconds) is not int or trial_seconds not in (5,30):raise ValueError("Trial must be 5 or 30 seconds")
        self.trial_seconds=trial_seconds
        self.data=data;self.plan_hash=plan_hash;self.nav=PreviewCore()
        self.reviewer=PreparedPathReview(data['path']);self.settings=Settings()
        self.origin=data['snapshot']['reference_origin'];self.heading=data['snapshot']['reference_heading']
        self.scan=None;self.review=None;self.review_stamp=None;self.index=0
        self.motor=None;self.armed_at=None;self.epoch=None;self.retired=None
        self.initial_checked=False;self.previous_pose=None

    def local_pose(self,pose):
        co,si=math.cos(self.heading),math.sin(self.heading)
        dx,dy=pose.x-self.origin[0],pose.y-self.origin[1]
        return Pose(co*dx+si*dy,-si*dx+co*dy,wrap(pose.yaw-self.heading))

    def motor_update(self,state,mono):
        self.motor=(state,mono)
        if state.get('armed') is True:
            if not self.initial_checked:self.retired='STOP_ARMED_BEFORE_PLAN_READY'
            if self.armed_at is None:
                self.armed_at=mono;self.epoch=state.get('epoch')
            elif state.get('epoch')!=self.epoch:self.retired='STOP_SESSION_CHANGED'
        elif self.armed_at is not None:self.retired='STOP_SESSION_FINISHED'

    def scan_update(self,stamp,rx,points,body_points):
        # Clear old geometry first, including rejected or out-of-order packets.
        previous=self.scan;self.scan=None;self.review=None;self.review_stamp=None
        if previous is not None and stamp<=previous[0]:raise ValueError('STOP_SCAN_ORDER_INVALID')
        if not points or not body_points or not all(math.isfinite(v) for p in points+body_points for v in p):
            raise ValueError('STOP_SCAN_INVALID')
        self.scan=(stamp,rx,points,min(clearance(x,y,self.settings) for x,y in body_points))

    def evaluate(self,now,mono):
        row=dict(controller_kind=KIND,plan_sha256=self.plan_hash,trial_max_seconds=self.trial_seconds,
                 reason='STOP_NOT_READY',source_stamp_ns=0,output_v=0.,output_w=0.,path_index=self.index)
        def stop(reason):row['reason']=reason;return row
        if self.retired:return stop(self.retired)
        if self.armed_at is not None and mono-self.armed_at>=self.trial_seconds-.25:
            self.retired='STOP_LOCAL_TRIAL_TIME_LIMIT';return stop(self.retired)
        if self.motor is None:return stop('STOP_MOTOR_MISSING')
        state,rx=self.motor
        if (not 0<=mono-rx<=.3 or state.get('mode')!='HARDWARE_CANDIDATE'
                or state.get('input_kind')!='guard' or state.get('trial_max_rpm')!=10
                or state.get('trial_max_seconds')!=self.trial_seconds or state.get('enabled') is not True
                or state.get('fault') or state.get('io_failed') is not False
                or state.get('feedback_fresh') is not True):return stop('STOP_REQUIRE_FRESH_MATCHING_GUARD_DRIVER')
        if self.armed_at is None and (state.get('armed') is not False or state.get('measured_rpm')!=[0,0]):
            return stop('STOP_REQUIRE_DISARMED_ZERO')
        for name in ('path','leader','follower'):
            item=self.nav.inputs.get(name)
            if item is None:return stop('STOP_'+name.upper()+'_MISSING')
            source=stamp_ns(item[0])-(50_000_000 if name in ('leader','path') else 0)
            if not (source>0 and 0<=(now-source)/1e9<=.3 and 0<=mono-item[1]<=.3):
                return stop('STOP_'+name.upper()+'_STALE')
        if self.scan is None:return stop('STOP_SCAN_OR_TF_INVALID')
        stamp,rx,points,original_clearance=self.scan
        source=min(stamp,oldest_pursuit_source(self.nav.inputs,remote_clock_margin_ns=50_000_000))
        row['source_stamp_ns']=source
        if not (source>0 and 0<=(now-source)/1e9<=.3 and 0<=mono-rx<=.3):return stop('STOP_SCAN_OR_SOURCE_STALE')
        pose=self.local_pose(Pose(*self.nav.inputs['follower'][2][:3]))
        if not self.initial_checked:
            first=self.data['path'][0]
            row.update(initial_position_error_m=math.hypot(pose.x-first[0],pose.y-first[1]),
                       initial_heading_error_deg=math.degrees(wrap(pose.yaw-first[2])),
                       current_reference_pose=[pose.x,pose.y,pose.yaw],plan_start_pose=list(first[:3]))
            if row['initial_position_error_m']>.05 or abs(row['initial_heading_error_deg'])>5:
                return stop('STOP_INITIAL_PLAN_POSE_MISMATCH')
            self.initial_checked=True
        previous=self.previous_pose
        if previous is not None and (math.hypot(pose.x-previous.x,pose.y-previous.y)>.10 or abs(wrap(pose.yaw-previous.yaw))>math.radians(10)):
            self.retired='STOP_LOCALIZATION_JUMP';return stop(self.retired)
        self.previous_pose=pose
        if self.review_stamp!=stamp:
            self.review=self.reviewer.review(points,self.index);self.review_stamp=stamp
        row.update(self.review)
        if row['reason']!='LIVE_PATH_GEOMETRY_CLEAR':return row
        speed,_,gate=self.nav.speed_gate(now,mono)
        if speed<=0:return stop(gate)
        v,w,index,tracking=pursue(self.data['path'],pose,min(.06,speed),self.index,.15)
        self.index=index;row.update(path_index=index,tracking_reason=tracking)
        if not tracking.startswith('DETOUR_TRACK'):return stop(tracking)
        # All returns are retained; compensate scan points to the current body pose.
        co,si=math.cos(pose.yaw),math.sin(pose.yaw)
        local=[(co*(x-pose.x)+si*(y-pose.y),-si*(x-pose.x)+co*(y-pose.y)) for x,y in points]
        d=min(original_clearance,min(clearance(x,y,self.settings) for x,y in local))
        row['clearance_m']=d
        if d<=.30:return stop('STOP_FOOTPRINT_CLEARANCE')
        factor=min(1.,(d-.30)/.30);v*=factor;w*=factor
        # Independently check the commanded one-second swept rectangular body.
        for k in range(1,11):
            t=k*.1;angle=w*t
            x=v*t if abs(w)<1e-9 else v/w*math.sin(angle)
            y=0. if abs(w)<1e-9 else v/w*(1-math.cos(angle))
            co,si=math.cos(angle),math.sin(angle)
            movement=(v+math.hypot(.27,.30)*abs(w))*.1
            if any(clearance(co*(px-x)+si*(py-y),-si*(px-x)+co*(py-y),self.settings)<=.30+movement for px,py in local):
                return stop('STOP_SWEPT_FOOTPRINT')
        rpm=max(abs(v-w*.28),abs(v+w*.28))/.065*60/(2*math.pi)
        if not (all(math.isfinite(n) for n in (v,w)) and 0<v<=.06 and abs(w)<=.2 and v>=abs(w)*.28 and rpm<=10.+1e-9):
            return stop('STOP_DRIVE_BOUNDS')
        row.update(reason='DETOUR_READY',output_v=v,output_w=w,max_wheel_rpm=rpm)
        return row


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true');p.add_argument('--ack-5s-only',action='store_true')
    p.add_argument('--ack-bounded-trial',action='store_true')
    p.add_argument('--trial-seconds',type=int,choices=(5,30),default=5)
    p.add_argument('--plan',type=Path,default=PLAN);a=p.parse_args()
    if not a.run:p.print_help();return
    if not (a.ack_bounded_trial or (a.ack_5s_only and a.trial_seconds==5)) or os.environ.get('ROS_DOMAIN_ID')!='10':
        p.error('Explicit bounded trial acknowledgement and ROS_DOMAIN_ID=10 required')
    data,digest=load_plan(a.plan);core=DetourCore(data,digest,a.trial_seconds)
    import rclpy
    from rclpy.time import Time
    from geometry_msgs.msg import Twist,TwistStamped
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener,TransformException
    from follower_remote_qos import latest_state_qos
    from convoy_preview_preflight import wait_for_inputs
    from watch_apf_case3 import project
    rclpy.init(args=[]);node=rclpy.create_node(NODE)
    pubs=[];status_pub=None
    try:
        wait_for_inputs(node,lambda:rclpy.spin_once(node,timeout_sec=.1))
        pubs=[node.create_publisher(TwistStamped if 'stamped' in t else Twist,t,latest_state_qos()) for t in TOPICS]
        status_pub=node.create_publisher(String,STATUS,latest_state_qos())
        instance=f'{os.getpid()}-{time.time_ns()}'
        started_ros_ns=node.get_clock().now().nanoseconds
        buffer=Buffer();listener=TransformListener(buffer,node,qos=transform_history_qos())
        deferred=DeferredScan(core)
        def resolve_scan(m):
            try:
                mount=buffer.lookup_transform('follower/base_link','follower/laser',Time.from_msg(m.header.stamp))
                tf=buffer.lookup_transform('map','follower/base_link',Time.from_msg(m.header.stamp))
            except TransformException as exc:
                if 'future' in str(exc).lower():raise AwaitTransform(str(exc)) from exc
                raise ValueError('STOP_SCAN_TF_INVALID: '+str(exc)) from exc
            q=tf.transform.rotation;t=tf.transform.translation
            if (tf.header.frame_id!='map' or tf.child_frame_id!='follower/base_link'
                    or not all(math.isfinite(v) for v in (t.x,t.y,t.z,q.x,q.y,q.z,q.w))
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01 or abs(q.x)>.01 or abs(q.y)>.01):
                raise ValueError('STOP_MAP_TF_INVALID')
            body=project(m,mount);yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
            points=to_reference(body,Pose(t.x,t.y,yaw),core.origin,core.heading)
            return points,body
        def scan_cb(m):
            now=node.get_clock().now().nanoseconds;rx=time.monotonic()
            vals=(m.angle_min,m.angle_increment,m.range_min,m.range_max)
            if (m.header.frame_id!='follower/laser' or not all(math.isfinite(v) for v in vals)
                    or m.angle_increment==0 or not 0<m.range_min<m.range_max or len(m.ranges)>50000
                    or not any(math.isfinite(r) and m.range_min<=r<=m.range_max for r in m.ranges)):
                deferred.invalidate('STOP_SCAN_INVALID');return
            deferred.offer(stamp_ns(m.header.stamp),rx,m,now)
            deferred.retry(now,rx,resolve_scan)
        def motor_cb(m):
            try:core.motor_update(json.loads(m.data),time.monotonic())
            except (ValueError,TypeError):core.motor=None
        subs=[node.create_subscription(LaserScan,'/follower/scan',scan_cb,latest_state_qos()),
              node.create_subscription(String,'/follower/motor_candidate/status',motor_cb,latest_state_qos()),
              node.create_subscription(RosPath,'/leader/path',lambda m:core.nav.path_callback(m,node.get_clock().now().nanoseconds,time.monotonic()),latest_state_qos())]
        for key,topic in [('leader','/leader/odom'),('follower','/follower/odom_map')]:
            subs.append(node.create_subscription(Odometry,topic,lambda m,k=key:core.nav.odom_callback(k,m,node.get_clock().now().nanoseconds,time.monotonic()),latest_state_qos()))
        last_report=0.
        def tick():
            nonlocal last_report
            deferred.retry(node.get_clock().now().nanoseconds,time.monotonic(),resolve_scan)
            row=core.evaluate(node.get_clock().now().nanoseconds,time.monotonic())
            row.update(preview_instance=instance,preview_started_ros_ns=started_ros_ns,
                       armed_observed_mono=core.armed_at,session_epoch=core.epoch,
                       scan_tf_detail=deferred.detail,pending_scan_source_ns=None if deferred.pending is None else deferred.pending[0])
            # Fail closed on duplicate control writers; no graph-cache grace while moving.
            for topic in ('/leader/odom','/leader/path','/follower/odom_map','/follower/scan'):
                if len(node.get_publishers_info_by_topic(topic))!=1:
                    row.update(reason='STOP_DUPLICATE_OR_MISSING_INPUT',output_v=0.,output_w=0.)
            for topic in TOPICS+(STATUS,):
                writers=node.get_publishers_info_by_topic(topic)
                if len(writers)!=1 or writers[0].node_name!=NODE:
                    row.update(reason='STOP_DUPLICATE_OR_MISSING_WRITER',output_v=0.,output_w=0.)
            now=node.get_clock().now().nanoseconds;source=row['source_stamp_ns']
            if row['output_v']>0 and (not source>0 or not 0<=(now-source)/1e9<=.3):
                row.update(reason='STOP_INPUT_AGED_DURING_REVIEW',output_v=0.,output_w=0.)
            row.update(stamp_ns=now,plain_reason=row['reason'],stamped_reason=row['reason'],
                       source_age_s=(now-source)/1e9 if source>0 else None,apf_stage=KIND)
            msg=Twist();msg.linear.x=row['output_v'];msg.angular.z=row['output_w']
            stamped=TwistStamped();stamped.header.frame_id='follower/base_link'
            stamped.header.stamp.sec=source//10**9;stamped.header.stamp.nanosec=source%10**9;stamped.twist=msg
            for publisher,topic in zip(pubs,TOPICS):publisher.publish(stamped if 'stamped' in topic else msg)
            out=String();out.data=json.dumps(row,allow_nan=False);status_pub.publish(out)
            if time.monotonic()-last_report>=1.:
                node.get_logger().info(out.data);last_report=time.monotonic()
        timer=node.create_timer(.025,tick)
        node.get_logger().info(f'임시 경로 {a.trial_seconds}초 후보. Gateway 연결됨. 자체 arm 없음. plan_sha256='+digest)
        rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            for publisher,topic in zip(pubs,TOPICS):
                msg=TwistStamped() if 'stamped' in topic else Twist()
                if 'stamped' in topic:msg.header.frame_id='follower/base_link'
                publisher.publish(msg)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
