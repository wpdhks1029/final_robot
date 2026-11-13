#!/usr/bin/env python3
"""Test-only rectangular-footprint scan gate; no APF steering or motor output.

Clearances are trial margins, not braking certification. The scan cannot cover
sensor blind areas, all heights, or a validated swept collision volume.
"""
import math
import json
from pathlib import Path
import time

INPUT_TOPIC = '/follower/cmd_vel_test'
OUTPUT_TOPIC = '/follower/cmd_vel_guard_test'
BASE = 'follower/base_link'
LASER = 'follower/laser'
STATUS_TOPIC='/follower/obstacle_preview/status'


def clearance_zone(core):
    if core.scan is None:return 'invalid'
    d=core.scan[2]
    return 'inside' if d<=0 else 'stop' if d<=core.hard else 'slow' if d<core.slow else 'clear'


def status_event_key(core,reason,stamped_reason,source_ns,v,w):
    return (reason,stamped_reason,source_ns<=0,v==0 and w==0,clearance_zone(core))


def guard_status(core,now_ns,plain_reason,stamped_reason,source_ns,v,w):
    scan=core.scan
    return dict(version=1,stamp_ns=now_ns,plain_reason=plain_reason,
                stamped_reason=stamped_reason,source_stamp_ns=source_ns,
                source_age_s=None if source_ns<=0 else (now_ns-source_ns)/1e9,
                clearance_m=None if scan is None else scan[2],
                scan_age_s=None if scan is None else (now_ns-scan[0])/1e9,
                scan_source_stamp_ns=None if scan is None else scan[0],
                clearance_zone=clearance_zone(core),
                nearest_points=[] if scan is None or scan[2]>core.hard else [
                    dict(clearance_m=d,index=i,scan_angle_deg=a,raw_range_m=r,base_x_m=x,base_y_m=y)
                    for d,i,a,r,x,y in core.nearest_points],
                output_v=v,output_w=w)


def validate_output_topic(topic):
    if topic != OUTPUT_TOPIC:
        raise RuntimeError('Only /follower/cmd_vel_guard_test output is permitted')


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


class ObstacleCore:
    def __init__(self, body_width=.60, front_extent=.27, rear_extent=.27,
                 hard_stop_clearance=.30, slow_clearance=.60):
        if not all(math.isfinite(v) and v>0 for v in
                   (body_width,front_extent,rear_extent,hard_stop_clearance,slow_clearance)):
            raise ValueError('Geometry and trial margins must be finite and positive')
        if hard_stop_clearance>=slow_clearance:
            raise ValueError('slow_clearance must exceed hard_stop_clearance')
        self.half_width=body_width/2
        self.front,self.rear=front_extent,rear_extent
        self.hard,self.slow=hard_stop_clearance,slow_clearance
        self.timeout=.5
        self.scan=None
        self.raw=None
        self.nearest_points=[]

    def clearance(self,x,y):
        # Euclidean distance to the measured closed rectangular footprint.
        dx=max(-self.rear-x,x-self.front,0.)
        dy=max(abs(y)-self.half_width,0.)
        return math.hypot(dx,dy)

    def update_raw(self,msg,received):
        self.raw=None
        v,w=msg.linear.x,msg.angular.z
        values=(v,w,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)
        if not all(math.isfinite(n) for n in values):
            return
        if any(abs(n)>1e-9 for n in values[2:]) or v<0:
            return
        # Rotation in place needs a swept-footprint policy; preview stops it.
        if v==0 and w!=0:
            return
        self.raw=(received,min(v,.10),max(-.20,min(.20,w)))

    def update_scan(self,msg,transform,now_ns,received):
        self.scan=None
        self.nearest_points=[]
        stamp=stamp_ns(msg.header.stamp)
        age=(now_ns-stamp)/1e9
        if (stamp<=0 or not 0<=age<=self.timeout or not msg.ranges
                or len(msg.ranges)>50000
                or msg.header.frame_id not in (LASER,BASE)
                or not all(math.isfinite(v) for v in
                           (msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max))
                or msg.angle_increment==0 or not 0<msg.range_min<msg.range_max):
            return
        if msg.header.frame_id==BASE:
            tx,ty,qx,qy,qz,qw=0.,0.,0.,0.,0.,1.
        else:
            if (transform is None or transform.header.frame_id!=BASE
                    or transform.child_frame_id!=LASER):
                return
            t,q=transform.transform.translation,transform.transform.rotation
            if not all(math.isfinite(v) for v in (t.x,t.y,t.z,q.x,q.y,q.z,q.w)):
                return
            if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>0.01:
                return
            tx,ty,qx,qy,qz,qw=t.x,t.y,q.x,q.y,q.z,q.w
        closest=float('inf')
        count=0
        nearest=[]
        for i,r in enumerate(msg.ranges):
            if not math.isfinite(r) or not msg.range_min<=r<=msg.range_max:
                continue
            angle=msg.angle_min+i*msg.angle_increment
            if not math.isfinite(angle):
                return
            x,y=r*math.cos(angle),r*math.sin(angle)
            # Full quaternion rotation of (x,y,0), projected onto base XY.
            bx=tx+(1-2*(qy*qy+qz*qz))*x+2*(qx*qy-qw*qz)*y
            by=ty+2*(qx*qy+qw*qz)*x+(1-2*(qx*qx+qz*qz))*y
            if not math.isfinite(bx) or not math.isfinite(by):
                return
            clearance=self.clearance(bx,by)
            closest=min(closest,clearance)
            nearest.append((clearance,i,math.degrees(angle),r,bx,by))
            count+=1
        if count:
            self.scan=(stamp,received,closest,count)
            self.nearest_points=sorted(nearest)[:8]

    def command(self,now_ns,received):
        if self.raw is None or not 0<=received-self.raw[0]<=self.timeout:
            return 0.,0.,'STOP_COMMAND_MISSING_OR_STALE'
        if self.scan is None:
            return 0.,0.,'STOP_SCAN_OR_TF_INVALID'
        stamp,rx,clearance,_=self.scan
        if not (0<stamp and 0<=(now_ns-stamp)/1e9<=self.timeout
                and 0<=received-rx<=self.timeout):
            return 0.,0.,'STOP_SCAN_STALE'
        _,v,w=self.raw
        if v==0 and w==0:
            return 0.,0.,'STOP_UPSTREAM'
        # No leader mask: every valid return is included before deciding stop.
        if clearance<=self.hard:
            return 0.,0.,'STOP_FOOTPRINT_CLEARANCE'
        factor=min(1.,max(0.,(clearance-self.hard)/(self.slow-self.hard)))
        return v*factor,w*factor,'PREVIEW_PASS' if factor==1 else 'PREVIEW_SLOW'


def main():
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--enable-apf-candidate',action='store_true')
    parser.add_argument('--enable-apf-rejoin',action='store_true')
    parser.add_argument('--confirm-obstacle-visible',action='store_true')
    parser.add_argument('--obstacle-center-y',type=float)
    args=parser.parse_args()
    apf_options=None
    if args.enable_apf_candidate:
        from apf_case3_candidate import Settings
        if not args.confirm_obstacle_visible or args.obstacle_center_y is None:
            parser.error('APF requires confirmed visibility and measured obstacle-center-y')
        settings=Settings(lidar_height=.25,visibility_confirmed=True,influence=1.5 if args.enable_apf_rejoin else .9)
        passage=settings.passage(args.obstacle_center_y)
        if not (passage['left_possible'] or passage['right_possible']):parser.error('No parallel passage with current margins')
        apf_options=dict(settings=settings,preferred_side=1 if passage['left_gap_m']>=passage['right_gap_m'] else -1,
                         enable_rejoin=args.enable_apf_rejoin)
    elif args.enable_apf_rejoin:parser.error('APF candidate must be enabled')
    import yaml
    import rclpy
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import Twist,TwistStamped
    from follower_stamped_preview import StampedGuard,PURSUIT_TOPIC,GUARD_TOPIC,FRAME
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from follower_remote_qos import latest_state_qos
    from tf2_ros import Buffer,TransformListener,TransformException

    class PreviewNode(Node):
        def __init__(self):
            super().__init__('follower_obstacle_preview')
            validate_output_topic(self.resolve_topic_name(OUTPUT_TOPIC))
            if self.resolve_topic_name(GUARD_TOPIC)!=GUARD_TOPIC:raise RuntimeError('Stamped guard remap rejected')
            config=Path(__file__).with_name('follower_hardware.yaml')
            params=yaml.safe_load(config.read_text())['follower_obstacle_preview']['ros__parameters']
            if apf_options is None:
                self.core=ObstacleCore(**params)
                self.stamped=StampedGuard(ObstacleCore(**params))
            else:
                from apf_guard_connection import APFGuardCore
                self.core=APFGuardCore(**params,**apf_options)
                self.stamped=StampedGuard(APFGuardCore(**params,**apf_options))
                if args.enable_apf_rejoin:
                    from follower_pursuit_preview import PreviewCore
                    from nav_msgs.msg import Odometry,Path as RosPath
                    from follower_remote_qos import latest_remote_qos
                    navigation=PreviewCore()
                    self.core.navigation=self.stamped.core.navigation=navigation
                    def navigation_odom(m):
                        navigation.odom_callback('follower',m,self.get_clock().now().nanoseconds,time.monotonic())
                        self.tick()
                    def navigation_path(m):
                        navigation.path_callback(m,self.get_clock().now().nanoseconds,time.monotonic())
                        self.tick()
                    self.nav_odom_sub=self.create_subscription(Odometry,'/follower/odom_map',
                        navigation_odom,latest_state_qos())
                    self.nav_path_sub=self.create_subscription(RosPath,'/leader/path',
                        navigation_path,latest_remote_qos())
            self.last_source_ns=None
            self.buffer=Buffer()
            self.listener=TransformListener(self.buffer,self,qos=latest_state_qos())
            self.pub=self.create_publisher(Twist,OUTPUT_TOPIC,latest_state_qos())
            self.stamped_pub=self.create_publisher(TwistStamped,GUARD_TOPIC,latest_state_qos())
            self.status_pub=self.create_publisher(String,STATUS_TOPIC,10)
            self.last_status_key=None;self.last_status_at=0.
            self.scan_sub=self.create_subscription(LaserScan,'/follower/scan',self.scan_cb,latest_state_qos())
            self.cmd_sub=self.create_subscription(Twist,INPUT_TOPIC,
                lambda msg:self.core.update_raw(msg,time.monotonic()),latest_state_qos())
            self.stamped_sub=self.create_subscription(TwistStamped,PURSUIT_TOPIC,
                self.stamped_cb,latest_state_qos())
            self.last_report=0.
            self.timer=self.create_timer(.025,self.tick)
            self.get_logger().info(f'TEST ONLY: {INPUT_TOPIC} -> {OUTPUT_TOPIC}; no motor/leader mask')

        def scan_cb(self,msg):
            self.core.scan=None
            self.stamped.core.scan=None
            self.core.nearest_points=[]
            transform=None
            if msg.header.frame_id==LASER:
                try:
                    transform=self.buffer.lookup_transform(BASE,LASER,Time.from_msg(msg.header.stamp),
                                                           timeout=Duration(seconds=0.0))
                except TransformException:
                    self.publish_new_source()
                    return
            self.core.update_scan(msg,transform,self.get_clock().now().nanoseconds,time.monotonic())
            self.stamped.core.scan=self.core.scan
            if apf_options is not None:self.stamped.core.points=list(self.core.points)
            self.publish_new_source()

        def stamped_cb(self,msg):
            self.stamped.update(msg,self.get_clock().now().nanoseconds,time.monotonic())
            self.publish_new_source()

        def publish_new_source(self):
            _,_,source_ns,_=self.stamped.command(self.get_clock().now().nanoseconds,time.monotonic())
            if (source_ns!=self.last_source_ns or self.last_status_key is None
                    or clearance_zone(self.core)!=self.last_status_key[-1]):
                self.tick()

        def tick(self):
            now=time.monotonic()
            v,w,reason=self.core.command(self.get_clock().now().nanoseconds,now)
            msg=Twist();msg.linear.x=v;msg.angular.z=w
            sv,sw,source_ns,stamped_reason=self.stamped.command(self.get_clock().now().nanoseconds,now)
            if apf_options is not None and args.enable_apf_rejoin:
                nav_stamp=self.stamped.core.navigation_source_ns
                source_ns=min(source_ns,nav_stamp) if source_ns>0 and nav_stamp>0 else 0
            self.last_source_ns=source_ns
            stamped=TwistStamped();stamped.header.frame_id=FRAME
            stamped.header.stamp.sec=source_ns//10**9;stamped.header.stamp.nanosec=source_ns%10**9
            stamped.twist.linear.x=sv;stamped.twist.angular.z=sw
            self.stamped_pub.publish(stamped)
            self.pub.publish(msg)
            key=status_event_key(self.core,reason,stamped_reason,source_ns,sv,sw)
            if key!=self.last_status_key or now-self.last_status_at>=1.:
                state=guard_status(self.core,self.get_clock().now().nanoseconds,
                                   reason,stamped_reason,source_ns,sv,sw)
                if apf_options is not None:
                    state['apf']=self.stamped.core.apf_status
                    state['apf_stage']='STRAIGHT_REJOIN_CANDIDATE' if args.enable_apf_rejoin else 'INITIAL_STEERING_ONLY; PASS/REJOIN stops'
                self.status_pub.publish(String(data=json.dumps(state,allow_nan=False)))
                if key!=self.last_status_key:
                    self.get_logger().info('GUARD_DECISION '+json.dumps(state,allow_nan=False))
                self.last_status_key=key;self.last_status_at=now
            if now-self.last_report>=2:
                clearance=self.core.scan[2] if self.core.scan else None
                self.get_logger().info(f'{reason}: clearance={clearance}, preview_v={v:.3f}, preview_w={w:.3f}')
                self.last_report=now

    rclpy.init(args=[])
    node=None
    try:
        node=PreviewNode();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        # ROS 종료 과정의 문맥 닫힘만 허용하고 실행 중 오류는 숨기지 않는다.
        if rclpy.ok():raise
    finally:
        if node is not None:
            if rclpy.ok():node.pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    main()
