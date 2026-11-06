#!/usr/bin/env python3
"""Read-only real-scene readiness report. No command publishers, motor ports or service calls.

Does not arm, enable, start either robot, or grant physical safety approval.
"""
import argparse
import json
import math
from pathlib import Path
import time


def source_age(msg,now):
    if msg is None:return None
    stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    return None if stamp<=0 else (now-stamp)/1e9


def speed_summary(values):
    values=[v for v in values if math.isfinite(v)]
    return dict(samples=len(values),minimum=None if not values else min(values),
                maximum=None if not values else max(values),
                above_stop_threshold=sum(v>.03 for v in values))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=int,default=10)
    a=p.parse_args()
    if not 5<=a.seconds<=20:p.error('seconds must be 5..20')
    import rclpy
    from nav_msgs.msg import Odometry,Path as RosPath
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from follower_pursuit_preview import PreviewCore
    from follower_remote_qos import latest_remote_qos
    rclpy.init(args=[]);node=rclpy.create_node('follower_follow_start_report')
    inputs={};counts={};subs=[];core=PreviewCore();status=None;status_rx=None
    leader_speeds=[];observations=[]
    def receive(name,m):
        inputs[name]=m;counts[name]=counts.get(name,0)+1
        if name=='leader':leader_speeds.append(m.twist.twist.linear.x)
        if name=='path':core.path_callback(m,node.get_clock().now().nanoseconds,time.monotonic())
        elif name in ('leader','follower'):core.odom_callback(name,m,node.get_clock().now().nanoseconds,time.monotonic())
    def motor(m):
        nonlocal status,status_rx
        try:status=json.loads(m.data);status_rx=time.monotonic()
        except ValueError:status=None
    try:
        for name,topic,kind in [('leader','/leader/odom',Odometry),('path','/leader/path',RosPath),
                                ('follower','/follower/odom_map',Odometry),
                                ('guard','/follower/cmd_vel_guard_stamped_test',TwistStamped)]:
            qos=latest_remote_qos() if name in ('leader','path','follower','guard') else 20
            subs.append(node.create_subscription(kind,topic,lambda m,n=name:receive(n,m),qos))
        subs.append(node.create_subscription(String,'/follower/motor_candidate/status',motor,20))
        print(f'READ ONLY {a.seconds}s: keep current setup; no movement or arm requested.',flush=True)
        started=time.monotonic();until=started+a.seconds;next_report=started;last_reason=None
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
            mono=time.monotonic()
            if mono>=next_report:
                now=node.get_clock().now().nanoseconds;v,w,reason=core.command(now,mono)
                m=inputs.get('leader')
                row=dict(elapsed_s=round(mono-started,3),ros_ns=now,reason=reason,
                         leader_v=None if m is None else m.twist.twist.linear.x,
                         leader_source_age_s=source_age(m,now),preview_v=v,preview_w=w)
                observations.append(row)
                if reason!=last_reason:
                    print(json.dumps(row,allow_nan=False),flush=True);last_reason=reason
                next_report=mono+.2
        now=node.get_clock().now().nanoseconds;mono=time.monotonic()
        v,w,reason=core.command(now,mono)
        report=dict(received=counts,source_age_s={k:source_age(m,now) for k,m in inputs.items()},
                    path_points=len(inputs['path'].poses) if 'path' in inputs else None,
                    computed_preview_v=v,computed_preview_w=w,preview_reason=reason)
        report['leader_speed_during_sampling']=speed_summary(leader_speeds)
        if 'leader' in inputs:
            m=inputs['leader'];report.update(leader_v=m.twist.twist.linear.x,leader_w=m.twist.twist.angular.z)
        if 'guard' in inputs:
            m=inputs['guard'];report.update(guard_v=m.twist.linear.x,guard_w=m.twist.angular.z)
        if 'leader' in inputs and 'follower' in inputs:
            l,f=inputs['leader'].pose.pose,inputs['follower'].pose.pose
            yaw=math.atan2(2*(f.orientation.w*f.orientation.z+f.orientation.x*f.orientation.y),
                           1-2*(f.orientation.y*f.orientation.y+f.orientation.z*f.orientation.z))
            dx,dy=l.position.x-f.position.x,l.position.y-f.position.y
            report.update(leader_distance_m=math.hypot(dx,dy),leader_ahead_m=math.cos(yaw)*dx+math.sin(yaw)*dy,
                          leader_left_m=-math.sin(yaw)*dx+math.cos(yaw)*dy)
        if status is not None:
            report['motor']={k:status.get(k) for k in ['enabled','armed','fault','feedback_fresh','measured_rpm',
                                'accepted_commands','input_kind','last_io_error','trial_max_rpm','trial_max_seconds']}
            report['motor_status_rx_age_s']=mono-status_rx
        else:report['motor']='NO DATA'
        print(json.dumps(report,indent=2,allow_nan=False),flush=True)
        directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
        path=directory/f'follow_start_{time.time_ns()}.json'
        path.write_text(json.dumps(dict(report,observations=observations),indent=2)+'\n')
        print('JSON='+str(path));print('Readiness observation only, not permission to arm or a physical safety verdict.')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
