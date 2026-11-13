#!/usr/bin/env python3
"""Read-only map/path/session observation; no publisher, arm or service."""
import argparse,json,math,os,time
from pathlib import Path
from directed_path_tracker import DirectedPathTracker
from follower_yaw_diagnostics import rpy


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=int,choices=(10,20),default=10);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10 required')
    import rclpy
    from nav_msgs.msg import Odometry,Path as RosPath
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init(args=[]);node=rclpy.create_node('directed_follow_shadow_readonly')
    q=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT);latest={};rows=[];tracker=DirectedPathTracker();last_reason=None
    def odom(key,m):
        roll,pitch,yaw=rpy(m.pose.pose.orientation);v=m.pose.pose.position
        latest[key]=(m.header.stamp.sec*10**9+m.header.stamp.nanosec,time.monotonic(),(v.x,v.y,yaw,m.twist.twist.linear.x,m.twist.twist.angular.z))
    def path(m):latest['path']=(m.header.stamp.sec*10**9+m.header.stamp.nanosec,time.monotonic(),[(r.pose.position.x,r.pose.position.y) for r in m.poses])
    def status(m):latest['motor']=(json.loads(m.data),time.monotonic())
    subs=[node.create_subscription(Odometry,'/leader/odom',lambda m:odom('leader',m),q),node.create_subscription(Odometry,'/follower/odom_map',lambda m:odom('follower',m),q),node.create_subscription(RosPath,'/leader/path',path,q),node.create_subscription(String,'/follower/motor_candidate/status',status,q)]
    print('조회 전용: 양쪽 정지·후발 비무장 유지. 경로 방향/이력/목표만 검사. 주행·arm·서비스 호출 없음.',flush=True)
    start=time.monotonic();last_sample=0.
    try:
        while time.monotonic()-start<a.seconds:
            rclpy.spin_once(node,timeout_sec=.05);mono=time.monotonic();now=node.get_clock().now().nanoseconds
            if mono-last_sample<.2:continue
            last_sample=mono;row=dict(ros_ns=now,control_connected=False)
            try:
                if 'motor' not in latest or mono-latest['motor'][1]>.3:raise ValueError('MOTOR_STATUS_MISSING_OR_STALE')
                motor=latest['motor'][0]
                row['motor']=dict(armed=motor.get('armed'),fault=motor.get('fault'),distance_test_only=motor.get('distance_test_only'),raw_linear_feedback_scale=motor.get('raw_linear_feedback_scale'))
                if motor.get('armed') is not False:raise ValueError('KEEP_FOLLOWER_DISARMED')
                if motor.get('distance_test_only'):raise ValueError('RESTART_NORMAL_AVOIDANCE_DRIVER')
                ages={}
                for key in ('leader','follower','path'):
                    if key not in latest:raise ValueError(key.upper()+'_MISSING')
                    stamp,rx,data=latest[key];stamp-=50_000_000 if key in ('leader','path') else 0
                    ages[key]=(now-stamp)/1e9
                    if not 0<=ages[key]<=.8 or not 0<=mono-rx<=.8:raise ValueError(key.upper()+'_STALE')
                row.update(source_age_s=ages,leader_map_pose=latest['leader'][2],follower_map_pose=latest['follower'][2],live_path_points=latest['path'][2])
                row['selection']=tracker.select(latest['path'][2],latest['follower'][2],.5,tip_heading=latest['leader'][2][2])
                row['reason']='DIRECTED_PATH_READY'
            except ValueError as exc:row['reason']=str(exc)
            rows.append(row)
            if row['reason']!=last_reason:
                print(json.dumps({k:v for k,v in row.items() if k!='live_path_points'},ensure_ascii=False),flush=True);last_reason=row['reason']
        owners={topic:[e.node_namespace+'/'+e.node_name for e in node.get_publishers_info_by_topic(topic)] for topic in ('/leader/odom','/leader/path','/follower/odom_map')}
        out=Path(__file__).resolve().parent/'logs'/f'directed_follow_shadow_{time.time_ns()}.json'
        out.write_text(json.dumps(dict(owners=owners,observations=rows,control_connected=False),allow_nan=False)+'\n')
        print(json.dumps(dict(last_reason=last_reason,owners=owners,control_connected=False),ensure_ascii=False));print('JSON='+str(out))
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
