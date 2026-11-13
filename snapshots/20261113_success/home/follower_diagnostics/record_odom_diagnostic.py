#!/usr/bin/env python3
"""Subscribe only. Saves raw/EKF/IMU/status history; no motor commands or services."""
import argparse
import csv
import json
import math
from pathlib import Path
import time


def summarize(rows):
    if not rows:return 'NO DATA'
    gaps=[b['rx_s']-a['rx_s'] for a,b in zip(rows,rows[1:])]
    result=f'n={len(rows)}, max_rx_gap={max(gaps,default=0):.4f}s'
    odom=[r for r in rows if r.get('x') is not None]
    if odom:
        a,b=odom[0],odom[-1]
        result+=(f", delta_xy=({b['x']-a['x']:.6f},{b['y']-a['y']:.6f})m"
                 f", max_abs_vx={max(abs(r['vx']) for r in odom):.6f}m/s")
    ages=[r['age_s'] for r in rows if r.get('age_s') is not None]
    if ages:result+=f', age_min/max={min(ages):.4f}/{max(ages):.4f}s'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=30)
    args=parser.parse_args()
    if not 5<=args.seconds<=120:parser.error('seconds must be 5..120')
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init(args=[]);node=rclpy.create_node('follower_odom_diagnostic_recorder')
    rows=[];subs=[];start=time.monotonic()
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
    path=directory/f'odom_{time.time_ns()}.csv'
    fields=['topic','rx_s','stamp_ns','age_s','frame','child','x','y','yaw','vx','vy','wz','status']
    def receive(topic,msg):
        row=dict.fromkeys(fields);row.update(topic=topic,rx_s=time.monotonic()-start)
        if hasattr(msg,'header'):
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            row.update(stamp_ns=stamp,age_s=(node.get_clock().now().nanoseconds-stamp)/1e9,
                       frame=msg.header.frame_id)
        if isinstance(msg,Odometry):
            p=msg.pose.pose;q=p.orientation;t=msg.twist.twist
            row.update(child=msg.child_frame_id,x=p.position.x,y=p.position.y,
                       yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                       vx=t.linear.x,vy=t.linear.y,wz=t.angular.z)
        elif isinstance(msg,Imu):row.update(wz=msg.angular_velocity.z)
        else:row.update(status=msg.data)
        rows.append(row)
    topics=[('/follower/odom_raw',Odometry),('/follower/odometry/filtered',Odometry),
            ('/follower/odom_position_test',Odometry),
            ('/follower/imu/data',Imu),('/follower/motor_locked/status',String)]
    try:
        for topic,kind in topics:
            subs.append(node.create_subscription(kind,topic,lambda m,t=topic:receive(t,m),qos))
        print(f'Subscribe only: {args.seconds}s. Keep robot stationary; keep locked driver ON.',flush=True)
        while time.monotonic()-start<args.seconds:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:print('Interrupted; saving partial capture.')
    finally:
        with path.open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
        for topic,_ in topics:print(topic+': '+summarize([r for r in rows if r['topic']==topic]))
        statuses=[r['status'] for r in rows if r['status']]
        if statuses:
            try:print('last_motor_status='+json.dumps(json.loads(statuses[-1])))
            except ValueError:print('last_motor_status='+statuses[-1])
        print('CSV='+str(path));print('No commands/services/port access. CSV contains received samples, not packet-loss proof.')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
