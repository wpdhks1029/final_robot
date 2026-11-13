#!/usr/bin/env python3
"""Offline CSV -> fresh EKF, Domain217 localhost only. No motor interface.

CSV lacks original covariance/filter state: reconstruction is approximate,
using the known raw/gyro covariances. It cannot exactly restore the old EKF.
"""
import argparse
import csv
import math
import os
from pathlib import Path
import subprocess
import time


def load_inputs(path):
    with Path(path).open() as f:
        rows=[r for r in csv.DictReader(f) if r['topic'] in
              ('/follower/odom_raw','/follower/imu/data')]
    if not rows:raise ValueError('No raw/IMU input')
    for r in rows:
        for k in ('rx_s','age_s','wz'):
            if not math.isfinite(float(r[k])):raise ValueError('Nonfinite '+k)
        if not 0<=float(r['rx_s'])<=120:raise ValueError('Capture time out of bounds')
    return sorted(rows,key=lambda r:float(r['rx_s']))


def fill_stationary_gaps(rows):
    """Counterfactual only: zero-speed measurements during confirmed stationary gaps.

    Never used for real odom. Refuses gaps whose preceding raw speed is nonzero.
    """
    raw=[r for r in rows if r['topic']=='/follower/odom_raw']
    if not raw:raise ValueError('No raw odom')
    finish=max(float(r['rx_s']) for r in rows)
    synthetic=[]
    for i,a in enumerate(raw):
        begin=float(a['rx_s'])
        end=float(raw[i+1]['rx_s']) if i+1<len(raw) else finish
        if end-begin<=.5:continue
        if any(abs(float(a[k]))>1e-9 for k in ('vx','vy','wz')):
            raise ValueError('Refusing to invent stopped feedback after nonzero measurement')
        t=begin+.1
        while t<end-.04:
            r=a.copy();r['rx_s']=str(t);r['age_s']='0.01'
            r['stamp_ns']=str(int(a['stamp_ns'])+round((t-begin)*1e9))
            synthetic.append(r);t+=.1
    return sorted(rows+synthetic,key=lambda r:float(r['rx_s'])),len(synthetic)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',type=Path)
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--fill-stationary-gaps',action='store_true',
                        help='Offline comparison only: invent zero-speed samples in stationary input gaps')
    args=parser.parse_args()
    rows=load_inputs(args.csv)
    if args.fill_stationary_gaps:
        rows,count=fill_stationary_gaps(rows)
        print(f'COUNTERFACTUAL: inserted {count} synthetic zero-speed samples, not actual feedback.',flush=True)
    if not args.run:
        print(f'Loaded {len(rows)} samples; use --run for isolated approximate replay.');return
    os.environ['ROS_DOMAIN_ID']='217';os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init(args=[]);node=rclpy.create_node('offline_ekf_replay')
    child=None;outputs=[]
    directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
    output=directory/f'ekf_replay_{time.time_ns()}.csv'
    log=output.with_suffix('.log')
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        if any(n!=node.get_name() for n,ns in node.get_node_names_and_namespaces()):
            raise RuntimeError('Domain217 already in use; no replay started')
        raw=node.create_publisher(Odometry,'/follower/odom_raw',100)
        imu=node.create_publisher(Imu,'/follower/imu/data',100)
        qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
        start=[None]
        def receive(m):
            outputs.append(dict(rx_s=time.monotonic()-start[0] if start[0] else -1,
                                x=m.pose.pose.position.x,y=m.pose.pose.position.y,
                                vx=m.twist.twist.linear.x,vy=m.twist.twist.linear.y))
        subscription=node.create_subscription(Odometry,'/offline/filtered',receive,qos)
        broadcaster=StaticTransformBroadcaster(node)
        tf=TransformStamped();tf.header.stamp=node.get_clock().now().to_msg()
        tf.header.frame_id='follower/base_link';tf.child_frame_id='follower/imu_link'
        tf.transform.translation.z=.16;tf.transform.rotation.w=1.
        broadcaster.sendTransform(tf)
        config=Path.home()/'robot_ws/src/cnu_capstone_2026/config/follower_ekf_A_selected.yaml'
        with log.open('x') as logfile:
            child=subprocess.Popen(['/opt/ros/humble/lib/robot_localization/ekf_node','--ros-args',
                '-r','__node:=follower_ekf_filter_node','--params-file',str(config),
                '-r','odometry/filtered:=/offline/filtered'],env=os.environ.copy(),stdout=logfile,stderr=subprocess.STDOUT)
            end=time.monotonic()+3
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
            if child.poll() is not None:raise RuntimeError('Replay EKF exited; inspect '+str(log))
            print('ISOLATED Domain217 localhost. Fresh EKF, reconstructed covariances; no motor access.',flush=True)
            print('Replaying receipt timing including input gaps (~60s).',flush=True)
            start[0]=time.monotonic();base_ns=node.get_clock().now().nanoseconds
            for r in rows:
                due=start[0]+float(r['rx_s'])
                while time.monotonic()<due:
                    if child.poll() is not None:raise RuntimeError('Replay EKF exited')
                    rclpy.spin_once(node,timeout_sec=min(.02,max(0.,due-time.monotonic())))
                stamp=base_ns+round((float(r['rx_s'])-float(r['age_s']))*1e9)
                if r['topic']=='/follower/odom_raw':
                    m=Odometry();m.child_frame_id='follower/base_link'
                    m.pose.pose.position.x=float(r['x']);m.pose.pose.position.y=float(r['y'])
                    yaw=float(r['yaw']);m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2)
                    m.twist.twist.linear.x=float(r['vx']);m.twist.twist.linear.y=float(r['vy'])
                    m.twist.twist.angular.z=float(r['wz'])
                    m.pose.covariance[0]=m.pose.covariance[7]=.05;m.pose.covariance[35]=.1
                    m.twist.covariance[0]=m.twist.covariance[7]=.02;m.twist.covariance[35]=.05
                    publisher=raw
                else:
                    m=Imu();m.orientation.w=1.;m.orientation_covariance[0]=-1.
                    m.angular_velocity.z=float(r['wz'])
                    m.angular_velocity_covariance[0]=m.angular_velocity_covariance[4]=m.angular_velocity_covariance[8]=.01
                    publisher=imu
                m.header.frame_id=r['frame'];m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9
                publisher.publish(m)
                rclpy.spin_once(node,timeout_sec=0.)
            end=time.monotonic()+2
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        if not outputs:raise RuntimeError('No replay EKF output')
        a,b=outputs[0],outputs[-1]
        print(f'output_count={len(outputs)}, delta_xy=({b["x"]-a["x"]:.6f},{b["y"]-a["y"]:.6f})m')
        print(f'x_min/max={min(r["x"] for r in outputs):.6f}/{max(r["x"] for r in outputs):.6f}m')
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        with output.open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=['rx_s','x','y','vx','vy']);writer.writeheader();writer.writerows(outputs)
        print('REPLAY_CSV='+str(output));print('REPLAY_LOG='+str(log))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
