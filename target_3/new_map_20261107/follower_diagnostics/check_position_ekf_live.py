#!/usr/bin/env python3
"""Stationary live TEST EKF comparison. No motor commands or physical port access.

Requires locked owner publishing /follower/odom_position_test. TEST EKF publishes
no dynamic TF; gyro is copied to preview-only frame/topic using the same mount.
"""
import argparse
import copy
import csv
import os
from pathlib import Path
import subprocess
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true');p.add_argument('--seconds',type=int,default=30)
    a=p.parse_args()
    if not a.run:p.print_help();return
    if not 5<=a.seconds<=60:p.error('seconds must be 5..60')
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('Set ROS_DOMAIN_ID=10 in this terminal')
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init(args=[]);node=rclpy.create_node('position_ekf_stationary_check')
    child=None;rows=[];subs=[];imu_count=0
    directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
    output=directory/f'position_ekf_live_{time.time_ns()}.csv';log=output.with_suffix('.log')
    topics=['/follower/odom_position_test','/follower/odometry/position_filtered_test','/follower/odometry/filtered']
    start=time.monotonic()
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        if node.get_publishers_info_by_topic(topics[1]):raise RuntimeError('Existing TEST EKF publisher; do not duplicate')
        if len(node.get_publishers_info_by_topic(topics[0]))!=1:raise RuntimeError('Require exactly one position TEST source')
        imu_pub=node.create_publisher(Imu,'/follower/imu_position_test',20)
        if node.get_publishers_info_by_topic('/follower/imu_position_test') and len(node.get_publishers_info_by_topic('/follower/imu_position_test'))>1:
            raise RuntimeError('Existing preview gyro relay')
        qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
        def relay(m):
            nonlocal imu_count
            copied=copy.deepcopy(m);copied.header.frame_id='follower_position_preview/imu_link'
            imu_pub.publish(copied);imu_count+=1
        subs.append(node.create_subscription(Imu,'/follower/imu/data',relay,qos))
        def receive(topic,m):
            rows.append(dict(topic=topic,rx_s=time.monotonic()-start,
                stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                x=m.pose.pose.position.x,y=m.pose.pose.position.y,
                vx=m.twist.twist.linear.x,wz=m.twist.twist.angular.z))
        for topic in topics:subs.append(node.create_subscription(Odometry,topic,lambda m,t=topic:receive(t,m),qos))
        broadcaster=StaticTransformBroadcaster(node)
        tf=TransformStamped();tf.header.stamp=node.get_clock().now().to_msg()
        tf.header.frame_id='follower_position_preview/base_link';tf.child_frame_id='follower_position_preview/imu_link'
        tf.transform.translation.z=.16;tf.transform.rotation.w=1.
        broadcaster.sendTransform(tf)
        config=Path(__file__).with_name('follower_position_ekf_test.yaml')
        with log.open('x') as logfile:
            child=subprocess.Popen(['/opt/ros/humble/lib/robot_localization/ekf_node','--ros-args',
                '-r','__node:=ekf','-r','__ns:=/follower_position_ekf_test','--params-file',str(config),
                '-r','odometry/filtered:=/follower/odometry/position_filtered_test',
                '-r','tf:=/tf','-r','tf_static:=/tf_static'],stdout=logfile,stderr=subprocess.STDOUT)
            print('Keep robot stationary and locked. TEST EKF only, no motor commands, no dynamic TF.',flush=True)
            end=time.monotonic()+a.seconds
            while time.monotonic()<end:
                if child.poll() is not None:raise RuntimeError('TEST EKF exited; inspect '+str(log))
                rclpy.spin_once(node,timeout_sec=.02)
        print('gyro_relay_received='+str(imu_count))
        for topic in topics:
            rr=[r for r in rows if r['topic']==topic]
            if not rr:print(topic+': NO DATA');continue
            delta=(rr[-1]['x']-rr[0]['x'],rr[-1]['y']-rr[0]['y'])
            print(f'{topic}: n={len(rr)}, delta_xy=({delta[0]:.6f},{delta[1]:.6f})m, '
                  f'max_abs_vx={max(abs(r["vx"]) for r in rr):.6f}m/s')
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        with output.open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=['topic','rx_s','stamp_ns','x','y','vx','wz'])
            writer.writeheader();writer.writerows(rows)
        print('CSV='+str(output));print('LOG='+str(log))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
