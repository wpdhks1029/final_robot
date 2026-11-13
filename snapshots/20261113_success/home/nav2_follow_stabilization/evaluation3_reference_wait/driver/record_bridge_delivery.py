#!/usr/bin/env python3
"""선두·후발에서 동시에 실행하는 조회 전용 시각 기록기. 모터/서비스 접근 없음."""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import socket
import time


def summarize(rows):
    groups=defaultdict(list)
    for row in rows:groups[row['name']].append(row)
    report={}
    for name,a in groups.items():
        gaps=[(y['rx_mono_ns']-x['rx_mono_ns'])/1e9 for x,y in zip(a,a[1:])]
        ages=[(x['rx_ros_ns']-x['source_ns'])/1e9 for x in a if x['source_ns']>0]
        long=[]
        for x,y in zip(a,a[1:]):
            gap=(y['rx_mono_ns']-x['rx_mono_ns'])/1e9
            if gap>=.2:long.append(dict(gap_s=gap,before_source_ns=x['source_ns'],after_source_ns=y['source_ns'],before_rx_ros_ns=x['rx_ros_ns'],after_rx_ros_ns=y['rx_ros_ns']))
        report[name]=dict(received=len(a),max_rx_gap_s=max(gaps,default=None),max_source_age_s=max(ages,default=None),long_gaps=long)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role',choices=('leader','follower'),required=True)
    p.add_argument('--seconds',type=int,default=90)
    a=p.parse_args()
    if not 10<=a.seconds<=180:p.error('seconds는 10~180초 범위입니다.')
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10이 필요합니다.')
    import rclpy
    from nav_msgs.msg import Odometry,Path as RosPath
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
    rclpy.init(args=[])
    n=rclpy.create_node('bridge_delivery_observer_'+a.role)
    qos=QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    rows=[];subs=[]
    def record(name,msg):
        mono=time.monotonic_ns();ros=n.get_clock().now().nanoseconds
        stamp=msg.header.stamp
        rows.append(dict(name=name,source_ns=stamp.sec*10**9+stamp.nanosec,rx_ros_ns=ros,rx_mono_ns=mono))
    topics=[('leader','/leader/odom',Odometry),('path','/leader/path',RosPath)]
    if a.role=='leader':topics += [('raw','/odom',Odometry),('filtered','/odometry/filtered',Odometry)]
    else:topics += [('follower','/follower/odom_map',Odometry)]
    for name,topic,kind in topics:
        subs.append(n.create_subscription(kind,topic,lambda m,k=name:record(k,m),qos))
    def heartbeat():
        rows.append(dict(name='observer_tick',source_ns=0,rx_ros_ns=n.get_clock().now().nanoseconds,rx_mono_ns=time.monotonic_ns()))
    timer=n.create_timer(.05,heartbeat)
    start=time.monotonic();interrupted=False
    print(f'조회 전용 {a.seconds}초, 역할={a.role}. 이동·arm·서비스 요청 없음. 종료 후 파일 저장.',flush=True)
    try:
        while rclpy.ok() and time.monotonic()-start<a.seconds:rclpy.spin_once(n,timeout_sec=.05)
    except KeyboardInterrupt:interrupted=True
    finally:
        directory=Path.home()/('leader_diagnostics' if a.role=='leader' else 'follower_diagnostics')/'logs'
        directory.mkdir(parents=True,exist_ok=True)
        path=directory/f'bridge_delivery_{a.role}_{time.time_ns()}.json'
        data=dict(role=a.role,host=socket.gethostname(),elapsed_s=time.monotonic()-start,interrupted=interrupted,summary=summarize(rows),rows=rows)
        path.write_text(json.dumps(data,ensure_ascii=False,separators=(',',':'))+'\n')
        print(json.dumps(data['summary'],ensure_ascii=False,indent=2));print('JSON='+str(path),flush=True)
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
