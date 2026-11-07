#!/usr/bin/env python3
"""조회 전용: 위치 입력·EKF·TF·bridge의 원본 시각과 QoS별 수신을 비교."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import time


def summarize(rows):
    if not rows:return dict(received=0)
    ages=sorted(r[0] for r in rows)
    stamps=[r[2] for r in rows]
    gaps=[b[1]-a[1] for a,b in zip(rows,rows[1:])]
    return dict(received=len(rows),age_min_p95_max_s=[ages[0],ages[int((len(ages)-1)*.95)],ages[-1]],
                age_over_0_3s=sum(a>.3 for a in ages),future_over_0_05s=sum(a<-.05 for a in ages),
                max_rx_gap_s=max(gaps) if gaps else None,
                repeated_stamp=sum(a==b for a,b in zip(stamps,stamps[1:])))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=int,default=10)
    a=p.parse_args()
    if not 5<=a.seconds<=20:p.error('seconds는5~20이어야 합니다.')
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped
    from tf2_msgs.msg import TFMessage
    rclpy.init(args=[]);node=rclpy.create_node('timing_pipeline_observer')
    rows=defaultdict(list);invalid=defaultdict(int);names=[];subs=[]
    reliable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
    best_effort=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    def record(name,stamp):
        value=stamp.sec*10**9+stamp.nanosec
        if value>0:rows[name].append(((node.get_clock().now().nanoseconds-value)/1e9,time.monotonic(),value))
        else:invalid[name]+=1
    def tf_cb(msg):
        for t in msg.transforms:
            if (t.header.frame_id,t.child_frame_id)==('follower/odom','follower/base_link'):
                record('tf_odom_base',t.header.stamp)
            elif (t.header.frame_id,t.child_frame_id)==('map','follower/odom'):
                record('tf_map_odom',t.header.stamp)
    for name,topic,qos in [
        ('position_input','/follower/odom_position_candidate',best_effort),
        ('filtered','/follower/odometry/filtered',best_effort),
        ('follower_bridge_reliable','/follower/odom_map',reliable),
        ('follower_bridge_best_effort','/follower/odom_map',best_effort),
        ('leader_bridge_reliable','/leader/odom',reliable),
        ('leader_bridge_best_effort','/leader/odom',best_effort)]:
        names.append(name)
        subs.append(node.create_subscription(Odometry,topic,lambda m,n=name:record(n,m.header.stamp),qos))
    for base,topic in [('pursuit','/follower/cmd_vel_stamped_test'),
                       ('guard','/follower/cmd_vel_guard_stamped_test')]:
        for suffix,qos in [('reliable',reliable),('best_effort',best_effort)]:
            name=base+'_'+suffix;names.append(name)
            subs.append(node.create_subscription(TwistStamped,topic,lambda m,n=name:record(n,m.header.stamp),qos))
    names+=['tf_odom_base','tf_map_odom']
    subs.append(node.create_subscription(TFMessage,'/tf',tf_cb,
                QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)))
    try:
        print(f'조회 전용 {a.seconds}초: 모터·서비스·설정 변경 없음. 양쪽 정지 유지.',flush=True)
        end=time.monotonic()+a.seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        report={name:dict(summarize(rows[name]),zero_stamp=invalid[name]) for name in names}
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
        directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
        path=directory/f'timing_pipeline_{time.time_ns()}.json'
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print('JSON='+str(path),flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
