#!/usr/bin/env python3
"""Stationary APF pivot observation only: subscriptions and file output."""
import argparse,json,math,os,time
from pathlib import Path
import numpy as np
from apf_pivot_candidate import APFPivot
from apf_rejoin_candidate import connected_groups
from local_avoidance_limits import SOURCE_TIMEOUT_S

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=int,default=20)
    a=p.parse_args()
    if not 5<=a.seconds<=60 or os.environ.get('ROS_DOMAIN_ID')!='10':p.error('seconds 5..60 and ROS_DOMAIN_ID=10 required')
    import rclpy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer,TransformListener
    from follower_remote_qos import latest_state_qos
    from watch_apf_case3 import project
    rclpy.init(args=[]);node=rclpy.create_node('apf_pivot_shadow_readonly')
    buffer=Buffer();listener=TransformListener(buffer,node);latest=None;rows=[];last=0.
    def cb(msg):
        nonlocal latest
        latest=(msg,time.monotonic())
    subscription=node.create_subscription(LaserScan,'/follower/scan',cb,latest_state_qos())
    print('조회 전용: 양쪽 정지 유지. APF 회전/전진 후보만 계산. 발행·arm·서비스 호출 없음.',flush=True)
    begin=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-begin<a.seconds:
            rclpy.spin_once(node,timeout_sec=.05)
            if latest is None or time.monotonic()-last<1.:continue
            last=time.monotonic();msg,rx=latest;stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            row=dict(control_connected=False,evaluation_pass=False,scan_stamp_ns=stamp)
            try:
                if not 0<=(node.get_clock().now().nanoseconds-stamp)/1e9<=SOURCE_TIMEOUT_S or last-rx>SOURCE_TIMEOUT_S:
                    raise ValueError('STOP_SCAN_STALE')
                tf=buffer.lookup_transform('follower/base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                points=project(msg,tf)
                front=[(x,y) for x,y in points if .35<x<1.6 and abs(y)<.65]
                groups=[g for g in connected_groups(front,.1) if len(g)>=5 and .08<=max(y for x,y in g)-min(y for x,y in g)<=.4 and max(x for x,y in g)-min(x for x,y in g)<=.4]
                if not groups:raise ValueError('STOP_BOX_NOT_CONFIRMED')
                box=min(groups,key=lambda g:min(x for x,y in g));near=min(x for x,y in box)
                low=min(y for x,y in box);high=max(y for x,y in box);far=max(near+.2,max(x for x,y in box))
                # Preserve scan points and add the known box's unseen depth.
                cloud=list(points)
                for f in np.linspace(0,1,41):
                    cloud.extend([(far,low+(high-low)*f),(near+(far-near)*f,low),(near+(far-near)*f,high)])
                candidate=APFPivot((0.,0.,0.),dict(near=near,far=far,high=high))
                started=time.monotonic();choice=candidate.choose((0.,0.,0.),np.asarray(cloud))
                row.update(compute_s=time.monotonic()-started,points=cloud,
                           box=dict(near=near,far=far,low=low,high=high))
                if choice is None:raise ValueError('STOP_APF_NO_CHECKED_STEP')
                row.update(reason='APF_PIVOT_STEP_CHECKED',heading_deg=math.degrees(choice[0]),
                           step_target=list(choice[1]),**candidate.last_diagnostic)
                row['source_age_after_compute_s']=(node.get_clock().now().nanoseconds-stamp)/1e9
                if row['source_age_after_compute_s']>SOURCE_TIMEOUT_S:row['reason']='STOP_INPUT_AGED_DURING_COMPUTE'
            except Exception as exc:
                row.update(reason='STOP_OBSERVATION_INVALID',detail=str(exc))
            rows.append(row);print(json.dumps({k:v for k,v in row.items() if k!='points'},ensure_ascii=False),flush=True)
    finally:
        folder=Path(__file__).parent/'logs';folder.mkdir(exist_ok=True)
        path=folder/f'apf_pivot_shadow_{time.time_ns()}.json';path.write_text(json.dumps(dict(rows=rows,control_connected=False),allow_nan=False)+'\n')
        print('JSON='+str(path));node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
