#!/usr/bin/env python3
"""후발 scan 조회 전용. 고정 mount(x=.26,yaw=0) 기준 차체 내부점의 전후 스캔을 보존한다."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time


def inspect_scan(m):
    hits=[]
    if m.header.frame_id!='follower/laser':return hits
    if not all(math.isfinite(x) for x in (m.angle_min,m.angle_increment,m.range_min,m.range_max)):return hits
    for i,r in enumerate(m.ranges):
        if not math.isfinite(r) or not m.range_min<=r<=m.range_max:continue
        a=m.angle_min+i*m.angle_increment
        x=.26+r*math.cos(a);y=r*math.sin(a)
        if -.27<=x<=.27 and abs(y)<=.30:
            neighbors=[]
            for j in range(max(0,i-4),min(len(m.ranges),i+5)):
                v=m.ranges[j];strength=m.intensities[j] if j<len(m.intensities) else None
                neighbors.append(dict(index=j,range_m=v if math.isfinite(v) else None,
                    intensity=strength if strength is not None and math.isfinite(strength) else None))
            hits.append(dict(index=i,angle_deg=math.degrees(a),range_m=r,base_x_m=x,base_y_m=y,neighbors=neighbors))
    return hits


def pack(m,now):
    clean=lambda a:[float(v) if math.isfinite(v) else None for v in a]
    return dict(source_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,rx_ros_ns=now,
                frame=m.header.frame_id,angle_min=m.angle_min,angle_max=m.angle_max,
                angle_increment=m.angle_increment,time_increment=m.time_increment,scan_time=m.scan_time,
                range_min=m.range_min,range_max=m.range_max,ranges=clean(m.ranges),intensities=clean(m.intensities))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=int,default=60);a=p.parse_args()
    if not 10<=a.seconds<=120:p.error('seconds는 10~120초입니다.')
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10이 필요합니다.')
    import rclpy
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(args=[]);n=rclpy.create_node('scan_anomaly_capture')
    previous=deque(maxlen=3);events=[];pending=[];count=0;hit_count=0;frames=set()
    def receive(m):
        nonlocal count,hit_count
        count+=1;frames.add(m.header.frame_id)
        hits=inspect_scan(m)
        row=pack(m,n.get_clock().now().nanoseconds)
        for event in pending:
            event["following"].append(row)
        pending[:]=[event for event in pending if len(event["following"])<2]
        if hits:
            hit_count+=1
            # 저장량만 제한하며 제어 처리에는 관여하지 않는다.
            if len(events)<100:
                event=dict(hits=hits,previous=list(previous),scan=row,following=[])
                events.append(event);pending.append(event)
        previous.append(row)
    sub=n.create_subscription(LaserScan,'/follower/scan',receive,qos_profile_sensor_data)
    print(f'조회 전용 {a.seconds}초. 모터·서비스·scan 변경 없음. 내부점과 전3/후2 스캔을 종료 후 저장합니다.',flush=True)
    start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<a.seconds:rclpy.spin_once(n,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        summary=dict(scans=count,frames=sorted(frames),inside_scans=hit_count,saved_events=len(events),
                     mount_assumption=dict(x_m=.26,y_m=0,yaw_rad=0),body_assumption=dict(front_m=.27,rear_m=.27,half_width_m=.30))
        d=Path(__file__).parent/'logs';d.mkdir(exist_ok=True);path=d/f'scan_anomalies_{time.time_ns()}.json'
        path.write_text(json.dumps(dict(summary=summary,events=events),ensure_ascii=False,allow_nan=False)+'\n')
        print(json.dumps(summary,ensure_ascii=False,indent=2));print('JSON='+str(path),flush=True)
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
