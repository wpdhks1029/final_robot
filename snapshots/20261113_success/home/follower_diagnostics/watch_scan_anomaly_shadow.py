#!/usr/bin/env python3
"""원본 scan 이상점의 실시간 진단 전용. 발행·서비스·모터 접근 없음."""
import argparse
from collections import Counter,deque
import json
import os
from pathlib import Path
import time
from analyze_scan_anomalies import analyze_event
from capture_scan_anomalies import inspect_scan,pack


class Shadow:
    def __init__(self):
        self.previous=deque(maxlen=3);self.pending=[];self.scans=0;self.inside_scans=0
        self.last_stamp=0;self.invalid_order=0

    def add(self,row,hits):
        self.scans+=1;stamp=row['source_ns'];completed=[]
        if stamp<=0 or stamp<=self.last_stamp:
            self.invalid_order+=1
            # 시각이 역전되면 기존 시계열을 종료하고 다시 수집한다.
            completed=self.finish();self.previous.clear();self.last_stamp=max(0,stamp)
            return completed
        self.last_stamp=stamp
        waiting=[]
        for e in self.pending:
            if stamp-e['scan']['source_ns']<=400_000_000:e['following'].append(row)
            if len(e['following'])>=2 or stamp-e['scan']['source_ns']>400_000_000:
                completed.append(analyze_event(e))
            else:waiting.append(e)
        self.pending=waiting
        if hits:
            self.inside_scans+=1
            self.pending.append(dict(scan=row,hits=hits,previous=list(self.previous),following=[]))
        self.previous.append(row)
        return completed

    def finish(self):
        result=[analyze_event(e) for e in self.pending];self.pending=[]
        return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=int,default=90);a=p.parse_args()
    if not 10<=a.seconds<=180:p.error('seconds는 10~180초입니다.')
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10이 필요합니다.')
    import rclpy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(args=[]);n=rclpy.create_node('scan_anomaly_shadow_observer');core=Shadow();results=[];guards=deque(maxlen=2000)
    def completed(events):
        for e in events:
            results.append(e)
            for h in e['hits']:
                print(json.dumps(dict(source_ns=e['source_ns'],angle_deg=h['angle_deg'],range_m=h['range_m'],finding=h['finding'],control_action='UNCHANGED_STOP'),ensure_ascii=False),flush=True)
    def scan_cb(m):
        completed(core.add(pack(m,n.get_clock().now().nanoseconds),inspect_scan(m)))
    def guard_cb(m):
        try:d=json.loads(m.data)
        except (ValueError,TypeError):return
        if isinstance(d,dict) and d.get('clearance_zone') in ('inside','stop'):
            guards.append(dict(rx_ros_ns=n.get_clock().now().nanoseconds,diagnostic=d))
    subs=[n.create_subscription(LaserScan,'/follower/scan',scan_cb,qos_profile_sensor_data),
          n.create_subscription(String,'/follower/obstacle_preview/status',guard_cb,qos_profile_sensor_data)]
    start=time.monotonic()
    print(f'진단 전용 {a.seconds}초. 입력 scan/guard만 구독. 제어 출력·arm·필터 변경 없음.',flush=True)
    try:
        while rclpy.ok() and time.monotonic()-start<a.seconds:rclpy.spin_once(n,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        completed(core.finish())
        guard_stamps={g['diagnostic'].get('scan_source_stamp_ns') for g in guards}
        for e in results:e['matching_guard_inside_or_stop']=e['source_ns'] in guard_stamps
        summary=dict(scans=core.scans,inside_scans=core.inside_scans,invalid_stamp_or_order=core.invalid_order,
                     findings=dict(Counter(h['finding'] for e in results for h in e['hits'])),
                     matched_guard_events=sum(e['matching_guard_inside_or_stop'] for e in results),live_filter_changed=False)
        d=Path(__file__).parent/'logs';d.mkdir(exist_ok=True);path=d/f'scan_shadow_{time.time_ns()}.json'
        path.write_text(json.dumps(dict(summary=summary,events=results,guard_events=list(guards)),ensure_ascii=False,allow_nan=False)+'\n')
        print(json.dumps(summary,ensure_ascii=False,indent=2));print('JSON='+str(path),flush=True)
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
