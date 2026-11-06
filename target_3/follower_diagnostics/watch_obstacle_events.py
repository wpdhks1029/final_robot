#!/usr/bin/env python3
"""정지 중 보호 판단 관찰. 모터·scan 처리·필터·mask 변경 없음."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=int,default=20)
    a=p.parse_args()
    if not 5<=a.seconds<=60:p.error('seconds는5~60이어야 합니다.')
    import rclpy
    from std_msgs.msg import String
    from follower_remote_qos import latest_state_qos
    rclpy.init(args=[]);node=rclpy.create_node('obstacle_event_observer')
    rows=[];last_scan=None;legacy=False
    def receive(m):
        nonlocal last_scan,legacy
        try:d=json.loads(m.data)
        except ValueError:return
        if 'clearance_zone' not in d:
            legacy=True;return
        rows.append(d)
        if d['clearance_zone'] in ('inside','stop') and d.get('scan_source_stamp_ns')!=last_scan:
            last_scan=d.get('scan_source_stamp_ns')
            print('근접 검출: '+json.dumps(d,ensure_ascii=False),flush=True)
    sub=node.create_subscription(String,'/follower/obstacle_preview/status',receive,latest_state_qos())
    try:
        print(f'조회 전용 {a.seconds}초. 양쪽 정지 유지. 모든 scan의 근접 구간 변화 진단을 관찰합니다.',flush=True)
        end=time.monotonic()+a.seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        summary=dict(received=len(rows),zones=dict(Counter(d['clearance_zone'] for d in rows)),
                     unsafe_messages=sum(d['clearance_zone'] in ('inside','stop') for d in rows),
                     old_diagnostic_seen=legacy)
        print(json.dumps(summary,ensure_ascii=False,indent=2))
        if legacy:print('이전 진단 버전이 보입니다. preview launch 재시작 여부를 확인하세요.')
        directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
        path=directory/f'obstacle_events_{time.time_ns()}.jsonl'
        with path.open('x') as f:
            for row in rows:f.write(json.dumps(row,ensure_ascii=False)+'\n')
        print('TRACE='+str(path))
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
