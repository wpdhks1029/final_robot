#!/usr/bin/env python3
"""정지 상태 조회 전용: 보호 출력 원본 시각을 선두·경로·후발·scan에 대조한다."""
import argparse
from collections import Counter,defaultdict
import json
import os
from pathlib import Path
import time

REMOTE_MARGIN_NS=50_000_000
RAW_NAMES=('leader','path','follower','scan')


def age_summary(values):
    if not values:return None
    a=sorted(values)
    return dict(min_s=a[0],p95_s=a[int((len(a)-1)*.95)],max_s=a[-1])


class BudgetTrace:
    def __init__(self):
        self.rows=[];self.parents=defaultdict(dict);self.decisions=[]

    def add(self,name,stamp_ns,now_ns,mono):
        row=dict(name=name,source_ns=stamp_ns,ros_ns=now_ns,mono=mono,
                 age_s=None if stamp_ns<=0 else (now_ns-stamp_ns)/1e9)
        self.rows.append(row)
        if name in RAW_NAMES and stamp_ns>0:
            effective=stamp_ns-(REMOTE_MARGIN_NS if name in ('leader','path') else 0)
            # 같은 stamp 재발행으로 원본의 최초 관측 시각을 새로 만들지 않는다.
            self.parents[effective].setdefault(name,row)

    def owners(self,stamp):
        return sorted(self.parents.get(stamp,{})) if stamp>0 else []

    def report(self,start,warmup=2.):
        rows=[r for r in self.rows if r['mono']-start>=warmup]
        result=dict(excluded_first_seconds=warmup,remote_clock_margin_s=REMOTE_MARGIN_NS/1e9)
        inputs={}
        for name in RAW_NAMES:
            a=[r for r in rows if r['name']==name and r['source_ns']>0]
            gaps=[b['mono']-a0['mono'] for a0,b in zip(a,a[1:])]
            stamp_gaps=[(b['source_ns']-a0['source_ns'])/1e9 for a0,b in zip(a,a[1:])
                        if b['source_ns']>a0['source_ns']]
            inputs[name]=dict(received=len(a),raw_age=age_summary([r['age_s'] for r in a]),
                              max_rx_gap_s=max(gaps) if gaps else None,
                              max_stamp_gap_s=max(stamp_gaps) if stamp_gaps else None)
        result['inputs']=inputs
        for name in ('pursuit','guard'):
            a=[r for r in rows if r['name']==name]
            valid=[r for r in a if r['source_ns']>0]
            near=[r for r in valid if r['age_s']>=.25]
            groups=Counter('+'.join(self.owners(r['source_ns'])) or 'unmatched' for r in near)
            unique={}
            for r in near:
                old=unique.get(r['source_ns'])
                if old is None or old['age_s']<r['age_s']:unique[r['source_ns']]=r
            examples=[]
            for r in sorted(unique.values(),key=lambda r:r['age_s'],reverse=True)[:6]:
                owners=self.owners(r['source_ns'])
                examples.append(dict(source_ns=r['source_ns'],effective_age_s=r['age_s'],owners=owners,
                    raw_age_when_first_observed_s={owner:self.parents[r['source_ns']][owner]['age_s'] for owner in owners}))
            result[name]=dict(received=len(a),zero_stamp=sum(r['source_ns']<=0 for r in a),
                              effective_age=age_summary([r['age_s'] for r in valid]),
                              over_0_3s=sum(r['age_s']>.3 for r in valid),
                              near_limit_owners=dict(groups),near_limit_examples=examples)
        result['guard_decisions']=dict(Counter(d['reason'] for d in self.decisions if d['mono']-start>=warmup))
        return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=int,default=20)
    a=p.parse_args()
    if not 5<=a.seconds<=30:p.error('seconds는5~30이어야 합니다.')
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10이 필요합니다.')
    import rclpy
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from follower_remote_qos import latest_state_qos
    rclpy.init(args=[]);node=rclpy.create_node('source_budget_observer');trace=BudgetTrace();subs=[]
    def record(name,m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        trace.add(name,stamp,node.get_clock().now().nanoseconds,time.monotonic())
    def diagnostic(m):
        try:
            d=json.loads(m.data)
            trace.decisions.append(dict(mono=time.monotonic(),reason=d.get('stamped_reason','UNKNOWN')))
        except ValueError:pass
    for name,topic,kind in [
        ('leader','/leader/odom',Odometry),('path','/leader/path',RosPath),
        ('follower','/follower/odom_map',Odometry),('scan','/follower/scan',LaserScan),
        ('pursuit','/follower/cmd_vel_stamped_test',TwistStamped),
        ('guard','/follower/cmd_vel_guard_stamped_test',TwistStamped)]:
        subs.append(node.create_subscription(kind,topic,lambda m,n=name:record(n,m),latest_state_qos()))
    subs.append(node.create_subscription(String,'/follower/obstacle_preview/status',diagnostic,latest_state_qos()))
    try:
        print(f'조회 전용 {a.seconds}초. 양쪽 정지 유지. 명령·arm·설정 변경 없음.',flush=True)
        start=time.monotonic();end=start+a.seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        report=trace.report(start)
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
        directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
        path=directory/f'source_budget_{time.time_ns()}.json'
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        raw=path.with_suffix('.jsonl')
        with raw.open('x') as f:
            for row in trace.rows:f.write(json.dumps(row)+'\n')
        print('JSON='+str(path));print('TRACE='+str(raw))
        print('출처는 stamp 일치로 대조하며 unmatched는 원인 미확정입니다. 수신 횟수만으로 손실을 단정하지 않습니다.')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
