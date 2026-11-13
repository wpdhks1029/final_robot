#!/usr/bin/env python3
"""저장된 scan 이상점의 공간·시간 지지 증거 계산. ROS·모터·필터 변경 없음."""
import argparse
import json
import math
from pathlib import Path


def nearby_support(scan, angle, target, angular_window_deg=2.0, distance_tolerance=.08, exclude_index=None):
    """0/NaN/범위 밖은 빈 공간 증거가 아니라 미관측으로 분류한다."""
    window=math.radians(angular_window_deg);near=[];far=0;unknown=0
    for i,r in enumerate(scan['ranges']):
        if i==exclude_index:continue
        a=scan['angle_min']+i*scan['angle_increment']
        delta=math.atan2(math.sin(a-angle),math.cos(a-angle))
        if abs(delta)>window:continue
        if r is None or not math.isfinite(r) or not scan['range_min']<=r<=scan['range_max']:
            unknown+=1;continue
        if abs(r-target)<=distance_tolerance:near.append(i)
        elif r>target+distance_tolerance:far+=1
    return dict(near_indices=near,far_count=far,unknown_count=unknown)


def analyze_event(event):
    s=event['scan'];out=[]
    for h in event['hits']:
        angle=math.radians(h['angle_deg']);target=h['range_m']
        spatial=nearby_support(s,angle,target,exclude_index=h['index'])
        temporal=[]
        for label in ('previous','following'):
            for other in event[label]:
                dt=(other['source_ns']-s['source_ns'])/1e9
                # 시각이 멀리 떨어진 기록은 비교하지 않는다.
                if not 0<abs(dt)<=.4:continue
                support=nearby_support(other,angle,target)
                temporal.append(dict(side=label,dt_s=dt,**support))
        supported=sum(bool(t['near_indices']) for t in temporal)
        sides={t['side'] for t in temporal}
        isolated=not spatial['near_indices']
        if isolated and supported==0 and sides=={'previous','following'}:
            finding='ISOLATED_TRANSIENT_CANDIDATE'
        elif spatial['near_indices'] or supported:
            finding='SUPPORTED_RETURN'
        else:finding='INSUFFICIENT_CONTEXT'
        out.append(dict(index=h['index'],angle_deg=h['angle_deg'],range_m=target,
                        spatial=spatial,temporal=temporal,temporal_support_scans=supported,
                        finding=finding,control_action='UNCHANGED_STOP'))
    return dict(source_ns=s['source_ns'],hits=out)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture',type=Path);a=p.parse_args()
    data=json.loads(a.capture.read_text());events=[analyze_event(e) for e in data['events']]
    report=dict(input=str(a.capture),parameters=dict(angular_window_deg=2.,distance_tolerance_m=.08,max_context_age_s=.4),
                limitation='후보 분류는 물체가 없다는 증명이 아니다. 미검출은 빈 공간의 증거가 아니다.',
                events=events,live_filter_changed=False)
    output=a.capture.with_name(a.capture.stem+'_analysis.json');output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    for e in events:
        for h in e['hits']:print(json.dumps({k:v for k,v in h.items() if k not in ('spatial','temporal')},ensure_ascii=False))
    print('JSON='+str(output))


if __name__=='__main__':main()
