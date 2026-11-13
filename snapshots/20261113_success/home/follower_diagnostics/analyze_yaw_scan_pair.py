#!/usr/bin/env python3
"""Offline successful yaw/scan comparison. No ROS, motor or live calibration."""
import json,math,time
from pathlib import Path
import numpy as np
from local_stop_turn_go import pose_at
from analyze_local_scan_alignment import analyze
from follower_yaw_diagnostics import synchronized_summary

ROOT=Path(__file__).resolve().parent

def clouds(d):
    rows=[(r['stamp_ns'],r['rx_mono'],(r['x'],r['y'],r['yaw'],r['v'],r['w'])) for r in d['streams']['filtered']]
    begin=d['summary']['measurement_begin_ns'];origin,_,_=pose_at(rows,begin,max_v=.09,max_w=.33)
    mount=d['mount'];tx,ty,_=mount['translation'];x,y,z,w=mount['rotation_xyzw']
    mount_yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));snapshots=[]
    for s in d.get('motion_scans',[]):
        try:p,_,_=pose_at(rows,s['source_ns'],max_v=.09,max_w=.33)
        except ValueError:continue
        local=[]
        for i,r in enumerate(s['ranges']):
            if r is None or not s['range_min']<=r<=s['range_max']:continue
            a=s['angle_min']+i*s['angle_increment']+mount_yaw
            bx=tx+r*math.cos(a);by=ty+r*math.sin(a)
            wx=p.x+math.cos(p.yaw)*bx-math.sin(p.yaw)*by-origin.x
            wy=p.y+math.sin(p.yaw)*bx+math.cos(p.yaw)*by-origin.y
            local.append((math.cos(origin.yaw)*wx+math.sin(origin.yaw)*wy,-math.sin(origin.yaw)*wx+math.cos(origin.yaw)*wy))
        dx,dy=p.x-origin.x,p.y-origin.y
        snapshots.append((s['source_ns'],local,(math.cos(origin.yaw)*dx+math.sin(origin.yaw)*dy,-math.sin(origin.yaw)*dx+math.cos(origin.yaw)*dy,p.yaw-origin.yaw)))
    initial=min(snapshots,key=lambda s:abs(s[0]-begin))
    final=snapshots[-1]
    if final[0]-initial[0]<2e9:raise ValueError('SCAN_WINDOW_TOO_SHORT')
    return analyze([dict(event='planning_input',points=initial[1],start=initial[2]),dict(event='stopped_control',points=final[1],control=dict(pose=final[2]))])

def main():
    paths=[ROOT/'logs'/('follower_yaw_trial_'+str(s)+'.json') for s in (1793857922493753783,1793858071674686175)]
    results=[]
    for p in paths:
        d=json.loads(p.read_text());end=min(v[-1]['stamp_ns'] for v in d['streams'].values())
        summary=synchronized_summary(d['streams'],begin_ns=d['summary']['measurement_begin_ns'],end_ns=end)
        fit=clouds(d);best=min(fit['fits'],key=lambda f:f['median_nearest_m'])
        result=dict(source=str(p),sensor_comparison=summary,scan_alignment=fit)
        results.append(result)
        print(json.dumps(dict(source=p.name,angles_deg={k:v.get('delta_deg') for k,v in summary['sources'].items()},scan_before=fit['before'],diagnostic_best_fit=best),ensure_ascii=False),flush=True)
    out=ROOT/'logs'/f'yaw_scan_pair_analysis_{time.time_ns()}.json'
    out.write_text(json.dumps(dict(results=results,automatic_calibration_applied=False,control_input_changed=False),indent=2)+'\n');print('JSON='+str(out))
if __name__=='__main__':main()
