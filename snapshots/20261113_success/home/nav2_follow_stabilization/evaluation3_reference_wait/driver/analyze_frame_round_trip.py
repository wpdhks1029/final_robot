#!/usr/bin/env python3
"""Offline pivot/20cm/return diagnostic, no live correction or commands."""
import json,math,time
from pathlib import Path
from local_stop_turn_go import pose_at
from analyze_local_scan_alignment import analyze
from analyze_yaw_scan_pair import clouds

ROOT=Path(__file__).resolve().parent

def main():
    trial=ROOT/'logs/local_avoidance_trial_1793858373313526816.jsonl'
    yaw=ROOT/'logs/follower_yaw_trial_1793858643094477410.json'
    rows=list(map(json.loads,trial.read_text().splitlines()));initial=next(r for r in rows if r['event']=='planning_input')
    d=json.loads(yaw.read_text());history=[(r['stamp_ns'],r['rx_mono'],(r['x'],r['y'],r['yaw'],r['v'],r['w'])) for r in d['streams']['filtered']]
    snapshot=None
    for s in reversed(d['motion_scans']):
        try:p,_,_=pose_at(history,s['source_ns'],max_v=.09,max_w=.33);snapshot=s;break
        except ValueError:pass
    if snapshot is None:raise ValueError('NO_MEASURED_SCAN_SUPPORT')
    anchor=initial['pose_anchor'];start=initial['start'];rotation=start[2]-anchor[2];co,si=math.cos(rotation),math.sin(rotation)
    dx,dy=p.x-anchor[0],p.y-anchor[1]
    pose=(start[0]+co*dx-si*dy,start[1]+si*dx+co*dy,p.yaw+rotation)
    tx,ty,_=d['mount']['translation'];x,y,z,w=d['mount']['rotation_xyzw'];mount_yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    cloud=[]
    for i,r in enumerate(snapshot['ranges']):
        if r is None or not snapshot['range_min']<=r<=snapshot['range_max']:continue
        a=snapshot['angle_min']+i*snapshot['angle_increment']+mount_yaw
        bx,by=tx+r*math.cos(a),ty+r*math.sin(a)
        cloud.append((pose[0]+math.cos(pose[2])*bx-math.sin(pose[2])*by,pose[1]+math.sin(pose[2])*bx+math.cos(pose[2])*by))
    result=analyze([initial,dict(event='stopped_control',points=cloud,control=dict(pose=pose))])
    right=clouds(d)
    best=min(result['fits'],key=lambda f:f['median_nearest_m']);right_best=min(right['fits'],key=lambda f:f['median_nearest_m'])
    out=ROOT/'logs'/f'frame_round_trip_analysis_{time.time_ns()}.json'
    output=dict(source_translation_trial=str(trial),source_return_trial=str(yaw),final_local_pose=pose,initial_to_final_scan_alignment=result,return_turn_scan_alignment=right,
                control_input_changed=False,automatic_calibration_applied=False,evaluation_pass=False)
    out.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(dict(final_local_pose=pose,combined_before=result['before'],combined_best_fit=best,return_best_fit=right_best),ensure_ascii=False));print('JSON='+str(out))
if __name__=='__main__':main()
