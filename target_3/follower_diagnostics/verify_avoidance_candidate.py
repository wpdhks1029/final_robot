#!/usr/bin/env python3
"""Verification of fixed selected candidate; no live control access."""
import argparse,json,math,time,hashlib
from pathlib import Path
import numpy as np
from compare_avoidance_strategies import dogleg,simulate,StopTurnGo,PreparedPathReview
ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--return-buffer',type=float,choices=(.75,.85,.95),default=.75)
    parser.add_argument('--output',type=Path,default=ROOT/'logs/avoidance_candidate_verification.json')
    args=parser.parse_args()
    planfile=ROOT/'logs/current_scene_left_detour_plan.json';raw=planfile.read_bytes();data=json.loads(raw);digest=hashlib.sha256(raw).hexdigest()
    points=list(data['snapshot']['points']);blockers=set()
    trial=ROOT/'logs/joint_follow_trial_1793709530898986785.jsonl'
    for line in trial.read_text().splitlines():
        row=json.loads(line);d=row.get('diagnostic',{});b=d.get('blocking')
        if d.get('plan_sha256')==digest and b:blockers.add(tuple(b['point']))
    points.extend(sorted(blockers))
    box=[(x,y) for x,y in points if .65<x<1.35 and abs(y)<.4]
    near=min(x for x,y in box);far=max(data['obstacle_far_x_m'],max(x for x,y in box),near+.2)
    low=min(y for x,y in box);high=max(y for x,y in box)
    for q in np.linspace(0,1,41):points.extend([(near,low+(high-low)*q),(far,low+(high-low)*q),(near+(far-near)*q,low),(near+(far-near)*q,high)])
    start=data['path'][0][:3];config=dict(advance_x=.16,offset=.82,return_x=far+args.return_buffer,speed=.06,turn=.2,pulse=True)
    route,goals=dogleg(start,config['advance_x'],config['offset'],config['return_x'])
    review=PreparedPathReview(route).review(points)
    if review['reason']!='LIVE_PATH_GEOMETRY_CLEAR':raise RuntimeError('Selected geometry blocked')
    results=[]
    cases=[dict(noise=.003,delay=.1,geometry_uncertainty=0.,odom_scale=0.,yaw_bias=0.),
           dict(noise=.005,delay=.2,geometry_uncertainty=.01,odom_scale=.01,yaw_bias=math.radians(.5)),
           dict(noise=.005,delay=.2,geometry_uncertainty=.02,odom_scale=-.01,yaw_bias=math.radians(-.5))]
    for case,settings in enumerate(cases):
        for seed in (101,113,127,149,163):
            slip=.03 if seed in (101,127,163) else -.03
            r=simulate(np.asarray(points),start,StopTurnGo(goals,config['speed'],config['turn'],config['pulse']),far,config['return_x'],
                       seed=seed,slip=slip,quantized=True,limit=900.,leader_policy='coordinated_stop_allowed',**settings)
            r.update(case=case,config=config);results.append(r)
            print(json.dumps({k:v for k,v in r.items() if k!='trace'}),flush=True)
    output=dict(source_plan=str(planfile),source_sha256=digest,source_trial=str(trial),
                points=points,retained_original_points=len(data['snapshot']['points']),additional_live_blockers=len(blockers),
                start=start,route=route,goals=goals,config=config,geometry=review,box_bounds=dict(near=near,far=far,low=low,high=high),
                results=results,simulation_only=True,evaluation_pass=False)
    dest=args.output;dest.write_text(json.dumps(output,allow_nan=False)+'\n')
    print('JSON='+str(dest),flush=True)

if __name__=='__main__':main()
