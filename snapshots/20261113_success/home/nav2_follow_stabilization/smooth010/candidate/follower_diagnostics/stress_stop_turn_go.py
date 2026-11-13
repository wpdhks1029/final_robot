#!/usr/bin/env python3
"""Offline robustness sweep; does not import ROS or access hardware."""
import json,math,time
from pathlib import Path
import numpy as np
from compare_avoidance_strategies import dogleg,simulate,StopTurnGo,PreparedPathReview
ROOT=Path(__file__).resolve().parent

def main():
    data=json.loads((ROOT/'logs/current_scene_left_detour_plan.json').read_text())
    base=json.loads((ROOT/'logs/avoidance_strategy_comparison.json').read_text())
    pts=list(data['snapshot']['points']);source=ROOT/'logs/joint_follow_trial_1793709530898986785.jsonl'
    for line in source.read_text().splitlines():
        r=json.loads(line);d=r.get('diagnostic',{});b=d.get('blocking')
        if d.get('plan_sha256')==base['source_sha256'] and b:pts.append(tuple(b['point']))
    box=base['box_bounds'];near=box['near'];far=max(data['obstacle_far_x_m'],box['far']);low=box['low'];high=box['high']
    for q in np.linspace(0,1,41):pts.extend([(near,low+(high-low)*q),(far,low+(high-low)*q),(near+(far-near)*q,low),(near+(far-near)*q,high)])
    points=np.asarray(pts);start=data['path'][0][:3];attempts=[];qualified=[]
    for offset in (.78,.82):
        route,goals=dogleg(start,.16,offset,far+.75);geometry=PreparedPathReview(route).review(pts)
        if geometry['reason']!='LIVE_PATH_GEOMETRY_CLEAR':
            print('geometry blocked '+str(offset),flush=True);continue
        for speed,turn,pulse in ((.05,.12,False),(.06,.2,False),(.06,.2,True)):
            c=dict(advance_x=.16,offset=offset,return_x=far+.75,speed=speed,turn=turn,pulse=pulse)
            results=[]
            for seed,slip in ((7,0.),(19,.03),(31,-.03)):
                r=simulate(points,start,StopTurnGo(goals,speed,turn,pulse),far,far+.75,
                           seed,.003,.1,slip,True)
                r.update(config=c,stage='selection');results.append(r)
                print(json.dumps({k:v for k,v in r.items() if k!='trace'}),flush=True)
            attempts.extend(results)
            if all(r['passed'] for r in results):qualified.append((max(r['duration_s'] for r in results),c,goals,geometry))
    if not qualified:raise RuntimeError('No strategy qualified; no success claim')
    qualified.sort(key=lambda q:q[0]);_,chosen,goals,geometry=qualified[0];robust=[]
    cases=[(.003,.1,0.,0.,0.),(.005,.2,.01,.01,math.radians(.5)),(.005,.2,.02,-.01,math.radians(-.5))]
    for number,(noise,delay,uncertainty,scale,bias) in enumerate(cases):
        for seed in (7,19,31,43,59):
            slip=.03 if seed%2 else -.03
            if seed in (19,43):slip=-.03
            r=simulate(points,start,StopTurnGo(goals,chosen['speed'],chosen['turn'],chosen['pulse']),far,far+.75,
                       seed,noise,delay,slip,True,limit=900.,geometry_uncertainty=uncertainty,odom_scale=scale,yaw_bias=bias)
            r.update(config=chosen,case=number,stage='robustness');robust.append(r)
            print(json.dumps({k:v for k,v in r.items() if k!='trace'}),flush=True)
    output=dict(source_comparison=str(ROOT/'logs/avoidance_strategy_comparison.json'),
                corrected_box_far_m=far,points=pts,start=start,selection_trials=attempts,
                chosen=chosen,geometry=geometry,robustness_trials=robust,
                simulation_only=True,evaluation_pass=False)
    target=ROOT/'logs/stop_turn_go_robustness.json';target.write_text(json.dumps(output,allow_nan=False)+'\n')
    print('JSON='+str(target),flush=True)

if __name__=='__main__':main()
