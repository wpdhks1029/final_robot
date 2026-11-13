#!/usr/bin/env python3
"""Offline recorded-scene replay. No ROS, motor, arm, or services."""
import json,time
import replay_apf_pivot as replay
from gentle_pivot_candidate import GentlePivot,build_gentle_plan
rows=[json.loads(s) for s in open('logs/local_avoidance_trial_1793781323794712578.jsonl')]
r=next(r for r in rows if r['event']=='planning_input');plan=build_gentle_plan(r['points'],r['start'])
class Controller(GentlePivot):
 def __init__(self,start,box):super().__init__(start,box,plan['gentle_route'])
replay.APFPivot=Controller
results=[dict(replay.replay(rows,seed=s),controller_kind='GENTLE_PIVOT_CHECKED_ROUTE_V1') for s in (7,11,19)]
p='logs/gentle_pivot_replay_'+str(time.time_ns())+'.json'
open(p,'w').write(json.dumps(dict(source='local_avoidance_trial_1793781323794712578.jsonl',config=plan['config'],geometry=plan['geometry'],results=results)))
for r in results:print({k:v for k,v in r.items() if k!='trace'})
print(p)
