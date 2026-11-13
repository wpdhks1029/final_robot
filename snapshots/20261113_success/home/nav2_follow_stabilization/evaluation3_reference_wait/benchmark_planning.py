"""Offline comparison. No ROS, serial, robot commands or settings changes."""
import cProfile
import importlib.util
import io
import json
from pathlib import Path
import pstats
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'candidate/follower_diagnostics'))
import continuous_avoidance_candidate as optimized
import continuous_avoidance_reference as reference
reference.CLEARANCE=optimized.CLEARANCE
from apf_pivot_candidate import obstacle_scene


def main():
    rows=[]
    for fixture in sorted((ROOT/'fixtures').glob('*.json')):
        data=json.loads(fixture.read_text());scene=obstacle_scene(data['points'],data['start'])
        box=scene['box'];start=data['start'];cloud=scene['points']
        # Fixed identical candidate; compare geometric outcome and full trace.
        path=reference.route(start,box,box['high']+.30+.20+.04,.65,1.)
        measurements={};outputs={}
        for name,module in [('reference',reference),('optimized',optimized)]:
            elapsed=[]
            for _ in range(3):
                begin=time.perf_counter();result=module.replay(cloud,start,path)
                elapsed.append(time.perf_counter()-begin)
            measurements[name]=elapsed;outputs[name]=result
        assert outputs['reference']==outputs['optimized'],fixture.name
        row=dict(fixture=fixture.name,obstacle_points=len(cloud),path_poses=len(path),
                 full_result_identical=True,result_reason=outputs['reference']['reason'],
                 elapsed_s=measurements,speedup=statistics.median(measurements['reference'])/statistics.median(measurements['optimized']))
        rows.append(row);print(json.dumps(row),flush=True)
    report=dict(hardware_executed=False,scope='fixed candidate replay only; no Raspberry Pi end-to-end claim',results=rows)
    (ROOT/'benchmark_results.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
