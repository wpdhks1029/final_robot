#!/usr/bin/env python3
"""Read recorded timing only; no ROS access and no new source timestamps."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def main():
    source=ROOT/'logs/joint_follow_trial_1793709530898986785.jsonl'
    rows=[json.loads(x) for x in source.read_text().splitlines()]
    start=next(r['ros_ns'] for r in rows if r['event']=='leader_start')
    end=next(r['ros_ns'] for r in rows if r['event']=='leader_stop_requested')
    samples=sorted([r for r in rows if r['event']=='timing_sample'],key=lambda r:r['rx_ros_ns'])
    reports={}
    for label,keys in [('map_pose_inputs',('leader','path','follower','scan')),('local_odom_inputs',('leader','filtered','scan'))]:
        latest={};index=0;ages=[];missing=0;owners={}
        for now in range(start,end,25_000_000):
            while index<len(samples) and samples[index]['rx_ros_ns']<=now:
                r=samples[index];latest[r['name']]=r;index+=1
            if any(k not in latest for k in keys):missing+=1;continue
            effective={k:latest[k]['source_ns']-(50_000_000 if k in ('leader','path') else 0) for k in keys}
            oldest=min(effective,key=effective.get);age=(now-effective[oldest])/1e9
            ages.append(age);owners[oldest]=owners.get(oldest,0)+1
        reports[label]=dict(samples=len(ages),missing_samples=missing,
            source_age_p95_s=float(np.percentile(ages,95)),max_source_age_s=max(ages),
            over_0_3s=sum(v>.3 for v in ages),over_0_3s_with_40ms_processing_delivery=sum(v+.04>.3 for v in ages),
            oldest_owners=owners)
    out=dict(source=str(source),leader_window_s=(end-start)/1e9,reports=reports,
             simulation_only=True,controller_and_tf_wait_not_replayed=True,new_network_delivery_not_validated=True)
    target=ROOT/'logs/local_avoidance_timing_replay.json';target.write_text(json.dumps(out)+'\n');print(json.dumps(out,ensure_ascii=False))
if __name__=='__main__':main()
