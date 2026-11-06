#!/usr/bin/env python3
"""Offline comparison of independent subscriber reception around an input expiry."""
import argparse,json
from pathlib import Path

def gaps(rows,window):
    inside=[r for r in rows if window[0]<=r[0]<=window[1]]
    pairs=[dict(source_gap_s=(b[0]-a[0])/1e9,receive_gap_s=b[1]-a[1],begin_ns=a[0],end_ns=b[0]) for a,b in zip(inside,inside[1:])]
    if not pairs:return dict(samples=len(inside),maximum_receive_gap_s=None,maximum_source_gap_s=None)
    return dict(samples=len(inside),maximum_receive_gap_s=max(x['receive_gap_s'] for x in pairs),maximum_source_gap_s=max(x['source_gap_s'] for x in pairs),worst=max(pairs,key=lambda x:x['receive_gap_s']))

def compare(trial,observation):
    fail=next(x for x in reversed(trial) if x['event']=='input_expired');window=(fail['ros_ns']-3_000_000_000,fail['ros_ns']+1_000_000_000)
    evidence=next(x for x in reversed(trial) if x['event']=='stop_scan_evidence')
    result=dict(reason='INPUT_DELIVERY_COMPARISON',failure_ros_ns=fail['ros_ns'],expired_age_s=fail['control']['source_age_after_compute_s'],compute_s=fail['control'].get('compute_s'),control_motion=fail['control']['motion'],channels={},control_connected=False)
    for channel,key in (('filtered','pose_odometry_history'),('imu','imu_orientation_history')):
        operator=gaps([(x[0],x[1]) for x in evidence[key]],window)
        observer=gaps([(x['stamp_ns'],x['rx_mono']) for x in observation['streams'].get(channel,[])],window)
        a,b=operator['maximum_receive_gap_s'],observer['maximum_receive_gap_s']
        label='INCONCLUSIVE'
        if a is not None and b is not None:
            if a>.3 and b<.2:label='OPERATOR_DELIVERY_GAP_NOT_SEEN_BY_OBSERVER'
            elif a>.3 and b>.3:label='GAP_VISIBLE_TO_BOTH_SUBSCRIBERS'
        result['channels'][channel]=dict(operator=operator,observer=observer,classification=label)
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',type=Path,required=True);p.add_argument('--observation',type=Path,required=True);a=p.parse_args()
    result=compare(list(map(json.loads,a.trial.read_text().splitlines())),json.loads(a.observation.read_text()))
    result.update(source_trial=str(a.trial),source_observation=str(a.observation),note='Two subscribers cannot by themselves prove the sensor publisher stopped; QoS and host scheduling remain possible causes')
    target=a.trial.with_name(a.trial.stem+'_input_delivery.json');target.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));print('JSON='+str(target))

if __name__=='__main__':main()
