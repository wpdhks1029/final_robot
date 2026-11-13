#!/usr/bin/env python3
"""Offline trimmed scan registration. Estimates only; no control changes."""
import argparse,json,math,time
from pathlib import Path
import numpy as np

def analyze(rows):
    initial=next(r for r in rows if r['event']=='planning_input')
    stopped=next(r for r in reversed(rows) if r['event']=='stopped_control')
    target=np.array(initial['points']);source=np.array(stopped['points'])
    pivot=np.array(stopped['control']['pose'][:2])
    target=target[np.linalg.norm(target-np.array(initial['start'][:2]),axis=1)<3.5]
    source=source[np.linalg.norm(source-pivot,axis=1)<3.5]
    def squared_distances(q):return ((q[:,None,:]-target[None,:,:])**2).sum(axis=2)
    def metrics(q):
        d=np.sqrt(squared_distances(q).min(axis=1))
        return dict(median_nearest_m=float(np.median(d)),matched_within_10cm=int((d<.1).sum()))
    fits=[]
    for initial_degrees in (-30,-15,0,15,30):
        a=math.radians(initial_degrees);rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        trans=pivot-rot@pivot
        for _ in range(50):
            q=source@rot.T+trans;d=squared_distances(q);ids=d.argmin(axis=1)
            nearest=np.sqrt(d[np.arange(len(q)),ids]);mask=nearest<=min(.6,float(np.quantile(nearest,.65)))
            u=q[mask];v=target[ids[mask]]
            if len(u)<20:break
            uc=u.mean(axis=0);vc=v.mean(axis=0);U,_,V=np.linalg.svd((u-uc).T@(v-vc));R=V.T@U.T
            if np.linalg.det(R)<0:V[-1]*=-1;R=V.T@U.T
            T=vc-R@uc;rot=R@rot;trans=R@trans+T
        fits.append(dict(initial_guess_deg=initial_degrees,correction_yaw_deg=math.degrees(math.atan2(rot[1,0],rot[0,0])),
                         translation_m=trans.tolist(),robot_position_delta_m=(rot@pivot+trans-pivot).tolist(),
                         **metrics(source@rot.T+trans)))
    good=[f for f in fits if f['median_nearest_m']<.025 and f['matched_within_10cm']>.6*len(source)]
    return dict(before=metrics(source),source_points=len(source),target_points=len(target),fits=fits,
                good_fit_yaw_range_deg=[min(f['correction_yaw_deg'] for f in good),max(f['correction_yaw_deg'] for f in good)] if good else None,
                control_input_changed=False,automatic_calibration_applied=False,
                limitations=['Offline initial/stop scan association can be ambiguous',
                             'Moving objects and partial visibility exist',
                             'Trimmed registration is diagnostic only; no live point masking',
                             'Independent map/IMU heading confirmation is required'])

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial-log',type=Path,required=True);a=p.parse_args()
    result=dict(source=str(a.trial_log),**analyze([json.loads(s) for s in a.trial_log.read_text().splitlines()]))
    out=Path(__file__).parent/'logs'/f'local_scan_alignment_{time.time_ns()}.json'
    out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));print('JSON='+str(out))

if __name__=='__main__':main()
