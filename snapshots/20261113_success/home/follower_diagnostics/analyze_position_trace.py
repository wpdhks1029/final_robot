#!/usr/bin/env python3
"""Offline comparison of an assumed 60 position units/revolution; no ROS/serial.

Stops are analysis windows, not a live motion filter. Never hides raw changes.
"""
import argparse
import json
import math
from pathlib import Path
import statistics

from follower_feedback_trace import delta32


def analyze(rows,units=60.):
    if len(rows)<2:raise ValueError('Need at least two samples')
    if not math.isfinite(units) or units<=0:raise ValueError('Invalid assumed units/revolution')
    normalized=[];previous=None;total=[0,0]
    for row in rows:
        if previous is not None:
            if row['elapsed']<=previous['elapsed']:raise ValueError('Nonincreasing sample time')
            total[0]-=delta32(row['m1_position'],previous['m1_position'])
            total[1]+=delta32(row['m2_position'],previous['m2_position'])
        normalized.append(dict(t=row['elapsed'],phase=row['phase'],left=total[0],right=total[1],
                               rpm_left=-row['m1_rpm'],rpm_right=row['m2_rpm']))
        previous=row
    done=[r for r in normalized if r['phase']=='DONE']
    if not done:raise ValueError('No DONE samples; no confirmed completion window')
    if any(r['m1_status'] or r['m2_status'] for r in rows):raise ValueError('Nonzero device status; inspect before comparison')
    scale=math.pi*.130/units
    def metric(left,right):
        return dict(left_units=left,right_units=right,
                    left_angle_deg=left*360/units,right_angle_deg=right*360/units,
                    mean_distance_mm=(left+right)*.5*scale*1000)
    def median_metric(rr):
        return metric(statistics.median(r['left'] for r in rr),statistics.median(r['right'] for r in rr))
    early=[r for r in done if r['t']<=done[0]['t']+2]
    late=[r for r in done if r['t']>=done[-1]['t']-2]
    movement=[]
    for a,b in zip(done,done[1:]):
        dl=b['left']-a['left'];dr=b['right']-a['right']
        if dl or dr:movement.append(dict(t=b['t'],delta_left=dl,delta_right=dr,
                                      rpm_left=b['rpm_left'],rpm_right=b['rpm_right']))
    return dict(assumption_only=True,units_per_rev=units,diameter_m=.130,
                samples=len(rows),seconds=normalized[-1]['t']-normalized[0]['t'],
                first_done=metric(done[0]['left'],done[0]['right']),
                early_done_2s_median=median_metric(early),late_done_2s_median=median_metric(late),
                final_raw=metric(normalized[-1]['left'],normalized[-1]['right']),
                rpm_odom_final_x_mm=(rows[-1]['odom_x']-rows[0]['odom_x'])*1000,
                done_position_range_units=dict(left=[min(r['left'] for r in done),max(r['left'] for r in done)],
                                               right=[min(r['right'] for r in done),max(r['right'] for r in done)]),
                done_change_events=movement,
                note='No calibration or live stationary suppression. Zero reported RPM does not prove zero physical motion.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trace',type=Path)
    args=p.parse_args()
    rows=[json.loads(line) for line in args.trace.read_text().splitlines() if line.strip()]
    result=analyze(rows)
    output=args.trace.with_name(args.trace.stem+'_position_review.json')
    output.write_text(json.dumps(result,indent=2)+'\n')
    for key in ['samples','seconds','units_per_rev','first_done','early_done_2s_median',
                'late_done_2s_median','final_raw','rpm_odom_final_x_mm','done_position_range_units']:
        print(key+'='+json.dumps(result[key]))
    print('done_change_events='+str(len(result['done_change_events'])))
    print('ANALYSIS_JSON='+str(output));print(result['note'])


if __name__=='__main__':main()
