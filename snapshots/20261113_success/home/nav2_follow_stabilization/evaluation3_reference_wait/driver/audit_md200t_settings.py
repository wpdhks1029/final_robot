#!/usr/bin/env python3
"""Opt-in PID4-only setting read and comparison with the saved older record.

No motor commands, setting writes, resets, alarm clearing, or ROS services.
Changing values back is deliberately not implemented.
"""
import json
from pathlib import Path
import time
import read_md200t_settings as reader

BASELINE=Path.home()/'follower_diagnostics/logs/md200t_settings_1793259986665866485.json'


def compare_settings(baseline,current):
    def index(rows):
        result={}
        for row in rows:
            pid=row['pid']
            if pid in result:raise ValueError('DUPLICATE_PID')
            result[pid]=row
        return result
    old,new=index(baseline),index(current)
    rows=[]
    for pid in sorted(old.keys()|new.keys()):
        a,b=old.get(pid),new.get(pid)
        known=(a is not None and b is not None and a.get('result')=='OK'
               and b.get('result')=='OK' and type(a.get('raw_unsigned')) is int
               and type(b.get('raw_unsigned')) is int)
        rows.append(dict(pid=pid,name=(b or a).get('name'),
            result=('MATCH' if a['raw_unsigned']==b['raw_unsigned'] else 'DIFFERENT') if known else 'UNKNOWN',
            baseline_raw=a.get('raw_unsigned') if a and a.get('result')=='OK' else None,
            current_raw=b.get('raw_unsigned') if b and b.get('result')=='OK' else None))
    return dict(settings=rows,different_pids=[r['pid'] for r in rows if r['result']=='DIFFERENT'],
        unknown_pids=[r['pid'] for r in rows if r['result']=='UNKNOWN'],
        physical_sensor_health_verified=False,motion_authorized=False)


def main():
    if not BASELINE.is_file():raise SystemExit('Saved baseline missing: '+str(BASELINE))
    baseline=json.loads(BASELINE.read_text())
    collected=[];original=reader.value
    def collect(pid,frame):
        result=original(pid,frame);collected.append(dict(result));return result
    reader.value=collect
    try:
        result=reader.main()
    finally:
        reader.value=original
    if collected:
        report=dict(baseline=str(BASELINE),**compare_settings(baseline,collected))
        report['meaning']='Raw settings comparison only; equal values do not clear an encoder fault or authorize motion.'
        out=Path(__file__).parent/'logs'/f'md200t_settings_comparison_{time.time_ns()}.json'
        out.parent.mkdir(exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
        print('COMPARISON='+str(out),flush=True)
    return result


if __name__=='__main__':raise SystemExit(main())
