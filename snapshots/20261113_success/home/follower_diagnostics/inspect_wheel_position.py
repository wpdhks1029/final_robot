#!/usr/bin/env python3
"""PID4/216-only position sampling; no torque-off, enable, speed or setting writes.

Does NOT release motor holding torque. Lift wheels; never force rotation against
motor resistance. Default invocation opens nothing. Position units remain unknown.
"""
import argparse
import json
from pathlib import Path
import time
from read_md200t_settings import ReadOnlyPort,query
from md200t_pid216_v2 import decode


def delta32(current,previous):return ((current-previous+2**31)%2**32)-2**31


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-read-only',action='store_true')
    args=parser.parse_args()
    if not args.run_read_only:parser.print_help();return 0
    import yaml
    cfg=yaml.safe_load(Path(__file__).with_name('follower_hardware.yaml').read_text())['follower_md200t_pid4']['ros__parameters']
    if cfg['driver_id']!=1 or cfg['baudrate']!=19200 or not cfg['port'].endswith('usb-0:1.4:1.0-port0'):
        raise RuntimeError('Confirmed serial configuration changed')
    port=None;rows=[];first=None;previous=None;total=[0,0];missing=0
    try:
        port=ReadOnlyPort.open(cfg['port'],19200)
        first=decode(query(port,216))
        if first is None or first['m1_rpm']!=0 or first['m2_rpm']!=0:
            raise RuntimeError('Initial zero feedback not confirmed; nothing changed')
        previous=[first['m1_position'],first['m2_position']]
        print('READ ONLY 45s: motor torque is NOT released. No hand rotation unless wheels are raised and freely turnable.',flush=True)
        start=time.monotonic()
        while time.monotonic()-start<45:
            frame=query(port,216);data=decode(frame) if frame else None
            if data is None:
                missing+=1;print('NO_REPLY: measurement incomplete',flush=True)
            else:
                positions=[data['m1_position'],data['m2_position']]
                for i in (0,1):total[i]+=delta32(positions[i],previous[i])
                previous=positions
                row=dict(elapsed=round(time.monotonic()-start,3),**data)
                rows.append(row)
                print(json.dumps(dict(**row,delta_m1=total[0],delta_m2=total[1])),flush=True)
            time.sleep(.1)
        print(f'POSITION_DELTA raw M1={total[0]}, M2={total[1]}; missing={missing}. Units not assumed.',flush=True)
    finally:
        if port is not None:port.close()
        if rows:
            directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
            target=directory/f'wheel_position_{time.time_ns()}.json'
            target.write_text(json.dumps(dict(initial=first,samples=rows,delta=total,missing=missing),indent=2)+'\n')
            print('JSON='+str(target))
    return 0


if __name__=='__main__':raise SystemExit(main())
