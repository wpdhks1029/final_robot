#!/usr/bin/env python3
"""Reconstruct two offline replay inputs from comparison CSV; optional Domain217 runs.

No hardware. New EKFs, reconstructed covariance/yaw/receipt age: approximate only.
This does not test the already-running Domain10 EKF or modify its configuration.
"""
import argparse
import csv
from pathlib import Path
import subprocess
import sys


def convert(rows,source):
    result=[]
    if not any(r['stamp_ns'] for r in rows):raise ValueError('No source stamps')
    for r in rows:
        if r['topic'] not in (source,'/follower/imu/data'):continue
        imu=r['topic']=='/follower/imu/data'
        result.append(dict(topic='/follower/imu/data' if imu else '/follower/odom_raw',
            rx_s=r['rx_s'],stamp_ns=r['stamp_ns'],age_s='0',
            frame='follower/imu_link' if imu else 'follower/odom',child='' if imu else 'follower/base_link',
            x='' if imu else r['x'],y='' if imu else r['y'],yaw='' if imu else '0',
            vx='' if imu else r['vx'],vy='' if imu else '0',wz=r['wz'],status=''))
    if not any(r['topic']=='/follower/odom_raw' for r in result):raise ValueError('Missing '+source)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('csv',type=Path)
    p.add_argument('--run',action='store_true');args=p.parse_args()
    with args.csv.open() as f:rows=list(csv.DictReader(f))
    for label,source in [('rpm','/follower/odom_raw'),('position','/follower/odom_position_test')]:
        converted=convert(rows,source)
        target=args.csv.with_name(args.csv.stem+'_fresh_'+label+'.csv')
        with target.open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(converted[0]));writer.writeheader();writer.writerows(converted)
        print(f'RECONSTRUCTED_{label.upper()}_INPUT={target}',flush=True)
        if args.run:
            print(f'Fresh isolated EKF comparison: {label}; no physical movement.',flush=True)
            result=subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('replay_ekf_diagnostic.py')),
                                   str(target),'--run'])
            if result.returncode:raise RuntimeError(f'{label} replay failed; next replay not started')
    print('Covariance/yaw/age reconstructed. Do not treat as exact bag replay or physical accuracy validation.')


if __name__=='__main__':main()
