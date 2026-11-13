#!/usr/bin/env python3
"""Offline one-dimensional model, not hardware or physical safety validation.

Assumptions: straight line, first-order velocity response tau=.15s, 100ms
measurement delay, fresh 10Hz poses, every third leader RPM sample reports zero.
No wheel slip, obstacles, steering, encoder faults or DDS behavior is modeled.
"""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as S

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'candidate/follower_diagnostics'))
from follower_motor_candidate import MotorCore,Config
from nav2_convoy_gate_core import Nav2ConvoyGateCore
from smooth_tracked_pursuit import SmoothTrackedPreviewCore
from tracked_pursuit_candidate import TrackedPreviewCore


def odom(x,v,ns,child):
    z=S(x=0.,y=0.,z=0.)
    return S(header=S(frame_id='map',stamp=S(sec=ns//10**9,nanosec=ns%10**9)),
        child_frame_id=child,pose=S(pose=S(position=S(x=x,y=0.,z=0.),
            orientation=S(x=0.,y=0.,z=0.,w=1.))),
        twist=S(twist=S(linear=S(x=v,y=0.,z=0.),angular=z)))


def run(smooth, steps=3000, *, initial_gap=1.57, response_tau=.15, delay_steps=5, zero_every=3):
    controller=SmoothTrackedPreviewCore() if smooth else TrackedPreviewCore()
    controller.timeout=.8
    gate=Nav2ConvoyGateCore(max_linear=.1);gate.graph_ok=True
    motor=MotorCore(config=Config(max_rpm=15))
    lx,fx,lv,fv=initial_gap,0.,0.,0.
    history=[];records=[]
    for i in range(steps):
        t=i*.02;ns=100_000_000_000+round(t*1e9)
        history.append((lx,fx,lv,fv))
        source_i=max(0,i-delay_steps);sample_i=source_i-source_i%5
        pl,pf,pv,pfv=history[sample_i]
        source=100_000_000_000+round(sample_i*.02*1e9)
        reported=0. if zero_every and (sample_i//5)%zero_every==0 else pv
        controller.odom_callback('leader',odom(pl,reported,source,'base_link'),ns,t)
        controller.odom_callback('follower',odom(pf,pfv,source,'follower/base_link'),ns,t)
        stamp=S(sec=source//10**9,nanosec=source%10**9)
        controller.inputs['path']=(stamp,t,((-5.,0.),(30.,0.)))
        v,w,reason=controller.command(ns,t)
        v=min(.1,max(0.,v))
        raw=motor.rpm(v,w)
        target_f=(-raw[0]+raw[1])*math.pi*.065/60
        if smooth:controller.feedback_bounded(v,w,t)
        nav_v=0. if 30.<=t<35. else .12
        gate.update_nav(nav_v,0.,t)
        status_reason={'STOP_LEADER_STOPPED':'FOLLOW_WAITING_LEADER',
                       'STOP_FOLLOW_DISTANCE':'FOLLOW_WAITING_DISTANCE'}.get(reason,reason)
        gate.update_follower(dict(trial_phase='FOLLOW',reason='TRACKED_PURSUIT' if v>0 else status_reason,
            follower_follow_reason=reason,follower_requested_v=v,smooth_follow=smooth),t)
        gate.update_motor(dict(armed=True,fault=None,io_failed=False,feedback_fresh=True),t)
        gate.update_leader_pose(pl,0.,t);gate.update_follower_pose(pf,0.,t)
        out=gate.evaluate(t)
        gain=1.-math.exp(-.02/response_tau)
        lv+=(out.linear_x-lv)*gain;fv+=(target_f-fv)*gain
        lx+=lv*.02;fx+=fv*.02
        records.append(dict(t=t,gap=lx-fx,follower_request=v,
            leader_request=out.linear_x,nav_request=nav_v,reason=reason))
    moving=[r for r in records if r['t']>2. and not 29.5<=r['t']<=37.]
    active=[r['follower_request']>0 for r in moving]
    return dict(smooth=smooth,initial_gap=initial_gap,response_tau=response_tau,
        delay_s=delay_steps*.02,zero_every=zero_every,
        modeled_drive_stop_edges=sum(a!=b for a,b in zip(active,active[1:])),
        gap_min=min(r['gap'] for r in records),gap_max=max(r['gap'] for r in records),
        max_follower_request=max(r['follower_request'] for r in records),
        minimum_gap_during_scheduled_stop=min(r['gap'] for r in records if 30.<=r['t']<35.),
        final_gap=records[-1]['gap'])


def main():
    report=dict(mode='offline_straight_line_assumption_model',
        assumptions=__doc__,results=[run(False),run(True),
            run(True,initial_gap=1.45,response_tau=.3,delay_steps=10),
            run(True,initial_gap=1.8,response_tau=.3,delay_steps=10),
            run(True,zero_every=0)],hardware_io=False,
        physical_validation=False)
    output=ROOT/'straight_model_comparison.json'
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
