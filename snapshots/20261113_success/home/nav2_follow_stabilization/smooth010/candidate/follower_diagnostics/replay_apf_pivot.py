#!/usr/bin/env python3
"""Replay captured scans only. Never publishes or opens motor devices."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
from apf_pivot_candidate import APFPivot,swept_clearance
from compare_avoidance_strategies import advance

ROOT=Path(__file__).resolve().parent

def replay(rows,seed=7,limit=600,from_stop=False,stage_candidate=False,box_shift_y=0.):
    initial=next(r for r in rows if r.get('event')=='planning_input')
    plan=next(r['plan'] for r in rows if r.get('event')=='checked_local_plan')
    cloud=np.asarray(plan['points']);start=tuple(initial['start']);pose=start
    # Known 20cm box depth is retained along with every recorded scan point.
    front=cloud[(cloud[:,0]>.35)&(cloud[:,0]<1.3)&(abs(cloud[:,1])<.4)]
    near=plan['box_far']-.2
    boxpoints=front[abs(front[:,0]-near)<.04]
    if not len(boxpoints):raise ValueError('BOX_NOT_IDENTIFIED')
    box=dict(near=near,far=plan['box_far'],high=float(boxpoints[:,1].max()))
    if 'box' in plan:box=plan['box']
    rng=np.random.RandomState(seed)
    if from_stop:
        stopped=next(r for r in reversed(rows) if r.get('event')=='stopped_control')
        pose=tuple(stopped['control']['pose'])
        cloud=np.asarray(stopped['points']+plan['box_boundary'])
    shifted=0
    if box_shift_y:
        if 'low' not in box:raise ValueError('Explicit box bounds required for hypothetical movement')
        cloud=cloud.copy()
        mask=((cloud[:,0]>=box['near']-.03)&(cloud[:,0]<=box['far']+.03)&
              (cloud[:,1]>=box['low']-.03)&(cloud[:,1]<=box['high']+.03))
        shifted=int(mask.sum());cloud[mask,1]+=box_shift_y
        box=dict(box,low=box['low']+box_shift_y,high=box['high']+box_shift_y)
    controller_type=APFPivot
    if stage_candidate:
        from apf_pivot_stage_candidate import APFPivot as controller_type
    controller=controller_type(start,box)
    history=[pose]*3;velocity=np.zeros(2);brake=0;previous=None;minimum=99.;trace=[];joined=None
    reason='SIM_TIME_LIMIT';stalled=0;previous_position=pose
    for i in range(int(limit/.1)):
        measured=np.array(history[max(0,len(history)-2)])+rng.normal(0,.003,3)
        v,w,phase=controller.command(tuple(measured),cloud)
        if phase.startswith('STOP'):reason=phase;break
        if phase in ('TURN','DRIVE') and phase!=previous:
            brake=3;previous=phase
        if brake:brake-=1;v=w=0.
        d=swept_clearance(cloud,[pose])-.01;minimum=min(minimum,d)
        if d<=.3:reason='STOP_TRUE_CLEARANCE';break
        factor=min(1.,max(0.,(d-.3)/.3));v*=factor;w*=factor
        rpm=np.round(np.array([v-w*.28,v+w*.28])/.065*60/(2*math.pi))
        if w and not np.any(rpm):rpm=np.array([-math.copysign(1.,w),math.copysign(1.,w)])
        if abs(rpm).max()>10:raise ValueError('RPM_LIMIT')
        qv=float(rpm.mean()*.065*2*math.pi/60);qw=float((rpm[1]-rpm[0])*.065*2*math.pi/60/.56)
        future=[advance(pose,qv,qw,k*.1) for k in range(11)]
        swept=swept_clearance(cloud,future)-.01-(abs(qv)+math.hypot(.27,.3)*abs(qw))*.1
        if swept<=.3:reason='STOP_SWEPT_CLEARANCE';break
        velocity+=(np.array([qv,qw])-velocity)*(.1/.12)
        pose=advance(pose,*velocity,.1);history.append(pose)
        if pose[0]>4.5 or abs(pose[1])>1.25:
            reason='STOP_SIM_WORKSPACE_LIMIT';break
        if math.hypot(pose[0]-previous_position[0],pose[1]-previous_position[1])>.006 or abs(pose[2]-previous_position[2])>.015:
            stalled=i;previous_position=pose
        if i-stalled>150:reason='STOP_SIM_STALLED';break
        if i%10==0:trace.append(dict(t=i*.1,pose=list(pose),phase=phase,clearance_m=d,**controller.last_diagnostic))
        rear=pose[0]-.27*abs(math.cos(pose[2]))-.3*abs(math.sin(pose[2]))
        if rear>box['far'] and pose[0]>=controller.join_x and abs(pose[1])<=.1 and abs(pose[2])<=math.radians(10):
            if joined is None:joined=pose[0]
            if pose[0]-joined>=1:reason='SIM_PASS_BODY_CLEAR_REJOIN_PLUS_1M';break
        else:joined=None
    return dict(reason=reason,passed=reason=='SIM_PASS_BODY_CLEAR_REJOIN_PLUS_1M',duration_s=i*.1,
                seed=seed,min_clearance_m=minimum,final_pose=list(pose),trace=trace,
                replay_start='RECORDED_STOP_POSE' if from_stop else 'INITIAL_POSE',
                stage_candidate=stage_candidate,hypothetical_box_shift_y_m=box_shift_y,shifted_box_points=shifted,
                simulation_only=True,evaluation_pass=False,control_connected=False,
                limitations=['Static initial scan; no unseen obstacles','Ideal fresh inputs; no wireless model',
                             'Follower only; leader coordination not validated','3mm pose noise, 0.1s delay; no wheel slip'])

def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--trial-log',type=Path,default=ROOT/'logs/local_avoidance_trial_1793770821520744310.jsonl')
    a.add_argument('--seeds',type=int,nargs='+',default=[7,11,19])
    a.add_argument('--from-stop',action='store_true',help='Offline only: retain original frame/box and replay from recorded stop')
    a.add_argument('--stage-candidate',action='store_true',help='Offline only: experimental stage/short-step candidate')
    a.add_argument('--box-shift-y',type=float,default=0.,help='Offline hypothetical box movement; does not modify recorded files')
    cfg=a.parse_args()
    rows=[json.loads(s) for s in cfg.trial_log.read_text().splitlines()]
    results=[replay(rows,seed,from_stop=cfg.from_stop,stage_candidate=cfg.stage_candidate,box_shift_y=cfg.box_shift_y) for seed in cfg.seeds]
    out=ROOT/'logs'/f'apf_pivot_replay_{time.time_ns()}.json'
    out.write_text(json.dumps(dict(source=str(cfg.trial_log),results=results),allow_nan=False)+'\n')
    for r in results:print(json.dumps({k:v for k,v in r.items() if k!='trace'}))
    print('JSON='+str(out))

if __name__=='__main__':main()
