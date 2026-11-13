#!/usr/bin/env python3
"""Offline strategy comparison. No ROS, services, ports or live control changes."""
import argparse,json,math,time,hashlib
from pathlib import Path
import numpy as np
from temporary_detour_candidate import Pose,pursue,plan,Settings
from apf_case3_candidate import wrap
from apf_rejoin_candidate import StraightRejoin
from watch_temporary_detour_shadow import PreparedPathReview

ROOT=Path(__file__).resolve().parent
R=.065;TRACK=.560;DT=.1

def clearance(points,pose):
    x,y,a=pose;dx=points[:,0]-x;dy=points[:,1]-y;co,si=math.cos(a),math.sin(a)
    return float(np.hypot(np.maximum(abs(co*dx+si*dy)-.27,0),np.maximum(abs(-si*dx+co*dy)-.3,0)).min())

def body_points(points,pose):
    x,y,a=pose;dx=points[:,0]-x;dy=points[:,1]-y;co,si=math.cos(a),math.sin(a)
    return np.column_stack((co*dx+si*dy,-si*dx+co*dy))

def advance(p,v,w,dt):
    x,y,a=p;b=a+w*dt
    if abs(w)<1e-10:return (x+v*math.cos(a)*dt,y+v*math.sin(a)*dt,b)
    return (x+v*(math.sin(b)-math.sin(a))/w,y+v*(math.cos(a)-math.cos(b))/w,b)

def dogleg(start,ax,offset,back):
    """Stop, turn and translate separately; no reverse translation."""
    x,y,a=start;out=[(*start,0.)];goals=[]
    def turn(b):
        nonlocal a
        delta=wrap(b-a);steps=max(1,math.ceil(abs(delta)/math.radians(1)))
        for k in range(1,steps+1):out.append((x,y,a+delta*k/steps,0.))
        a=b;goals.append(('TURN',(x,y,a)))
    def move(xx,yy):
        nonlocal x,y
        steps=max(1,math.ceil(math.hypot(xx-x,yy-y)/.01))
        for k in range(1,steps+1):out.append((x+(xx-x)*k/steps,y+(yy-y)*k/steps,a,0.))
        x,y=xx,yy;goals.append(('DRIVE',(x,y,a)))
    turn(0.);move(ax,y);turn(math.pi/2);move(ax,offset)
    turn(0.);move(back,offset);turn(-math.pi/2);move(back,0.)
    turn(0.);move(back+1.2,0.)
    return out,goals

class StopTurnGo:
    def __init__(self,goals,v=.05,w=.12,pulse=False):self.goals=goals;self.i=0;self.v=v;self.w=w;self.pulse=pulse
    def command(self,pose):
        x,y,a=pose
        while self.i<len(self.goals):
            phase,(gx,gy,ga)=self.goals[self.i]
            err=wrap(ga-a)
            if phase=='TURN':
                if abs(err)<=math.radians(1.5):self.i+=1;continue
                command=max(-self.w,min(self.w,1.0*err))
                if self.pulse:command=math.copysign(max(.12,abs(command)),err)
                return 0.,command,phase
            along=math.cos(ga)*(gx-x)+math.sin(ga)*(gy-y)
            lateral=-math.sin(ga)*(gx-x)+math.cos(ga)*(gy-y)
            if along<=(.015 if self.pulse else .008):self.i+=1;continue
            if abs(lateral)>.10:return 0.,0.,'STOP_CROSS_ERROR'
            v=self.v if self.pulse else min(self.v,max(.012,.7*along))
            w=1.2*err+1.0*lateral
            w=max(-min(.12,.98*v/.28),min(min(.12,.98*v/.28),w))
            wheel_speed_limit=10*R*2*math.pi/60
            scale=min(1.,.98*wheel_speed_limit/(v+abs(w)*.28))
            return v*scale,w*scale,phase
        return 0.,0.,'END'

class CurvePP:
    def __init__(self,path,lookahead=.15):self.path=path;self.i=0;self.lookahead=lookahead
    def command(self,pose):
        v,w,self.i,state=pursue(self.path,Pose(*pose),.06,self.i,self.lookahead)
        return v,w,state

class APFController:
    def __init__(self):self.core=StraightRejoin(Settings(lidar_height=.25,visibility_confirmed=True),1);self.stamp=1
    def command(self,pose,points):
        self.stamp+=1;r=self.core.command(.06,0.,body_points(points,pose).tolist(),pose,[(0.,0.),(6.,0.)],self.stamp)
        return r['v'],r['w'],r['reason']

def simulate(points,start,controller,far,join_x,seed=7,noise=.003,delay=.1,slip=0.,quantized=True,limit=600.,geometry_uncertainty=0.,odom_scale=0.,yaw_bias=0.,leader_policy="constant"):
    rng=np.random.RandomState(seed);pose=tuple(start);history=[];trace=[];minimum=999.;joined=None;reason='TIME_LIMIT'
    maxrpm=0.;lastmotion=0.;wheel=np.zeros(2);leaderx=None;previous_forward=0.;min_center_gap=999.
    # Same leader policy for all strategies; leader geometry is an explicit assumption.
    leader0=.27*abs(math.cos(start[2]))+.3*abs(math.sin(start[2]))+1.+.27
    for i in range(round(limit/DT)):
        t=i*DT;history.append(pose)
        past=history[max(0,len(history)-1-round(delay/DT))]
        measured=(past[0]*(1+odom_scale)+rng.normal(0,noise),
                  past[1]*(1+odom_scale)+rng.normal(0,noise),past[2]+yaw_bias+rng.normal(0,noise))
        if leaderx is None:leaderx=leader0
        if leader_policy=='constant':leaderx=leader0+.04*t;leader_v=.04
        elif leader_policy=='coordinated_stop_allowed':
            leader_v=max(0.,min(.04,previous_forward+.5*(pose[0]+leader0-start[0]-leaderx)))
            if i:leaderx+=leader_v*DT
        else:raise ValueError('Unknown leader policy')
        min_center_gap=min(min_center_gap,leaderx-pose[0]);q=np.linspace(0,1,12)
        leader=np.array([(leaderx-.27,-.3+.6*z) for z in q]+[(leaderx+.27,-.3+.6*z) for z in q]+[(leaderx-.27+.54*z,-.3) for z in q]+[(leaderx-.27+.54*z,.3) for z in q])
        obstacles=np.concatenate((points,leader))
        d=clearance(obstacles,pose)-geometry_uncertainty;minimum=min(minimum,d)
        if d<=.3:reason='STOP_TRUE_CLEARANCE';break
        if isinstance(controller,APFController):v,w,state=controller.command(measured,obstacles)
        else:v,w,state=controller.command(measured)
        if state.startswith('STOP'):reason=state;break
        if state=='END':reason='END_BEFORE_EVALUATION';break
        # Same existing all-direction 0.30/0.60 m guard for every candidate.
        sensed=clearance(obstacles,measured)-geometry_uncertainty;factor=max(0.,min(1.,(sensed-.3)/.3));v*=factor;w*=factor
        requested=np.array([(v-w*.28)/R*60/(2*math.pi),(v+w*.28)/R*60/(2*math.pi)])
        if abs(requested).max()>10.+1e-8:reason='STOP_RPM_LIMIT';break
        if quantized:requested=np.round(requested)
        maxrpm=max(maxrpm,float(abs(requested).max()))
        # 0.12s first-order motor response and unequal left/right travel.
        wheel+=(requested-wheel)*min(1.,DT/.12)
        linear=wheel*R*2*math.pi/60*np.array([1-slip,1+slip])
        actualv=float(linear.mean());actualw=float((linear[1]-linear[0])/TRACK)
        # Conservative 1s swept rectangular check including quantized wheel response.
        nearby=obstacles[(abs(obstacles[:,0]-pose[0])<1.)&(abs(obstacles[:,1]-pose[1])<1.)]
        if len(nearby):
            margin=.3+(abs(actualv)+math.hypot(.27,.3)*abs(actualw))*.1
            future=np.asarray([advance(pose,actualv,actualw,k*.1) for k in range(1,11)])
            dx=nearby[:,0][None,:]-future[:,0,None];dy=nearby[:,1][None,:]-future[:,1,None]
            co=np.cos(future[:,2,None]);si=np.sin(future[:,2,None])
            distances=np.hypot(np.maximum(abs(co*dx+si*dy)-.27,0),np.maximum(abs(-si*dx+co*dy)-.3,0))
            if float(distances.min())-geometry_uncertainty<=margin:
                reason='STOP_SWEPT_CLEARANCE';break
        newpose=advance(pose,actualv,actualw,DT)
        previous_forward=actualv*math.cos(pose[2])
        # Independent midpoint check of physical integration.
        mid=clearance(obstacles,advance(pose,actualv,actualw,DT/2))-geometry_uncertainty;minimum=min(minimum,mid)
        if mid<=.3:reason='STOP_TRUE_CLEARANCE';break
        if abs(actualv)+abs(actualw)>.001:lastmotion=t
        if t-lastmotion>15.:reason='STALLED_QUANTIZATION_OR_LOCAL_MINIMUM';break
        if i%5==0:trace.append(dict(t=round(t,2),x=pose[0],y=pose[1],yaw=pose[2],clearance=d,v=actualv,w=actualw,phase=state,leader_x=leaderx,leader_v=leader_v))
        x,y,a=pose;rear=x-.27*abs(math.cos(a))-.3*abs(math.sin(a))
        if rear>far and x>=join_x and abs(y)<=.10 and abs(wrap(a))<=math.radians(10):
            if joined is None:joined=x
            if x-joined>=1.:reason='SIM_PASS_BODY_CLEAR_REJOIN_PLUS_1M';break
        else:joined=None
        pose=newpose
    return dict(reason=reason,passed=reason=='SIM_PASS_BODY_CLEAR_REJOIN_PLUS_1M',duration_s=round(t,2),
        min_true_clearance_m=minimum,max_abs_wheel_rpm=maxrpm,seed=seed,pose_noise_std=noise,pose_delay_s=delay,
        wheel_slip_fraction=slip,integer_rpm=quantized,geometry_uncertainty_m=geometry_uncertainty,
        odometry_scale_error=odom_scale,yaw_bias_rad=yaw_bias,leader_policy=leader_policy,
        leader_travel_m=leaderx-leader0,min_longitudinal_center_gap_m=min_center_gap,final_pose=pose,trace=trace,simulation_only=True,evaluation_pass=False)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,default=ROOT/'logs/avoidance_strategy_comparison.json')
    a=p.parse_args();planfile=ROOT/'logs/current_scene_left_detour_plan.json';raw=planfile.read_bytes();data=json.loads(raw)
    original=list(data['snapshot']['points']);points=list(original)
    livefile=ROOT/'logs/joint_follow_trial_1793709530898986785.jsonl';blockers=[]
    if livefile.exists():
        for line in livefile.read_text().splitlines():
            row=json.loads(line)
            if row.get('event')=='guard_diagnostic':
                d=row['diagnostic'];b=d.get('blocking')
                if d.get('plan_sha256')==hashlib.sha256(raw).hexdigest() and b:blockers.append(tuple(b['point']))
    blockers=sorted(set(blockers));points.extend(blockers)
    # Fill the known 20cm box; never remove or translate recorded points.
    box=[(x,y) for x,y in points if .65<x<1.35 and abs(y)<.4]
    near=min(x for x,y in box);far=max(data['obstacle_far_x_m'],max(x for x,y in box),near+.2)
    low=min(y for x,y in box);high=max(y for x,y in box)
    for q in np.linspace(0,1,41):points.extend([(near,low+(high-low)*q),(far,low+(high-low)*q),(near+(far-near)*q,low),(near+(far-near)*q,high)])
    pts=np.asarray(points);start=data['path'][0][:3];attempts=[];candidates=[]
    # Compare the prior fixed path to live evidence, not just its original snapshot.
    oldreview=PreparedPathReview(data['path']).review(points)
    candidates.append(('fixed_curve_pp',lambda:CurvePP(data['path']),data['join_x_m'],dict(geometry=oldreview)))
    # Replan curves with the augmented actual scene. The original planner retains all points.
    for padding in (.02,.04,.06):
        import temporary_detour_candidate as module
        original_clear=module.path_clear
        def inflated(path,cloud,c,padding=padding):return original_clear(path,cloud,c,padding=padding)
        module.path_clear=inflated
        try:r=plan(points,Settings(),side=1,initial_cross=start[1],initial_yaw=start[2],objective='time',nominal_speed=.06)
        finally:module.path_clear=original_clear
        attempts.append(dict(method='replanned_curve_pp',padding=padding,reason=r['reason']))
        print(json.dumps(attempts[-1]),flush=True)
        if r.get('path'):
            route=r['path'];name=f'replanned_curve_pp_margin_{padding:.2f}'
            candidates.append((name,lambda route=route:CurvePP(route),r['join_x_m'],dict(padding=padding,geometry=PreparedPathReview(route).review(points))))
    # Stop/rotate/straight: search full-footprint valid geometry first.
    doglegs=[]
    for ax in (.08,.12,.16,.20,.24):
        for offset in (.74,.78,.82,.86,.90,.94):
            for back in (far+.75,far+.95,far+1.15):
                route,goals=dogleg(start,ax,offset,back);review=PreparedPathReview(route).review(points)
                attempts.append(dict(method='stop_turn_go',advance_x=ax,offset=offset,return_x=back,reason=review['reason'],clearance=review.get('min_path_clearance_m')))
                if review['reason']=='LIVE_PATH_GEOMETRY_CLEAR':
                    cost=sum(math.hypot(route[i][0]-route[i-1][0],route[i][1]-route[i-1][1])/.04+abs(wrap(route[i][2]-route[i-1][2]))/.10 for i in range(1,len(route)))
                    doglegs.append((cost,goals,back,dict(advance_x=ax,offset=offset,return_x=back,geometry=review)))
    print('checked dogleg geometries='+str(len(doglegs)),flush=True)
    for k,(_,goals,back,config) in enumerate(sorted(doglegs,key=lambda r:r[0])[:5]):
        candidates.append((f'stop_turn_go_{k}',lambda goals=goals:StopTurnGo(goals),back,config))
    candidates.append(('apf_straight_rejoin',lambda:APFController(),far+.8,{}))
    results=[]
    scenarios=[('ideal',0.,0.,0.,False,7),('quantized',0.,0.,0.,True,7),
               ('noise_delay',.003,.1,0.,True,7),('left_right_slip',.003,.1,.03,True,19),
               ('opposite_slip',.003,.1,-.03,True,31)]
    for name,make,join,config in candidates:
        for case,noise,delay,slip,quantized,seed in scenarios:
            r=simulate(pts,start,make(),far,join,seed,noise,delay,slip,quantized)
            r.update(method=name,case=case,config=config);results.append(r)
            print(json.dumps({k:v for k,v in r.items() if k not in ('trace','config')}),flush=True)
    out=dict(source_plan=str(planfile),source_sha256=hashlib.sha256(raw).hexdigest(),
             original_scan_points=len(original),additional_live_blocker_points=len(blockers),all_recorded_points_retained=True,
             simulation_obstacle_points=points,
             box_bounds=dict(near=near,far=far,low=low,high=high),attempts=attempts,results=results,
             limitations=['Static 2D observed points; no visibility/occlusion model','Leader 0.54x0.60m dimensions assumed; constant 0.04m/s',
                          'Ideal fresh local inputs; ROS/network success not claimed','Stop-turn-go requires separately validated pivot permission; current driver forbids it',
                          'AMCL/IMU drift, friction and motor response are simplified','No real evaluation pass; all methods simulated only'])
    a.output.write_text(json.dumps(out,allow_nan=False)+'\n');print('JSON='+str(a.output),flush=True)

if __name__=='__main__':main()
