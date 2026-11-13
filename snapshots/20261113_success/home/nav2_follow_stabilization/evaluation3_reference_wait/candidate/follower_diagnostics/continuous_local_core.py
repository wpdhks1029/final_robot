"""Explicit continuous20cm profile; separate from the legacy local core."""
import math
from evaluation3_clearance_policy import PLAN_CLEARANCE_M,STOP_CLEARANCE_M,PROFILE
from types import SimpleNamespace as S
import numpy as np
from local_stop_turn_go import LocalCore,minimum_clearance
from apf_pivot_candidate import obstacle_scene,swept_clearance
from continuous_avoidance_candidate import search,route
from compare_avoidance_strategies import advance


def build_plan(points,start):
    old=obstacle_scene(points,start)
    result=search(old['points'],start,old['box'])
    if result['reason']!='SIM_REJOIN_PLUS_1M':raise ValueError(result['reason'])
    path=route(start,old['box'],result['offset_m'],result['approach_m'],result['rejoin_m'])
    old['return_x']=old['box_far']+1.4
    old.update(attempts=result.get('attempts',[]),evaluation_pass=False,path=[[*p,0.] for p in path],continuous_path=path,live_map_rejoin=True)
    old['config']=dict(profile=PROFILE,clearance_m=PLAN_CLEARANCE_M,stop_clearance_m=STOP_CLEARANCE_M,max_v=.025,**{k:result[k] for k in ('offset_m','approach_m','rejoin_m')})
    old['geometry']=dict(reason='CONTINUOUS_STATIC_REPLAY_PASS',scope='STATIC_CAPTURE_ONLY',min_path_clearance_m=result['minimum_m'])
    return old

def route_still_clear(plan,points):
    # Fine path samples plus 3cm reserve; runtime checks remain independent.
    return swept_clearance(points,plan['continuous_path'])>PLAN_CLEARANCE_M+.03

class ContinuousCore(LocalCore):
    def __init__(self,**kw):
        super().__init__(**kw);self.index=0;self.last_curve=None;self.follow_workspace_max_x=4.5
    def install(self,plan):
        self.plan=plan;self.index=0
        self.fsm=S(controller=S(stage=0,phase='CURVE',map_guidance=None))
    def handoff_to_follow(self,motor,*,follow_distance_m=1.):
        if not motor.get('continuous20') or motor.get('armed') is not True or motor.get('feedback_fresh') is not True:
            raise ValueError('CONTINUOUS_HANDOFF_REQUIRES_FRESH_ARMED_FEEDBACK')
        if follow_distance_m not in (1.,2.):raise ValueError('INVALID_POST_REJOIN_DISTANCE')
        if follow_distance_m>1. and self.last_pose is not None:
            self.follow_workspace_max_x=max(4.5,self.last_pose[0]+follow_distance_m+.5)
        self.plan=None;self.fsm=None;self.geometry_complete=False
    def command(self,now,mono,motor,follow_command=None):
        scene=self.scene(now,mono);pose=scene['pose'];p=(pose.x,pose.y,pose.yaw)
        cloud=scene['points'];u=scene['uncertainty']
        if self.plan is not None:cloud=np.concatenate((cloud,np.asarray(self.plan['box_boundary'])))
        d=minimum_clearance(cloud,p)-u
        out=dict(v=0.,w=0.,motion='STOP',reason='STOP_CONTINUOUS_NOT_READY',source_stamp_ns=scene['source'],scan_stamp_ns=scene['scan_stamp'],
            clearance_m=d,swept_clearance_m=d,pose=list(p),uncertainty_m=u,geometry_complete=False,input_source_stamps=scene['source_stamps'])
        if d<=STOP_CLEARANCE_M:out['reason']='STOP_FOOTPRINT_CLEARANCE';return out
        max_x=self.follow_workspace_max_x if self.fsm is None else 4.5
        if p[0]>max_x or abs(p[1])>1.25:out['reason']='STOP_APF_WORKSPACE_BOUND';return out
        if self.last_pose is not None and (math.dist(p[:2],self.last_pose[:2])>.1 or abs(math.atan2(math.sin(p[2]-self.last_pose[2]),math.cos(p[2]-self.last_pose[2])))>.12):
            out['reason']='STOP_FILTERED_ODOM_JUMP';return out
        self.last_pose=p
        if self.fsm is None:
            v,w=follow_command or (0.,0.)
            ratio=min(1.,.025/v) if v>0 else 0.
            v=min(.025,max(0.,v)*ratio);w=max(-.2,min(.2,w*ratio))
            if not v:return dict(out,reason='FOLLOW_WAITING_LEADER')
        else:
            c=self.fsm.controller
            if p[0]>self.plan['box']['near']-.2:c.stage=max(1,c.stage)
            rear=p[0]-.27*abs(math.cos(p[2]))-.3*abs(math.sin(p[2]))
            if rear>self.plan['box_far']+.30:c.stage=max(2,c.stage)
            if c.stage>=2:
                g=c.map_guidance
                if g is None:return dict(out,reason='STOP_MAP_GUIDANCE_MISSING')
                target=g['target_local']
                c.stage=3 if g['path_distance_m']<=.10 and abs(g['path_heading_error_deg'])<=10 else 2
                out['source_stamp_ns']=min(out['source_stamp_ns'],g['source_stamp_ns'])
            else:
                xy=np.asarray(self.plan['continuous_path'])[:,:2];end=min(len(xy),self.index+80)
                self.index+=int(np.argmin(np.linalg.norm(xy[self.index:end]-p[:2],axis=1)))
                j=self.index
                while j<len(xy)-1 and np.linalg.norm(xy[j]-p[:2])<.12:j+=1
                target=xy[j]
            dx,dy=target[0]-p[0],target[1]-p[1];co,si=math.cos(p[2]),math.sin(p[2]);x=co*dx+si*dy;y=-si*dx+co*dy
            if x<=0:return dict(out,reason='STOP_TARGET_BEHIND')
            curvature=2*y/max(dx*dx+dy*dy,.001)
            if abs(curvature)>8:return dict(out,reason='STOP_CURVATURE_INFEASIBLE')
            v=.025;w=v*curvature
            out['apf']=dict(stage=c.stage,stage_name=('MOVE_OUT','PASS_BOX','REJOIN','TRACK_REFERENCE')[c.stage],phase='CURVE',selected_heading_rad=math.atan2(dy,dx))
        scale=min(1.,10*.065*2*math.pi/60/(v+abs(w)*.28));v*=scale;w*=scale
        dt=.05 if self.last_curve is None else min(.1,max(.001,mono-self.last_curve[0]))
        old_v,old_w=(0.,0.) if self.last_curve is None or mono-self.last_curve[0]>.3 else self.last_curve[1:]
        v=min(v,old_v+.05*dt)
        w=max(old_w-.4*dt,min(old_w+.4*dt,w))
        w=max(-8*v,min(8*v,w))
        if v<=0:w=0.
        # Slewing yaw separately can retain a larger turn than the new target.
        # Recheck both wheels after that slew, not only before it.
        final_scale=min(1.,10*.065*2*math.pi/60/max(v+abs(w)*.28,1e-12))
        v*=final_scale;w*=final_scale
        rpm=np.round(np.array([v-w*.28,v+w*.28])/.065*60/(2*math.pi))
        qv=float(rpm.mean()*.065*2*math.pi/60);qw=float((rpm[1]-rpm[0])*.065*2*math.pi/60/.56)
        out['wheel_diagnostic']=dict(rounded_rpm=rpm.tolist(),quantized_v=qv,quantized_w=qw,final_scale=final_scale)
        if qv==0 and max(abs(rpm))<=10:
            # Tiny positive translation may round to [-1,+1]. Send STOP,
            # retaining the virtual ramp so the next step can move forward.
            self.last_curve=(mono,v,w)
            return dict(out,reason='CONTINUOUS_RAMP_WARMUP')
        if qv<=0 or max(abs(rpm))>10:return dict(out,reason='STOP_WHEEL_BOUND')
        actual=self.inputs[self.pose_source][2]
        sweeps=[]
        for sv,sw in ((qv*1.35,qw*1.35),(actual[3],actual[4])):
            bound=u+(abs(sv)+math.hypot(.27,.30)*abs(sw))*.1
            sweeps.append(swept_clearance(cloud,[advance(p,sv,sw,k*.1) for k in range(11)])-bound)
        out['swept_clearance_m']=min(d,*sweeps)
        if out['swept_clearance_m']<=STOP_CLEARANCE_M:return dict(out,reason='STOP_SWEPT_CLEARANCE')
        self.last_curve=(mono,v,w)
        return dict(out,v=v,w=w,motion='DRIVE',reason='CONTINUOUS_READY')
