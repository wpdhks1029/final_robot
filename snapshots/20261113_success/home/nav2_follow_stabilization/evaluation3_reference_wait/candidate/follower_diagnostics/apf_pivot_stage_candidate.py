"""EXPERIMENTAL ONLY: stage switching and adaptive step exploration. No ROS or serial access.

APF chooses a heading; every rotation and straight segment is independently
checked with the rectangular body. The left bypass preference stays locked.
"""
import math
import numpy as np
from apf_case3_candidate import wrap
from compare_avoidance_strategies import advance,body_points


def swept_clearance(points,poses):
    p=np.asarray(poses);cloud=np.asarray(points)
    dx=cloud[:,0][None,:]-p[:,0,None];dy=cloud[:,1][None,:]-p[:,1,None]
    co=np.cos(p[:,2,None]);si=np.sin(p[:,2,None])
    return float(np.hypot(np.maximum(abs(co*dx+si*dy)-.27,0),
                          np.maximum(abs(-si*dx+co*dy)-.30,0)).min())


def checked_step(points,pose,heading,length,margin=.33):
    delta=wrap(heading-pose[2]);n=max(1,math.ceil(abs(delta)/math.radians(1)))
    turn=[(pose[0],pose[1],pose[2]+delta*k/n) for k in range(n+1)]
    n=max(1,math.ceil(length/.01))
    drive=[advance((pose[0],pose[1],heading),length*k/n,0,1) for k in range(n+1)]
    # Bounds between samples: <=1 degree corner motion and <=1cm translation.
    clearance=swept_clearance(points,turn+drive)
    return clearance>margin+.01,clearance,turn+drive


class APFPivot:
    def __init__(self,start,box,step=.10):
        self.box=box;self.step=step;self.stage=0;self.phase='SELECT'
        self.target=None;self.heading=None;self.last_diagnostic={}
        # Intermediate attraction targets enforce a left bypass and then rejoin.
        self.offset=max(start[1]+.65,box['high']+.3+.3+.08)
        self.targets=[(max(start[0],box['near']-.65),self.offset),
                      (box['far']+.70,self.offset),
                      (box['far']+1.40,0.),(box['far']+2.70,0.)]
        self.join_x=box['far']+1.40

    def choose(self,pose,points):
        x,y,a=pose
        rear=x-.27*abs(math.cos(a))-.3*abs(math.sin(a))
        # The outward waypoint is guidance, not a mandatory arrival point.
        # Transition when a forward-progressing checked step is available.
        if self.stage==0 and y>=self.box['high']+.45:
            self.stage=1;choice=self._choose_step(pose,points)
            if choice is not None and choice[1][0]-x>=.4*math.hypot(choice[1][0]-x,choice[1][1]-y):
                self.last_diagnostic['stage_transition']='MOVE_OUT_TO_PASS_BOX'
                return choice
            self.stage=0
        if self.stage==1 and rear>self.box['far']+.30:
            self.stage=2;choice=self._choose_step(pose,points)
            if choice is not None and (choice[1][1]<y-.02 or abs(y)<.08):
                self.last_diagnostic['stage_transition']='PASS_BOX_TO_REJOIN'
                return choice
            self.stage=1
        if self.stage==2 and rear>self.box['far']+.30 and abs(y)<.08:
            self.stage=3
        return self._choose_step(pose,points)

    def _choose_step(self,pose,points):
        x,y,a=pose
        # After rejoining, continue along the reference line. A fixed final
        # attraction point would make APF turn back and circle at that point.
        target=np.array((x+.35,0.) if self.stage==len(self.targets)-1 else self.targets[self.stage])
        delta=target-np.array([x,y]);distance=float(np.linalg.norm(delta))
        attraction=delta/max(distance,.001)
        # Limit scan density influence: use the nearest return per 10-degree bin.
        local=body_points(np.asarray(points),pose);ranges=np.linalg.norm(local,axis=1)
        angles=np.arctan2(local[:,1],local[:,0]);repulsion=np.zeros(2)
        for index in range(36):
            mask=(np.floor((angles+math.pi)/(2*math.pi)*36).astype(int)%36)==index
            ids=np.flatnonzero(mask)
            if not len(ids):continue
            i=ids[np.argmin(ranges[ids])];v=np.asarray(points[i])-[x,y];r=float(ranges[i])
            if .001<r<1.1:repulsion-=v/r*min(2.,(.0+1/r-1/1.1)/r**2)*.035
        force=attraction+repulsion;desired=math.atan2(force[1],force[0])
        choices=[]
        for heading in np.linspace(-math.pi,math.pi,73)[:-1]:
            for step in (self.step,.06,.04,.02):
                length=min(step,distance)
                if length<.015:continue
                end=np.array([x+length*math.cos(heading),y+length*math.sin(heading)])
                # Do not move back to the right of the box while bypassing its side.
                if self.stage<=1 and end[0]<self.box['far']+.35 and end[1]<y-.025:continue
                turn_cost=.005 if self.stage==len(self.targets)-1 else .035
                # Compare progress per metre so a long outward step cannot
                # dominate a shorter forward step and lead into a dead end.
                cost=(float(np.linalg.norm(target-end))-distance)/length
                cost+=turn_cost*abs(wrap(heading-a))+.06*(1-math.cos(wrap(heading-desired)))+.002/length
                choices.append((cost,float(heading),length))
        # Geometry is a feasibility condition, not part of the cost: checking in
        # cost order gives exactly the same optimum without testing every ray.
        for checked,(_,h,length) in enumerate(sorted(choices),1):
            ok,clearance,path=checked_step(points,pose,h,length)
            if not ok:continue
            self.last_diagnostic=dict(stage=self.stage,apf_heading_rad=desired,selected_heading_rad=h,
                                      checked_clearance_m=clearance,side='left',checked_candidates=checked,
                                      step_length_m=length,
                                      stage_name=('MOVE_OUT','PASS_BOX','REJOIN','TRACK_REFERENCE')[self.stage])
            return h,(x+length*math.cos(h),y+length*math.sin(h)),path
        return None

    def command(self,pose,points):
        if self.phase=='SELECT':
            selected=self.choose(pose,points)
            if selected is None:return 0.,0.,'STOP_APF_NO_CHECKED_STEP'
            self.heading,self.target,_=selected;self.segment_length=math.hypot(self.target[0]-pose[0],self.target[1]-pose[1]);self.phase='TURN'
        if self.phase=='TURN':
            err=wrap(self.heading-pose[2])
            if abs(err)>math.radians(2):return 0.,math.copysign(min(.2,max(.12,abs(err))),err),'TURN'
            self.phase='DRIVE'
            return 0.,0.,'BRAKE'
        delta=np.array(self.target)-pose[:2]
        along=float(delta@[math.cos(self.heading),math.sin(self.heading)])
        if along<=min(.012,.25*getattr(self,'segment_length',.1)):
            self.phase='SELECT';return 0.,0.,'BRAKE'
        # Straight motion only. Heading corrections happen after a stop.
        if abs(wrap(self.heading-pose[2]))>math.radians(5):
            self.phase='TURN';return 0.,0.,'BRAKE'
        return .06,0.,'DRIVE'


class APFPivotFSM:
    """Adapter for the existing bounded local motor contract and brake gate."""
    def __init__(self,start,box):
        self.controller=APFPivot(start,box);self.points=None;self.i=-1
        self.goals=[];self.previous=None
        self.reselect_count=0;self.reselect_position=None

    def command(self,pose):
        if self.reselect_position is None or math.hypot(pose[0]-self.reselect_position[0],pose[1]-self.reselect_position[1])>.05:
            self.reselect_position=pose[:2];self.reselect_count=0
        c=self.controller;selecting=c.phase=='SELECT';v,w,phase=c.command(pose,self.points)
        if selecting:self.previous=None
        if phase=='BRAKE':return 0.,0.,'BRAKE'
        if phase.startswith('STOP'):return v,w,phase
        if phase!=self.previous:
            self.previous=phase;self.i+=1
            self.goals.append((phase,(*c.target,c.heading)))
        length=math.hypot(c.target[0]-pose[0],c.target[1]-pose[1])
        ok,clearance,_=checked_step(self.points,pose,c.heading,length)
        if not ok:
            self.reselect_count+=1
            c.last_diagnostic.update(blocked_reselects=self.reselect_count,blocked_step_clearance_m=clearance)
            if self.reselect_count>3:return 0.,0.,'STOP_APF_STEP_BLOCKED'
            # Stop before selecting a replacement. LocalCore requires 3 fresh
            # zero-wheel feedback samples before SELECT may run again.
            c.phase='SELECT';self.previous=None
            return 0.,0.,'BRAKE'
        return v,w,phase


def build_apf_pivot_plan(points,start):
    from apf_rejoin_candidate import connected_groups
    points=[tuple(p) for p in points]
    front=[(x,y) for x,y in points if start[0]+.35<x<start[0]+1.6 and abs(y-start[1])<.65]
    groups=[g for g in connected_groups(front,.1) if len(g)>=5 and
            .08<=max(y for x,y in g)-min(y for x,y in g)<=.4 and
            max(x for x,y in g)-min(x for x,y in g)<=.4]
    if not groups:raise ValueError('NO_OBSTACLE_CLUSTER')
    group=min(groups,key=lambda g:min(x for x,y in g))
    near=min(x for x,y in group);low=min(y for x,y in group);high=max(y for x,y in group)
    far=max(near+.2,max(x for x,y in group));box=dict(near=near,far=far,low=low,high=high)
    boundary=[]
    for f in np.linspace(0,1,41):
        boundary.extend([(near,low+(high-low)*f),(far,low+(high-low)*f),
                         (near+(far-near)*f,low),(near+(far-near)*f,high)])
    cloud=points+boundary;controller=APFPivot(start,box)
    choice=controller.choose(start,np.asarray(cloud))
    if choice is None:raise ValueError('STOP_APF_NO_CHECKED_STEP')
    h,target,path=choice
    return dict(controller_kind='APF_PIVOT_STEP_V1',box=box,box_boundary=boundary,
                box_far=far,return_x=controller.join_x,start=list(start),points=cloud,
                path=[[*p,0.] for p in path],goals=[],
                config=dict(side='left',step_m=.1,initial_heading_deg=math.degrees(h)),
                geometry=dict(reason='APF_INITIAL_STEP_CLEAR',scope='INITIAL_STEP_ONLY',
                              min_path_clearance_m=controller.last_diagnostic['checked_clearance_m']),
                attempts=[controller.last_diagnostic],evaluation_pass=False)
