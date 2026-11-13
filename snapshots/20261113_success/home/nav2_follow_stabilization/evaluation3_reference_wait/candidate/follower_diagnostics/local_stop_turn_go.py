"""Pure local-frame stop/turn/go core. No ROS/serial access."""
import math
import time
from collections import deque
from local_avoidance_limits import SOURCE_TIMEOUT_S
import numpy as np
from compare_avoidance_strategies import StopTurnGo,dogleg,advance
from temporary_detour_candidate import Pose
from apf_case3_candidate import wrap
from apf_rejoin_candidate import connected_groups
from watch_temporary_detour_shadow import PreparedPathReview


def pose_at(history,stamp,*,max_v=.06,max_w=.2):
    """Interpolate measured raw odometry; only <=50ms bounded hold at the end."""
    if not history:raise ValueError('RAW_ODOM_MISSING')
    for i in range(len(history)-1):
        a,b=history[i],history[i+1]
        if a[0]<=stamp<=b[0] and 0<b[0]-a[0]<=200_000_000:
            f=(stamp-a[0])/(b[0]-a[0]);pa,pb=a[2],b[2]
            pose=Pose(pa[0]+f*(pb[0]-pa[0]),pa[1]+f*(pb[1]-pa[1]),wrap(pa[2]+f*wrap(pb[2]-pa[2])))
            return pose,min(a[0],b[0]),0.
    last=history[-1];gap=(stamp-last[0])/1e9
    if 0<=gap<=.05:
        return Pose(*last[2][:3]),last[0],(max_v+math.hypot(.27,.3)*max_w)*gap
    raise ValueError('RAW_ODOM_SCAN_BRACKET_MISSING')


def minimum_clearance(points,pose):
    x,y,a=pose;dx=points[:,0]-x;dy=points[:,1]-y;co,si=math.cos(a),math.sin(a)
    return float(np.hypot(np.maximum(abs(co*dx+si*dy)-.27,0),np.maximum(abs(-si*dx+co*dy)-.3,0)).min())


def body_points_for_diagnostic(points,pose):
    x,y,a=pose;dx=points[:,0]-x;dy=points[:,1]-y;co,si=math.cos(a),math.sin(a)
    return np.column_stack((co*dx+si*dy,-si*dx+co*dy))


class LocalPlanFailure(ValueError):
    def __init__(self,reason,diagnostic):
        super().__init__(reason);self.diagnostic=diagnostic


def build_local_plan(points,start,search_seconds=8.):
    """Current full scan, known 20cm depth, left-side geometry search."""
    points=[tuple(p) for p in points]
    front=[(x,y) for x,y in points if start[0]+.35<x<start[0]+1.6 and abs(y-start[1])<.65
           and math.hypot(max(abs(x-start[0])-.27,0),max(abs(y-start[1])-.3,0))<.9]
    groups=[g for g in connected_groups(front,.10) if len(g)>=5]
    if not groups:raise ValueError('NO_OBSTACLE_CLUSTER')
    box=min(groups,key=lambda g:min(x for x,y in g));xs=[x for x,y in box];ys=[y for x,y in box]
    if not .08<=max(ys)-min(ys)<=.4 or max(xs)-min(xs)>.4:raise ValueError('BOX_GEOMETRY_NOT_CONFIRMED')
    near=min(xs);far=max(max(xs),near+.20);low=min(ys);high=max(ys)
    cloud=list(points)
    for q in np.linspace(0,1,41):cloud.extend([(near,low+(high-low)*q),(far,low+(high-low)*q),(near+(far-near)*q,low),(near+(far-near)*q,high)])
    attempts=[];began=time.monotonic()
    preferred=[(.16,.82,.85),(.16,.78,.85),(.16,.82,.95),(.12,.82,.85),(.20,.82,.85),(.16,.86,.85)]
    # Adapt the lateral goal to the measured box edge while retaining all points.
    offsets=sorted(set(round(high+.3+.3+extra,3) for extra in (.04,.08,.12,.16,.20)) |
                   {round(y,3) for y in np.arange(.66,1.101,.04)},key=lambda y:abs(y-.82))
    advances=sorted((0.,.04,.08,.12,.16,.20,.24,.28,.32),key=lambda x:abs(x-.16))
    extended=[(ax,offset,buffer) for offset in offsets for ax in advances for buffer in (.75,.85,.95,1.05,1.15)]
    extended.sort(key=lambda q:abs(q[0]-.16)+abs(q[1]-.82)+.4*abs(q[2]-.85))
    seen=set();exhausted=False
    for ax,offset,buffer in preferred+extended:
        if (ax,offset,buffer) in seen:continue
        seen.add((ax,offset,buffer))
        if time.monotonic()-began>search_seconds:exhausted=True;break
        path,goals=dogleg(start,start[0]+ax,offset,far+buffer)
        geometry=PreparedPathReview(path).review(cloud)
        attempts.append(dict(advance=ax,offset=offset,buffer=buffer,**geometry))
        if geometry['reason']=='LIVE_PATH_GEOMETRY_CLEAR':
            return dict(path=path,goals=goals,geometry=geometry,points=cloud,box_far=far,return_x=far+buffer,
                        config=dict(advance=ax,offset=offset,buffer=buffer),attempts=attempts,
                        planning_s=time.monotonic()-began,simulation_only=False,evaluation_pass=False)
    diagnostic=dict(start=list(start),box=dict(near=near,far=far,low=low,high=high),
                    point_count=len(points),attempts=attempts,search_time_limit_hit=exhausted,
                    planning_s=time.monotonic()-began)
    raise LocalPlanFailure('STOP_NO_CHECKED_STOP_TURN_GO_ROUTE',diagnostic)



class LocalCore:
    def __init__(self,pose_source='raw'):
        if pose_source not in ('raw','filtered'):raise ValueError('INVALID_POSE_SOURCE')
        self.pose_source=pose_source;self.raw_history=deque(maxlen=100);self.imu_history=deque(maxlen=100)
        self.imu_anchor=None;self.raw_yaw_anchor=None
        self.inputs={};self.history=deque(maxlen=100);self.scan=None;self.anchor=None
        self.plan=None;self.fsm=None;self.last_goal=None;self.braking=False
        self.last_pose=None;self.progress_at=None;self.progress_pose=None
        self.stage_started=None;self.stage_limit=90.;self.joined=None;self.geometry_complete=False
        self.scan_cache=None;self.last_scan_stamp=0
        self.last_alignment_failure=None

    def raw(self,stamp,rx,data):
        self._odom('raw',stamp,rx,data)
    def filtered(self,stamp,rx,data):
        self._odom('filtered',stamp,rx,data)
    def imu(self,stamp,rx,data):
        self._odom('imu',stamp,rx,data)
    def _odom(self,key,stamp,rx,data):
        if len(data)!=5 or not all(math.isfinite(x) for x in data):self.inputs.pop(key,None);return
        previous=self.inputs.get(key)
        if previous is not None and stamp<=previous[0]:self.inputs.pop(key,None);return
        if key=='raw' and self.raw_history and stamp<=self.raw_history[-1][0]:self.inputs.pop(key,None);return
        if key==self.pose_source and self.history and stamp<=self.history[-1][0]:self.inputs.pop(key,None);return
        row=(stamp,rx,tuple(data));self.inputs[key]=row
        if key=='raw':self.raw_history.append(row)
        if key=='imu':self.imu_history.append(row)
        if key==self.pose_source:self.history.append(row)
    def offer_scan(self,stamp,rx,body_points):
        self.scan=None;self.scan_cache=None
        if stamp<=self.last_scan_stamp or not body_points or not all(math.isfinite(x) for p in body_points for x in p):return
        self.last_scan_stamp=stamp
        self.scan=(stamp,rx,np.asarray(body_points,dtype=float))
    def freeze(self,raw_pose,reference_pose):
        self.anchor=(Pose(*raw_pose[:3]),Pose(*reference_pose[:3]))
        self.imu_anchor=self.inputs['imu'][2][2] if 'imu' in self.inputs else None
        self.raw_yaw_anchor=self.inputs['raw'][2][2] if 'raw' in self.inputs else None
    def reference_pose(self,raw):
        if self.anchor is None:raise ValueError('ANCHOR_MISSING')
        origin,start=self.anchor;a=wrap(start.yaw-origin.yaw);dx=raw.x-origin.x;dy=raw.y-origin.y
        return Pose(start.x+math.cos(a)*dx-math.sin(a)*dy,
                    start.y+math.sin(a)*dx+math.cos(a)*dy,wrap(start.yaw+wrap(raw.yaw-origin.yaw)))
    def scene(self,now,mono,*,require_leader=True):
        for key in (('raw','filtered','imu','leader') if self.pose_source=='filtered' else ('raw','leader')):
            if key=='leader' and not require_leader:continue
            row=self.inputs.get(key)
            if row is None:raise ValueError(key.upper()+'_MISSING')
            stamp=row[0]-(50_000_000 if key=='leader' else 0)
            if not (stamp>0 and 0<=(now-stamp)/1e9<=SOURCE_TIMEOUT_S and 0<=mono-row[1]<=SOURCE_TIMEOUT_S):raise ValueError(key.upper()+'_STALE')
        if require_leader:
            leader=self.inputs['leader'][2]
            if leader[3]<-.01 or leader[3]>.1 or abs(leader[4])>.1:raise ValueError('LEADER_UNEXPECTED_MOTION')
        heading_check={}
        if self.pose_source=='filtered':
            if self.anchor is None:raise ValueError('ANCHOR_MISSING')
            filtered=self.inputs['filtered'];imu=self.inputs['imu']
            delta=wrap(filtered[2][2]-self.anchor[0].yaw)
            if self.imu_anchor is None:self.imu_anchor=wrap(imu[2][2]-delta)
            imu_delta=wrap(imu[2][2]-self.imu_anchor);error=wrap(delta-imu_delta)
            limit=math.radians(8)+.5*abs(filtered[0]-imu[0])/1e9
            heading_check=dict(filtered_heading_delta_deg=math.degrees(delta),imu_heading_delta_deg=math.degrees(imu_delta),
                               filtered_imu_error_deg=math.degrees(error),filtered_imu_limit_deg=math.degrees(limit))
            if abs(error)>limit:raise ValueError('FILTERED_IMU_HEADING_DIVERGENCE')
        if self.scan is None:raise ValueError('SCAN_MISSING_OR_INVALID')
        stamp,rx,body=self.scan
        if not (0<=(now-stamp)/1e9<=SOURCE_TIMEOUT_S and 0<=mono-rx<=SOURCE_TIMEOUT_S):raise ValueError('SCAN_STALE')
        try:
            scan_odom,pose_support,error=pose_at(self.history,stamp,max_v=.09 if self.pose_source=='filtered' else .06,
                                               max_w=.33 if self.pose_source=='filtered' else .2)
        except ValueError as exc:
            self.last_alignment_failure=dict(pose_source=self.pose_source,scan_stamp_ns=stamp,
                pose_first_ns=self.history[0][0] if self.history else None,
                pose_last_ns=self.history[-1][0] if self.history else None,
                scan_ahead_of_pose_s=(stamp-self.history[-1][0])/1e9 if self.history else None,
                kind='WAITING_FOR_NEWER_POSE' if self.history and stamp>self.history[-1][0] else 'POSE_HISTORY_GAP',
                history_tail=[row[0] for row in list(self.history)[-8:]])
            raise ValueError(str(exc).replace('RAW_ODOM','FILTERED_ODOM') if self.pose_source=='filtered' else str(exc)) from exc
        self.last_alignment_failure=None
        scan_pose=self.reference_pose(scan_odom);current=self.reference_pose(Pose(*self.inputs[self.pose_source][2][:3]))
        key=(stamp,pose_support,error)
        if self.scan_cache is None or self.scan_cache[0]!=key:
            co,si=math.cos(scan_pose.yaw),math.sin(scan_pose.yaw)
            points=np.column_stack((scan_pose.x+co*body[:,0]-si*body[:,1],scan_pose.y+si*body[:,0]+co*body[:,1]))
            self.scan_cache=(key,points)
        points=self.scan_cache[1]
        sources=dict(scan=stamp,scan_pose_support=pose_support,raw=self.inputs['raw'][0])
        if require_leader:sources['leader']=self.inputs['leader'][0]-50_000_000
        if self.pose_source=='filtered':
            sources['filtered']=self.inputs['filtered'][0];sources['imu']=self.inputs['imu'][0]
        pose_age=(now-self.inputs[self.pose_source][0])/1e9
        # Allow the requested data lifetime while reserving extra body-space
        # uncertainty when the current measured robot pose itself is delayed.
        age_uncertainty=(.24 if self.pose_source=='filtered' else .15)*max(0.,pose_age-.1)
        return dict(pose=current,points=points,source=min(sources.values()),source_stamps=sources,
                    scan_stamp=stamp,uncertainty=.01+error+age_uncertainty,pose_source=self.pose_source,heading_check=heading_check)
    def install(self,plan):
        self.plan=plan
        if plan.get('controller_kind')=='APF_PIVOT_STEP_V1':
            from apf_pivot_candidate import APFPivotFSM
            self.fsm=APFPivotFSM(plan['start'],plan['box'])
            if 'gentle_route' in plan:
                from gentle_pivot_candidate import GentlePivot
                self.fsm.controller=GentlePivot(plan['start'],plan['box'],plan['gentle_route'])
                self.fsm.controller.map_rejoin=plan.get('live_map_rejoin',False)
        else:self.fsm=StopTurnGo(plan['goals'],v=.06,w=.2,pulse=True)
        self.last_goal=None;self.braking=False;self.stage_started=None
    def handoff_to_follow(self,motor):
        if motor.get('measured_rpm')!=[0,0] or motor.get('zero_feedback_samples',0)<3:
            raise ValueError('REJOIN_HANDOFF_REQUIRES_MEASURED_ZERO')
        self.plan=None;self.fsm=None;self.last_goal=None;self.braking=False
        self.stage_started=None;self.joined=None;self.geometry_complete=False
    def command(self,now,mono,motor,follow_command=None):
        scene=self.scene(now,mono);pose=scene['pose'];p=(pose.x,pose.y,pose.yaw);cloud=scene['points'];uncertainty=scene['uncertainty']
        apf=self.plan is not None and self.plan.get('controller_kind')=='APF_PIVOT_STEP_V1'
        if apf:
            cloud=np.concatenate((cloud,np.asarray(self.plan['box_boundary'])))
        d=minimum_clearance(cloud,p)-uncertainty
        result=dict(v=0.,w=0.,motion='STOP',reason='STOP_NOT_READY',source_stamp_ns=scene['source'],
                    scan_stamp_ns=scene['scan_stamp'],clearance_m=d,swept_clearance_m=d,pose=list(p),
                    uncertainty_m=uncertainty,geometry_complete=False,input_source_stamps=scene['source_stamps'],
                    input_source_timeout_s=SOURCE_TIMEOUT_S)
        result['pose_source']=self.pose_source
        if scene['heading_check']:result['heading_check']=scene['heading_check']
        body=body_points_for_diagnostic(cloud,p)
        distances=np.hypot(np.maximum(abs(body[:,0])-.27,0),np.maximum(abs(body[:,1])-.3,0))
        nearest_index=int(np.argmin(distances));nearest=body[nearest_index]
        result['nearest_scan_point']=dict(base_x_m=float(nearest[0]),base_y_m=float(nearest[1]),
            laser_range_m=float(math.hypot(nearest[0]-.26,nearest[1])),
            clearance_before_uncertainty_m=float(distances.min()),
            point_origin='known_box_boundary' if apf and nearest_index>=len(scene['points']) else 'live_scan')
        if d<=.3:result['reason']='STOP_FOOTPRINT_CLEARANCE';return result
        if apf:
            result['controller_kind']='APF_PIVOT_STEP_V1'
            if p[0]>4.5 or abs(p[1])>1.25:result['reason']='STOP_APF_WORKSPACE_BOUND';return result
            # New heading selection is performed at a confirmed stop only.
            if self.fsm.controller.phase=='SELECT' and (motor.get('measured_rpm')!=[0,0] or motor.get('zero_feedback_samples',0)<3):
                result['reason']='BRAKE_BEFORE_APF_SELECTION';return result
            self.fsm.points=cloud
        if self.last_pose is not None:
            old=self.last_pose
            if math.hypot(p[0]-old[0],p[1]-old[1])>.10 or abs(wrap(p[2]-old[2]))>.12:
                result['reason']='STOP_FILTERED_ODOM_JUMP' if self.pose_source=='filtered' else 'STOP_RAW_ODOM_JUMP';return result
        self.last_pose=p
        if self.fsm is None:
            if follow_command is None:result['reason']='PLAN_REQUIRED';return result
            v,w=follow_command
            if v<=0:result['reason']='FOLLOW_WAITING_LEADER';return result
            v=min(.06,v);limit=min(.2,.98*v/.28);w=max(-limit,min(limit,w))
            scale=min(1.,.98*10*.065*2*math.pi/60/(v+abs(w)*.28));v*=scale;w*=scale
            phase='DRIVE';goal=-1
        else:
            v,w,phase=self.fsm.command(p);goal=self.fsm.i
        if apf:result['apf']=dict(self.fsm.controller.last_diagnostic,phase=self.fsm.controller.phase)
        if phase=='BRAKE':result['reason']='BRAKE_BEFORE_PHASE_CHANGE';return result
        if goal!=self.last_goal:
            self.braking=self.last_goal is not None;self.last_goal=goal
            self.stage_started=mono;self.progress_at=mono;self.progress_pose=p
            if self.fsm is not None and 0<=goal<len(self.fsm.goals):
                kind,target=self.fsm.goals[goal]
                if kind=='TURN':self.stage_limit=min(120.,max(30.,abs(wrap(target[2]-p[2]))/.02+15.))
                else:self.stage_limit=min(210.,max(30.,math.hypot(target[0]-p[0],target[1]-p[1])/.008+15.))
            else:self.stage_limit=360.
        if phase=='END':result['reason']='LOCAL_ROUTE_END';return result
        if phase.startswith('STOP'):result['reason']=phase;return result
        if self.braking:
            if (motor.get('measured_rpm')==[0,0] and motor.get('zero_feedback_samples',0)>=3):self.braking=False
            else:result['reason']='BRAKE_BEFORE_PHASE_CHANGE';return result
        if mono-self.stage_started>self.stage_limit:
            result['reason']='STOP_PHASE_TIME_LIMIT';return result
        if self.progress_pose is not None:
            old=self.progress_pose
            moved=math.hypot(p[0]-old[0],p[1]-old[1])>.006 or abs(wrap(p[2]-old[2]))>.015
            if moved:self.progress_pose=p;self.progress_at=mono
            elif mono-self.progress_at>15.:result['reason']='STOP_PHASE_STALLED';return result
        factor=min(1.,max(0.,(d-.3)/.3));v*=factor;w*=factor
        # Quantized command and measured current wheel motion are both swept.
        rpm=np.round(np.array([(v-w*.28),v+w*.28])/.065*60/(2*math.pi))
        # A sub-half-RPM pivot rounds to zero on the device. Check the smallest
        # executable pivot through the same full-body sweep before allowing it.
        if phase=='TURN' and w!=0 and not np.any(rpm):
            w=math.copysign(2*.065*2*math.pi/60/.560,w)
            rpm=np.array([-math.copysign(1.,w),math.copysign(1.,w)])
            result['minimum_pivot_rpm_applied']=True
        result['candidate_wheel_rpm']=rpm.astype(int).tolist()
        if abs(rpm).max()>10:result['reason']='STOP_RPM_LIMIT';return result
        qv=float(rpm.mean()*.065*2*math.pi/60);qw=float((rpm[1]-rpm[0])*.065*2*math.pi/60/.560)
        f=self.inputs[self.pose_source][2];actual=(f[3],f[4])
        forecasts=[];margins=[]
        motions=[(qv,qw),actual]
        if self.pose_source=='filtered':
            # Measured yaw/raw ratios were 1.22–1.27. Reserve a larger predicted
            # motion envelope without changing wheel conversion/calibration.
            motions.append((qv*1.35,qw*1.35));result['swept_prediction_scale']=1.35
        for sv,sw in motions:
            bound=(abs(sv)+math.hypot(.27,.3)*abs(sw))*.1
            for k in range(1,11):
                forecasts.append(advance(p,sv,sw,k*.1));margins.append(uncertainty+bound)
        forecasts=np.asarray(forecasts)
        dx=cloud[:,0][None,:]-forecasts[:,0,None];dy=cloud[:,1][None,:]-forecasts[:,1,None]
        co=np.cos(forecasts[:,2,None]);si=np.sin(forecasts[:,2,None])
        distances=np.hypot(np.maximum(abs(co*dx+si*dy)-.27,0),np.maximum(abs(-si*dx+co*dy)-.3,0))
        swept=min(d,float(np.min(distances.min(axis=1)-np.asarray(margins))))
        result['swept_clearance_m']=swept
        if swept<=.3:result['reason']='STOP_SWEPT_CLEARANCE';return result
        rear=p[0]-.27*abs(math.cos(p[2]))-.3*abs(math.sin(p[2]))
        aligned=self.plan is not None and rear>self.plan['box_far'] and p[0]>=self.plan['return_x'] and abs(p[1])<=.10 and abs(wrap(p[2]))<=math.radians(10)
        if aligned:
            if self.joined is None:self.joined=p[0]
            self.geometry_complete=p[0]-self.joined>=1.
        else:self.joined=None
        result.update(v=v,w=w,motion=phase,reason='LOCAL_AVOIDANCE_READY',goal_index=goal,
                      geometry_complete=self.geometry_complete,phase_time_budget_s=self.stage_limit)
        return result
