"""Map-path target expressed in the measured filtered-odom control frame.

Does not transform scans or reset odometry; only the attraction target changes.
"""
import math
from local_stop_turn_go import pose_at

class MapPoseBracketMissing(ValueError):
    def __init__(self,diagnostic):
        super().__init__('MAP_FILTERED_POSE_BRACKET_MISSING');self.diagnostic=diagnostic

def guidance(inputs,core,now,mono,lookahead=.5,tracker=None):
    m=inputs.get('follower');path=inputs.get('path')
    if m is None or path is None:raise ValueError('MAP_REJOIN_INPUT_MISSING')
    stamps={}
    for key,item in (('map',m),('path',path)):
        stamp=item[0].sec*10**9+item[0].nanosec
        if key=='path':stamp-=50_000_000
        if not 0<=(now-stamp)/1e9<=.8 or not 0<=mono-item[1]<=.8:raise ValueError('MAP_REJOIN_INPUT_STALE')
        stamps[key]=stamp
    # Match the map pose to odometry at its own measured timestamp.
    try:odom,support,error=pose_at(core.history,stamps['map'],max_v=.09,max_w=.33)
    except ValueError:
        latest=core.history[-1][0] if core.history else None
        raise MapPoseBracketMissing(dict(map_stamp_ns=stamps['map'],latest_filtered_stamp_ns=latest,
            first_filtered_stamp_ns=core.history[0][0] if core.history else None,
            map_ahead_s=(stamps['map']-latest)/1e9 if latest is not None else None,
            waiting_for_newer_pose=latest is not None and stamps['map']>latest)) from None
    local=core.reference_pose(odom);p=m[2];pts=path[2];best=None
    for i,(a,b) in enumerate(zip(pts,pts[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        if length<.001:continue
        t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(length*length)))
        q=(a[0]+t*dx,a[1]+t*dy);distance=math.hypot(p[0]-q[0],p[1]-q[1])
        candidate=(distance,i,q,math.atan2(dy,dx))
        if best is None or distance<best[0]:best=candidate
    if best is None:raise ValueError('MAP_REJOIN_PATH_TOO_SHORT')
    distance,index,q,path_heading=best;remaining=lookahead;target=q
    for b in pts[index+1:]:
        length=math.hypot(b[0]-target[0],b[1]-target[1])
        if length>=remaining:
            target=(target[0]+remaining*(b[0]-target[0])/length,target[1]+remaining*(b[1]-target[1])/length);remaining=0.;break
        remaining-=length;target=b
    # Do not extend a path beyond observed leader history.
    if tracker is None and remaining>.25:raise ValueError('MAP_REJOIN_PATH_END_TOO_NEAR')
    selection=None
    if tracker is not None:
        selection=tracker.select(pts,p,lookahead,tip_heading=inputs['leader'][2][2] if 'leader' in inputs else None)
        target=selection['target_map'];distance=selection['path_distance_m'];path_heading=selection['path_heading_rad']
        if selection['remaining_path_m']<.25:raise ValueError('MAP_REJOIN_PATH_END_TOO_NEAR')
    rotation=local.yaw-p[2];co,si=math.cos(rotation),math.sin(rotation)
    dx,dy=target[0]-p[0],target[1]-p[1]
    target_local=(local.x+co*dx-si*dy,local.y+si*dx+co*dy)
    heading_error=math.atan2(math.sin(p[2]-path_heading),math.cos(p[2]-path_heading))
    return dict(path_selection=selection,target_local=target_local,target_map=target,map_pose=list(p[:3]),
                path_distance_m=distance,path_heading_error_deg=math.degrees(heading_error),
                source_stamp_ns=min(*stamps.values(),support),map_pose_support_ns=support,
                map_odom_rotation_deg=math.degrees(rotation),hold_uncertainty_m=error)


class MapPoseContinuity:
    """Reject localization jumps; never turn them into an immediate steering target."""
    def __init__(self):self.previous=None
    def check(self,item):
        stamp=item[0].sec*10**9+item[0].nanosec;p=item[2]
        if self.previous is not None:
            old_stamp,old=self.previous
            dt=(stamp-old_stamp)/1e9
            if dt<0:raise ValueError('MAP_REJOIN_TIME_REVERSED')
            if dt==0:return
            dyaw=abs(math.atan2(math.sin(p[2]-old[2]),math.cos(p[2]-old[2])))
            if math.hypot(p[0]-old[0],p[1]-old[1])>.1+.12*dt or dyaw>math.radians(12)+.5*dt:
                raise ValueError('MAP_REJOIN_LOCALIZATION_JUMP')
        self.previous=(stamp,p)
