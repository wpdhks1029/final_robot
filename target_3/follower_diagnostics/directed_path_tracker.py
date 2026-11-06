"""Session-stable, arc-length path selection. No ROS or motor access.

Reject ambiguous sharp reversals rather than silently deleting path points.
Curves can change global heading; only local tangent discontinuity is checked.
"""
import hashlib,json,math


def wrap(a):return math.atan2(math.sin(a),math.cos(a))

class DirectedPathTracker:
    def __init__(self):
        self.points=None;self.progress=None;self.last_position=None;self.session=None
        self.last_observed_position=None
    def observe_position(self,pose):
        position=tuple(float(v) for v in pose[:2])
        if len(position)!=2 or not all(math.isfinite(v) for v in position):
            raise ValueError('PATH_INVALID_FOLLOWER_POSITION')
        if self.last_observed_position is not None and math.dist(position,self.last_observed_position)>.20:
            raise ValueError('PATH_FOLLOWER_MAP_JUMP')
        self.last_observed_position=position
    def select(self,points,pose,lookahead=.5,tip_heading=None):
        pts=tuple(tuple(float(v) for v in p[:2]) for p in points)
        if len(pts)<2:raise ValueError('PATH_NEEDS_TWO_POINTS')
        if len(pts)>10000 or any(not math.isfinite(v) for p in pts for v in p):raise ValueError('PATH_INVALID_POINTS')
        if not .05<=lookahead<=1.5:raise ValueError('PATH_INVALID_LOOKAHEAD')
        if self.points is not None:
            # This candidate handles append-only leader history. A reset or a
            # rolling prefix change requires a new operator session, never a
            # silent progress reset while armed.
            if len(pts)<len(self.points) or pts[:len(self.points)]!=self.points:
                raise ValueError('PATH_SESSION_CHANGED_RESTART')
        segments=[];arc=0.;previous=None
        for i,(a,b) in enumerate(zip(pts,pts[1:])):
            dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
            if length<.001:continue
            heading=math.atan2(dy,dx)
            if previous is not None and abs(wrap(heading-previous))>math.radians(120):
                raise ValueError('PATH_SHARP_REVERSAL_RESTART_BRIDGE')
            segments.append((i,a,b,length,arc,heading));arc+=length;previous=heading
        if not segments:raise ValueError('PATH_NEEDS_TWO_DISTINCT_POINTS')
        if tip_heading is not None and abs(wrap(segments[-1][-1]-tip_heading))>math.pi/2:
            raise ValueError('PATH_TIP_DIRECTION_MISMATCH_RESTART_BRIDGE')
        candidates=[]
        self.observe_position(pose)
        motion=0. if self.last_position is None else math.dist(pose[:2],self.last_position)
        for i,a,b,length,s,heading in segments:
            dx,dy=b[0]-a[0],b[1]-a[1]
            t=max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/(length*length)))
            projected=s+t*length
            if self.progress is not None:
                if projected<self.progress-.03 or projected>self.progress+max(.30,2*motion+.05):continue
            q=(a[0]+t*dx,a[1]+t*dy)
            candidates.append((math.dist(pose[:2],q),projected,i,q,heading))
        if not candidates:raise ValueError('PATH_PROGRESS_DISCONTINUITY')
        distance,projected,index,q,heading=min(candidates,key=lambda c:(c[0],c[1]))
        progress=projected if self.progress is None else max(self.progress,projected)
        target_arc=min(arc,progress+lookahead);target=None
        for i,a,b,length,s,h in segments:
            if target_arc<=s+length+1e-9:
                f=max(0.,min(1.,(target_arc-s)/length));target=(a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]));break
        if target is None:target=pts[-1]
        if self.session is None:self.session=hashlib.sha256(json.dumps(pts[:2]).encode()).hexdigest()[:12]
        self.points=pts;self.progress=progress;self.last_position=tuple(pose[:2])
        return dict(target_map=list(target),projection_map=list(q),path_distance_m=distance,
                    path_heading_rad=heading,path_index=index,path_progress_m=progress,
                    path_projected_arc_m=projected,path_target_arc_m=target_arc,path_length_m=arc,
                    remaining_path_m=arc-progress,path_session=self.session,
                    path_heading_error_deg=math.degrees(wrap(pose[2]-heading)))
