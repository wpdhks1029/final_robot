"""Independent map/path check for local-odometry completion claims."""
import math

class LiveRejoinVerification:
    def __init__(self,required_distance_m=1.):
        if not math.isfinite(required_distance_m) or required_distance_m<=0:
            raise ValueError('INVALID_FOLLOW_VERIFICATION_DISTANCE')
        self.required_distance_m=required_distance_m
        self.distance=0.;self.previous=None
    def check(self,inputs,now,mono,local_pose,origin,heading,eligible,*,require_frozen_alignment=True,path_selection=None):
        out=dict(verified=False,forward_on_live_path_m=0.,reason='MAP_PATH_NOT_READY')
        try:
            m=inputs['follower'];path=inputs['path']
            for key,item in (('follower',m),('path',path)):
                stamp=item[0].sec*10**9+item[0].nanosec-(50_000_000 if key=='path' else 0)
                if not 0<=(now-stamp)/1e9<=.8 or not 0<=mono-item[1]<=.8:raise ValueError('MAP_PATH_STALE')
            p=m[2];points=path[2];best=None
            for a,b in zip(points,points[1:]):
                dx,dy=b[0]-a[0],b[1]-a[1];ll=dx*dx+dy*dy
                if ll<1e-6:continue
                t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/ll))
                q=(a[0]+t*dx,a[1]+t*dy);d=math.hypot(p[0]-q[0],p[1]-q[1])
                if best is None or d<best[0]:best=(d,math.atan2(dy,dx),q)
            if best is None:raise ValueError('MAP_PATH_TOO_SHORT')
            if path_selection is not None:
                best=(path_selection['path_distance_m'],path_selection['path_heading_rad'],path_selection['projection_map'])
                out['path_selection']=path_selection
            co,si=math.cos(heading),math.sin(heading)
            expected=(origin[0]+co*local_pose[0]-si*local_pose[1],origin[1]+si*local_pose[0]+co*local_pose[1])
            position_error=math.hypot(p[0]-expected[0],p[1]-expected[1])
            angle=lambda v:math.atan2(math.sin(v),math.cos(v))
            yaw_error=abs(angle(p[2]-heading-local_pose[2]))
            path_heading_error=abs(angle(p[2]-best[1]))
            out.update(map_pose=list(p[:3]),predicted_map_pose=[*expected,heading+local_pose[2]],
                       live_path_points=[list(q) for q in points],path_distance_m=best[0],
                       path_heading_error_deg=math.degrees(path_heading_error),
                       map_local_position_error_m=position_error,map_local_heading_error_deg=math.degrees(yaw_error))
            if require_frozen_alignment and (position_error>.15 or yaw_error>math.radians(10)):raise ValueError('MAP_LOCAL_POSE_MISMATCH')
            if not eligible or best[0]>.10 or path_heading_error>math.radians(10):raise ValueError('NOT_REJOINED_LIVE_PATH')
            stamp=m[0].sec*10**9+m[0].nanosec
            if self.previous is not None:
                old_stamp,old_p,old_mono=self.previous
                if stamp>old_stamp:
                    step=math.hypot(p[0]-old_p[0],p[1]-old_p[1])
                    if step>.10 or mono-old_mono>.8:raise ValueError('MAP_PROGRESS_DISCONTINUITY')
                    self.distance=max(0.,self.distance+(p[0]-old_p[0])*math.cos(best[1])+(p[1]-old_p[1])*math.sin(best[1]))
            self.previous=(stamp,p,mono)
            out.update(reason='LIVE_MAP_PATH_ALIGNED',verified=self.distance>=self.required_distance_m,forward_on_live_path_m=self.distance,
                       required_distance_m=self.required_distance_m)
            return out
        except (KeyError,ValueError,TypeError,IndexError) as exc:
            self.previous=None;self.distance=0.;out['reason']=str(exc) if isinstance(exc,ValueError) else 'MAP_PATH_NOT_READY'
            return out
