"""평가3 계산 후보. ROS·포트·모터 명령 없음. 단일 직선 기준 경로용."""
from dataclasses import dataclass
import math


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


@dataclass(frozen=True)
class Settings:
    half_width:float=.30
    front:float=.27
    rear:float=.27
    hard:float=.30
    influence:float=.90
    track:float=.560
    max_v:float=.10
    max_w:float=.20
    obstacle_width:float=.20
    obstacle_length:float=.20
    obstacle_height:float=None
    visibility_confirmed:bool=False
    space_width:float=2.50
    lidar_height:float=None

    def __post_init__(self):
        values=[v for k,v in vars(self).items() if k not in ('lidar_height','obstacle_height','visibility_confirmed')]
        if not all(math.isfinite(v) and v>0 for v in values) or self.influence<=self.hard:
            raise ValueError('기하/상한 설정이 유효하지 않습니다.')
        if self.obstacle_height is not None and (not math.isfinite(self.obstacle_height) or self.obstacle_height<=0):
            raise ValueError('장애물 높이가 유효하지 않습니다.')
        if self.lidar_height is not None and (not math.isfinite(self.lidar_height) or self.lidar_height<=0):
            raise ValueError('실측 라이다 높이가 유효하지 않습니다.')

    def passage(self,obstacle_center_y):
        if not math.isfinite(obstacle_center_y):raise ValueError('장애물 위치가 유효하지 않습니다.')
        left=self.space_width/2-obstacle_center_y-self.obstacle_width/2
        right=self.space_width/2+obstacle_center_y-self.obstacle_width/2
        required=2*self.half_width+2*self.hard
        return dict(left_gap_m=left,right_gap_m=right,required_gap_m=required,
                    left_possible=left>required,right_possible=right>required)


def clearance(x,y,c):
    return math.hypot(max(-c.rear-x,x-c.front,0.),max(abs(y)-c.half_width,0.))


def front_blocker(x,y,d,c):
    return x>c.front and abs(y)<c.half_width+c.hard+.15 and d<c.influence


def scan_diagnostic(points,c):
    rows=[dict(x_m=x,y_m=y,clearance_m=clearance(x,y,c)) for x,y in points]
    front=[r for r in rows if r['x_m']>c.front and abs(r['y_m'])<c.half_width+c.hard+.15]
    return dict(valid_points=len(rows),front_corridor_half_width_m=c.half_width+c.hard+.15,
                influence_clearance_m=c.influence,
                front_blocker_count=sum(front_blocker(r['x_m'],r['y_m'],r['clearance_m'],c) for r in rows),
                nearest_points=sorted(rows,key=lambda r:r['clearance_m'])[:5],
                nearest_front_points=sorted(front,key=lambda r:r['clearance_m'])[:5])


class APF:
    def __init__(self,settings=Settings(),preferred_side=None):
        if preferred_side not in (None,-1,1):raise ValueError('회피 방향 설정이 유효하지 않음')
        self.c=settings;self.preferred_side=preferred_side;self.side=None;self.phase='TRACK'

    def step(self,v,w,points,*,whole_body_passed=False,reference_heading_error=0.):
        c=self.c
        def stop(reason):return dict(v=0.,w=0.,phase=self.phase,reason=reason,side=self.side)
        if c.lidar_height is None:return stop('STOP_LIDAR_HEIGHT_UNCONFIRMED')
        if c.obstacle_height is None and not c.visibility_confirmed:return stop('STOP_OBSTACLE_VISIBILITY_UNCONFIRMED')
        if c.obstacle_height is not None and c.lidar_height>=c.obstacle_height:return stop('STOP_OBSTACLE_BELOW_OR_AT_SCAN_PLANE')
        if not all(math.isfinite(n) for n in (v,w,reference_heading_error)) or v<0 or abs(w)>c.max_w or v>c.max_v:
            return stop('STOP_INVALID_PURSUIT')
        if not points:return stop('STOP_NO_VALID_SCAN_POINTS')
        if any(len(p)!=2 or not all(math.isfinite(n) for n in p) for p in points):return stop('STOP_INVALID_SCAN_POINTS')
        distances=[clearance(x,y,c) for x,y in points]
        if min(distances)<=c.hard:return stop('STOP_FOOTPRINT_CLEARANCE')
        if v==0:return stop('STOP_UPSTREAM')
        front=[(x,y,d) for (x,y),d in zip(points,distances) if front_blocker(x,y,d,c)]
        if front and self.side is None:
            left=min((d for (x,y),d in zip(points,distances) if y>.05 and x>-c.rear),default=float('inf'))
            right=min((d for (x,y),d in zip(points,distances) if y<-.05 and x>-c.rear),default=float('inf'))
            self.side=self.preferred_side if self.preferred_side is not None else (1 if left>=right else -1);self.phase='AVOID'
        if self.side is not None and whole_body_passed:self.phase='REJOIN'
        elif self.side is not None and not front:self.phase='PASS'
        elif front:self.phase='AVOID'
        fx=fy=0.;count=0
        for (x,y),d in zip(points,distances):
            if d>=c.influence:continue
            mag=min(2.,.025*(1/d-1/c.influence)/(d*d))
            norm=math.hypot(x,y)
            if norm>1e-6:fx-=mag*x/norm;fy-=mag*y/norm;count+=1
        if count:fx/=count;fy/=count
        bend=math.atan2(fy,max(.2,1.+fx))
        if self.phase=='AVOID':
            candidate_w=w+.15*bend+self.side*.08
        elif self.phase=='PASS':
            candidate_w=.8*wrap(reference_heading_error)+.15*bend
        else:candidate_w=w+.15*bend
        candidate_v=v*(.6 if self.phase in ('AVOID','PASS') else 1.)
        candidate_v=min(candidate_v,c.max_v)
        limit=min(c.max_w,2*candidate_v/c.track)
        candidate_w=max(-limit,min(limit,candidate_w))
        return dict(v=candidate_v,w=candidate_w,phase=self.phase,reason='APF_CANDIDATE',side=self.side,
                    min_clearance_m=min(distances),evaluation_pass=False)


class RejoinEvidence:
    """실측 장애물 후단·직선 기준에 대한 기하 증거. 접촉0회 실측을 대체하지 않는다."""
    def __init__(self,origin,heading,obstacle_end_s,*,cross_tolerance,heading_tolerance,settings=Settings()):
        vals=(*origin,heading,obstacle_end_s,cross_tolerance,heading_tolerance)
        if len(origin)!=2 or not all(math.isfinite(n) for n in vals) or cross_tolerance<=0 or heading_tolerance<=0:raise ValueError('기준 실측값이 유효하지 않음')
        self.origin=origin;self.heading=heading;self.end=obstacle_end_s;self.cross_tol=cross_tolerance;self.heading_tol=heading_tolerance;self.c=settings
        self.join_s=None;self.last_stamp=None;self.passed=False

    def step(self,stamp_ns,x,y,yaw):
        if type(stamp_ns) is not int or stamp_ns<=0 or (self.last_stamp is not None and stamp_ns<=self.last_stamp):raise ValueError('pose 시각은 단조 증가해야 함')
        if not all(math.isfinite(v) for v in (x,y,yaw)):raise ValueError('pose가 유효하지 않음')
        self.last_stamp=stamp_ns;c=self.c;co=math.cos(self.heading);si=math.sin(self.heading)
        dx=x-self.origin[0];dy=y-self.origin[1];s=co*dx+si*dy;cross=-si*dx+co*dy
        corners=[]
        for bx in (-c.rear,c.front):
            for by in (-c.half_width,c.half_width):
                wx=x+math.cos(yaw)*bx-math.sin(yaw)*by;wy=y+math.sin(yaw)*bx+math.cos(yaw)*by
                corners.append(co*(wx-self.origin[0])+si*(wy-self.origin[1]))
        self.passed=min(corners)>self.end
        aligned=abs(cross)<=self.cross_tol and abs(wrap(yaw-self.heading))<=self.heading_tol
        if not self.passed:self.join_s=None;phase='NOT_FULLY_PASSED'
        elif not aligned:self.join_s=None;phase='REJOIN'
        else:
            if self.join_s is None:self.join_s=s
            phase='GEOMETRIC_COMPLETE_CANDIDATE' if s-self.join_s>=1. else 'FOLLOW_AFTER_REJOIN'
        return dict(phase=phase,whole_body_passed=self.passed,cross_error_m=cross,
                    post_rejoin_net_progress_m=0. if self.join_s is None else max(0.,s-self.join_s),
                    evaluation_pass=False,contact_observation_required=True)
