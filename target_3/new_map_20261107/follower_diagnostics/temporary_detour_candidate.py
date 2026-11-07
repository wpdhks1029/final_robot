"""정적 20cm 상자·직선 기준 임시 경로/Pure Pursuit. ROS/모터 접근 없음."""
import math
from dataclasses import dataclass
from apf_case3_candidate import Settings,clearance,wrap
from apf_rejoin_candidate import connected_groups


@dataclass(frozen=True)
class Pose:
    x:float
    y:float
    yaw:float


def reference_frame(path,pose):
    """짧은 선분 각도 대신 전체 점의 주축과 잔차로 직선성을 확인한다."""
    if len(path)<3 or not all(math.isfinite(v) for p in path for v in p):raise ValueError('REFERENCE_MISSING')
    mx=sum(p[0] for p in path)/len(path);my=sum(p[1] for p in path)/len(path)
    xx=sum((x-mx)**2 for x,y in path);yy=sum((y-my)**2 for x,y in path);xy=sum((x-mx)*(y-my) for x,y in path)
    heading=.5*math.atan2(2*xy,xx-yy)
    if math.cos(heading-pose.yaw)<0:heading=wrap(heading+math.pi)
    co,si=math.cos(heading),math.sin(heading)
    along=[co*(x-mx)+si*(y-my) for x,y in path]
    if max(along)-min(along)<.4:raise ValueError('REFERENCE_TOO_SHORT')
    residual=max(abs(-si*(x-mx)+co*(y-my)) for x,y in path)
    if residual>.10:raise ValueError('REFERENCE_NOT_STRAIGHT')
    if abs(wrap(pose.yaw-heading))>math.radians(20):raise ValueError('REFERENCE_HEADING_MISMATCH')
    cross=-si*(pose.x-mx)+co*(pose.y-my)
    origin=(pose.x+si*cross,pose.y-co*cross)
    return origin,heading,cross,residual


def to_reference(points,pose,origin,heading):
    result=[];co,si=math.cos(heading),math.sin(heading)
    for px,py in points:
        wx=pose.x+math.cos(pose.yaw)*px-math.sin(pose.yaw)*py-origin[0]
        wy=pose.y+math.sin(pose.yaw)*px+math.cos(pose.yaw)*py-origin[1]
        result.append((co*wx+si*wy,-si*wx+co*wy))
    return result


def transition(x0,x1,y0,y1,slope0=0.,step=.02):
    length=x1-x0
    if length<=0:raise ValueError('TRANSITION_TOO_SHORT')
    delta=y1-y0
    a3=10*delta/length**3-6*slope0/length**2
    a4=-15*delta/length**4+8*slope0/length**3
    a5=6*delta/length**5-3*slope0/length**4
    count=max(1,math.ceil(length/step));out=[]
    for i in range(count+1):
        t=length*i/count
        y=y0+slope0*t+a3*t**3+a4*t**4+a5*t**5
        dy=slope0+3*a3*t*t+4*a4*t**3+5*a5*t**4
        ddy=6*a3*t+12*a4*t*t+20*a5*t**3
        out.append((x0+t,y,math.atan(dy),ddy/(1+dy*dy)**1.5))
    return out


def arc_transition(x0,y0,yaw0,y1,radius,step=.008):
    """전진 두 원호로 평행 이동. 짧은 접근에서 긴 다항식 경로의 한계를 보완."""
    side=1 if y1>y0 else -1
    theta0=side*yaw0;delta=side*(y1-y0)
    cosine=(math.cos(theta0)+1-delta/radius)/2
    if not -1<=cosine<=1:return []
    angle=math.acos(cosine)
    if angle<theta0 or angle>math.radians(85):return []
    x,y,yaw=x0,y0,yaw0;out=[(x,y,yaw,0.)]
    for curvature,length in ((side/radius,radius*(angle-theta0)),(-side/radius,radius*angle)):
        count=max(1,math.ceil(length/step));ds=length/count
        for _ in range(count):
            end=yaw+curvature*ds
            x+=(math.sin(end)-math.sin(yaw))/curvature
            y+=(math.cos(yaw)-math.cos(end))/curvature
            yaw=end;out.append((x,y,yaw,curvature))
    return out


def lateral_arc_transition(x0,y0,yaw0,y1,radius,step=.002):
    """전진 원호→측방 직진→원호. 역회전/제자리 회전 없이 큰 측방 이동."""
    side=1 if y1>y0 else -1
    theta0=side*yaw0
    extra=side*(y1-y0)-radius*(math.cos(theta0)+1)
    if extra<0 or theta0>=math.pi/2:return []
    x,y,yaw=x0,y0,yaw0;out=[(x,y,yaw,0.)]
    motions=((side/radius,radius*(math.pi/2-theta0)),(0.,extra),(-side/radius,radius*math.pi/2))
    for curvature,length in motions:
        count=max(1,math.ceil(length/step));ds=length/count
        for _ in range(count):
            end=yaw+curvature*ds
            if curvature:
                x+=(math.sin(end)-math.sin(yaw))/curvature
                y+=(math.cos(yaw)-math.cos(end))/curvature
            else:x+=ds*math.cos(yaw);y+=ds*math.sin(yaw)
            yaw=end;out.append((x,y,yaw,curvature))
    return out


def path_clear(path,points,c,padding=.02):
    body_radius=math.hypot(max(c.front,c.rear),c.half_width)
    for i,(x,y,yaw,k) in enumerate(path):
        if abs(k)>.98*2/c.track:return False
        motion=0. if i==0 else math.hypot(x-path[i-1][0],y-path[i-1][1])+body_radius*abs(wrap(yaw-path[i-1][2]))
        margin=padding+motion
        radius=body_radius+c.hard+margin
        co,si=math.cos(yaw),math.sin(yaw)
        for px,py in points:
            dx,dy=px-x,py-y
            if abs(dx)>radius or abs(dy)>radius:continue
            if clearance(co*dx+si*dy,-si*dx+co*dy,c)<=c.hard+margin:return False
    return True


def plan(points,c,*,side,initial_cross=0.,initial_yaw=0.,objective='length',nominal_speed=.04):
    if side not in (-1,1):raise ValueError('INVALID_SIDE')
    if objective not in ('length','time') or not 0<nominal_speed<=.06:raise ValueError('INVALID_PLANNING_OBJECTIVE')
    if not points or not all(math.isfinite(v) for p in points for v in p):raise ValueError('SCAN_INVALID')
    # 군집은 계획 대상 선택에만 사용한다. 경로 충돌 검사는 원본 전체 점으로 수행한다.
    front=[(x,y) for x,y in points if .3<x<2.5 and abs(y)<1.8]
    # 먼저 전체 물체를 군집화한 뒤 진행 보호 구간과 겹치는 물체를 고른다.
    # 경계에서 상자 절반을 잘라 폭/중심을 잘못 추정하지 않는다.
    groups=[g for g in connected_groups(front) if len(g)>=5
            and any(abs(y)<c.half_width+c.hard for x,y in g)]
    if not groups:return dict(reason='NO_OBSTACLE_CLUSTER',path=[])
    obstacle=min(groups,key=lambda g:min(x for x,y in g))
    xmin=min(x for x,y in obstacle);xmax=max(x for x,y in obstacle)
    ymin=min(y for x,y in obstacle);ymax=max(y for x,y in obstacle)
    if ymax-ymin>.35 or xmax-xmin>.4:return dict(reason='STOP_UNSUPPORTED_OBSTACLE',path=[])
    # 관측으로 보이지 않는 뒷면도 확인된 시험물 치수로 보수적으로 포함한다.
    far=max(xmax,xmin+c.obstacle_length)
    center=(ymin+ymax)/2
    geometry=dict(obstacle_near_x_m=xmin,obstacle_far_x_m=far,
                  observed_center_y_m=center,observed_width_m=ymax-ymin,
                  approach_finish_max_x_m=xmin-.45)
    lo=min(ymin,center-c.obstacle_width/2);hi=max(ymax,center+c.obstacle_width/2)
    obstacle_model=[]
    for i in range(21):
        f=i/20
        obstacle_model.extend([(xmin,lo+(hi-lo)*f),(far,lo+(hi-lo)*f),
                               (xmin+(far-xmin)*f,lo),(xmin+(far-xmin)*f,hi)])
    collision_points=list(points)+obstacle_model
    candidates=[]
    for extra in (.04,.08,.12,.16,.20):
        offset=(lo-c.half_width-c.hard-extra) if side<0 else (hi+c.half_width+c.hard+extra)
        for finish in (xmin-.45,xmin-.65,xmin-.85):
            if finish<.5:continue
            for return_length in (1.,1.3,1.6):
                end_parallel=far+c.rear+c.hard+.25
                path=transition(0.,finish,initial_cross,offset,math.tan(initial_yaw))
                path+=transition(finish,end_parallel,offset,offset)[1:]
                path+=transition(end_parallel,end_parallel+return_length,offset,0.)[1:]
                path+=transition(end_parallel+return_length,end_parallel+return_length+1.2,0.,0.)[1:]
                if path_clear(path,collision_points,c):
                    length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:]))
                    candidates.append((length,path,offset,end_parallel+return_length))
        # 근접 배치도 고정 거리로 거부하지 않고 실제 회전 차체로 검사한다.
        for radius in (.286,.29,.30,.32,.35,.40,.50):
            for builder in (arc_transition,lateral_arc_transition):
                approach=builder(0.,initial_cross,initial_yaw,offset,radius)
                if not approach:continue
                end_parallel=max(far+c.rear+c.hard+.25,approach[-1][0]+.05)
                path=approach+transition(approach[-1][0],end_parallel,offset,offset)[1:]
                back=builder(end_parallel,offset,0.,0.,radius)
                if not back:continue
                path+=back[1:];join_x=path[-1][0]
                path+=transition(join_x,join_x+1.2,0.,0.)[1:]
                if path_clear(path,collision_points,c):
                    length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:]))
                    candidates.append((length,path,offset,join_x))
    if not candidates:
        return dict(reason='STOP_NO_COLLISION_CHECKED_DETOUR',path=[],**geometry)
    estimates=[]
    scored=candidates if objective=='time' else [min(candidates,key=lambda row:row[0])]
    for length,path,offset,join_x in scored:
        estimated=0.
        for previous,current in zip(path,path[1:]):
            x,y,a,k=current;co,si=math.cos(a),math.sin(a)
            near_points=[(px,py) for px,py in collision_points if abs(px-x)<1.05 and abs(py-y)<1.05]
            d=min((clearance(co*(px-x)+si*(py-y),-si*(px-x)+co*(py-y),c) for px,py in near_points),default=.60)
            factor=min(1.,max(0.,(d-c.hard)/.30))
            speed=min(nominal_speed,.98*(10*.065*2*math.pi/60)/(1+abs(k)*c.track/2))
            ds=math.hypot(x-previous[0],y-previous[1])
            estimated+=ds/max(1e-6,speed*factor)
        estimates.append((estimated,length,path,offset,join_x))
    estimated,length,path,offset,join_x=min(estimates,key=lambda row:row[0] if objective=='time' else row[1])
    return dict(reason='DETOUR_PLANNED',path=path,offset_m=offset,**geometry,
                join_x_m=join_x,candidates=len(candidates),side=side,evaluation_pass=False,
                objective=objective,nominal_speed_m_s=nominal_speed,estimated_follow_time_s=estimated,
                limitation='STATIC_OBSERVED_POINTS_AND_KNOWN_BOX_ONLY')


def pursue(path,pose,v,previous=0,lookahead=.25):
    """단조 진행, 작은 lookahead로 경로 모서리 안쪽 절단을 줄인다."""
    if not path or not 0<=v<=.1 or not .08<=lookahead<=.30:raise ValueError('PATH_OR_SPEED_INVALID')
    end=min(len(path),previous+81)
    closest=min(range(previous,end),key=lambda i:math.hypot(path[i][0]-pose.x,path[i][1]-pose.y))
    target=closest;distance=0.
    while target<len(path)-1 and distance<lookahead:
        distance+=math.hypot(path[target+1][0]-path[target][0],path[target+1][1]-path[target][1]);target+=1
    dx=path[target][0]-pose.x;dy=path[target][1]-pose.y
    x=math.cos(pose.yaw)*dx+math.sin(pose.yaw)*dy;y=-math.sin(pose.yaw)*dx+math.cos(pose.yaw)*dy
    error=math.hypot(path[closest][0]-pose.x,path[closest][1]-pose.y)
    if x<=0 or error>.15:return 0.,0.,closest,'STOP_DETOUR_TRACKING'
    if math.hypot(dx,dy)<.03:return 0.,0.,closest,'DETOUR_END'
    curvature=2*y/(x*x+y*y)
    # 외측 바퀴도 현재 10RPM(.065m 반경) 상한을 넘지 않도록 속도를 낮춘다.
    wheel_limit=10*.065*2*math.pi/60
    v=min(v,.98*wheel_limit/(1+abs(curvature)*.560/2))
    w=curvature*v;limit=min(.2,.98*2*v/.560)
    limited=abs(w)>limit+1e-9
    # 추종 오차로 요구 곡률이 일시적으로 커지면 가능한 조향으로 제한한다.
    # 출력의 전진/RPM 한계와 호출부의 회전 차체 검사는 그대로 유지한다.
    return v,max(-limit,min(limit,w)),closest,'DETOUR_TRACK_CURVATURE_LIMITED' if limited else 'DETOUR_TRACK'
