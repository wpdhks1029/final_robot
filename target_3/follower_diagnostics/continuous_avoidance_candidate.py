"""Offline-only continuous20cm-clearance candidate. No ROS or motor output."""
import math,time
import numpy as np
from apf_pivot_candidate import swept_clearance
from compare_avoidance_strategies import advance

CLEARANCE=.20

def route(start,box,offset,approach,rejoin):
    x0,y0,a0=start
    # Quintic interpolation has zero lateral slope/curvature at each junction.
    def leg(xa,xb,ya,yb):
        n=max(2,math.ceil((xb-xa)/.01));out=[]
        for t in np.linspace(0,1,n+1):
            q=10*t**3-15*t**4+6*t**5
            derivative=(yb-ya)*(30*t*t-60*t**3+30*t**4)/(xb-xa)
            out.append((xa+t*(xb-xa),ya+(yb-ya)*q,math.atan(derivative)))
        return out
    end=x0+approach;passed=max(end+.1,box['far']+.6)
    return leg(x0,end,y0,offset)+leg(end,passed,offset,offset)[1:]+leg(passed,passed+rejoin,offset,0.)[1:]+leg(passed+rejoin,passed+rejoin+1.2,0.,0.)[1:]

def replay(points,start,path,limit=500,deadline=None):
    p=tuple(start);idx=0;minimum=99.;trace=[]
    cloud=np.asarray(points);xy=np.asarray(path)[:,:2]
    for i in range(int(limit/.05)):
        if deadline is not None and time.monotonic()>=deadline:return dict(reason='PLANNING_WALL_BUDGET',minimum_m=minimum)
        search=xy[idx:min(len(xy),idx+80)]
        idx+=int(np.argmin(np.linalg.norm(search-np.array(p[:2]),axis=1)))
        target_index=idx
        while target_index<len(xy)-1 and np.linalg.norm(xy[target_index]-p[:2])<.12:target_index+=1
        dx,dy=xy[target_index]-p[:2];c,s=math.cos(p[2]),math.sin(p[2]);x=c*dx+s*dy;y=-s*dx+c*dy
        if x<=0:return dict(reason='TARGET_BEHIND',minimum_m=minimum)
        curvature=2*y/max(dx*dx+dy*dy,.001)
        if abs(curvature)>8.:return dict(reason='CURVATURE_INFEASIBLE',minimum_m=minimum)
        v=.025;w=v*curvature
        scale=min(1.,10*.065*2*math.pi/60/(v+abs(w)*.28));v*=scale;w*=scale
        rpm=np.round(np.array([v-w*.28,v+w*.28])/.065*60/(2*math.pi))
        if max(abs(rpm))>10:return dict(reason='WHEEL_BOUND',minimum_m=minimum)
        qv=float(rpm.mean()*.065*2*math.pi/60);qw=float((rpm[1]-rpm[0])*.065*2*math.pi/60/.56)
        future=[advance(p,qv*1.35,qw*1.35,k*.1) for k in range(11)]
        d=swept_clearance(cloud,future)-.02;minimum=min(minimum,d)
        if d<=CLEARANCE:return dict(reason='SWEPT_CLEARANCE',minimum_m=minimum)
        p=advance(p,qv,qw,.05)
        if i%20==0:trace.append(dict(t=i*.05,pose=p,v=qv,w=qw))
        if idx>=len(xy)-5:return dict(reason='SIM_REJOIN_PLUS_1M',minimum_m=minimum,duration_s=i*.05,trace=trace)
    return dict(reason='TIME_LIMIT',minimum_m=minimum)

def search(points,start,box):
    results=[];deadline=time.monotonic()+60.
    for extra in (.04,.08,.12):
        for approach in (.65,.8,.95,1.1):
            for rejoin in (1.,1.4,1.8):
                if time.monotonic()>=deadline:return dict(reason='CONTINUOUS_PLANNING_BUDGET_EXCEEDED',attempts=results)
                offset=box['high']+.30+CLEARANCE+extra
                path=route(start,box,offset,approach,rejoin)
                if swept_clearance(points,path)<=CLEARANCE+.07:continue
                result=replay(points,start,path,deadline=deadline)
                result.update(offset_m=offset,approach_m=approach,rejoin_m=rejoin)
                results.append(result)
                if result['reason']=='PLANNING_WALL_BUDGET':return dict(reason='CONTINUOUS_PLANNING_BUDGET_EXCEEDED',attempts=results)
                if result['reason']=='SIM_REJOIN_PLUS_1M':return result
    return dict(reason='NO_VALIDATED_CONTINUOUS_ROUTE',attempts=[{k:v for k,v in r.items() if k!='trace'} for r in results])
