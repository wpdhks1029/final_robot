"""APF 조향 후보와 기존 보호의 결합. 재합류 미구현 구간은 정지한다."""
import math
from apf_case3_candidate import APF,Settings,clearance
from follower_obstacle_preview import ObstacleCore


def swept_clear(points,v,w,c,horizon=1.):
    # 현재 관측한 정적 점에 대해 1초간 사각 차체를 검사한다.
    # 동적 장애물 예측이나 미관측 공간의 안전을 보장하지 않는다.
    speed_bound=abs(v)+abs(w)*math.hypot(max(c.front,c.rear),c.half_width)
    margin=speed_bound*horizon/40.
    nearby=[(x,y) for x,y in points if clearance(x,y,c)<=c.hard+speed_bound*horizon+margin]
    for i in range(1,41):
        t=horizon*i/40.;a=w*t
        x=v*t if abs(w)<1e-9 else v*math.sin(a)/w
        y=0. if abs(w)<1e-9 else v*(1-math.cos(a))/w
        co,si=math.cos(a),math.sin(a)
        for px,py in nearby:
            dx,dy=px-x,py-y
            # 샘플 사이 이동도 보수적으로 포함한다.
            if clearance(co*dx+si*dy,-si*dx+co*dy,c)<=c.hard+margin:return False
    return True


class APFGuardCore(ObstacleCore):
    def __init__(self,*,settings,preferred_side,enable_rejoin=False,**kwargs):
        super().__init__(**kwargs)
        if (self.half_width,self.front,self.rear,self.hard)!=(settings.half_width,settings.front,settings.rear,settings.hard):
            raise ValueError('APF/guard geometry mismatch')
        self.apf=APF(settings,preferred_side);self.points=[];self.apf_status=None
        self.navigation=None;self.navigation_source_ns=0
        self.rejoin=None
        if enable_rejoin:
            from apf_rejoin_candidate import StraightRejoin
            self.rejoin=StraightRejoin(settings,preferred_side)

    def update_scan(self,msg,transform,now_ns,received):
        self.points=[]
        super().update_scan(msg,transform,now_ns,received)
        if self.scan is not None:
            from watch_apf_case3 import project
            try:self.points=project(msg,transform)
            except (ValueError,AttributeError):self.scan=None

    def command(self,now_ns,received):
        v,w,reason=super().command(now_ns,received)
        if reason.startswith('STOP'):return v,w,reason
        # APF 계산 뒤에 기존의 감속·정지 보호를 적용한다.
        _,raw_v,raw_w=self.raw
        if self.rejoin is None:self.apf_status=self.apf.step(raw_v,raw_w,self.points)
        else:
            if self.navigation is None:return 0.,0.,'STOP_APF_NAVIGATION_MISSING'
            stamps=[]
            for key in ('path','follower'):
                row=self.navigation.inputs.get(key)
                if row is None:return 0.,0.,'STOP_APF_NAVIGATION_MISSING'
                stamp=row[0].sec*10**9+row[0].nanosec
                if key=='path':stamp-=50_000_000
                if not (stamp>0 and 0<=(now_ns-stamp)/1e9<=.3 and 0<=received-row[1]<=.3):
                    return 0.,0.,'STOP_APF_NAVIGATION_STALE'
                stamps.append(stamp)
            self.navigation_source_ns=min(stamps)
            pose=self.navigation.inputs['follower'][2][:3]
            pose_stamp=self.navigation.inputs['follower'][0]
            self.apf_status=self.rejoin.command(raw_v,raw_w,self.points,pose,
                self.navigation.inputs['path'][2],pose_stamp.sec*10**9+pose_stamp.nanosec)
        r=self.apf_status
        if r['reason'] not in ('APF_CANDIDATE','APF_REJOIN_CANDIDATE'):return 0.,0.,r['reason']
        if self.rejoin is None and r['phase'] in ('PASS','REJOIN'):return 0.,0.,'STOP_APF_REJOIN_NOT_CONNECTED'
        old=self.raw
        try:
            self.raw=(old[0],r['v'],r['w'])
            v,w,reason=super().command(now_ns,received)
        finally:self.raw=old
        # 경계값에 딱 맞춘 조향은 감속/직렬화의 반올림으로 전진 조건을
        # 위반할 수 있다. 감독·드라이버 검사를 완화하지 않고 출력에 여유를 둔다.
        limit=min(self.apf.c.max_w,.98*2*v/self.apf.c.track)
        w=max(-limit,min(limit,w))
        if not swept_clear(self.points,v,w,self.apf.c):return 0.,0.,'STOP_APF_SWEPT_CLEARANCE'
        return v,w,reason
