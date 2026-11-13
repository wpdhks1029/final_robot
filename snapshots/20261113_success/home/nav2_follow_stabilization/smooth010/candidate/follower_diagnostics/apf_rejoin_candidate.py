"""단일 정적 상자·직선 경로 APF 후보. ROS 접근 없음. 실물 검증 전."""
import math
from apf_case3_candidate import RejoinEvidence,clearance,front_blocker,wrap


def connected_groups(points,gap=.10):
    """폭 추정용 공간 군집. 안전 검사에 쓰이는 원본 점은 변경하지 않는다."""
    cells={}
    for i,(x,y) in enumerate(points):cells.setdefault((math.floor(x/gap),math.floor(y/gap)),set()).add(i)
    remaining=set(range(len(points)));groups=[]
    while remaining:
        first=remaining.pop();group=[points[first]];queue=[first]
        while queue:
            i=queue.pop();x,y=points[i];cx,cy=math.floor(x/gap),math.floor(y/gap)
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    for j in tuple(cells.get((cx+dx,cy+dy),())):
                        if j in remaining and math.hypot(points[j][0]-x,points[j][1]-y)<=gap:
                            remaining.remove(j);queue.append(j);group.append(points[j])
        groups.append(group)
    return groups


class StraightRejoin:
    def __init__(self,settings,side):
        self.c=settings;self.side=side;self.reference=None;self.evidence=None
        self.phase='TRACK';self.last_evidence=None
        self.last_pose=None;self.invalidated=False
        self.rejoining=False

    def command(self,v,w,points,pose,path,stamp):
        c=self.c;x,y,yaw=pose
        if (not all(math.isfinite(n) for n in (v,w,x,y,yaw)) or v<0 or v>c.max_v or abs(w)>c.max_w
                or type(stamp) is not int or stamp<=0 or len(path)<2):return self.stop('STOP_APF_REFERENCE_INVALID')
        if self.invalidated:return self.stop('STOP_APF_POSE_JUMP')
        if self.last_pose is not None and stamp>self.last_pose[0] and math.hypot(x-self.last_pose[1],y-self.last_pose[2])>.15:
            self.invalidated=True;return self.stop('STOP_APF_POSE_JUMP')
        if self.last_pose is None or stamp>self.last_pose[0]:self.last_pose=(stamp,x,y)
        if self.reference is None:
            blocking=[(px,py) for px,py in points if front_blocker(px,py,clearance(px,py,c),c)]
            if not blocking:return dict(v=v,w=w,phase='TRACK',reason='APF_REJOIN_CANDIDATE')
            if len(blocking)<5:return self.stop('STOP_APF_OBSTACLE_CLUSTER_INSUFFICIENT')
            groups=[g for g in connected_groups(blocking) if len(g)>=5]
            if not groups:return self.stop('STOP_APF_OBSTACLE_CLUSTER_INSUFFICIENT')
            # 가장 가까운 충분한 군집만 기하 기준으로 잡는다. 넓은 군집은
            # 잘라내거나 건너뛰지 않고 아래 폭 검사에서 계속 거부한다.
            blocking=min(groups,key=lambda g:min(clearance(px,py,c) for px,py in g))
            if len(path)<2:return self.stop('STOP_APF_REFERENCE_INVALID')
            # 현재 위치에 가장 가까운 유효 선분을 사용한다. 교차/곡선 경로는 거부한다.
            segments=[]
            for a,b in zip(path,path[1:]):
                dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
                if length<.03:continue
                t=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/length**2))
                segments.append((math.hypot(x-a[0]-t*dx,y-a[1]-t*dy),a,math.atan2(dy,dx)))
            if not segments:return self.stop('STOP_APF_REFERENCE_INVALID')
            _,origin,heading=min(segments,key=lambda s:s[0])
            if abs(wrap(yaw-heading))>math.radians(15):return self.stop('STOP_APF_REFERENCE_HEADING')
            if any(abs(wrap(h-heading))>math.radians(10) for _,_,h in segments):return self.stop('STOP_APF_CURVED_REFERENCE')
            co,si=math.cos(heading),math.sin(heading)
            world=[(x+math.cos(yaw)*px-math.sin(yaw)*py,y+math.sin(yaw)*px+math.cos(yaw)*py) for px,py in blocking]
            coords=[(co*(wx-origin[0])+si*(wy-origin[1]),-si*(wx-origin[0])+co*(wy-origin[1])) for wx,wy in world]
            lateral=[p[1] for p in coords]
            if max(lateral)-min(lateral)>.35:return self.stop('STOP_APF_OBSTACLE_WIDTH')
            center=(min(lateral)+max(lateral))/2
            # 확인된 20cm 시험물 치수를 사용한다. 일반 물체에는 자동 적용하지 않는다.
            end=min(p[0] for p in coords)+c.obstacle_length
            target=center+self.side*(c.obstacle_width/2+c.half_width+c.hard+.05)
            self.reference=(origin,heading,target)
            self.evidence=RejoinEvidence(origin,heading,end,cross_tolerance=.10,
                heading_tolerance=math.radians(10),settings=c)
        origin,heading,target=self.reference
        if any(abs(-math.sin(heading)*(px-origin[0])+math.cos(heading)*(py-origin[1]))>.15 for px,py in path):
            return self.stop('STOP_APF_REFERENCE_CHANGED')
        # 동일 pose는 재적산하지 않고 최근 기하 판정을 재사용한다.
        if self.evidence.last_stamp is None or stamp>self.evidence.last_stamp:
            self.last_evidence=self.evidence.step(stamp,x,y,yaw)
        elif stamp<self.evidence.last_stamp:return self.stop('STOP_APF_POSE_ORDER')
        e=self.last_evidence
        along=math.cos(heading)*(x-origin[0])+math.sin(heading)*(y-origin[1])
        relative=wrap(yaw-heading)
        rear_along=along-max(c.front,c.rear)*abs(math.cos(relative))-c.half_width*abs(math.sin(relative))
        # 차체 통과만으로 즉시 꺾지 않는다. 후방 상자와 회전 여유를 확보한다.
        turn_clear=rear_along>self.evidence.end+c.hard+.15
        if turn_clear:self.rejoining=True
        if e['whole_body_passed'] and self.rejoining:
            self.phase='REJOIN' if e['phase']=='REJOIN' else e['phase']
            target=0.
        else:
            has_front=any(front_blocker(px,py,clearance(px,py,c),c) for px,py in points)
            self.phase='AVOID' if has_front else 'PASS'
        cross=-math.sin(heading)*(x-origin[0])+math.cos(heading)*(y-origin[1])
        # APF 인력: 통과 중에는 측방 가상 목표, 통과 후에는 원래 직선으로 복귀한다.
        desired=heading+max(-math.radians(40),min(math.radians(40),math.atan2(3.0*(target-cross),1.)))
        fy=0.;count=0
        for px,py in points:
            d=clearance(px,py,c)
            if d<=c.hard:return self.stop('STOP_FOOTPRINT_CLEARANCE')
            if d<c.influence:
                norm=math.hypot(px,py)
                if norm>1e-6:
                    fy-=min(2.,.025*(1/d-1/c.influence)/(d*d))*py/norm;count+=1
        repulsion=0. if count==0 else .15*math.atan2(fy/count,1.)
        out_v=min(v*.6,.04)
        out_w=.8*wrap(desired-yaw)+repulsion
        limit=min(c.max_w,2*out_v/c.track)
        out_w=max(-limit,min(limit,out_w))
        if e['phase'] in ('FOLLOW_AFTER_REJOIN','GEOMETRIC_COMPLETE_CANDIDATE'):
            out_v=v;out_w=max(-min(c.max_w,2*v/c.track),min(min(c.max_w,2*v/c.track),w))
        return dict(v=out_v,w=out_w,phase=self.phase,reason='APF_REJOIN_CANDIDATE',evidence=e,
                    side=self.side,evaluation_pass=False)

    def stop(self,reason):return dict(v=0.,w=0.,phase=self.phase,reason=reason,evaluation_pass=False)
