"""저장 스캔의 전진 전용 운동학 경로 탐색. 가상 계산만 수행한다."""
import argparse,heapq,json,math,time
from pathlib import Path
import numpy as np
from apf_case3_candidate import wrap


def search(points,start,goal,*,limit=60000,timeout=40.):
    px=np.asarray([p[0] for p in points]);py=np.asarray([p[1] for p in points])
    def clear(p):
        x,y,a=p;co,si=math.cos(a),math.sin(a)
        mask=(abs(px-x)<.75)&(abs(py-y)<.75)
        dx=px[mask]-x;dy=py[mask]-y
        bx=co*dx+si*dy;by=-si*dx+co*dy
        return not np.any(np.hypot(np.maximum(abs(bx)-.27,0),np.maximum(abs(by)-.3,0))<=.315)
    def key(p):return (round(p[0]/.035),round(p[1]/.035),round(p[2]/math.radians(5)))
    def heuristic(p):return math.hypot(goal[0]-p[0],goal[1]-p[1])+.15*abs(wrap(goal[2]-p[2]))
    if not clear(start):return dict(reason='START_COLLISION',path=[])
    poses=[start];parents=[None];segments=[[]];costs=[0.];best={key(start):0.}
    queue=[(heuristic(start),0)];began=time.monotonic();expanded=0
    while queue and expanded<limit and time.monotonic()-began<timeout:
        _,index=heapq.heappop(queue);pose=poses[index];expanded+=1
        if math.hypot(pose[0]-goal[0],pose[1]-goal[1])<.12 and abs(wrap(pose[2]-goal[2]))<math.radians(10):
            route=[];i=index
            while parents[i] is not None:route.append(segments[i]);i=parents[i]
            path=[(*start,0.)]
            for segment in reversed(route):path.extend(segment)
            return dict(reason='SEARCH_PATH_FOUND',path=path,expanded=expanded,seconds=time.monotonic()-began)
        for k in (-3.35,-2.,-1.,0.,1.,2.,3.35):
            x,y,a=pose;segment=[];valid=True
            for j in range(16):
                ds=.005;end=a+k*ds
                if k:x+=(math.sin(end)-math.sin(a))/k;y+=(math.cos(a)-math.cos(end))/k
                else:x+=ds*math.cos(a);y+=ds*math.sin(a)
                a=end
                if not (-.5<x<3.8 and -.65<y<1.4 and abs(a)<math.radians(110)) or not clear((x,y,a)):
                    valid=False;break
                segment.append((x,y,a,k))
            if not valid:continue
            nxt=(x,y,a);newcost=costs[index]+.08+.002*abs(k);cell=key(nxt)
            if newcost>=best.get(cell,float('inf')):continue
            best[cell]=newcost;poses.append(nxt);parents.append(index);segments.append(segment);costs.append(newcost)
            heapq.heappush(queue,(newcost+1.2*heuristic(nxt),len(poses)-1))
    return dict(reason='SEARCH_LIMIT' if queue else 'SEARCH_EXHAUSTED',path=[],expanded=expanded,seconds=time.monotonic()-began)


def main():
    p=argparse.ArgumentParser();p.add_argument('--snapshot',type=Path,required=True)
    p.add_argument('--earlier-start',type=float,default=0.)
    p.add_argument('--box-forward',type=float,default=0.,help='가상 상자만 진행방향으로 이동. 실제 설정 변경 없음')
    p.add_argument('--seconds',type=float,default=40.)
    args=p.parse_args();data=json.loads(args.snapshot.read_text());s=data['snapshot']
    if args.snapshot.name!='temporary_detour_check_1793703177417779781.json':
        p.error('이 실험은 명시된 기록 스냅샷 전용입니다. 다른 스캔에는 상자 ROI를 다시 검증해야 합니다.')
    points=list(s['points']);box_roi=(.87,1.13,.10,.39)
    if args.box_forward:
        points=[(x+args.box_forward,y) if box_roi[0]<x<box_roi[1] and box_roi[2]<y<box_roi[3] else (x,y) for x,y in points]
    # 알려진 상자 크기의 미관측 후면도 포함한다.
    for i in range(41):
        f=i/40;near=.9002417809897474+args.box_forward;far=1.1012517062222593+args.box_forward
        low=.1335417078129606;high=.359110280756642
        points.extend([(near,low+(high-low)*f),(far,low+(high-low)*f),
                       (near+(far-near)*f,low),(near+(far-near)*f,high)])
    result=search(points,(-args.earlier_start,s['cross_error_m'],s['pose']['yaw']-s['reference_heading']),
                  (2.8,0.,0.),timeout=args.seconds)
    result.update(snapshot=str(args.snapshot),earlier_start_m=args.earlier_start,box_forward_m=args.box_forward,
                  points=points,simulation_only=True,control_connected=False)
    dest=args.snapshot.parent/f'detour_search_{time.time_ns()}.json';dest.write_text(json.dumps(result)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('path','points')}),flush=True);print('JSON='+str(dest),flush=True)


if __name__=='__main__':main()
