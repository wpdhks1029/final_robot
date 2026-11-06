"""정적 기하 + Pure Pursuit 모의. ROS/모터/네트워크 접근 없음."""
import json,math,time
from pathlib import Path
from temporary_detour_candidate import Settings,Pose,plan,pursue
from apf_case3_candidate import clearance
from apf_guard_connection import swept_clear


def main():
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--one-meter-gap',action='store_true',help='후발 앞끝~상자 .60m, 20cm상자, 선두 뒤끝까지 .20m인 개발 배치')
    args=parser.parse_args()
    box_x=.87 if args.one_meter_gap else 1.6
    c=Settings();world=[]
    for i in range(201):world.extend([(-1+i*.04,-1.25),(-1+i*.04,1.25)])
    for i in range(21):
        t=i/20;world.extend([(box_x,.25+.2*t),(box_x+.2,.25+.2*t),(box_x+.2*t,.25),(box_x+.2*t,.45)])
    result=plan(world,c,side=-1)
    if not result['path']:raise RuntimeError(result['reason'])
    x=y=yaw=0.;index=0;rows=[];reason='SIMULATION_LIMIT';passed=False;joined_at=None
    for i in range(10000):
        v,w,index,state=pursue(result['path'],Pose(x,y,yaw),.04,index)
        co,si=math.cos(yaw),math.sin(yaw)
        points=[(co*(px-x)+si*(py-y),-si*(px-x)+co*(py-y)) for px,py in world]
        distance=min(clearance(px,py,c) for px,py in points)
        factor=min(1.,max(0.,(distance-c.hard)/.3));v*=factor;w*=factor
        rows.append(dict(t=i*.1,x=x,y=y,yaw=yaw,clearance=distance,v=v,w=w,index=index))
        if state.startswith('STOP') or not swept_clear(points,v,w,c):reason=state if state.startswith('STOP') else 'STOP_SWEPT';break
        if distance<=c.hard:reason='STOP_CLEARANCE';break
        rear=x-c.rear*abs(math.cos(yaw))-c.half_width*abs(math.sin(yaw))
        passed=rear>result['obstacle_far_x_m']
        aligned=passed and abs(y)<.1 and abs(yaw)<math.radians(10) and x>=result['join_x_m']
        if not aligned:joined_at=None
        elif joined_at is None:joined_at=x
        if joined_at is not None and x-joined_at>=1.:reason='SIMULATED_PASS_REJOIN_PLUS_1M';break
        x+=v*math.cos(yaw)*.1;y+=v*math.sin(yaw)*.1;yaw+=w*.1
    output=dict(reason=reason,simulation_only=True,evaluation_pass=False,physical_contact_observed=False,
                layout='ONE_METER_BODY_GAP' if args.one_meter_gap else 'LONG_APPROACH',
                leader_motion_model='NOT_SIMULATED; STATIC_SCAN_RECHECK_REQUIRED',
                duration_s=rows[-1]['t'],min_clearance_m=min(r['clearance'] for r in rows),plan=result,trace=rows)
    filename='temporary_detour_one_meter_simulation.json' if args.one_meter_gap else 'temporary_detour_simulation.json'
    path=Path(__file__).parent/'logs'/filename;path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(output,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in output.items() if k not in ('trace','plan')},ensure_ascii=False),flush=True)
    print('JSON='+str(path),flush=True)
    if reason!='SIMULATED_PASS_REJOIN_PLUS_1M':raise SystemExit(1)


if __name__=='__main__':main()
