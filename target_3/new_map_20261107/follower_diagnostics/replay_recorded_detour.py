"""기록 스캔의 좌회피 폐루프 모의. 실제 ROS/모터/설정 변경 없음."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
from temporary_detour_candidate import Pose,pursue


def coordinated_speed(leader_x,follower_x,follower_yaw,follower_v,target_distance,*,enabled=True):
    if not enabled:return 0.
    if not all(math.isfinite(v) for v in (leader_x,follower_x,follower_yaw,follower_v,target_distance)):
        raise ValueError('Invalid coordinated policy input')
    desired=follower_v*math.cos(follower_yaw)+.4*(follower_x+target_distance-leader_x)
    return max(.012,min(.04,desired))


def run(data,lookahead=.15,noise=0.,seed=7,pose_delay_s=0.,follower_speed=.04,leader_policy='constant'):
    if not 0<follower_speed<=.06 or leader_policy not in ('constant','coordinated'):raise ValueError('Invalid simulation policy')
    rng=np.random.RandomState(seed);route=data['path'];s=data['snapshot'];points=list(data['points'])
    shift=data['box_forward_m']
    near=data.get('box_near_x_m',.9002417809897474+shift)
    far=data.get('box_model_far_x_m',1.1012517062222593+shift)
    low=data.get('box_model_low_y_m',.1335417078129606);high=data.get('box_model_high_y_m',.359110280756642)
    for i in range(41):
        q=i/40;points.extend([(near,low+(high-low)*q),(far,low+(high-low)*q),
                             (near+(far-near)*q,low),(near+(far-near)*q,high)])
    px=np.array([p[0] for p in points]);py=np.array([p[1] for p in points])
    x,y,yaw=route[0][:3];index=0;rows=[];join_x=None;minimum=999.;reason='SIMULATION_LIMIT'
    leader_x0=.27*abs(math.cos(yaw))+.3*abs(math.sin(yaw))+1.+.27
    # 초기 차체 간격 1m. 선두가 15cm 전진하고 후발 정지 중 상자를 투입하는 가상 시점.
    leader_speed=.04;prelude=max(shift/leader_speed,1. if leader_policy=='coordinated' else 0.);pose_history=[]
    lx=leader_x0+leader_speed*prelude;previous_v=0.;max_gap=0.;max_leader_x=lx;min_leader_v=.04
    for i in range(20000):
        t=i*.05
        if leader_policy=='constant':lx=leader_x0+leader_speed*(prelude+t)
        else:
            leader_speed=coordinated_speed(lx,x,yaw,previous_v,leader_x0-route[0][0])
            if i:lx+=leader_speed*.05
        max_leader_x=max(max_leader_x,lx);min_leader_v=min(min_leader_v,leader_speed)
        max_gap=max(max_gap,lx-x)
        pose_history.append((x,y,yaw))
        delayed=pose_history[max(0,len(pose_history)-1-round(pose_delay_s/.05))]
        measured=Pose(*(a+rng.normal(0,noise) for a in delayed))
        v,w,index,state=pursue(route,measured,follower_speed,index,lookahead)
        # 선두 차체도 매 주기 실제 점과 동일한 보호 검사에 포함한다.
        q=np.linspace(0,1,17)
        lead_px=np.concatenate([np.full(17,lx-.27),np.full(17,lx+.27),lx-.27+.54*q,lx-.27+.54*q])
        lead_py=np.concatenate([-.3+.6*q,-.3+.6*q,np.full(17,-.3),np.full(17,.3)])
        allx=np.concatenate([px,lead_px]);ally=np.concatenate([py,lead_py])
        def distance_at(xx,yy,aa):
            dx=allx-xx;dy=ally-yy;co=np.cos(aa);si=np.sin(aa)
            return float(np.hypot(np.maximum(abs(co*dx+si*dy)-.27,0),np.maximum(abs(-si*dx+co*dy)-.3,0)).min())
        distance=distance_at(x,y,yaw);minimum=min(minimum,distance)
        factor=min(1.,max(0.,(distance-.3)/.3));v*=factor;w*=factor
        # 1초 예측 사각 차체 검사. 움직이는 선두는 현 위치에 고정해 보수적으로 검사.
        if v>0:
            future=np.linspace(.05,1.,20);angles=yaw+w*future
            xx=x+v*future*np.cos(yaw) if abs(w)<1e-9 else x+v*(np.sin(angles)-math.sin(yaw))/w
            yy=y+v*future*np.sin(yaw) if abs(w)<1e-9 else y+v*(math.cos(yaw)-np.cos(angles))/w
            speed=abs(v)+abs(w)*math.hypot(.27,.3)
            nearby=(abs(allx-x)<.85)&(abs(ally-y)<.85)
            if np.any(nearby):
                dx=allx[nearby][None,:]-xx[:,None];dy=ally[nearby][None,:]-yy[:,None]
                co=np.cos(angles)[:,None];si=np.sin(angles)[:,None]
                predicted=float(np.hypot(np.maximum(abs(co*dx+si*dy)-.27,0),
                                         np.maximum(abs(-si*dx+co*dy)-.3,0)).min())
            else:predicted=float('inf')
            if predicted<=.3+speed*.05:
                reason='STOP_SWEPT_CLEARANCE';break
        rows.append(dict(t=t,x=x,y=y,yaw=yaw,v=v,w=w,index=index,clearance_m=distance,leader_x=lx,leader_v=leader_speed))
        previous_v=v
        if state.startswith('STOP'):reason=state;break
        if distance<=.3:reason='STOP_CLEARANCE';break
        if (v+abs(w)*.28)/(.065*2*math.pi)*60>10.+1e-8:reason='STOP_WHEEL_LIMIT';break
        rear=x-.27*abs(math.cos(yaw))-.3*abs(math.sin(yaw))
        aligned=rear>far and x>=data['join_x_m'] and abs(y)<=.1 and abs(yaw)<=math.radians(10)
        if not aligned:join_x=None
        elif join_x is None:join_x=x
        if join_x is not None and x-join_x>=1.:
            reason='SIMULATED_PASS_REJOIN_PLUS_1M';break
        dt=.05
        if abs(w)<1e-9:x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt
        else:
            end=yaw+w*dt;x+=v*(math.sin(end)-math.sin(yaw))/w;y+=v*(math.cos(yaw)-math.cos(end))/w
        yaw+=w*dt
    return dict(reason=reason,lookahead_m=lookahead,pose_noise_std=noise,seed=seed,
                duration_s=i*.05,min_clearance_m=minimum,trace=rows,simulation_only=True,evaluation_pass=False,
                pose_delay_s=pose_delay_s,
                nominal_follower_speed_m_s=follower_speed,leader_policy=leader_policy,
                leader_travel_m=max_leader_x-leader_x0,max_longitudinal_center_gap_m=max_gap,
                min_leader_speed_m_s=min_leader_v,
                initial_body_gap_m=1.,leader_prelude_s=prelude,box_forward_m=shift,
                static_snapshot_condition='All recorded scan points unchanged; no box translation' if shift==0 else 'All non-box scan points unchanged; box moved forward only',
                network_model='IDEAL_FRESH_INPUT; communications not validated',
                leader_geometry_assumption_m=dict(front=.27,rear=.27,half_width=.3))


def main():
    p=argparse.ArgumentParser();p.add_argument('--plan',type=Path,default=Path(__file__).parent/'logs/recorded_left_detour_plan.json')
    p.add_argument('--lookahead',type=float,default=.15);p.add_argument('--noise',type=float,default=0.)
    p.add_argument('--seed',type=int,default=7);p.add_argument('--pose-delay',type=float,default=0.)
    p.add_argument('--follower-speed',type=float,default=.04)
    p.add_argument('--leader-policy',choices=['constant','coordinated'],default='constant')
    a=p.parse_args();data=json.loads(a.plan.read_text());result=run(data,a.lookahead,a.noise,a.seed,a.pose_delay,a.follower_speed,a.leader_policy)
    result['plan_file']=str(a.plan.resolve())
    dest=a.plan.parent/f'left_closed_loop_{time.time_ns()}.json';dest.write_text(json.dumps(result)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='trace'},ensure_ascii=False),flush=True);print('JSON='+str(dest),flush=True)
    return 0 if result['reason']=='SIMULATED_PASS_REJOIN_PLUS_1M' else 1


if __name__=='__main__':raise SystemExit(main())
