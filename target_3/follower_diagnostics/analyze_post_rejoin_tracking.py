"""Offline model comparison; recorded localization drift is imposed, not corrected."""
import json,math
from pathlib import Path
import numpy as np
from tracked_pursuit_candidate import TrackedPreviewCore
from compare_avoidance_strategies import advance

SOURCE=Path(__file__).parent/'logs/local_avoidance_trial_1793865762811694697.jsonl'
def simulate(gain):
    rows=[r for line in SOURCE.read_text().splitlines() if 'post_rejoin_map_verification' in (r:=json.loads(line))]
    times=np.array([(r['ros_ns']-rows[0]['ros_ns'])/1e9 for r in rows])
    biases=[]
    for r in rows:
        v=r['post_rejoin_map_verification'];m=v['map_pose'];e=v['predicted_map_pose']
        biases.append([m[0]-e[0],m[1]-e[1],math.atan2(math.sin(m[2]-e[2]),math.cos(m[2]-e[2]))])
    biases=np.array(biases);biases[:,2]=np.unwrap(biases[:,2]);biases-=biases[0]
    p=tuple(rows[0]['post_rejoin_map_verification']['map_pose']);nav=TrackedPreviewCore(.35);nav.speed_gate=lambda *args:(.025,0.,'FORWARD_ALLOWED')
    va=wa=0.;trace=[];verified=0.;best=0.;previous=None
    for k in range(1200):
        t=k*.05;bias=np.array([np.interp(t,times,biases[:,j]) for j in range(3)])
        m=(p[0]+bias[0],p[1]+bias[1],p[2]+bias[2]);idx=min(len(rows)-1,int(np.searchsorted(times,t)))
        path=rows[idx]['post_rejoin_map_verification']['live_path_points']
        nav.inputs={'path':(0,0,path),'follower':(0,0,(*m,0.,0.)),'leader':(0,0,(*path[-1],math.atan2(path[-1][1]-path[-2][1],path[-1][0]-path[-2][0]),.04,0.))}
        v,w,reason=nav.command(0,0)
        if reason!='TRACKED_PURSUIT':return {'reason':reason}
        s=nav.last_selection;error=math.atan2(math.sin(s['path_heading_rad']-m[2]),math.cos(s['path_heading_rad']-m[2]))
        w=max(-.2,min(.2,w+gain*v*error));w=max(-8*v,min(8*v,w))
        rpm=np.round(np.array([v-w*.28,v+w*.28])/.065*60/(2*math.pi));qv=float(rpm.mean()*.065*2*math.pi/60);qw=float((rpm[1]-rpm[0])*.065*2*math.pi/60/.56)
        va+=(qv-va)*.25;wa+=(qw-wa)*.25;p=advance(p,va,wa,.05)
        aligned=s['path_distance_m']<=.1 and abs(error)<=math.radians(10)
        if aligned and previous is not None:verified+=max(0.,(m[0]-previous[0])*math.cos(s['path_heading_rad'])+(m[1]-previous[1])*math.sin(s['path_heading_rad']))
        elif not aligned:verified=0.
        best=max(best,verified);previous=m
        trace.append({'t':t,'lateral_m':s['path_distance_m'],'heading_deg':abs(math.degrees(error))})
    return {'gain_per_m':gain,'reason':'MODEL_COMPLETED','max_heading_deg':max(r['heading_deg'] for r in trace),
            'max_lateral_m':max(r['lateral_m'] for r in trace),'final_lateral_m':trace[-1]['lateral_m'],
            'longest_aligned_progress_m':best,'trace':trace}

if __name__=='__main__':
    results=[simulate(gain) for gain in (0.,1.,2.,3.,4.)]
    out={'simulation_only':True,'evaluation_pass':False,'source_trial':str(SOURCE),'limitation':'RECORDED_LOCALIZATION_DRIFT_IMPOSED_ON_SIMULATED_POSE; NOT_A_PHYSICAL_REPLAY; WORKSPACE_NOT_MODELLED','results':results}
    target=SOURCE.parent/'post_rejoin_heading_comparison.json';target.write_text(json.dumps(out))
    print(json.dumps([{k:v for k,v in r.items() if k!='trace'} for r in results]))
