"""검증 로그를 요약하고 외부 의존성 없는 SVG/HTML 결과를 만든다."""
import hashlib,html,json,math
from pathlib import Path


def main():
    import argparse
    root=Path(__file__).parent;logs=root/'logs'
    parser=argparse.ArgumentParser();parser.add_argument('--plan',type=Path,default=logs/'recorded_left_detour_plan.json')
    args=parser.parse_args();plan=json.loads(args.plan.read_text());target_source=plan['source']
    files=[];run_plans={}
    for p in logs.glob('left_closed_loop_*.json'):
        d=json.loads(p.read_text())
        if d['reason']!='SIMULATED_PASS_REJOIN_PLUS_1M':continue
        run_path=Path(d['plan_file']) if d.get('plan_file') else logs/('recorded_left_detour_time_plan.json' if d.get('leader_policy')=='coordinated' else 'recorded_left_detour_plan.json')
        run_plan=json.loads(run_path.read_text())
        if run_plan['source']!=target_source or d['box_forward_m']!=plan['box_forward_m']:continue
        run_plans[p.name]=run_plan;files.append((p,d))
    if not files:raise RuntimeError('Successful closed-loop evidence missing')
    results=[]
    for p,d in files:
        trace=d['trace'];max_rpm=max((r['v']+abs(r['w'])*.28)/(.065*2*math.pi)*60 for r in trace)
        min_forward=min(r['v']-abs(r['w'])*.28 for r in trace)
        if d['min_clearance_m']<=.3 or max_rpm>10.000001 or min_forward<0:
            raise RuntimeError('Claimed success violates envelope')
        leader_low=trace[0]['leader_x']-.27;leader_high=trace[-1]['leader_x']+.27
        leader_clearance=min(math.hypot(max(leader_low-x,x-leader_high,0.),max(abs(y)-.3,0.)) for x,y in run_plans[p.name]['points'])
        if leader_clearance<=0:raise RuntimeError('Leader model intersects recorded static points')
        results.append(dict(file=p.name,reason=d['reason'],noise=d['pose_noise_std'],pose_delay_s=d.get('pose_delay_s',0.),
                            leader_policy=d.get('leader_policy','constant'),
                            seed=d['seed'],duration_s=d['duration_s'],min_clearance_m=d['min_clearance_m'],
                            leader_model_min_static_clearance_m=leader_clearance,
                            max_wheel_rpm=max_rpm,min_forward_wheel_speed_m_s=min_forward))
    chosen_path,chosen=max(files,key=lambda item:item[0].stat().st_mtime_ns)
    plan=run_plans[chosen_path.name];source=logs/plan['source'];shift=plan['box_forward_m']
    prefix='current_scene_left_simulation' if shift==0 else 'left_simulation'
    summary=dict(source_snapshot=plan['source'],source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                 original_snapshot_same_pose_result='SIMULATED_SUCCESS_WITHOUT_BOX_TRANSLATION' if shift==0 else 'NO_CHECKED_PATH_FOUND_AT_ORIGINAL_BOX_LOCATION',
                 condition=dict(initial_body_gap_m=1.,leader_start_prelude_s=chosen['leader_prelude_s'],
                                box_forward_m=shift,box_lateral_change_m=0.,follower_start_unchanged=True,
                                follower_hard_clearance_m=.3,leader_dimensions_assumed_m=[.54,.60]),
                 success_definition=['entire body beyond box far end','rejoin original straight reference',
                                     'remain within .10m and 10deg for at least 1m net progress'],
                 results=results,simulation_only=True,physical_evaluation_pass=False,live_motor_connected=False,
                 limitations=['Recorded static point cloud plus known box rectangle',
                              'Ideal fresh network; pose latency/noise tested separately',
                              'Leader policy varies by run; original policy is constant .04m/s',
                              'Leader dimensions assumed; actual localization and contact not verified',
                              'Simulated duration exceeds current 30/60s hardware trial'])
    (logs/(prefix+'_summary.json')).write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    scale=140.;left=-1.3;top=2.;width=830;height=480
    def xy(x,y):return ((x-left)*scale+35,(top-y)*scale+20)
    def line(points,color,dash=''):
        text=' '.join(f'{xy(x,y)[0]:.1f},{xy(x,y)[1]:.1f}' for x,y in points)
        return f'<polyline points="{text}" fill="none" stroke="{color}" stroke-width="2" stroke-dasharray="{dash}"/>'
    drawing=[]
    for x,y in plan['points']:
        sx,sy=xy(x,y)
        if 0<sx<width and 0<sy<height:drawing.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="1.4" fill="#777"/>')
    drawing.append(line([(-.5,0),(4.2,0)],'#444','6 5'))
    drawing.append(line([(p[0],p[1]) for p in plan['path']],'#1671db'))
    drawing.append(line([(r['x'],r['y']) for r in chosen['trace'][::8]],'#e24c25'))
    trace=chosen['trace']
    selected=[0,max(range(len(trace)),key=lambda i:trace[i]['y']),len(trace)-1]
    for index in selected:
        row=trace[index];co,si=math.cos(row['yaw']),math.sin(row['yaw']);corners=[]
        for bx,by in ((-.27,-.3),(.27,-.3),(.27,.3),(-.27,.3),(-.27,-.3)):
            corners.append((row['x']+co*bx-si*by,row['y']+si*bx+co*by))
        drawing.append(line(corners,'#148348'))
    near=plan.get('box_near_x_m',1.0502417809897473);far=plan.get('box_model_far_x_m',1.2512517062222593)
    low=plan.get('box_model_low_y_m',.1335417078129606);high=plan.get('box_model_high_y_m',.359110280756642)
    drawing.append(line([(near,low),(far,low),(far,high),(near,high),(near,low)],'#b36b00'))
    if shift:
        drawing.append(line([(near-shift,low),(far-shift,low),(far-shift,high),(near-shift,high),(near-shift,low)],'#b36b00','3 3'))
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img" aria-label="실제 스캔 기반 가상 좌회피 경로와 폐루프 궤적">'+''.join(drawing)+'</svg>'
    (logs/(prefix+'_paths.svg')).write_text(svg)
    table=''.join(f"<tr><td>{r['seed']}</td><td>{html.escape(r['leader_policy'])}</td><td>{r['noise']}</td><td>{r['pose_delay_s']}</td><td>{r['duration_s']:.2f}</td><td>{100*r['min_clearance_m']:.2f}</td><td>{r['max_wheel_rpm']:.2f}</td></tr>" for r in results)
    note='현재 기록의 상자와 모든 점 위치를 유지한 가상 성공. 실물 검증 아님.' if shift==0 else '상자를 진행방향으로15cm 옮긴 가상 조건. 원래 위치의 성공은 아님.'
    condition=f"초기 차체 사이1m, 선두 시작 선행{chosen['leader_prelude_s']}초, 상자 전방 이동{shift}m."
    page='''<!doctype html><meta charset="utf-8"><title>좌회피 가상 검증</title><style>body{max-width:980px;margin:30px auto;font:16px sans-serif;line-height:1.6}svg{width:100%;border:1px solid #ccc}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ccc}strong{color:#a33}</style>
<h1>기록 스캔 기반 좌회피 가상 검증</h1><p><strong>'''+html.escape(note)+'''</strong></p><p>'''+html.escape(condition)+'''</p>
<p>회색: 기록 점 · 파랑: 계획 경로 · 주황: 실제 적분 궤적 · 초록: 차체 · 황갈색 점선: 원래 상자, 실선: 가상 변경 상자</p>'''+svg+'''<table><tr><th>seed</th><th>선두 정책</th><th>위치/각도 잡음 std(m/rad)</th><th>pose 지연(s)</th><th>가상 시간(s)</th><th>최소 후발 여유(cm)</th><th>최대 바퀴 RPM</th></tr>'''+table+'''</table><p>성공: 차체 전체 통과 → 기준 직선 재합류 → 1m 추가 진행. 후발 30cm 보호 및 바퀴 전진/10RPM 한계 검사.</p>
<p>한계: 가상 소요 시간은 위 표와 같다. 현재 하드웨어 시험 제한보다 길다. 무선 지연·실물 위치 오차·접촉 여부는 미검증.</p>'''
    (logs/(prefix+'_report.html')).write_text(page)
    print(json.dumps(dict(successful_runs=len(results),min_clearance_m=min(r['min_clearance_m'] for r in results),
                          report=str(logs/(prefix+'_report.html'))),ensure_ascii=False))


if __name__=='__main__':main()
