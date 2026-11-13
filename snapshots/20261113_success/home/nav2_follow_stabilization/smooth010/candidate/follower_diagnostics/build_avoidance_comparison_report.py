#!/usr/bin/env python3
import hashlib,json,html,math
from pathlib import Path
ROOT=Path(__file__).resolve().parent;LOG=ROOT/'logs'

def main():
    comparison=json.loads((LOG/'avoidance_strategy_comparison_v2.json').read_text())
    development=json.loads((LOG/'stop_turn_go_robustness.json').read_text())
    first=json.loads((LOG/'avoidance_candidate_verification.json').read_text())
    final=json.loads((LOG/'avoidance_candidate_verification_wider.json').read_text())
    results=final['results'];count=len(comparison['results'])+len(development['selection_trials'])+len(development['robustness_trials'])+len(first['results'])+len(results)
    summary=dict(development_closed_loop_runs=count,final_successes=sum(r['passed'] for r in results),final_runs=len(results),
                 min_clearance_m=min(r['min_true_clearance_m'] for r in results),
                 duration_s=[min(r['duration_s'] for r in results),max(r['duration_s'] for r in results)],
                 max_command_rpm=max(r['max_abs_wheel_rpm'] for r in results),
                 max_leader_travel_m=max(r['leader_travel_m'] for r in results),
                 min_center_gap_m=min(r['min_longitudinal_center_gap_m'] for r in results),
                 config=final['config'],simulation_only=True,evaluation_pass=False)
    archive=LOG/'avoidance_verified_sources';archive.mkdir(exist_ok=True)
    summary['source_files']={}
    for name in ('compare_avoidance_strategies.py','verify_avoidance_candidate.py'):
        raw=(ROOT/name).read_bytes();(archive/name).write_bytes(raw)
        summary['source_files'][name]=hashlib.sha256(raw).hexdigest()
    (LOG/'avoidance_strategy_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    selected=[]
    for method in ('fixed_curve_pp','apf_straight_rejoin'):
        r=next(r for r in comparison['results'] if r['method']==method and r['case']=='quantized')
        selected.append(dict(name=method,**r))
    for case in range(3):
        r=min((r for r in results if r['case']==case),key=lambda r:r['min_true_clearance_m'])
        selected.append(dict(name='추천 후보 / 오차 조건 '+str(case),**r))
    payload=dict(points=final['points'],box=final['box_bounds'],runs=selected,summary=summary)
    encoded=json.dumps(payload,ensure_ascii=False).replace('</','<\\/')
    document='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>후발 회피 방식 비교</title>
<style>body{font:16px sans-serif;max-width:1050px;margin:25px auto;color:#17202a}canvas{border:1px solid #ccd;background:#fff;width:100%}select,input{font:inherit}pre{white-space:pre-wrap}small{color:#566}</style>
<h1>후발 장애물 회피 방식 비교 — 가상 시험</h1><p>실제 기록점 유지 + 최근 경로 차단점 + 상자 후면 모델. 실물 합격 기록이 아닙니다.</p>
<select id="run"></select> <button id="play">재생</button><input id="step" type="range" min="0" value="0" style="width:45%">
<canvas id="map" width="1000" height="600"></canvas><pre id="status"></pre>
<p>회색: 관측점 / 빨강: 상자 모델 / 파랑: 후발 / 초록: 선두(치수 가정). 선두는 후발 진행에 맞춰 정지·저속 운용합니다.</p>
<small>미관측 공간·광학 오검출·실제 마찰·완전한 ROS 통신은 검증하지 않았습니다. 현재 드라이버는 제자리 회전을 허용하지 않으므로 이 가상 제어는 실물에 연결돼 있지 않습니다.</small>
<script>const D=__DATA__;const select=document.querySelector('#run'),slider=document.querySelector('#step'),canvas=document.querySelector('#map'),ctx=canvas.getContext('2d');
D.runs.forEach((r,i)=>{let o=document.createElement('option');o.value=i;o.textContent=r.name+' / '+r.reason;select.append(o)});
const sx=x=>50+(x+.7)*165,sy=y=>540-(y+1.25)*165;
function robot(x,y,a,color){ctx.beginPath();[[-.27,-.3],[.27,-.3],[.27,.3],[-.27,.3]].forEach(([u,v],i)=>{let X=sx(x+Math.cos(a)*u-Math.sin(a)*v),Y=sy(y+Math.sin(a)*u+Math.cos(a)*v);i?ctx.lineTo(X,Y):ctx.moveTo(X,Y)});ctx.closePath();ctx.fillStyle=color+'44';ctx.fill();ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();ctx.beginPath();ctx.moveTo(sx(x),sy(y));ctx.lineTo(sx(x+.35*Math.cos(a)),sy(y+.35*Math.sin(a)));ctx.stroke()}
function draw(){let r=D.runs[+select.value],trace=r.trace;slider.max=Math.max(0,trace.length-1);let n=Math.min(+slider.value,trace.length-1),p=trace[n];ctx.clearRect(0,0,1000,600);
ctx.lineWidth=1;ctx.strokeStyle='#dde';ctx.fillStyle='#667';ctx.font='12px sans-serif';for(let x=-.5;x<=4.5;x+=.5){ctx.beginPath();ctx.moveTo(sx(x),20);ctx.lineTo(sx(x),560);ctx.stroke();ctx.fillText(x.toFixed(1),sx(x),580)}for(let y=-1;y<=1.5;y+=.5){ctx.beginPath();ctx.moveTo(25,sy(y));ctx.lineTo(975,sy(y));ctx.stroke();ctx.fillText(y.toFixed(1),2,sy(y))}
ctx.fillStyle='#888';D.points.forEach(([x,y])=>{if(x>-.7&&x<4.9&&y>-1.25&&y<1.8)ctx.fillRect(sx(x)-1,sy(y)-1,2,2)});
let b=D.box;ctx.fillStyle='#dd442255';ctx.fillRect(sx(b.near),sy(b.high),(b.far-b.near)*165,(b.high-b.low)*165);
ctx.setLineDash([5,5]);ctx.strokeStyle='#aa7788';ctx.beginPath();ctx.moveTo(sx(0),sy(0));ctx.lineTo(sx(4.5),sy(0));ctx.stroke();ctx.setLineDash([]);
ctx.strokeStyle='#1769aa';ctx.lineWidth=2;ctx.beginPath();trace.slice(0,n+1).forEach((p,i)=>i?ctx.lineTo(sx(p.x),sy(p.y)):ctx.moveTo(sx(p.x),sy(p.y)));ctx.stroke();if(p){robot(p.x,p.y,p.yaw,'#1769aa');if(p.leader_x!==undefined)robot(p.leader_x,0,0,'#198754');}
document.querySelector('#status').textContent=JSON.stringify({method:r.name,t:p?.t,phase:p?.phase,body_clearance_m:p?.clearance,full_result:r.reason,total_s:r.duration_s,min_clearance_m:r.min_true_clearance_m,simulation_only:true,evaluation_pass:false},null,2)}
select.onchange=()=>{slider.value=0;draw()};slider.oninput=draw;let playing=false;document.querySelector('#play').onclick=()=>{playing=!playing;document.querySelector('#play').textContent=playing?'일시정지':'재생'};setInterval(()=>{if(playing){slider.value=Math.min(+slider.max,+slider.value+2);draw();if(+slider.value>=+slider.max)playing=false}},50);draw();</script></html>'''
    (LOG/'avoidance_strategy_comparison.html').write_text(document.replace('__DATA__',encoded))
    lines=['# 후발 회피 방식 비교와 추천','',
      '**추천: 기존 Pure Pursuit 추종을 유지하되, 장애물 검출 시 로컬 odom 기준 정지·회전·직진 상태기계로 전환.**','',
      '이번에 확정한 것은 가상 회피 가능성이다. 실제 회피·재합류 합격 기록은 아니다. 실물 모터/ROS 제어 설정은 변경하지 않았다.','',
      '## 실제 출발 실패 원인','',
      '최근 시험 joint_follow_trial_1793709530898986785.jsonl은 STOP_LIVE_PATH_BLOCKED 626건, STOP_SCAN_OR_SOURCE_STALE 10건, STOP_FOLLOWER_STALE 1건이었다. 선두가 이동해도 저장된 곡선의 회전 차체 여유가 현재 관측점에서 검사 기준을 만족하지 않아 후발 arm 조건이 성립하지 않았다. 초기 위치 오차나 세션 종료 실패와는 별개의 실제 경로 여유 문제다.','',
      '## 비교','',
      '| 방식 | 가상 결과 | 해석 |','|---|---|---|',
      '| 기존 고정 곡선 + PP | 0/5 완료 | 현재 차단점을 넣으면 감속·정체 |',
      '| 현재 관측점으로 곡선 재계획 + PP | 여유 설정 3종 모두 경로 생성 거부 | 전진 전용 곡선으로 접근 여유 확보 어려움 |',
      '| 현재 APF + 직선 재합류 후보 | 0/5 완료 | 근접 감속과 정수 RPM에서 정체 |',
      '| 정지·회전·직진 분리, 최종 후보 | 추가 15/15 완료 | 장애물 전체 통과 + 원래 선 재합류 + 1m |','',
      f'총 {count}회 폐루프 개발·검증을 기록했다. 개발 과정 실패도 원본 JSON에 보존했다. 초기에 고속 후보 8건에서 RPM 제한 실패가 나와 바퀴별 속도 제한을 보완했다. 최종 후보만의 결과와 개발 중 실패를 구분했다. Nav2 전체 스택/Gazebo를 실행한 비교는 아니다.','',
      '## 최종 후보','',
      '초기 위치에서 원래 선 방향으로 약16cm 전진한 뒤 정지 → 왼쪽90도 회전 → 기준선 왼쪽82cm 지점까지 직진 → 정지/오른쪽90도 → 상자 후면보다85cm 더 앞까지 직진 → 정지/오른쪽90도 → 원래 선으로 복귀 → 정지/왼쪽90도 → 1m 이상 추종. 이 값은 현재 장면의 결과이며 다른 배치에 그대로 쓰는 고정 타이머가 아니다. 실제 전환은 스캔·회전각·바퀴 이동량 피드백으로 판정해야 한다.','',
      f"- 최종 15조건 모두 성공. 전체 {summary['duration_s'][0]:.1f}–{summary['duration_s'][1]:.1f}초, 바퀴 명령 최대 {summary['max_command_rpm']:.0f}RPM.",
      f"- 오차를 보수적으로 반영한 최저 차체 여유 {summary['min_clearance_m']*100:.1f}cm. 기존 30cm 정지/60cm 감속 유지.",
      '- 위치 잡음 3–5mm, 자세 잡음 같은 수치(rad), 위치 지연0.1–0.2초, 좌우 이동량 차이±3%, 장애물 외곽 불확실성0/1/2cm, 오도메트리 거리 척도±1%, yaw 편향±0.5도, 정수RPM 반올림, 0.12초 모터 응답 모델을 적용했다. 이 모델 파라미터는 실제 시스템을 완전히 식별한 값이 아니다.',
      '- 선두는 후발 전방 진행에 맞춰0–0.04m/s로 조절. 선두 정지 중 회피를 허용하려면 일반 추종과 구분된 협조 회피 세션이 필요하다. 일반적인 선두 정지 신호를 무시하는 방식이 아니다.',
      f"- 초기 차체 간격1m. 회피 중 최소 종방향 중심 간격 {summary['min_center_gap_m']:.2f}m이므로 차체 간격1m를 계속 유지한 시험은 아니다.",
      f"- 선두 이동 최대 {summary['max_leader_travel_m']:.2f}m. 초기 선두 위치와 앞끝까지 포함하면 약4.6m 길이의 공간이 필요하다. 현재 공간 폭은 약2.5m 조건이다.",
      '- 원본 관측점447개와 최근 차단점35개를 모두 보존했다. 관측점 제거/상자 이동 없이, 상자 깊이20cm와 좌표 변화를 포괄하는 외곽을 추가했다. 미관측 영역은 자유 공간이라고 입증한 것이 아니다.','',
      '## 실물 연결에 필요한 작업','',
      '1. 현재 드라이버/Gateway는 전진 바퀴만 허용해 제자리 회전을 거부한다. 회전 상태에서만 허용되는 별도 제한 모드가 필요하며 RPM/응답/입력 만료/비상정지 보호는 유지한다.',
      '2. 회피 중 좌표를 odom에 고정하고 장착 TF만 사용해 map→base 시각 대기를 제어 루프에서 제거한다. PP/지도 위치는 재합류 판정에 사용한다. AMCL 보정으로 회피 중 로컬 경로가 갑자기 움직이지 않게 한다.',
      '3. 선두와 후발에 공동 회피 세션을 부여한다. 선두는 후발 진행에 맞춰 정지·저속, 후발은 명시적으로 허용된 회피 단계에서만 움직인다. 연결 만료/사람 접근/새 장애물은 정지한다.',
      '4. 현재 실물 드라이버 최대60초이므로, 수분짜리 전체 시험에는 새 전체 시간 상한과 단계별 watchdog 검증이 필요하다. 현재 제한을 삭제하거나 무제한 운용으로 변경하지 않았다.',
      '5. 초기 회전·직진을 실제 바퀴/IMU로 검증한 다음 전체 회피로 진행한다. 이번에는 실물 실행 명령을 만들거나 실행하지 않았다.','',
      '## 평가 범위','',
      '이번 선택은 현재20×20cm 바닥면 시험물과 저속 조건의 평가3 동작 구현을 위한 것이다. 보고서의 APF 알고리즘/속도0.30m/s/장애물0.2m²/간격 조건을 충족했다고 표시하지 않는다. 다른 알고리즘을 채택하면 방법과 실제 시험 조건을 보고서에도 반영해야 한다.','',
      'Nav2의 Humble RPP도 회전 정렬과 충돌 예측 기능을 별도로 제공한다. 이 문서는 회전·직진 분리 개념의 참고일 뿐, 이번 자체 상태기계가 Nav2 검증을 받았다는 뜻은 아니다: https://api.nav2.org/nav2-humble/html/md_nav2_regulated_pure_pursuit_controller_README.html','',
      '## 재현과 산출물','',
      '최종 검증: `python3 -B ~/follower_diagnostics/verify_avoidance_candidate.py --return-buffer 0.85 --output ~/follower_diagnostics/logs/avoidance_candidate_verification_wider.json` (가상 계산만 수행).',
      '추천 궤적 재생: logs/avoidance_strategy_comparison.html. 요약: logs/avoidance_strategy_summary.json. 최종 원본: logs/avoidance_candidate_verification_wider.json. 개발 실패 원본: logs/stop_turn_go_robustness.json. 비교 원본: logs/avoidance_strategy_comparison_v2.json.',
      '시뮬레이터 검사7개 통과: 회전 차체 여유, 제자리회전 운동학, 바퀴10RPM, 초기 충돌 차단, 차체통과·재합류후1m 판정. 최종 제어 소스와 SHA256은 logs/avoidance_verified_sources 및 요약 JSON에 보존.']
    (ROOT/'AVOIDANCE_STRATEGY_RECOMMENDATION_20261103.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
