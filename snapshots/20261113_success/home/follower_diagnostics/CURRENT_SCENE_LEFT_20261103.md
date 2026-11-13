# 현재 실측 장면의 좌회피 후보

원본: logs/temporary_detour_check_1793705648765334593.json.
이 문서의 결과는 이전179370317... 스캔의 상자 전방15cm 변경 실험과 분리한다.

## 변경/원인

현재 상자는 경로 기준 x=.9748648m, 중심 y=-.0344780m, 관측 폭.2227859m.
첫 좌회전 구간 대표 후보의 원시 차체 여유는 .3276245m였으나,5mm 간격 샘플의 이동 상계를 더한 계획 기준에서 거부됐다.
측방 원호 후보를2mm로 검사하고, 전진 조건을 만족하는 반경.286m 후보를 추가했다.
30cm 하드 여유와 추가2cm 계획 여유는 유지한다. 샘플 사이 병진/회전 여유도 각 간격에 맞게 계속 적용한다.
독립적인 중간 자세 보간 검사에서도30cm 이상임을 확인한다. 관측 점이나 상자 위치는 바꾸지 않는다.

## 결과

현재 기록을 그대로 입력해 DETOUR_PLANNED. 좌측 최대 경로 오프셋 약.876915m.
순수 Pure Pursuit 명령 적분/전진 바퀴/10RPM/감속/1초 사각 차체 검사를 적용한 모의:
seed7/19/31 모두 SIMULATED_PASS_REJOIN_PLUS_1M.
189.2~190.2초, 최소 후발 여유 .3273950m, 선두 가상 이동3.262~3.263m.
위치3mm/각도3mrad 잡음과pose지연.1초 포함. 초기 차체 간격1m를 가정한다.
선두는 시작1초간 .04m/s로 먼저 전진한 뒤 후발 진행에 맞춰 .012~.04m/s 협조 감속한다.
상자 전방/좌우 이동은0. 실제 선두 치수는.54×.60m로 가정했으며 물리적으로 검증하지 않았다.
네트워크는 신선한 입력을 가정한다. 현재 .3초 만료 문제를 해결한 결과가 아니다.
실물 제어에는 연결하지 않았다. 190초는 기존30/60초 시험보다 길다.

관련 단위 검사108개 통과. 실제 기록 회귀 검사3개에는 전체점/중간자세/원래상자위치 확인이 포함된다.

## 파일

- logs/current_scene_left_detour_plan.json: 현재 기록의 좌회피 계획.
- logs/current_scene_left_simulation_summary.json: 결과/입력 해시.
- logs/current_scene_left_simulation_report.html: 계획과 적분 궤적.

## 실제 입력 관찰(명령 발행 없음)

```bash
python3 -B ~/follower_diagnostics/watch_temporary_detour_shadow.py --seconds 20 --virtual-forward-probe --plan ~/follower_diagnostics/logs/current_scene_left_detour_plan.json
```

이 명령은 이전 가상 배치의recorded_left_detour_time_plan.json을 사용하지 않는다.
상자/로봇을 옮기지 않고 실제 입력으로 경로 적합성과 시각 유효성을 확인한다.
기존 APF/드라이버/실제 주행 시험 설정은 변경하지 않았다.
