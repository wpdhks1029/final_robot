# 다음 단계: 시간 비용 경로/선두 협조 감속/실제 입력 관찰

## 개선한 모의 결과

동일 기록과 초기1m 차체 간격, 선두15cm 선행 후 상자 전방15cm 투입 조건을 유지했다.
상자는 좌우로 이동시키지 않았다. 원래 상자 위치 그대로의 경로 문제를 해결했다고 주장하지 않는다.

기존 최단거리 경로/후발 기준 .04m/s/선두 고정 .04m/s:
391.6초, 최소 후발 여유 .3336818m, 선두15.814m 이동.

새 시간 비용 경로/후발 기준 .06m/s(바퀴10RPM으로 항상 제한)/선두 협조 감속:
seed7/19/31 모두 성공,250.45~250.70초, 최소 여유 .3466359m, 선두3.526~3.527m 이동.
잡음3mm/3mrad와 pose지연.10초를 동일하게 적용했다.
최대 선두·후발 중심 종방향 간격은 약2.556m로, 회피 중1m 간격을 고정 유지한 결과가 아니다.

협조 정책은 가상에서 선두 속도 .012~.04m/s를 사용한다. 이는 기존 추종기의 선두 정지 판정을 우회하지 않기 위한 개발 후보다.
정지/비활성 입력은 양수 최소속도보다 우선하도록 별도 검사한다. 실제 전송·가감속·무선 타이밍은 검증하지 않았다.
30cm 하드 정지/60cm 감속/회전 차체 검사/10RPM/전진 바퀴 조건은 변경하지 않았다.

## 실제 입력 관찰

새 임시 경로 제어는 아직 모터에 연결하지 않았다. 기존 APF launch의 제어와 구분한다.
watch_temporary_detour_shadow.py는 실제 scan/TF/leader/path/follower만 구독한다. ROS publisher/client/service를 만들지 않는다.
source/rx .3초를 유지하고, 원격 입력50ms 여유는 보수적으로 차감한다. 계산 중 입력이 늦어져도 STOP으로 표시한다.
전체 경로를 현재 점군으로 검사하고 가상 Pure Pursuit/기존 감속/선두 협조 속도를 출력만 한다.
정지 상태에서 계산을 보기 위한 --virtual-forward-probe는 모터 동작이나 arm 허가가 아니다.

```bash
python3 -B ~/follower_diagnostics/watch_temporary_detour_shadow.py --seconds 20 --virtual-forward-probe
```

현재 실제 상자가 가상 계획 조건과 다르면 STOP_LIVE_PATH_BLOCKED가 예상된다. 입력이 늦으면 STOP_*_STALE.
이 도구는 새 경로의 실물 주행을 시작하지 않는다. 현재 경로에 대해 물체를 임의 삭제하거나 실물 배치를 자동 변경하지 않는다.

## 재현

```bash
python3 -B ~/follower_diagnostics/optimize_recorded_left_detour.py
python3 -B ~/follower_diagnostics/replay_recorded_detour.py --plan ~/follower_diagnostics/logs/recorded_left_detour_time_plan.json --lookahead 0.15 --noise 0.003 --pose-delay 0.1 --seed 7 --follower-speed 0.06 --leader-policy coordinated
```

결과 비교: logs/left_simulation_optimization_summary.json.
관련 단위 검사105개 통과. 250초는 현재 하드웨어30/60초 시험 제한보다 길다. 제한은 변경하지 않았다.
전체 실물 연결 전에 단계별 감독, 새 경로 추종과 기존 선두/간격 보호 결합, 실제 입력 만료 문제를 검증해야 한다.
