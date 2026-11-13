> 입력 유효기간 갱신: 센서·선두0.8초, 모터 감시0.3초. 값을 적용하려면 드라이버와 시험 도구를 모두 재시작해야 한다. 드라이버 상태의 input_source_timeout_s=0.8 일치를 시험 도구가 확인한다.

# 로컬 정지·회전·직진 회피 후보

가상 비교 추천과 실물 연결 준비를 구분한다. 실제 모터로 새 회전 모드를 실행하지 않았다. 새 프로필은 명시적으로 선택해야 하며 기존 manual/guard 기본 동작을 변경하지 않는다.

## 구현

- local_avoidance_drive.py: 버전3 JSON 전용 계약과 별도 PivotDrivePort. 입력 토픽 /follower/motor_candidate/local_avoidance_command. 기존 Gateway가 보내는 버전2/guard 토픽은 이 모드에 연결되지 않는다.
- follower_drive_node.py: --input-kind local_avoidance --ack-bounded-pivot-candidate 선택 시에만 새 계약 사용. 10RPM, 30/60/360초 명시적 상한. 360초는 이 모드에만 허용하며 일반 guard는 기존 최대60초 유지.
- local_stop_turn_go.py: 휠 RPM 기반 /follower/odom_raw의 측정 이력으로 스캔 시각 자세를 보간. 끝에서 최대50ms만 정지 자세로 보관할 수 있으며 그 이동 오차 상한을 공간 여유에서 차감. 측정 시각을 현재시각으로 바꾸지 않는다. 회피 중 map→base TF 조회 없음. 장착 TF는 기존 실측 x=.26,y=0,z=.24,yaw=0과 일치해야 함.
- run_local_avoidance_trial.py: 매 실행마다 독립 세션. 한 도구에서 선두 속도·후발 명령·arm/disarm·양쪽 정지 확인. 입력/그래프/상태 검사 후 START를 받아야 출발. 현재 스캔으로 경로 검사 후 시작하고, 장애물이 없는 경우 일반 Pure Pursuit 추종. 주행 중 근접 전방점 5개 이상이 3개 새 스캔에서 지속되면 정지·계획·회피로 전환. 계획 연산은 별도 작업자에서 수행하며 양쪽 정지와 신선한 정지 명령을 유지.

## 유지되는 제한

사용자 요청으로 로컬 회피 모드의 센서·선두 원본/수신 유효기간은0.8초다. 모터 상태·명령 전달·장치 응답·피드백 감시는0.3초를 유지한다. 원격 시계 여유50ms 차감, 30cm 차체 정지/60cm 감속 유지. 관측점을 제외하거나 선두를 마스크하지 않는다. 추가1cm 기하 불확실성 및 자세 보관 오차를 여유에서 차감한다. 현재 raw 자세 자체가100ms보다 늦으면 초과 지연당0.15m/s의 추가 공간 불확실성을 적용한다. 0.8초 이내의 데이터라도 공간 여유가 부족하면 정지한다. 현재 명령/측정 속도 양쪽에 대해1초 회전 차체 예측. DRIVE는 전진만, TURN은 v=0 및 |w|<=.2. 두 경우 모두 명령 바퀴10RPM 이하. TURN에서는 raw M1==M2인 반대 방향 물리 바퀴만 허용.

DRIVE↔TURN 및 회전 방향 변경 전에 유효한 영속도 피드백3개를 필수로 확인. 포트의 독립 전체 기한, 입력 끊김/응답/피드백 기한 유지. 단계별 거리/회전각에 따른 별도 시간 상한,15초 진행 정체 차단. 전체 기한360초가 있더라도 이번 첫 실물 시험은30초로 제한.

## 실행 순서: 첫 30초

1. 현재 후발 Preview/APF/임시 경로 제어 터미널(기존5번)과 Gateway 터미널(기존6번)을 Ctrl+C로 종료한다. 센서·위치 추정·양쪽 Bridge·선두 모터는 유지한다. 선두 Nav2 제어기나 다른 /cmd_vel 발행자가 있으면 새 시험은 시작하지 않는다.
2. 후발 모터 드라이버 터미널(기존1번)을 Ctrl+C 후 다음으로 교체한다.

```bash
python3 -B ~/follower_diagnostics/follower_drive_node.py --run-hardware --ack-hardware-zero-rpm-tx --input-kind local_avoidance --ack-bounded-pivot-candidate --enable-motion-input --trial-max-rpm 10 --trial-seconds 30
```

3. 별도 후발 터미널에서 공통 ROS 환경(Domain10)을 적용한 뒤 다음을 실행한다.

```bash
python3 -B ~/follower_diagnostics/run_local_avoidance_trial.py --run --ack-detour-area-power-cut-ready --seconds 30
```

START+Enter 후 최대29.5초 후발 제어, 종료 시 양쪽 정지 요청. 현재 상자가 있으면 AVOID, 없으면 FOLLOW로 시작한다. 세션 도중 장애물 검출 시 PLAN_BRAKE→PLANNING→AVOID. 새 시험 도구는 매번 종료하므로 예전 preview의 STOP_SESSION_FINISHED 재사용 문제가 없다. 장애물·스캔·기준선 조건이 맞지 않으면 선두 출발 전에 거부한다.

이번 첫30초는 초기 전진/좌회전/정지 확인이다. 전체 회피는 가상 시험에서206~262초였고, 메모리 계약 연계는 약242초였다. 이30초의 trial_success는 제한 시험 완료를 뜻한다. 전체 차체 통과·재합류후1m는 geometric_complete_candidate로 별도 기록하고, 실물 비접촉 확인 없이 evaluation_pass를 true로 하지 않는다.

## 검증

기존 모터/드라이버 회귀를 포함한117개 검사 통과. 최종 계약·로컬 제어·가짜ROS운용·CLI 검사24개 통과. 실제 피드백 갱신 횟수에 맞춰 raw odom을 공급하는 메모리 폐루프도 전체 과정 완료: 가상242초, 명령4836건, 단계 사이 정지 제어52tick, 최종 RPM[0,0]. 피드백 응답/원본시각 만료/기한/잘못된 프로필/잘못된 회전 방향·전진 역전·영속도 미확인·10RPM 초과가 차단되는 것을 검사했다.

실제 회전 방향/회전량, 바닥 마찰, 미관측 공간, 라이다 오검출, 네트워크 장시간 지연, Nav2 지정 코스 연계는 새 프로필로 아직 검증하지 않았다. 모터 피드백 기반 오도메트리의 누적 오차가 외부 실측으로 확인된 것은 아니다. 이번 시험은 선두 직선 운용의 평가3 후보이며, 선두 Nav2 목표 주행과 혼합하는 단계는 별도다.

가상 비교: AVOIDANCE_STRATEGY_RECOMMENDATION_20261103.md. 궤적 재생: logs/avoidance_strategy_comparison.html. 실제 실행 로그는 logs/local_avoidance_trial_*.jsonl. 현재 단계/여유 상태는 /follower/local_avoidance/status에 게시.
