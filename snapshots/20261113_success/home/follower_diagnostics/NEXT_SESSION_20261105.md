# 내일 재개: 후발 회피 및 선두 경로 복귀

사용자 요청: 오늘 물리 시험 종료. 코드/로그 분석만 수행. 내일 평가3 우선 재개.
실물로 복귀하지 않은 이전 성공 로그는 성공 근거로 사용하지 않는다.

## 확정된 보정과 현재 상태

- 선두 실행: leader_nav_yaw_calibrated.launch.py. 직진 feedback_speed_scale1.173,
  회전 feedback_yaw_scale1.18. 바퀴 반경.085/트랙.560 및 명령 환산 유지.
  사용자 선두1m 실측 일치. 회전은 반복 IMU/눈대중 기반 임시값; 양방향 정밀 외부각 검증은 미완료.
- 후발 follower_odometry_calibration.json: raw_linear_feedback_scale1.1474.
  RPM raw 직진 피드백만 수정. 위치 카운터 기반 EKF와 yaw 및 명령 환산 유지.
  사용자 보정 후 실제1m 일치. 최신 로그 follower_distance_trial_1793787616612669186.json:
  raw1.00308m, filtered1.01094m. 후발 yaw는 filtered/IMU가 선택되어 있으며 기하 트랙값 변경 없음.
- 마지막 실행 드라이버는 거리시험 전용일 수 있음. --ack-distance-test-ignore-clearance
  모드를 회피 시험에 재사용하지 말 것. 회피 도구는 distance_test_only=true 드라이버를 거부함.
- 선두 및 후발 거리시험에서의 근접 검사 제외는 해당 진단 전용. 일반 회피 .30m 하드 보호 유지.
- 임의 필터/스캔 삭제/위조된 여유 값 없음. 모든 실물 평가 통과 미확정.

## 원인 분석 (확정 사실 / 미확정 원인 분리)

1. 좌표계와 완료 판정 불일치 (확정)
   local_avoidance_trial_1793783686459932449: frozen 기준선+filtered 적분 위치만으로
   GEOMETRIC_PASS_REJOIN_PLUS_1M. 실제 사용자 미복귀. 이후 정지상태 audit:
   선두 경로 거리.4957m, projected-local vs map 위치.5549m, 방향11.07deg 차이.
   당시 map 연속 궤적이 없어 차이 발생 시점/AMCL vs 오뎀 책임은 확정 불가.
   현재 --map-rejoin은 동일 측정시각의 filtered/map으로 목표만 변환한다.
   scan/오뎀 자체를 억지로 맞추지 않는다. 완료는 실제 map 경로 근접+1m 순진행 검증.

2. 선두 경로의 반대 방향 이력 (새로 확인, 우선 처리)
   logs/avoidance_handoff_analysis_20261104.json.
   직진 시험1793784599680490643에 기준선 반대21.2cm prefix,
   1793784890865692866에42.4/33.7cm prefix,
   1793785278116234041에46.6/13.1cm prefix. 단발 샘플이 아니라 반복 수신.
   map_rejoin_guidance.py와 live_rejoin_verification.py는 최근접 선분을
   방향/세션/진행도 없이 고른다. 실제 path_heading_error 약137~145deg 출력도 존재.
   원인 후보: 지난 이동 이력, AMCL 보정, 재배치. 어느 것이 주원인인지는 미확정.
   현재 직진 실험에서 역방향 prefix를 정상 복귀 방향으로 사용하면 안 된다.
   곡선/회전 경로까지 단순 고정 기준선으로 필터하면 안 됨. 새 세션 경로,
   경로 인덱스/호길이 진행 추적, 국소 접선 연속성 및 목표의 차체 전방 조건이 필요.

3. 회피와 Pure Pursuit 연결 미완성 (코드 확인)
   follower_pursuit_preview.py: 가까운 점 이후 lookahead.8m의 첫 점 선택,
   x<=0이면STOP_TARGET_BEHIND, 목표각45deg 초과STOP_ALIGN_REQUIRED.
   monotonic/session progress 추적 미구현이라고 자체 주석에 명시되어 있음.
   GENTLE_PIVOT_* 실제 회피는10cm stop/turn/go이며 Pure Pursuit 곡률 연속제어가 아님.
   operator는 회피 뒤 완료 시 종료한다. 일반 FOLLOW/PP로 원활한 인계가 없음.
   평가3의 정상 추종→장애물 회피→재합류→추종 지속을 아직 입증하지 못함.

4. 온라인 통과 여유와 초기 계획 불일치 (확정, 원인 일부 미확정)
   scan이 바뀌면 최초45deg 경로의 정면전환이34cm 계획기준 미달.
   1793784599680490643 정지장면: 바로 직진 최소.3197m,
   동일 대각선10cm 추가 후 직진 약.38m. 최대2회 검사된 대각선 연장 구현.
   1793785278116234041 여유.3403m, 기존 목표 잔여 구간이 막혀STOP_APF_STEP_BLOCKED.
   막힌 원인을 벽/상자/일시점/좌표 투영 변화로 분류할 증거가 부족할 수 있음.
   모든 점 제거 또는 보호거리 완화로 해결하지 말 것. 선택 실패의 실제 검사 pose/point 기록 필요.

5. 시각 정렬/드라이버 handshake (보완되었으나 실물 반복 검증 미완료)
   map→filtered 보간 실패가 공용함수 RAW 오류명으로 표시됨. 명확한 오류로 변경.
   미래 map stamp만 양쪽 zero 최대.2초 대기; 실제 측정 support 도착하면 재개,
   과거 gap/지속 실패는 종료. source TTL.8, motor deadlines.3 그대로.
   방향전환 zero-status race: driver가 실측 zero3회 전까지 zero만 유지,
   invalid epoch/값은 여전히 fault. 부분 UART reply시 zero-only 단발재시도도 유지.

## 내일 순서 (재시험만 반복하지 않기)

1. 현재 보정값 유지, 물리 재배치/양쪽 pose 실제일치 확인 후 leader bridge를 새 세션으로 시작.
2. 정지/가상 전진에서 live path 전체점+국소접선+현재선분인덱스 기록.
   새 세션과 역방향 prefix 처리, 목표의 전방성/진행도 보존부터 오프라인 테스트.
3. 회피 없이 map 기반 Pure Pursuit의 짧은 직선 추종 확인. 기존 lookahead와
   작은 시험공간/리더간격 관계 점검; 무작정 gain 튜닝 금지.
4. 회피→map 재합류→PP로 인계하는 명시적 상태 전이 구현/시뮬 검증.
   목표는 앞쪽 경로 호길이로 선택, 세션/방향 연속성 보장. 정지 확인 후 인계.
5. 보정 후 새 scan/pose 기록으로 회피/복귀 시뮬. 과거 미보정 raw 프레임 로그를
   그대로 현재 정확도 증거로 사용하지 않기. 물리 시험은 그 뒤에만.
6. RViz 경로 오차와 실물 통과를 동시에 확인, 재합류후1m 및 추종 지속 기록.
   마지막에 선두 Nav2 자율주행과 연결. 정량평가3 통과는 별도로 입증.

회피용 정상 후발 driver (거리시험 종료 후 재시작 필요):
python3 -B ~/follower_diagnostics/follower_drive_node.py --run-hardware --ack-hardware-zero-rpm-tx --input-kind local_avoidance --ack-bounded-pivot-candidate --enable-motion-input --trial-max-rpm 10 --trial-seconds 360
센서/EKF/AMCL/Bridge는 기존 순서 유지. 거리시험 전용 플래그 넣지 않기.
