# 가벼운 기록에서 확인한 곡선 정지

원본 bags/wheel_yaw_light_20261101_224642, sqlite quick_check ok.
시험 logs/joint_follow_trial_1793540890124986534.jsonl.
scene_failure1793540900492838007: STOP_LEADER_ROTATING.
직전 EKF 원본stamp1793540900434752069: vx=.0472652548 m/s, wz=.0676183274 rad/s.
즉 관측값 기준 곡선반경 약.699m. lv<.05 및 abs(lw)>.05 조건에 의해 제자리 회전으로 분류.
원본바퀴wz는 .0225394666 rad/s 단위 계단으로 변동하며 +/-값까지 관측.
원본 /odom twist covariance diagonal: vx0, vy.0004, wz0.
해당 EKF 출력 wz variance 약9.99999e-10, 측정 각속도 거의 그대로 반영.
바퀴 각속도 후보 검토 당시 0 측정공분산을 놓쳤음. 오프라인 총회전량 일치만으로 순간각속도품질 검증 안됨.

후발 RESPONSE_TIMEOUT은 정지요청 약.388초 뒤 상태에 기록됨.
waiting.318s, tick gap.020s, last feedback.400s, softwarebuffer19 bytes.
커널USB오류필터 결과없음, portowner1개(사용자출력). 물리통신정상확정은 아님.
회전정지판정/선두측정공분산 문제와 후발응답문제를 분리. 아직 보호기한/판정/모터코드 동작은 바꾸지 않음.

## 수정 준비 및 검증
- 선두 원본 leader_md200t_polling_current.py 보존.
- leader_md200t_polling_covariance_candidate.py: YawRateCovariancePublisher 추가,
  wheel_yaw_rate_variance 기본 .0009 (sigma .03rad/s, 미보정 초기 비교값).
  covariance[35]에 하한만 적용. 큰 기존 분산은 낮추지 않음.
  속도·stamp·포트·패킷·feedback gate·반환 프로토콜 불변.
- 후발 pursuit: 제자리회전 조건을 abs(w)>.05 및 abs(v)<=기존 정지속도 .01로 변경.
  원래 abs(v)<.05가 실제전진 .047과곡선 .0676을 잘못 분류했음.
  실제회전/역주행/정지/최소거리/큰heading/기한 조건 유지.
- 모의검사53개 통과. 실행 중 노드에는 재시작 전 반영되지 않음.
- no_deadband30초 기록의 격리 재생, yaw분산하한 .0009 적용:
  IMU기존15.2613도, 바퀴후보37.1000도. 사용자대략실측35도.
  logs/leader_ekf_comparison_1793541414121934798.
  실제 yaw추정 정확성·공분산통계적타당성·주행안전성 검증은 아님.
- 후발응답timeout은 해결 미완료. 원본timeout모터동작은 변경하지 않음.
