# 선두 실제 치수 확인 및 회전 환산 후보

사용자 실측: 외경.170m, 구동바퀴 중심간격.560m.
현재 설정: radius.085m(일치), track.466m(불일치), feedback_speed_scale1.18.
같은 RPM차에서 기존w는 track실측대비 .560/.466=1.2017배 커짐.
직진보정배율까지포함한 기존회전계수 대비, track.560/yaw_scale1.0 후보 계수는 .466/(.560*1.18)=.7052058.
이는 물리치수와 직진/회전분리 가정에 따른 후보이며 스캔정합결과에 맞춘 최적화가 아님.

준비파일:
- leader_md200t_polling_split_scale_candidate.py: 부모코드불변, odom에서만 직진/회전배율분리, 예외시공유속성복구.
- leader_controller_geometry_candidate.yaml: 기존paramdump 유지, track.560, feedback_yaw_scale1.0, yawvariancefloor.0009만명시.
- 실제선두에아직미적용. launch의명시파라미터덮어쓰기여부확인필요.

주의:
- 기하 track 수정은 명령Twist->RPM에도 영향을줄수있음. /cmd_vel .04/.03은 반올림때문에 기존/후보 모두약4,5RPM이지만 일반값에서는 달라질수있음.
- yaw분산하한 .0009는 실측통계보정값아님.
- 사용자대략35도와 점군추정24.6도 불일치. 후보결과만으로 실제각도를확정하지않음.
- 추종/정지성공은 지도정합 성공을 뜻하지않음.

## 실행 후보 묶음
leader_nav_geometry_candidate.launch.py는 원래실행파일을덮어쓰지않는별도후보.
기존 leader_sensors_check.launch.py를 그대로include,
EKF는별도 leader_ekf_wheel_yaw_candidate.yaml,
컨트롤러는별도 leader_md200t_polling_split_scale_candidate.py 사용.
allow_motion기본false, 포트/baud57600/ID1/max25RPM/cmdTTL.5/폴링10Hz 유지.
명시값: radius.085,track.560,linearfeedbackscale1.18,yawfeedbackscale1.0,yawvariancefloor.0009.
모의/설정 검사11개 및py_compile통과.
기존기하실행으로되돌리려면 후보기본launch를정상종료하고 leader_nav_015_base.launch.py를재실행.
두기본launch동시실행금지. EKF초기화후선두initialpose와bridge경로초기화필요.

기록의동일원본Twist입력 재환산 격리EKF비교(실제명령RPM변화는재현안함):
logs/leader_ekf_comparison_1793543243199085539
기존바퀴37.5777도 -> 후보26.1631도.
실물미적용/미검증. 후발serial오류도아직근본해결확정아님.
