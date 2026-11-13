# 실물 전방 장애물 정지 시험 준비

30초우회전1793547175079223970: 시간종료/양쪽zero/no postfault.
지도거리1.56147->1.81243m, 선두map yaw약-29.30도. 정밀간격유지는미완료.

run_joint_follow_trial.py --expect-obstacle-stop 추가. straight만허용.
제어출력/arm조건/원본시각/.3TTL/장애물hard.3 slow.6/RPM상한불변.
통과는GUARD_STOP_OR_INVALID종료와, 같은guard원본stamp를가진
.3초이내전방STOP_FOOTPRINT_CLEARANCE진단(clearance0<d<=.3, inside는통과아님),
양쪽freshzero와postfault없음이동시에필요.
단순시간종료/상류정지/오래된진단/잘못된frame/옆장애물/정지확인실패는통과아님.
제자리clearance를물리거리참값이나정지기한인증으로사용하지않음.
helper/stateflow검사30개통과. 가짜정방향장애물정지, 원인불명zero, 장애물없는시간종료 포함.
실제모터/물리제동/센서blindarea는이번모의검사로검증되지않음.

실물준비: 두로봇정지/직선정렬/선두약1.5~1.8m앞/선두진행경로확보.
사람대신가벼운빈상자를후발차체앞면에서약.7m앞에고정배치.
선두바퀴/차체/진행경로와상자접촉없음확인. 라이다높이.24m에서검출가능한크기.
주행중물체를손으로넣지않음. 전원차단준비. rosbag은사용안함.
드라이버10RPM30초와일치시켜 --trial-seconds30 --leader-speed.04 --leader-turnstraight
--expect-obstacle-stop --trace-timing 사용. 실제적용/주행은사용자가수행.
