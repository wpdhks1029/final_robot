# 정량목표 1번 — 사용자 확인 성공 기록

2026-11-06 사용자가 선두 Nav2 단독 주행을 정량목표1번 성공 코드로 지정하고 저장·백업을 요청했다. 후발 추종은 포함하지 않는다. planner_server/controller_server/bt_navigator/velocity_smoother 네 노드 active [3] 확인 후 RViz 목표 주행을 진행한 대화 기준이다. 별도 평가 코스 로그·영상·접촉 횟수는 이 장치에 제공되지 않았다.

선두 모터/센서/EKF: ros2 launch ~/leader_diagnostics/leader_nav_yaw_calibrated.launch.py allow_motion:=true
위치추정: ros2 launch nav2_bringup localization_launch.py map:=/home/sunuk/robot_ws/maps/leader_map.yaml params_file:=/home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml use_sim_time:=false autostart:=true
주행: ros2 launch nav2_bringup navigation_launch.py params_file:=/home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml use_sim_time:=false autostart:=true use_composition:=False
공통 환경: source /opt/ros/humble/setup.bash; source ~/robot_ws/install/setup.bash; ROS_DOMAIN_ID=10; ROS_LOCALHOST_ONLY=0
RViz: rviz2 -d ~/convoy_view.rviz; Fixed Frame map; 지도 /map Transient Local; 스캔 /scan Best Effort.
주의: use_composition은 Python 조건식 때문에 대문자 False를 사용한다.

백업 상태: 이 폴더는 성공 실행 기록과 후발 장치에 보관된 선두 관련 참고 사본을 보관한다. 선두 10.253.245.81 접근이 현재 도구 환경에서 차단되어 실제 최신 선두 파일·지도·설치 패키지 원본은 아직 확보하지 못했다. 참고 사본을 최신 성공 원본으로 간주하지 않는다. 선두 원본 백업 명령을 사용자에게 안내했다. 원본 백업 경로/해시 수신 시 이 상태 기록을 갱신한다.
