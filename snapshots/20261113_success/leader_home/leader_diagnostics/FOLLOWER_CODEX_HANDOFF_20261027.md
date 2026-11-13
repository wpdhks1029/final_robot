# 후발 Raspberry Pi Codex 인수인계 — 2026-10-27

## 우선 수행할 작업

이 문서를 끝까지 읽고, 후발 실제 소스/Git/설치/센서/모터/ROS 토픽을 읽기 전용으로 조사한다.
현재 사용자는 선두 Jetson에서 후발 Raspberry Pi 작업으로 전환 중이다.
첫 단계에서 파일 수정, 패키지 업데이트, 빌드, Git push/merge, 모터 포트 쓰기, 노드 launch, cmd_vel publish를 하지 않는다.
실제 후발 소스와 설정을 확인해 결과와 남은 문제를 보고한다. 이후 후발 변경/시험 범위는 사용자와 이어서 진행한다.
선두의 기존 성공 코드/백업을 보존하고, 현재 실행 중인 선두 구성을 임의로 중단하거나 변경하지 않는다.

## 장치와 접근

- 노트북: sunuk-Victus-by-HP-Gaming-Laptop-16-r0xxx, IP10.253.245.133(관찰 당시).
- 선두: Jetson Orin Nano, Ubuntu22.04/ROS2 Humble, hostname ubuntu, SSH sunuk@10.253.245.81.
- 후발: Raspberry Pi, Ubuntu22.04/ROS2 Humble, hostname follower, SSH sunuk@10.253.245.192.
- IP는 현재 세션에서 확인한 값이며 바뀔 수 있다. 비밀번호는 문서/대화에 기록하지 않는다.
- ROS_DOMAIN_ID=10, ROS_LOCALHOST_ONLY=0. 환경은 terminal별 source/export 필요.
- Jetson Codex 도구는 sandbox에서 장치/ROS DDS 접근에 제약이 있어 사용자가 일반터미널에서 하드웨어 명령을 실행했다.
  후발 Codex도 접근 가능 범위를 직접 확인하고 실제 측정과 소프트웨어 모의검증을 구분한다.

## 후발에서 실제 확인한 사실

- 후발에서 /leader/odom (map/base_link), /leader/path (map, length21) 실제 수신 성공.
- 후발 자체 /follower/odom_map, /follower/cmd_vel은 Unknown topic.
- 당시 후발 ps 출력에서 ROS/모터/센서 실행 프로세스 없음. 공용 ROS 노드 목록은 선두 노드이므로 후발 실행 증명이 아니다.
- ~/robot_ws/src 에 cnu_capstone_2026, leader_robot_control, ydlidar_ros2_driver, iahrs_ros2, sllidar_ros2 존재.
- ~/cnu_capstone_2026_review 에 HARDWARE_CODE_REVIEW_20261021.md, raspberrypi_follower,
  jetson_leader, convoy_robot, config, launch, docs, maps, urdf, setup.py 등 존재.
- 홈에 robot_ws_backups, YDLidar-SDK, rs485_test.py 존재.
- rg 미설치. 설치부터 하지 말고 find/grep으로 조사할 수 있다.

## 후발 조사 우선순위

1. AGENTS.md/인수인계/하드웨어 리뷰 지침을 읽고 Git 상태 및 실제 설치된 실행파일과 소스 연결 확인.
2. ~/robot_ws와 ~/cnu_capstone_2026_review 중 어느 것이 빌드/운용 기준인지 확인.
3. 후발 모터 종류·시리얼 포트·ID·baud·반지름·윤거·방향·watchdog·구동 잠금·feedback 파싱 확인.
   선두의 보정치/MD200T 설정을 후발에 그대로 적용하지 않는다.
4. 후발 센서 종류(YDLidar 등), IMU/EKF/TF/자체 위치 추정 및 공통 map 정합 방식을 확인.
5. 모든 후발 topic/frame/node 이름과 remap을 확인. /cmd_vel, /odom, /imu/data, /scan,
   /initialpose, map->odom->base_link 등을 선두와 공유하면 충돌할 수 있다.
   특히 같은 Domain10에서 무분별한 원래 bringup 실행은 양쪽 모터가 같은 명령을 받는 위험이 있다.
6. Jetson 측 통합 코드의 예상 후발 입력은 /follower/odom_map, map->follower/odom->follower/base_link,
   원시 추종 출력 /follower/cmd_vel_raw. 실제 후발 소스와 일치하는지 먼저 확인.
7. 추종 입력/bridge에 timestamp 검증, 선두정지/통신두절/TF유실/노드종료 시 정지가 있는지 검토.
   선두 bridge만 신선도검사가 수정되었고 후발 구현의 신선도검사는 아직 검증되지 않았다.
8. 시계 동기와 통신 간격을 검증. 선두->노트북 수신은 평균10Hz이나 최대gap0.484초 관찰.
9. 후발 무구동 센서/수신 -> 바퀴들기/실제정지 -> 저속지상 보정 -> 무구동 추종계산 -> 제한된실추종 순서.
   모터 구동은 실제 소스/격리/정지수단/사용자 준비 확인 후 진행한다.

## 현재 선두 최종 운용 후보(후발에 그대로 실행 금지)

- /home/sunuk/leader_diagnostics/leader_nav_015_base.launch.py
- /home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml
- 저장지도 AMCL localization + real_leader_bridge_node.py 별도 실행.
- 반지름0.085m, track0.466m, feedback_speed_scale1.18, max_rpm25, polling10Hz, vy=0제약.
- Nav2 명령상한 직진0.15m/s, 회전0.10rad/s. 피드백만 보정하므로 실제 명령속도 정확도는 미계측.
- 전체 외곽 폭0.60m 길이0.54m, 기준축 전후각0.27m. 충돌반경0.42m.
- laser TF xyz(0.26,0,0.24)m, yaw pi. 사용자 실장위치/수평방향 확인, scan/map 정합 확인.
- source MD 기본반지름/bringup 0.085 수정 외 polling/보정은 diagnostic wrapper. 성공 백업 보존.
- 기존DWB는 angular0.005 명령이 정수RPM0으로 반올림되어 정지. 이후 +/-1RPM 저속반복도 발생.
  RotationShimController 적용 후 회전->이동->목표도착->자동정지를 사용자가 모두 확인.
- current original015 YAML은 DWB, shim YAML은 별도 파일. 반드시 실제 FollowPath.plugin 확인.
- 선두 모터 M1왼쪽 M2오른쪽, FTDI AP2472IE,57600,ID1. 이는 후발 설정이 아님.
- leader bridge는 map->base_link TF + /odometry/filtered, 10Hz, 이동0.1m마다경로점, 최대5000.
  TF/odom0.5초 신선도검사, stale면 발행억제, 측정timestamp유지.
- bridge 시작 후 2D Pose Estimate로 위치를 바꾸면 경로가 불연속일 수 있다. 위치정합을 먼저하고 bridge경로수명 관리.
- 제어기 launch 재시작 시 EKF odom초기화. AMCL 초기위치 재지정 필요. Nav2만 재시작하면 EKF는유지.
- RViz는 노트북 로컬에서 실행. Jetson SSH는 DISPLAY없어서 xcb오류, 재설치불필요.
- 2D Pose Estimate=현재차체축중앙위치/앞방향, Nav2 Goal=도착차체중앙위치/도착방향.

아래는 선두 작업 전체 상태 기록이다. 과거 단계의 미검증/설정값은 뒤쪽 최신 기록으로 갱신된다.

---

# 선두 Jetson 작업 상태 — 2026-10-26

## 보호한 기준선과 현재 운용 후보

- 성공 백업 `~/robot_ws_LEADER_NAV_SUCCESS_20260906.tar.gz` SHA256 OK.
- 성공 Git clone `~/leader_nav_git_backup/repo`, 브랜치
  `leader-nav-success-2026-09-06`, 커밋 `4fe1b49`, 변경 없이 보존.
- 통합 저장소 `~/robot_ws/src/cnu_capstone_2026`는
  `real-leader-integration`, HEAD `6c73e45`. push/merge/커밋 안 함.
- 현재 MD200T 제어 소스는 바퀴 반지름 기본값만 0.085m로 수정했으며,
  모든 polling/횡속도 제약/구동 시험 코드는 이 진단 디렉터리에 별도 보관.
- 원본 IMU 소스와 원본 EKF YAML은 변경하지 않음.
- 본 세션 적용: bringup의 IMU 파라미터 키 serial_port -> port,
  bridge 설치 YAML map_frame odom -> map, bridge TF/odom 유효시간 검사.
  변경 전 파일은 ~/leader_config_backups에 보존.

## 실기체·네트워크 확인

- MD200T ID1, 57600bps, FT232R by-id. M1=왼쪽, M2=오른쪽,
  양수 RPM에서 두 바퀴 실제 전진·정지 확인.
- IMU /imu/data 약 20Hz, LiDAR /scan 약 12Hz.
- 고정 TF base_link -> imu_link / laser 확인.
- 기존 EKF, AMCL 지도 로딩·초기 위치 지정, map -> odom -> base_link 확인.
- 노트북 Domain 13을 10으로 맞춰 Jetson 토픽 수신 확인.
- /leader/odom, /leader/path를 map 기준으로 노트북에서 실제 수신.
  정지 상태 path 포즈 1개, 발행 약 10Hz.
- EKF 중단 시 bridge 발행 중단, 재시작 시 수신 복구 확인.
- Nav2 7개 lifecycle active, 실제 시간 사용, ComputePathToPose 성공.
  실제 Nav2 목표 주행은 이번 세션에서 아직 안 함.

## EKF 문제와 진단 결과

- 최초 lifted-wheel 기록에서 raw 변위 0.0585m에 EKF 위치가 수 m 점프.
- 원본 bag: bags/lifted_ekf_20261026_175317.
- 유효 TF/IMU 조건의 fresh EKF 재생에서도 raw +0.0585m,
  EKF x -0.0858m 재현.
- fresh vx 공분산만 변경하거나 vy 제약만 추가해도 x 문제는 남음.
- 초기 stale 피드백 구간을 줄이면 EKF +0.0587m로 일치.
- 따라서 명령이 없는 동안 실제 RPM 피드백 공백을 없애는 polling과
  차동구동 vy=0 모델 제약을 별도 후보로 결합.
- 주의: 19:08 첫 재생은 /tf_static QoS 충돌로 비교 무효.
  해당 결과 폴더의 COMPARISON_INVALID.md 참고.

## 현재 후보와 검증 범위

- 컨트롤러: md200t_polling_controller.py. 기존 컨트롤러를 상속하며
  같은 시리얼 소유자/단일 executor에서 PID4(PID216) 10Hz 조회.
- launch: leader_polling_constrained_ekf_check.launch.py.
  기본 allow_motion=false, 시험에만 명시적으로 true. max_rpm=5.
- config: leader_polling_constrained_ekf.yaml. 원본 대비 wheel vy 융합만 추가.
  wrapper는 vy=0 모델 제약에 분산 0.0004를 부여. 실측 보정값 아님.
- 30초 정지: stale 0, raw/EKF 위치 변화 0.
  y 분산은 polling 단독의 27.65 -> 29718 급증 대신 약 0.39 -> 1.89.
- 짧은 lifted 전진: raw 0.0580m, EKF 0.0589m; 실제 정지 확인.
- 동일 EKF에서 추가 180초 대기: stale 0, 위치 변화/최대 위치 step 0,
  헤더 최대 간격 raw 0.064s, EKF 0.061s.
  EKF 위치 분산 xy는 약 (20.24,20.24) -> (29.16,29.17).
- 이어서 재구동: raw 0.0619m, EKF 0.0613m,
  최대 관측 vx 0.0628m/s, raw 근정지 True, 사용자 실제 정지 확인.
  5RPM은 명령 상한으로, 실제 순간 RPM/속도의 절대 상한을 보증하지 않음.

## 다음 단계와 미검증

- 지상 20cm 직진 진단을 준비함. 아직 실행 결과 없음.
  leader_ground_straight_check.py: +0.04m/s, raw 전진량 0.20m 도달 또는
  10초 제한, 이어서 0속도. 시간 초과/역방향/위치 점프/피드백 유실 등
  fake ROS 테스트 9개 통과. 물리적 비상정지는 사용자가 준비한 전원 차단.
- 사용자가 실제 정지, 전방 약 1m 빈 공간과 줄자 준비 가능 상태를 확인함.
- 지상 실제 거리·직진성·회전 정확도, 장시간/통신 유실 복구, 실제 Nav2 목표
  주행 및 이동 중 /leader/path 증가와 정확도는 미검증.
- LiDAR 장착 위치를 옮겼으므로 AMCL/Nav2 전 실제 장착 xyz/rpy와
  기존 URDF laser TF의 일치 여부를 다시 확인해야 함. 임의 보정 금지.
- 후보의 정식 패키지 통합·배포·Git 커밋/push/merge는 미수행.
- 후발 Pi에는 접속·변경·시험하지 않음. Pi 수신/좌표 정합/추종은 미검증.

## 터미널 원칙

진단 launch 하나만 실행하고, Nav2/기존 모터 노드를 중복 실행하지 않는다.
실제 구동은 사용자가 Jetson 일반 SSH 터미널에서 명시적 실행 플래그로 수행.
도구의 격리 환경에서는 시리얼/DDS에 접근하지 못하므로, 본 에이전트의
실행 검증은 offline/fake 방식이고 실기체 결과는 사용자가 제공한 로그와 관찰임.

## 바퀴 치수 정정

- 사용자 실물 확인: 지름 170mm, 반지름 85mm (wheel_radius=0.085m).
- 현재 bringup, MD200T 기본값 및 진단 launch의 반지름을 0.075 -> 0.085m로 수정.
- 성공 백업/성공 clone/성공 launch 사본 및 과거 bag은 변경하지 않음.
- 이전 0.075m 설정에서 지상 시험 실측 3회 모두 27cm. 0.0972m 추정 후보는 적용하지 않음.
- 실행 중 노드에는 재시작 후 적용되며, 0.085m 설정의 지상 실측은 미검증.

## 진단용 피드백 배율 후보

- 지상 40cm 시험 raw 0.4086m, 실측 약 0.47m. 표시점 약 305~306도, /odom 역산 왼쪽 273.7도.
- 사용자 보정 요청에 따라 constrained diagnostic launch에 feedback_speed_scale=1.12 적용.
- wrapper에서 odom_callback 동안만 RPM 환산 배율 적용 후 원시 RPM 복원. 반지름 0.085m 및 송신 RPM 변환 유지.
- 양쪽 동일 배율로 선속도와 각속도 모두 영향. 회전/다른 속도 정확도 미검증.
- 재시작 및 실측 재검증 필요. 기존 성공 코드에는 적용하지 않음.

- 후속 1.12 배율 40cm 시험 실측 약 43cm. 사용자 추가 보정 요청으로 diagnostic launch 배율 1.18로 조정. 최종 raw 출력 미수신이므로 임시값, 재시작/실측 검증 필요. 반지름 0.085m 유지.

## 저속 지상 반복 검증 및 Nav2 재개 전 확인

- feedback_speed_scale=1.18, radius=0.085m, wheel_track=0.466m 유지.
- 사용자 실측 직진 40cm 5회 일치, 정지/쏠림 이상 없음.
- 좌회전 45도 3회 실측 약45도, raw46.11/EKF46.07도.
- 우회전 45도 3회 실측45도 및 실제 정지 확인.
- 검증 범위는 현재 저속/바닥 조건이며 Nav2 실제 주행 완료 아님.
- 현재 URDF laser TF는 xyz(0.15,0,0.22)m, yaw pi. 사용자 LiDAR 재배치 후 현재 실장 치수/방향 답변 대기.
- 원래 Nav2 max_vel_x0.15/theta0.30, smoother max[0.26,0,1.0], robot_radius0.22. 진단 MD max_rpm5와 상이하므로 기존 YAML 그대로 Nav2 목표 주행하지 않음. 실물 외곽 크기와 저속 planner/smoother 설정 점검 필요.

## LiDAR 実装位置修正

- 사용자 확인: 바퀴축 기준 전방26cm, 차체 좌우중앙, 바닥에서 스캔중심24cm.
- 현재 URDF의 base_link는 바퀴 반지름과 축높이를 같게 둔 지면 기준 구조이므로 laser xyz=(0.26,0,0.24)m 적용.
- laser yaw pi는 기존값 유지, 실제 장착 방향 확인 대기. 설치파일은 소스 symlink 확인, 재빌드 불필요. RSP 재시작 후 실제 TF 검증 필요.
- 성공 아카이브 및 성공 Git clone 변경 없음.

- 사용자 LiDAR 위치만 이동하고 수평 방향 동일 확인: 기존 yaw pi 유지. 다음 단계는 allow_motion=false 상태 AMCL/RViz 정합 확인. 변경 후 TF 실측 출력 및 AMCL 정합은 아직 미수신.

- 사용자 외곽 확인: 폭0.60m, 길이0.54m, 축에서 전방0.27/후방0.27m. 중심 기준 모서리 0.4036m, diagnostic costmap radius0.42m 유지 가능.
- AMCL 정적 scan/map 겹침 사용자 확인. 저속 Nav2 active 및 max x0.03/theta0.06 확인. 목표(3.673007965,1.103894949) 경로계산 SUCCEEDED(12.18ms). 현재 pose(1.558,0.321), yaw14.33deg에서 목표직선거리2.26m. 사용자가 이 목표 주행 요청. 실제 NavigateToPose 결과 아직 미검증.

- 사용자 속도 상향 요청: 별도 nav2_moderate_speed_check.yaml(x0.06,theta0.10) 및 leader_nav_moderate_base.launch.py(max_rpm10) 생성. 기존 저속 진단 launch(max5) 보존. 합성 명령9.36RPM<10 산술 확인. 실제 상향 주행 미검증, 원래 성공 코드 변경 없음.

- 사용자 15cm/s 시험 요청: nav2_015_speed_check.yaml 최대 x0.15/theta0.10, leader_nav_015_base.launch.py max_rpm25 생성. 기존 반지름0.085/배율1.18 유지. 가감속0.05m/s² 유지. 실제 속도/제동거리/장애물 회피는 미검증. 명령 변환과 피드백에 비대칭 배율이므로 실제 속도는 명령0.15m/s와 다를 수 있음.

## 2026-10-27 Nav2 진행실패 진단

- 사용자 0.15 설정 실제 장애물 우회 관찰. 이후 복귀 목표에서 완전 정지, Failed to make progress 반복, recovery spin90deg가10초 후 실패, wait5초, backup, BT action acknowledgement/service timeout 관찰. 최종 사용자 Ctrl+C로 navigation 종료. 정상 목표 SUCCEEDED는 미확인.
- 회전상한0.10rad/s에서1.57rad 회전 최소15.7초: 기존 Spin 10초 제한과 명백히 불일치. 자체 설정 누락 수정.
- 별도 BT navigate_015_recovery.xml Spin time_allowance45초. 015 YAML에 지정, progress 허용20->45초(기준거리2cm 유지), BT tick10->20ms, action응답20->200ms, DWB debug off. 속도/충돌영역/모터 설정 불변. YAML/XML 검증, 실기체 개선 미검증.
- controller의 정지 원인 자체는 cmdvel/odom/회피 후보 기록 없으므로 미확정. CPU/TF/충돌영역 및 작은 명령 정수RPM 반올림 조사 가능. planner expected20Hz 경고만으로 CPU 병목 단정 금지.

- 10/27 새목표 정지 root cause: 제어기 실제로그 angular0.005가 정수RPM0으로 환산, 699회 연속0확인. reports/nav2_deadzone_20261027.md 참조. 015 YAML 최소속도와 DWB/smoother가감속 조정. 실제 수정효과 미검증.

- 후속 1793082525..2535 로그: 회전 +/-0.03..0.04, 실제 명령 +/-1RPM, 응답0RPM164회 및 부호전환 반복. 0RPM 반올림 외 저속구동/명령진동 문제 잔존. 015 YAML 최소theta0.08(이전 지상회전 시험 명령), theta가감속 DWB/smoother1.6 설정. 최대theta0.10 유지. 실기체 개선 미검증, deadzone/좌우반복 원인 완전확정 아님.

- 사용자 수정후45도목표 회전/주행 관찰,180도목표 미출발 보고. 실제180도명령로그 미검토이므로 원인확정 아님. 설치된 rotation_shim_controller1.1.20 확인, 별도 nav2_015_rotation_shim_check.yaml 생성(DWB기존값유지, 경로방향차0.5rad이상 먼저0.1rad/s회전,0.2rad이내 해제). 회전중 충돌검사 유지, 직진0.15속도증가 안함. 활성화/실제개선미검증.

## 2026-10-27 Rotation Shim 실기체 검증 완료

- 사용자 실제 확인: shim 적용 후 버벅거림 없는 회전, 뒤쪽 목표로 회전 -> 이동 -> 목표 도착 -> 자동 정지 모두 확인.
- 사용자 보고에 근거한 동작 검증이며 NavigateToPose SUCCEEDED 로그 자체는 아직 수신하지 않음.
- 현재 운용 후보: leader_nav_015_base.launch.py (반지름0.085m, track0.466m, feedback_speed_scale1.18, max_rpm25, polling10Hz, lateral constraint) + nav2_015_rotation_shim_check.yaml (직진 명령상한0.15, 회전0.10rad/s). 실제 속도 계측 미완료.
- 성공 기준선은 보존, 위 구성은 별도 진단 경로. bridge를 포함한 전체 운용 launch 통합/재부팅 후 반복 검증은 아직 안 함.
- 후발 연동 전 남은 항목: 현재 보정/TF 구성에서 bridge 재기동, /leader/odom 및 실제주행 /leader/path 증가/연속성/신선도 검증, follower Pi 수신/좌표/시간동기/정지/추종 검증. 과거 노트북 수신 확인은 Pi 수신 증명이 아님.

## 2026-10-27 현재 구성 bridge 실주행 검증

- Domain10 노트북에서 /leader/odom(map/base_link), /leader/path(map) 수신 확인.
- 정지 시작 path1, 위치(3.549524,0.733643), 이후 주행 뒤 path13, 위치(4.839301,0.812857): 좌표 순변위 약1.292m. 경로 누적 확인.
- 이후 vx 약-3.1e-6m/s,wz0.00367rad/s. 이 메시지만으로 물리적 완전정지 확정하지 않음.
- 수신 평균 약10Hz이나 hz관측 최대gap0.484초, 최소0초(버스트) 존재. 네트워크/DDS/스케줄링 구분 미완료, 일정한 실시간 지연 보장 아님.
- 현재 구성 stale억제와 복구는 아직 실기체 재검증 필요(이전 구성 테스트와 구분). 후발Pi수신/시간동기/추종/통신두절정지 미검증.

## 후발 Raspberry Pi 네트워크 수신 증명

- 사용자 sunuk@follower 터미널 Domain10에서 /leader/odom(map/base_link) 및 /leader/path(map,length21) 실제 수신 확인. 선두정지근처 twist vx~-8e-46,wz~4e-7.
- 후발 자체 TF/odom/모터/시간동기/패킷두절정지/추종은 아직 미검증.
- Jetson 측 real follower 코드 입력조건은 /follower/odom_map 및 map->follower/base_link. 후발 실제설치 코드 및환경과 동일하다는 보장은없음. 확인전 추종node 시작금지.
