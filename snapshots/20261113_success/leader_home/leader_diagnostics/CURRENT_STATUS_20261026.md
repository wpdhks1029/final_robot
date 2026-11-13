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

## 2026-10-28 GitHub 선두 성공 스냅샷 게시

- 사용자 요청 저장소: https://github.com/wpdhks1029/capstone_leader (public).
- main commit ec89638be6cfef173b23e3b49448d2d98e6ee56a (GitHub connector로 게시, main파일 재조회 확인).
- 별도 로컬 export /home/sunuk/capstone_leader, 153 tracked payload files, original source hash manifest140개, software mock tests42개 통과.
- 모터/IMU/LiDAR SDK/TF/EKF/RotationShim/bridge/지도/실행문서 포함. 후발코드/로그/bag/인증정보 제외.
- 새 export 빌드 convoy_robot/iahrs_ros2완료, leader_robot_control setup에서 지연되어 fullbuild중단; 전체cleanbuild 재현성 미확정 docs/VALIDATION.md에 명시. 실행helper는 새 포팅이며 실기체재검증별도.
- 원래workspace/설치/보호Git/20260906성공archive그대로. archiveSHA재검증OK.
