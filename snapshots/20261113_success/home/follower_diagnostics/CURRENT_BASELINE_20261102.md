> 목표 범위 정정: 설계보고서 원문을 확인했다. 아래의 제한 시연 완료 항목은 정량평가 1·2·3 완료를 뜻하지 않는다. 최종 목표와 실제 차이는 EVALUATION_REQUIREMENTS_20261102.md를 우선한다. 특히 평가3은 APF 비접촉 우회+재합류후1m 주행이며, 지금까지의 전방정지시험으로 대체할 수 없다. 보고서조건0.30m/s·간격1.0m도 아직 검증하지 않았다.

# 현재 운용 기준 구성 — 2026-11-02

이 문서는 사용자 실행 결과·설정 파일·시험 로그에 따른 기준 구성이다.
에이전트의 프로세스 네임스페이스에서는 로봇의 실시간 프로세스를 직접 조회하지 못했다.
실제 실행 목록은 각 로봇의 터미널에서 ps 및 ros2 node list로 최종 대조할 것.
노드를 종료하거나 설정을 변경하지 않고 정리한 문서다.

## 공통 환경 — 각 로봇 새 터미널

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

## 선두 ubuntu / 10.253.245.81 — 상시 터미널 3개

### 1. 기본 센서·모터·EKF

```bash
ros2 launch ~/leader_diagnostics/leader_nav_geometry_candidate.launch.py allow_motion:=true
```

- 기존 leader_sensors_check.launch.py 포함: 라이다·IMU·장착TF.
- /md200t_controller: leader_md200t_polling_split_scale_candidate.py.
- /ekf_filter_node: leader_ekf_wheel_yaw_candidate.yaml.
- 바퀴 반지름 .085m, 중심간격 .560m.
- 직진 feedback_speed_scale1.18, 회전 feedback_yaw_scale1.0.
- 각속도 분산 하한 .0009(비보정 초기 후보값).
- EKF odom0=/odom, vx/vy/wz 사용. imu0 미사용.
- ID1/57600baud/기존 선두FTDI포트/max25RPM/명령TTL.5초.
- 기존 leader_nav_015_base.launch.py와 동시실행 금지.

### 2. 지도·AMCL

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/sunuk/robot_ws/maps/leader_map.yaml \
  params_file:=/home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml \
  use_sim_time:=false autostart:=true
```

주요 노드 /map_server, /amcl 및 localization lifecycle manager.
선두 초기위치 /initialpose. TF map->odom->base_link->laser.
laser 장착TF x=.260,z=.240,yaw180도는 유지. gyro deadband 제거판이 적용된 상태.

### 3. 선두 브릿지

```bash
python3 -B ~/leader_diagnostics/leader_bridge_latest_candidate.py --ros-args -p map_frame:=map -p base_frame:=base_link -p publish_rate:=20.0
```

노드 /real_leader_bridge, 출력 /leader/odom 및 /leader/path.
초기 위치를 다시 지정한 뒤 브릿지를 재시작해 이전 경로이력을 초기화.

## 후발 follower / 10.253.245.192 — 상시 터미널 6개

### 1. 후발 드라이버

```bash
python3 -B ~/follower_diagnostics/follower_drive_node.py \
  --run-hardware --ack-hardware-zero-rpm-tx --input-kind guard \
  --enable-motion-input --trial-max-rpm 10 --trial-seconds 30
```

노드 /follower_drive_candidate. 시작 비무장, 자동arm없음.
바퀴반지름 .065m, 중심간격 .560m. 차체폭 .600m, 앞뒤각각 .270m.
ID1/19200baud/USB by-path1.4 단일 소유자.
출력 /follower/odom_raw, /follower/odom_position_candidate, /follower/motor_candidate/status.
명령 /follower/motor_candidate/guard_command, arm/disarm 서비스.
max10RPM, trial30초, 원본/명령/모터응답 기한.3초 유지.
간헐 RESPONSE_TIMEOUT은 최근 시험에서 미재현했으나 근본 해결 확정 아님.
수신바이트·tick구간·ROS발행시간 오류 진단이 추가돼 있음.

### 2. 센서·장착TF·후발EKF

```bash
ros2 launch ~/follower_diagnostics/follower_position_fixed_lidar.launch.py enable_position_profile:=true
```

주요 노드:
/follower_imu_mount_tf, /follower_laser_mount_tf,
/follower_imu, /follower_lidar, /follower_ekf_filter_node.
다음 재기동부터 위 통합 launch를 사용한다. 기존 observation launch와 start_lidar_fixed_candidate.py를 모두 종료한 상태에서만 실행한다. 현재 실행 중인 두 터미널은 이 문서 갱신 때문에 종료할 필요 없다. 새 launch는 기본 라이다를 비활성화하고 수정 SDK·강도8비트 고정 후보를 정확히1개 실행한다. 나머지 IMU·EKF·TF 및 토픽은 유지한다.
기본 EKF30Hz, odom0=/follower/odom_position_candidate, IMU gyroZ 사용.
ekf_frequency20 옵션은 후보로 존재하지만 현재 기준은 인자없이 YAML30Hz.
센서포트: LiDAR1.1/230400, IMU1.3/115200. 모터포트 열지 않음.

### 3. 후발 지도·AMCL

```bash
ros2 launch ~/follower_diagnostics/follower_localization.launch.py enable_localization:=true
```

/follower/map_server, /follower/amcl, /follower/localization_manager.
지도 /follower/localization_map, 초기위치 /follower/initialpose.
TF map->follower/odom->follower/base_link->follower/laser.
과거 EKF 재시작 뒤 AMCL이 오래된TF를 참조한 사례가 있어 복구 시 TF현재시각 확인 필요.

### 4. 후발 브릿지

```bash
python3 -B ~/robot_ws/src/cnu_capstone_2026/convoy_robot/real_follower_bridge_tf_candidate.py
```

/real_follower_bridge, 출력 /follower/odom_map.
현재 EKF·TF의 오래된 원본시각을 보존하며 누락/기한초과 시 출력하지 않음.

### 5. 추종·장애물 보호 미리보기

```bash
ros2 launch ~/follower_diagnostics/follower_convoy_preview.launch.py enable_preview:=true
```

/follower_pursuit_preview 및 /follower_obstacle_preview.
입력 /leader/odom,/leader/path,/follower/odom_map,/follower/scan.
출력 /follower/cmd_vel_test,/follower/cmd_vel_stamped_test,
/follower/cmd_vel_guard_test,/follower/cmd_vel_guard_stamped_test.
중간 상태 출력 BestEffort KeepLast1, stamped우선, TF동기대기없음.
원본시각 보존, remote50ms 보수여유/.3초보호기한 유지.
장애물 hard.30m/slow.60m(차체표면기준), leader mask 없음.

### 6. 게이트웨이

```bash
python3 -B ~/follower_diagnostics/follower_guard_gateway.py --run
```

/follower_guard_gateway, stamped guard->epoch/source stamp 포함 String 모터명령.
드라이버로 가는 최종 명령 Reliable 유지. arm/rearm은 수행하지 않음.

## 노트북 RViz — 선택 상시 1개

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
rviz2 -d ~/convoy_view.rviz
```

FixedFrame map, 선두scan /scan, 후발scan /follower/scan, scan BestEffort/Depth1/Decay0.
기존profile보존. .rviz는 화면설정이며 로봇위치를 저장/복구하는 도구가 아님.

## 시험 도구 — 필요할 때만 1개

/follower의 run_joint_follow_trial.py는 실행 중에만 /joint_follow_trial_operator가 생김.
현재 기준 driver30초와 시험 --trial-seconds30 일치. 최대29.5초 추종 뒤 양쪽정지요청.
선두명령 .04m/s, turn right/left +/- .03rad/s, 실제동시출발·제동보장아님.
rosbag은 현재 시험에서 사용하지 않음. 개별leader_short_forward_trial과 동시실행 금지.
최신 우회전 log1793547175079223970: 시간종료/양쪽zero/no fault/성공.
지도거리1.56->1.81m로 증가했으므로 정밀간격유지는 남은과제.
선두스캔정합은 사용자관측상 크게개선, 미세잔차남음.
전방 장애물 실물 정지 완료: logs/obstacle_front_stop_confirmed_20261102.json, 진단 여유.298994m, 사용자 실측약.27m, 양쪽영속도·정지후오류없음. 반복 제안하지 않는다.
직진·좌·우회전30초 완료 기록 확보. 현재 수정SDK+뒤쪽판 조건의 좌회전 log1793620789981196357 성공. 같은 조건의 우회전 추가실시는 사용자 지시로 생략했으며 이전우회전성공과 구분한다.
Wi-Fi 절전OFF 유지(선두wlP1p1s0,후발wlan0). 재연결/재부팅 후 상태 재확인. 시험도구 START안내 후30초 수신대기. 무제한연속추종으로 확장하지 않았다.
뒤쪽 무광판은 임시 배경 조건이다. 판제거시 순간 내부점 재발. 필터로 지우지 않았으며 일반환경해결로 간주하지 않는다.

## 조회 — 새 프로세스 시작/종료 없음

각 호스트에서:
```bash
hostname
ros2 node list --no-daemon
```
같은Domain10의원격노드도 보일수있어 node list만으로호스트를구분하지않을것.

후발 상태:
```bash
ros2 topic echo /follower/motor_candidate/status --once --full-length
python3 -B ~/follower_diagnostics/check_follow_start.py --seconds 10
```


최신 업데이트: 통합센서 구성으로 판 없는 직진30초 성공(logs/joint_follow_trial_1793621988255863710.jsonl). 양쪽zero/오류없음, 지도간격1.391→1.549m. 순간라이다검출해결로 간주하지 않는다. 전체재부팅 재현은 아직 미실시이며 RESTART_ACCEPTANCE_20261102.md를 사용한다.


최신 제한시연 재검증: joint_follow_trial_1793624385286758908.jsonl 직진30초 시간만료/양쪽영속도확인/정지후오류없음. 주행중 INPUT_REJECTED 기록 0건. 경로이력초기화수정적용후결과. 현재기동구성의재현기록이며 일반환경장시간운용·라이다순간점·실물직진편향해결로간주하지않는다.
