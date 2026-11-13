# 전체 재부팅 후 제한 시연 재현

현재 노드를 추가로 실행하는 절차가 아니다. 양쪽 로봇을 정지하고 기존 시험/launch를 종료한 뒤 전체 재부팅했을 때 사용한다. 로스백·진단용 라이다 교체도구는 실행하지 않는다.

## 재부팅 전

마지막 시험의 양쪽 영속도 확인 결과를 확인하고 실물 정지도 확인한다. 후발에서 비무장을 명시한다.

```bash
ros2 service call /follower/motor_candidate/disarm std_srvs/srv/Trigger "{}"
```

각 로봇의 기존 실행 터미널을 Ctrl+C로 종료한 뒤 각 호스트에서 재부팅한다. RViz 노트북은 재부팅할 필요 없다.

```bash
sudo reboot
```

## 모든 새 ROS 터미널의 공통 환경

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

## 선두 ubuntu

Wi-Fi 절전은 재부팅/재연결 시 다시 켜질 수 있다. 선두 한 터미널에서:

```bash
sudo iw dev wlP1p1s0 set power_save off
iw dev wlP1p1s0 get power_save
```

선두 터미널1:

```bash
ros2 launch ~/leader_diagnostics/leader_nav_geometry_candidate.launch.py allow_motion:=true
```

선두 터미널2:

```bash
ros2 launch nav2_bringup localization_launch.py map:=/home/sunuk/robot_ws/maps/leader_map.yaml params_file:=/home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml use_sim_time:=false autostart:=true
```

선두 터미널3:

```bash
python3 -B ~/leader_diagnostics/leader_bridge_latest_candidate.py --ros-args -p map_frame:=map -p base_frame:=base_link -p publish_rate:=20.0
```

## 후발 follower

후발 한 터미널에서:

```bash
sudo iw dev wlan0 set power_save off
iw dev wlan0 get power_save
```

후발 터미널1 — 모터 피드백, 시작 비무장:

```bash
python3 -B ~/follower_diagnostics/follower_drive_node.py --run-hardware --ack-hardware-zero-rpm-tx --input-kind guard --enable-motion-input --trial-max-rpm 10 --trial-seconds 30
```

후발 터미널2 — IMU/EKF/TF와 수정 라이다를 함께 실행. 원래 observation launch나 단독 라이다를 추가 실행하지 않는다:

```bash
ros2 launch ~/follower_diagnostics/follower_position_fixed_lidar.launch.py enable_position_profile:=true
```

후발 터미널3:

```bash
ros2 launch ~/follower_diagnostics/follower_localization.launch.py enable_localization:=true
```

후발 터미널4:

```bash
python3 -B ~/robot_ws/src/cnu_capstone_2026/convoy_robot/real_follower_bridge_tf_candidate.py
```

양쪽 localization 활성화와 초기위치 정합을 확인한 뒤 후발 터미널5:

```bash
ros2 launch ~/follower_diagnostics/follower_convoy_preview.launch.py enable_preview:=true
```

후발 터미널6:

```bash
python3 -B ~/follower_diagnostics/follower_guard_gateway.py --run
```

## RViz·초기위치

노트북:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
rviz2 -d ~/convoy_view.rviz
```

Fixed Frame=map. scan은 /scan과 /follower/scan, BestEffort/Depth1/Decay0. 2D Pose Estimate의 topic을 선두 /initialpose, 후발 /follower/initialpose로 바꿔 각 실제 위치·방향을 맞춘다. 기존 pose 숫자를 현재 실제 배치와 무관하게 재사용하지 않는다. 선두 초기위치를 조정했다면 선두 bridge만 한 번 재시작해 경로이력을 초기화한다.

## 재현 확인

후발 별도 터미널:

```bash
python3 -B ~/follower_diagnostics/check_follow_start.py --seconds 10
```

양쪽 실제 배치와 지도 간격이 맞아야 한다. motor fault=null/armed=false/feedback_fresh=true/측정RPM[0,0], follower/leader/path 입력이 수신돼야 한다. 경로1점·선두정지 때문에 preview가0인 것은 초기 상태에서 예상된다. 조회 결과만으로 주행 준비를 자동 승인하지 않는다.

위치·공간·전원차단 준비를 확인한 뒤 기존 제한시연을1회 실행한다.

```bash
python3 -B ~/follower_diagnostics/run_joint_follow_trial.py --run --ack-clear-both-power-cut-ready --trial-seconds 30 --leader-speed 0.04 --leader-turn straight --trace-timing
```

START 안내 후30초 수신대기 뒤 START 입력. 시간만료·양쪽zero확인·post_stop_faults=[]이면 재부팅 후 제한시연 재현 통과. 실패하면 같은 주행을 반복하지 않고 로그 원인을 확인한다. 이번 재현은 무제한 연속추종이나 라이다의 일반환경 신뢰성을 인증하는 시험이 아니다.

## 최신 완료 근거

통합센서 재시작 뒤 판 없는 직진시험 joint_follow_trial_1793621988255863710.jsonl 성공. 양쪽영속도·정지후오류없음, 지도상간격1.391→1.549m. 전체컴퓨터재부팅 재현은 아직 미실시.


최신 제한시연 재검증: joint_follow_trial_1793624385286758908.jsonl 직진30초 시간만료/양쪽영속도확인/정지후오류없음. 주행중 INPUT_REJECTED 기록 0건. 경로이력초기화수정적용후결과. 현재기동구성의재현기록이며 일반환경장시간운용·라이다순간점·실물직진편향해결로간주하지않는다.
