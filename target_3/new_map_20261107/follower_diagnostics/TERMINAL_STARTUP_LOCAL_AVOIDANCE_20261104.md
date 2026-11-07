# 선두·후발 실행 터미널 — 로컬 회피 30초 / 입력0.8초

현재 구성은 선두0.04m/s 직선 + 후발 로컬 회피 시험이다. 선두3개, 후발상시4개+시험1개, 노트북RViz1개. 기존 Preview/APF/임시 경로 제어 및 Gateway는 종료한다. 센서·원격 선두 입력 유효기간0.8초, 모터 명령/응답/피드백 감시0.3초, 바퀴10RPM, 후발시험30초.

## 모든 선두·후발 ROS 터미널 공통

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

## 재부팅 후 Wi-Fi 절전 확인

선두:
```bash
sudo iw dev wlP1p1s0 set power_save off
iw dev wlP1p1s0 get power_save
```
후발:
```bash
sudo iw dev wlan0 set power_save off
iw dev wlan0 get power_save
```

## 선두1 — 모터·센서·EKF

```bash
ros2 launch ~/leader_diagnostics/leader_nav_geometry_candidate.launch.py allow_motion:=true
```

## 선두2 — 지도·AMCL

```bash
ros2 launch nav2_bringup localization_launch.py map:=/home/sunuk/robot_ws/maps/leader_map.yaml params_file:=/home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml use_sim_time:=false autostart:=true
```

## 선두3 — Bridge

```bash
python3 -B ~/leader_diagnostics/leader_bridge_latest_candidate.py --ros-args -p map_frame:=map -p base_frame:=base_link -p publish_rate:=20.0
```

## 후발1 — 로컬 회피 전용 모터 드라이버

```bash
python3 -B ~/follower_diagnostics/follower_drive_node.py --run-hardware --ack-hardware-zero-rpm-tx --input-kind local_avoidance --ack-bounded-pivot-candidate --enable-motion-input --trial-max-rpm 10 --trial-seconds 30
```

## 후발2 — LiDAR·IMU·장착TF·EKF

드라이버가 위치 입력을 발행하기 시작한 뒤 실행:

```bash
ros2 launch ~/follower_diagnostics/follower_position_fixed_lidar.launch.py enable_position_profile:=true
```

## 후발3 — 지도·AMCL

```bash
ros2 launch ~/follower_diagnostics/follower_localization.launch.py enable_localization:=true
```

## 후발4 — Bridge

```bash
python3 -B ~/robot_ws/src/cnu_capstone_2026/convoy_robot/real_follower_bridge_tf_candidate.py
```

## 노트북 — RViz

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
rviz2 -d ~/convoy_view.rviz
```

Fixed Frame=map. Map topic=/follower/localization_map, Transient Local. 선두 scan=/scan, 후발 scan=/follower/scan, 두 스캔 모두 Best Effort. 2D Pose Estimate 도구 Topic은 선두 /initialpose, 후발 /follower/initialpose로 지정해 각각 위치를 설정한다. 선두 pose 설정 완료 후 선두 Bridge를 한 번 재시작해 과거 경로 이력을 지운다.

## 후발5 — 상태 확인과 시험

```bash
ros2 lifecycle get /follower/map_server
ros2 lifecycle get /follower/amcl
ros2 topic echo /follower/motor_candidate/status --once --full-length
```

map_server/amcl 둘 다 active. 모터 상태 armed=false, fault=null, feedback_fresh=true, input_kind=local_avoidance, motion_profile=LOCAL_STOP_TURN_GO_V1, input_source_timeout_s=0.8, trial_max_seconds=30.

```bash
python3 -B ~/follower_diagnostics/run_local_avoidance_trial.py --run --ack-detour-area-power-cut-ready --seconds 30
```

시작 전 검사와 현재 스캔 경로 검사를 통과하면 START+Enter로 시작한다. 최대29.5초 후발 제어 후 양쪽 정지 요청. 장애물이 이미 보이면 AVOID, 없으면 FOLLOW로 시작하며 주행 중 장애물 검출 시 PLAN_BRAKE→PLANNING→AVOID로 전환한다. 현재 시험에서는 이 도구가 선두 /cmd_vel을 발행하므로 다른 선두 속도/자율주행 시험 도구를 함께 실행하지 않는다.

기존 check_follow_start.py는 이전 guard 체인을 관찰하는 도구다. 현재 시험 준비 판정은 새 run_local_avoidance_trial.py의 시작 전 검사로 수행한다. 전체 회피·재합류는 모델상수분이므로 첫30초 성공과 전체평가3 완료를 구분한다. 현재 기준선 파일 logs/current_scene_left_detour_plan.json을 사용하며 다른 지도/코스로 바꾸면 기준선 재확인이 필요하다.
