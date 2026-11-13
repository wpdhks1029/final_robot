# 정량목표 1~3 실행표 — 2026-11-13 현재 성공 구성

정량목표1: 선두 Nav2 Goal 주행 + 후발 실측 경로 추종.
정량목표2: 선두 단독 3m 전진을 3회. 후진은 별도 복귀용이다.
정량목표3: 후발 장애물 회피 → 선두 경로 재합류 → 추종 재개.
사용자가 정량목표3 대기/재개 수정 후 실물 동작 성공을 확인했다.
이 확인은 정식 평가 합격이나 모든 거리/오차 기준의 충족을 뜻하지 않는다.

## 모든 선두·후발 ROS 터미널 환경

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
```

각 코드는 별도 터미널에서 실행한다. 이미 실행 중인 같은 노드를 중복 실행하지 않는다.

## 정량목표1·3 공통 선두 노드

선두 1: 모터·센서·EKF
```bash
ros2 launch ~/leader_diagnostics/leader_nav_yaw_calibrated.launch.py allow_motion:=true
```

선두 2: 지도·AMCL
```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/sunuk/robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml \
  params_file:=/home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml \
  use_sim_time:=false autostart:=true
```

선두 3: Bridge
```bash
FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/convoy_udp_only_pilot.xml" \
python3 -B ~/leader_diagnostics/leader_bridge_latest_candidate.py \
  --ros-args -p map_frame:=map -p base_frame:=base_link -p publish_rate:=20.0
```

선두 4: 유니캐스트
```bash
FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/convoy_udp_only_pilot.xml" \
python3 -B ~/leader_diagnostics/convoy060_dfa647fff6de/convoy_unicast_relay.py \
  --role leader --peer 10.253.245.192
```

## 정량목표1·3 공통 후발 노드

후발 1은 아래 평가별 드라이버를 먼저 실행한다.
후발 2: 센서·EKF·지도·AMCL·Bridge 통합 성공 구성
```bash
bash ~/nav2_follow_stabilization/success_tf_recovery/start_localization.bash
```

이 통합 스크립트가 LiDAR/IMU/장착TF/EKF/지도/AMCL/Bridge를 실행한다.
과거 follower_private_tf_* 3개는 함께 실행하지 않는다. 현재 성공 구성은 /tf 및 /tf_static이다.
AMCL은 자동 활성화한다. 초기 위치 지정은 실제 차체 위치에 맞춰 별도로 한다.

후발 3: 유니캐스트
```bash
python3 -B ~/nav2_follow_stabilization/convoy_unicast_relay.py \
  --role follower --peer 10.253.245.81
```

## 노트북 RViz — 정량목표1·3

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
rviz2 -d ~/convoy_view.rviz
```

Fixed Frame=map. 선두 초기 위치 토픽=/initialpose, 후발=/follower/initialpose.
후발 LaserScan 표시 토픽=/follower/scan, Reliability=Best Effort.
표시 체크 여부와 별개로 공통 센서 실행 시 후발 스캔은 이미 발행한다.
로봇을 재배치하거나 드라이버/위치추정을 재시작했다면 실제 초기 위치·방향을 다시 확인한다.

## 정량목표1 — 선두 Nav2 + 후발 추종

후발 1:
```bash
bash ~/nav2_follow_stabilization/smooth010/start_follower_driver.bash
```

공통 노드와 초기 위치 준비 후 선두 5:
```bash
bash ~/leader_diagnostics/nav2_follow_stability_da0ab05848da/start_leader_nav2.bash
```

후발 4:
```bash
bash ~/nav2_follow_stabilization/success_tf_recovery/start_follow.bash 0.1
```

입력 준비/Goal 대기 메시지 후 RViz에서 선두 Nav2 Goal을 지정한다.
후발 최대0.1m/s, 15RPM, 총시간 제한 없음. 기존 일반 추종 구성이다.
이 평가1 구성은 라이다 안전거리 정지를 비활성화한 기존 모드이며 장애물 회피 평가3과 구별한다.
아래 평가3 신호/스캔 정렬 무기한 대기 변경을 평가1에 적용했다고 간주하지 않는다.

## 정량목표2 — 선두 단독 3m 전진 × 3회

선두 Nav2/gate·후발 추종/평가3·teleop을 종료한다. 후발은 비무장으로 경로 밖에 둔다.
선두 모터·센서·EKF(선두 1)가 필요하다. 지도/AMCL/Bridge/중계기는 거리 측정 시험의 필수 노드가 아니다.
기존 선두 1이 켜져 있다면 다시 실행하지 않는다.

아래를 1회씩 실행하고 START + Enter, 총3회 반복한다.
```bash
python3 -B ~/leader_diagnostics/eval2_distance_dbd45e9d9025/run_leader_distance_trial.py \
  --run --ack-clear-path-power-cut-ready \
  --direction forward --distance-m 3 --speed 0.1 \
  --distance-scale 0.935594233291917 --operator-clear-path
```

실측 합계9.05m에서 산출한 현재 잠정 계수다. 매회 동일한 차체 기준점으로 실측한다.
operator-clear-path는 운영자가 경로를 확인하며 라이다 정지를 사용하지 않는 명시적 모드다.
기존 설치 코드에 CLI 계수를 전달하므로 deploy_leader_candidate.py --apply를 매번 할 필요 없다.

### 선택: 복귀용 3m 후진 (정량목표2의 전진3회와 별도)

최대0.5m/s 후진은 선두 1을 종료한 뒤 전용 모터·센서 launch로 교체한다.
```bash
ros2 launch ~/leader_diagnostics/eval2_distance_0b10ece7e961/leader_eval2_speed050.launch.py \
  allow_motion:=true
```

다른 선두 터미널:
```bash
bash ~/leader_diagnostics/eval2_distance_0b10ece7e961/reverse_3m.bash
```

후진 계수1.0은 전진 보정과 별도다. 정량목표1·3 복귀 시 전용launch를 종료하고 일반 선두1로 교체한다.

## 정량목표3 — 장애물 회피·재합류·추종

선두 Nav2/gate, 평가2, teleop, 후발 일반 추종을 종료한다.
공통 선두1~4와 후발 통합 위치추정/유니캐스트는 유지한다.
선두 주행명령은 평가3 프로그램이 /cmd_vel로 전달한다. RViz Nav2 Goal을 찍어 시작하는 평가1과 구별한다.
새 시작 위치로 재배치했다면 선두 Bridge도 재시작하여 이전 실측 경로를 초기화한다.
후발 드라이버를 평가3 전용 하나로 교체한다.

후발 1:
```bash
bash ~/nav2_follow_stabilization/evaluation3_reference_wait/start_follower_driver.bash
```

드라이버 재시작 후 양쪽 초기 위치/방향을 맞추고 정지·후발 비무장 상태에서 후발 4:
```bash
bash ~/nav2_follow_stabilization/evaluation3_reference_wait/capture_reference.bash
```

REFERENCE_JSON 성공 확인 후 같은 터미널:
```bash
bash ~/nav2_follow_stabilization/success_tf_recovery/start_evaluation3_wait.bash
```

START + Enter. 계획 여유10cm, 정지 여유5cm, 후발10RPM, 재합류 후 추종 목표2m.
총6분 제한 없음. 원격 기준 입력 지연 및 스캔/오뎀 정렬 누락은 속도0으로 대기하고,
새 입력/정렬의 안정과 정지 피드백 확인 후 같은 arm 세션에서 재개한다.
모터/엔코더/로컬 센서 고장·명령/피드백 watchdog·정상 완료·Ctrl+C는 종료 조건으로 남는다.

## 평가 전환표

| 전환 | 종료/교체 | 유지 |
|---|---|---|
| 1 → 3 | 선두5 Nav2/gate, 후발 일반 추종 종료; 후발 드라이버 교체; 기준선 새 저장 | 선두1~4, 후발 통합 위치추정·중계기 |
| 3 → 1 | 평가3 종료; 후발 smooth010 드라이버로 교체; 선두5 및 일반 추종 실행 | 공통 센서·지도·Bridge·중계기 |
| 1 또는3 → 2 | Nav2/gate 및 추종/평가3 종료, 후발 비무장 | 선두 일반 모터·센서·EKF |

장치나 동일cmd_vel에 명령하는 실행을 중복하지 않는다. 실물 재배치/드라이버 재시작 후에는 초기 위치를 확인한다.
