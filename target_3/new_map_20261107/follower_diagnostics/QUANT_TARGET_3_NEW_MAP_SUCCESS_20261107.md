# 정량목표3 — 새 지도 적용 사용자 확인 성공 버전 (2026-11-07)

사용자가 새 지도 적용 후 정상 작동했다고 확인하고 현재 상태의 GitHub 보관을 요청했다. 기존 2026-11-06 성공 백업은 그대로 유지한다.

## 포함 변경

- 양쪽에서 사용하는 새 지도: `robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml` 및 `leader_map.pgm`.
- 후발 localization launch에 명시적인 `map:=` 경로 추가. 기존 지도 기본값과 기존 지도 해시 검사는 보존.
- 조회 전용 `capture_shared_map_reference.py`로 새 지도 기준선을 별도 저장. 새 기준선 형식을 시험에서 사용.
- 근접 정지 후 재출발 시 정수RPM 반올림으로 전진량0이 되는 경우 정지 출력을 유지하며 가속 램프 진행. slew 이후 좌우10RPM 제한 재적용, wheel_diagnostic 기록.

## 시험 증거와 범위

원본 로그: `local_avoidance_trial_1794032200970674450.jsonl` (전체는 원본 백업 압축 파일에 포함).
상자 감지 → 회피 경로 계산 → 지도 경로 합류 → FOLLOW 전환 기록. 마지막 경로 추적 진행량 약2.539m, 마지막 연속 지도 정렬 검증량 약0.260m. 최종 종료 STOP_APF_WORKSPACE_BOUND, 양쪽 정지 확인, post_stop_faults 없음. 자동 trial_success/evaluation_pass와 tracked_pursuit_handoff_confirmed는 false다. 사용자 확인 정상 동작과 자동2m 연속 검증 미통과를 구분해 보관한다. 기록 결과를 성공으로 바꿔 쓰지 않는다.

모터 보정/계획 여유20cm/정지10cm/전방감지70cm/lookahead35cm/최대10RPM/360초는 유지한다. 계산 최적화와 속도 상승은 아직 적용하지 않았다.

## 복원 경로와 실행

이 폴더의 `follower_diagnostics/`는 현재 코드·설정·시험 도구 사본이다. `robot_ws/maps/`는 새 지도다. `reference/new_map_reference_20261107.json`은 당시 시작 배치 기준선이다. 백업 tar.gz는 원래 홈 디렉터리 기준 상대경로를 사용하며 전체 시험 로그를 포함한다. 원본 manifest의 SHA256으로 확인할 수 있다.

새 지도와 YAML/PGM 해시는 기준선 기록과 일치함을 검증했다. 당시 기준선은 당시 배치에 해당하므로 다른 시작 배치에서는 조회 전용 도구로 별도 기준선을 생성해야 한다. 기존 성공 백업의 지도와 섞지 않는다.

공통 환경: source /opt/ros/humble/setup.bash; source ~/robot_ws/install/setup.bash; ROS_DOMAIN_ID=10; ROS_LOCALHOST_ONLY=0.

선두 모터·센서·EKF:
```bash
ros2 launch ~/leader_diagnostics/leader_nav_yaw_calibrated.launch.py allow_motion:=true
```
선두 지도·AMCL:
```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/sunuk/robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml \
  params_file:=/home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml \
  use_sim_time:=false autostart:=true
```
선두 Bridge:
```bash
python3 -B ~/leader_diagnostics/leader_bridge_latest_candidate.py --ros-args -p map_frame:=map -p base_frame:=base_link -p publish_rate:=20.0
```
후발 드라이버:
```bash
python3 -B ~/follower_diagnostics/follower_drive_node.py \
  --run-hardware --ack-hardware-zero-rpm-tx --input-kind local_avoidance \
  --ack-bounded-pivot-candidate --ack-continuous20-candidate --enable-motion-input \
  --trial-max-rpm 10 --trial-seconds 360
```
후발 센서·EKF:
```bash
ros2 launch ~/follower_diagnostics/follower_position_fixed_lidar.launch.py enable_position_profile:=true
```
후발 지도·AMCL:
```bash
ros2 launch ~/follower_diagnostics/follower_localization.launch.py \
  enable_localization:=true \
  map:=/home/sunuk/robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml
```
후발 Bridge:
```bash
python3 -B ~/robot_ws/src/cnu_capstone_2026/convoy_robot/real_follower_bridge_tf_candidate.py
```
시험:
```bash
python3 -B ~/follower_diagnostics/run_local_avoidance_trial.py \
  --run --ack-clear-both-power-cut-ready --apf-pivot --gentle-pivot --map-rejoin \
  --continue-follow-after-rejoin --continuous20 --dynamic-box --post-rejoin-follow-m 2 \
  --pose-source filtered \
  --reference-plan ~/follower_diagnostics/logs/new_map_reference_20261107.json \
  --seconds 360
```
평가3 중 선두 navigation_launch.py/SLAM/teleop은 종료한다. 양쪽 지도 초기위치 지정 후 선두 Bridge를 재시작한다.

선두 최신 leader_diagnostics 원본 전체는 이 장치에서 확보하지 못했다. 선두 실행 명령은 보관하지만 완전한 선두 원본 백업으로 표시하지 않는다.
