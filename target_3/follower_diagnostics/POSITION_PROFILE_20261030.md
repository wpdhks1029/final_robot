# 후발 위치 기반 EKF 선택 구성

이 구성은 모터를 0RPM 잠금 상태로 유지하며 위치 기반 odom 후보를 실제
후발 프레임으로 전달한다. 자동 추종/비영RPM 명령 구독은 추가하지 않는다.
회전/윤거/통신단절 정지/장기 위치 지터는 미검증 항목이다.

## 유지한 원본

- 원래 robot_ws/src/cnu_capstone_2026/config/follower_ekf_A_selected.yaml 변경 없음.
- 기존 관측 launch 기본 odom_profile=rpm, 기존 명령 동작 유지.
- 변경 전 관측 launch 전체 백업:
  follower_observation.launch.py.before_position_profile_20261030
  SHA256 3ce7bf5f229a4ce4860f6dbe7e7937362a99758be0a4c87474bed169c65244d0.
- 선두 성공 코드/프로젝트 소스/Git push/merge/build/install 변경 없음.

## 후보 데이터 경로

단일 locked 모터 소유자 -> /follower/odom_position_candidate
(follower/odom, follower/base_link) -> 별도 position EKF YAML
-> /follower/odometry/filtered 및 follower/odom -> follower/base_link TF.
기존 AMCL/bridge가 사용하는 토픽/프레임 계약은 유지.
기존 /follower/odom_raw는 RPM 기준으로 계속 발행하므로 비교/복귀 가능.

입력은 60단위/회전 가정/반지름.065/윤거.560/0.5초 속도창이며,
공분산은 기존 시험값(정밀 교정 아님). 실제 위치 측정이 없으면 발행하지 않는다.
정지 지터를 숨기거나 가상0을 만들어 피드백을 대체하지 않는다.

## 전환 순서 — 사용자가 실제 터미널에서 수행

1. 새 측정/회전 명령은 입력하지 않는다. 현재 포트 소유자와 관측 launch를
   프로세스로 확인하고 정확한 해당 터미널만 종료한다.
2. 기존 관측 launch(센서/기존EKF)를 종료하여 중복 센서/TF/EKF를 방지한다.
3. 모터 실행부 종료/포트 해제 후 새 locked 후보를 실행한다.

```bash
python3 -B ~/follower_diagnostics/follower_motor_locked.py --run-hardware-locked --ack-zero-rpm-tx --publish-position-candidate
```

4. 다른 터미널에서 새 관측 프로파일을 실행한다.

```bash
ros2 launch ~/follower_diagnostics/follower_position_observation.launch.py enable_position_profile:=true
```

기본 실행은 비활성. enable시 실제source 발행자1/정상후발frame/fresh시각과
기존filtered odom발행자0을 확인해야 센서와새EKF를 시작한다.
실제 시리얼은 locked가 단독소유하며 새 관측launch는 모터 포트를 열지 않는다.

5. EKF노드의 odom0가 /follower/odom_position_candidate인지 확인한다.
   filtered발행자1/후발동적TF소유자1을 확인한다. 실패시 이동하지 않는다.
6. EKF원점이 재설정되므로 AMCL 초기위치를 실제 지도 위치로 다시 지정한다.
   기존 localization/bridge가 살아있으면 중복 실행하지 않는다.
   그뒤 정지상태 map좌표/통신만 확인하며 바로 주행하지 않는다.

## 원래 RPM 프로파일로 복귀

새 관측 launch와모터소유자를정확히종료/포트해제 후:

```bash
python3 -B ~/follower_diagnostics/follower_motor_locked.py --run-hardware-locked --ack-zero-rpm-tx --publish-position-test
```

다른 터미널:

```bash
ros2 launch ~/follower_diagnostics/follower_observation.launch.py enable_sensor_tf:=true enable_lidar:=true enable_imu:=true enable_ekf:=true external_feedback_confirmed:=true
```

복귀시에도 AMCL초기위치재지정 필요. 두프로파일을동시에실행하지않는다.

## 작성 단계 검증

프로파일계약/기존YAML과입력토픽외설정동일/nativeframe/중복거부/기본비활성/
모터잠금/기존시험publisher관련37개모의검사통과.
후속 사용자 실행에서 실제 새 프로파일 전환 확인:
odom0=/follower/odom_position_candidate,발행자1/구독자1,정지twist0。
AMCL재지정/map TF정상,20초leader/follower수신약10Hz/age>.5초0。
현재는모터0RPM잠금상태연결확인까지완료이며주행/자동추종은미검증。
