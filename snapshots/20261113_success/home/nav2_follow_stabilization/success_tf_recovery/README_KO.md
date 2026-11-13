# GitHub 성공 백업과 현재 AMCL TF 정지 비교

출처: https://github.com/wpdhks1029/final_robot
비교한 Git tree: d5dc27a75b9490802141e30c093874dd37729cd9.
GitHub blob SHA1을 재계산해 로컬 final_robot의 818개 파일이 모두 일치함을 확인했다.
원본 GitHub·final_robot·기존 장치 설정은 덮어쓰지 않았다.

## 확인한 내용

현재 follower_localization.launch.py, follower_position_fixed_lidar.launch.py,
follower_observation.launch.py, follower_hardware.yaml은 GitHub 성공본과 동일하다.
성공본은 센서·EKF·AMCL·Bridge가 /tf와 /tf_static을 공유했다.
현재는 wrapper로 /follower/tf_internal 및 /follower/tf_static_internal로 분리했다.
현재 Bridge는 TF 큐100→10, 오뎀 발행 Reliable depth20→Best Effort depth1로 달라졌다.

사용자 로그에서는 AMCL의 초기 위치 수신은 동작했고, AMCL buffer의 odom TF 시각이
1794560000.393977에 고정됐다. 동시에 별도 Best Effort 구독자는 EKF와 TF를 약20Hz,
나이 약0.04~0.07초로 수신했다. 정적 TF가800초 전 시각인 것은 정적 데이터의
특성이므로 동적 TF 정지 근거로 사용하지 않는다.
따라서 문제 위치는 EKF 발행 중단보다 AMCL 동적 TF 전달/처리 경로다.
Reliable 수신 또는 listener 처리 정지가 의심되지만 DDS 내부 원인은 확정하지 않았다.
외부 follower_amcl_candidate.yaml은 GitHub에 포함되지 않아 해당 파일 변경 여부는
저장소만으로 확인할 수 없다. 시간이 지난 TF를 새 시각으로 찍어 내보내거나,
지도 위치를 가짜로 만들거나, 입력 시각 검사를 완화하지 않는다.

## 복구 비교 실행 — 지도 오뎀 확인 전 주행하지 않는다

평가 프로그램을 종료한다. 후발 기존 터미널2(센서),3(AMCL),4(Bridge)를 Ctrl+C로
종료한다. 모터 드라이버는 비무장 상태로 유지하고 유니캐스트도 유지한다.
다른 새 원격 설정이나 SSH 전송은 필요 없다. 아래 한 명령이 후발2~4를 함께 실행한다:

```bash
bash ~/nav2_follow_stabilization/success_tf_recovery/start_localization.bash
```

원본 센서·AMCL launch와 GitHub 원본 Bridge를 공개 TF 구성으로 실행한다.
EKF20Hz는 기존 현재 설정을 유지한다. 다른 모터 노드·속도 명령·arm 서비스는 생성하지 않는다.
AMCL은 원본 lifecycle manager의 autostart로 활성화한다.
활성화 후 RViz /follower/initialpose로 실제 후발 위치·방향을 지정한다.

```bash
python3 -B ~/nav2_follow_stabilization/diagnose_follower_odom.py --tf-scope public --seconds 15
```

FOLLOWER_MAP_ODOM_RECEIVED, 최신 filtered/odom_map, 공개 map→follower/odom와
follower/odom→follower/base_link가 지속 수신되는지 확인한다. 실제 성공은 아직 확인 전이다.

## 복구 구성에서 평가 실행

정량목표3은 기존10cm·5cm 드라이버와 현재 기준선 캡처를 사용한다.
공개 static TF와 맞추기 위해 운영 프로그램은 다음 wrapper로 실행한다:

```bash
bash ~/nav2_follow_stabilization/evaluation3_clearance10_stop05/capture_reference.bash
bash ~/nav2_follow_stabilization/success_tf_recovery/start_evaluation3.bash
```

첫 명령이 REFERENCE_JSON으로 성공한 뒤 두 번째 명령을 실행한다.
정량목표1 일반 추종으로 돌아갈 경우 원래 smooth010 드라이버로 교체하고
success_tf_recovery/start_follow.bash 0.1을 사용한다. 선두 Nav2/gate는 정량목표1에서만 켠다.
기존 smooth010/start_follow.bash의 기본 private 토픽은 유지하고 명시적인
CONVOY_TF_STATIC_TOPIC 환경 변수로만 공개 토픽을 선택하도록 했다.

## 검증 범위

GitHub 원본 Bridge와 helper 파일 해시 일치, 복구 launch의 세 노드 그룹과 TF 토폴로지,
원본 Bridge의 원본시각/수신시각 stale 거절, 일반/복구 추종 wrapper 토픽 선택 4개 검증 통과.
실제 ROS 수신, 장치 구동, AMCL 복구 또는 주행 성공을 검증했다고 주장하지 않는다.

## 사용자 복구 확인

공개 TF 전환 후 사용자가 제공한15초 관측에서 FOLLOWER_MAP_ODOM_RECEIVED,
odom_map152개(약10Hz), 종료 시각 source age 약0.072초, filtered288개,
map→follower/odom 및 follower/odom→follower/base_link 연결을 확인했다.
이 관측에서 지도 오뎀 복구를 확인했으며 장기 안정성·주행 성공 또는 DDS 내부 원인
확정을 뜻하지 않는다. 사용자 관측 요약은 operator_recovery_verified_20261113.json에 보존했다.
현재 평가3은 capture_reference 성공 후 이 폴더 start_evaluation3.bash로 실행한다.
