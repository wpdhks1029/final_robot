# 정량목표 3번 — 사용자 확인 성공 기준 코드

사용자가 2026-11-06 현재 버전을 정량목표 3번 성공 코드로 지정하고 백업을 요청했다. 목표의 공식 문구는 이 기록에서 임의로 정의하지 않는다.

기능: 후발 추종 중 상자 감지 → 정지 후 곡선 경로 계산 → 회피 → 지도 경로 합류 → Pure Pursuit 추종. 감지 전방70cm, 계획 여유20cm, 정지10cm, lookahead35cm, 최대10RPM, 합류 후 추종 목표2m, 전체360초. 지속 반복 재계획은 꺼짐. 거리 보정 및 방향/충돌/입력 유효기간 보호 유지.

사용자 확인: 회피·합류 성공. 최신 시험의 경로 추적 진행량 약2.31m. 자동 검증은 연속 정렬 거리 최대 약0.897m로 2m 판정 미통과, 최종 LEADER_STALE 및 Stale/future/pre-arm source 명령 거부. post_stop_faults에는 INVALID_LOCAL_AVOIDANCE_ENVELOPE, 양쪽 정지 확인. 사용자 추정 원인은 휴대폰과의 거리 증가이며 확정 원인으로 검증된 것은 아니다. 이후 /leader/odom 조회 평균 약17~22Hz, 최대 수신 간격0.384초. 이 조회만으로 시험 시각의 통신 지연 원인을 확정하지 않는다.

사용자 지정 성공 기준과 자동 evaluation_pass:false를 구분해 보관한다. 실제 하드웨어 테스트를 이 백업 작업에서 추가 실행하지 않았다.

## 실행

공통 환경: ROS Humble, robot_ws 설치 환경, ROS_DOMAIN_ID=10, ROS_LOCALHOST_ONLY=0.
기존 드라이버 오류가 잠긴 경우 종료 후 아래 명령으로 재실행한다.

```bash
python3 -B ~/follower_diagnostics/follower_drive_node.py --run-hardware --ack-hardware-zero-rpm-tx --input-kind local_avoidance --ack-bounded-pivot-candidate --ack-continuous20-candidate --enable-motion-input --trial-max-rpm 10 --trial-seconds 360
```

```bash
python3 -B ~/follower_diagnostics/run_local_avoidance_trial.py \
  --run \
  --ack-clear-both-power-cut-ready \
  --apf-pivot \
  --gentle-pivot \
  --map-rejoin \
  --continue-follow-after-rejoin \
  --continuous20 \
  --dynamic-box \
  --post-rejoin-follow-m 2 \
  --pose-source filtered \
  --seconds 360
```

관찰(별도 터미널): `python3 -B ~/follower_diagnostics/watch_follow_alignment.py --seconds 360 --observe-motion`

## 백업 범위

후발 진단 폴더의 코드/설정/문서/테스트 및 convoy_robot 소스, 최신 물리 시험 로그를 저장한다. leader_diagnostics는 이 장치에 없어 선두 장치의 파일은 포함하지 못했다. 선두 실행 기준은 leader_nav_yaw_calibrated.launch.py + 지도/AMCL + leader_bridge_latest_candidate.py다.

백업: `/home/sunuk/follower_diagnostics/backups/quant_target_3_success_20261106`. manifest.json에 파일별 SHA256, ARCHIVE.sha256에 압축 파일 SHA256을 기록한다. 이후 수정은 이 백업과 구분하며 사용자의 새 지시 없이 성공 기준 백업을 덮어쓰지 않는다.
