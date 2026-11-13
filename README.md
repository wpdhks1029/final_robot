# 전체 로봇 성공 구성 백업 — 정량목표1~3

현재 성공 구성: [2026-11-13 스냅샷](snapshots/20261113_success/README.md)

실행 명령: [현재 목표별 실행표](snapshots/20261113_success/RUNBOOK_KO.md)

- 선두 원본: snapshots/20261113_success/leader_home/
- 후발 원본: snapshots/20261113_success/home/
- 목표별 분리 저장소: capstone_goal1, capstone_goal2, capstone_goal3
- 기존 target_1~3 파일과 이전 Git 기록은 보존했습니다.
- 이전 main은 backup-before-20261113 브랜치에도 보존했습니다.

---

## 이전 백업 안내 (보존)

# 최신 사용자 확인 성공 버전 — 새 지도 적용 (2026-11-07)

**현재 평가3 기준은 [target_3/new_map_20261107](target_3/new_map_20261107/README.md)입니다.** 새 지도 YAML/PGM, 새 기준선, 현재 소스·설정, 재출발 RPM 수정, 전체 시험 로그 압축 백업을 포함합니다. 기존 지도 성공 백업은 아래 경로에 그대로 보존했습니다.

최신 시험은 회피·합류 후 FOLLOW까지 진행했고 사용자가 정상 동작을 확인했습니다. 자동 종료는 STOP_APF_WORKSPACE_BOUND, 양쪽 정지 확인 및 모터 오류 없음입니다. 자동 2m 연속 정렬 검증 통과로 표기하지 않습니다.

---

# final_robot — 정량목표 1·3 사용자 확인 성공 기준

2026-11-06 사용자가 지정한 선두·후발 성공 버전 보관 저장소입니다. 각 평가는 독립 실행합니다.

| 평가 | 구성 | 보관 상태 |
|---|---|---|
| 1 | 선두만 Nav2 자율주행, 후발 추종 없음 | 실행 명령·로컬 참고 사본 저장. **최신 선두 원본과 지도 백업은 아직 미확보** |
| 3 | 대열 추종 중 상자 감지 → 곡선 회피 → 지도 경로 합류 → 추종 | 사용자 지정 성공 시점 코드·설정·테스트·로그·SHA256 백업 저장 |

평가2 단순3m 전진은 이 성공 백업 범위에 포함하지 않습니다.

## 파일 구성

- `target_1/reference_copies/`: 후발 장치에 있던 선두 참고 사본. 실제 성공 당시 선두 파일과 동일하다는 검증은 하지 못했습니다.
- `target_1/backup/`: 목표1 실행 기록·참고 사본 원본 백업과 해시.
- `target_3/follower_diagnostics/`: 목표3 성공 지정 시점의 소스·설정·문서·테스트.
- `target_3/robot_ws/`: 목표3 백업에 포함된 convoy_robot 소스.
- `target_3/backup/`: 원본 tar.gz, 파일별 manifest, 압축 파일 해시. 최신 물리 시험 전체 JSONL은 이 압축 파일에 포함됩니다.
- `docs/`: 사용자 확정 평가 구조와 성공 기록.

### 공통 ROS 환경

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

## 평가1 — 선두 단독 Nav2

각각 별도 선두 터미널에서 실행합니다. 기존 동일 노드를 중복 실행하지 않습니다.

```bash
ros2 launch ~/leader_diagnostics/leader_nav_yaw_calibrated.launch.py allow_motion:=true
```

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/sunuk/robot_ws/maps/leader_map.yaml \
  params_file:=/home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml \
  use_sim_time:=false autostart:=true
```

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml \
  use_sim_time:=false autostart:=true use_composition:=False
```

`use_composition:=False`는 대문자 F가 필요합니다. planner_server/controller_server/bt_navigator/velocity_smoother가 모두 active인 것을 사용자 출력으로 확인했습니다. RViz에서 초기 위치와 Nav2 목표를 지정합니다.

**목표1 최신 선두 원본은 접근 제한 때문에 확보하지 못했습니다.** 위 실행 기록과 참고 사본을 최신 원본이라고 간주하지 마세요. 선두에서 별도 생성한 `leader_success_backups/quant_target_1_*/leader_success.tar.gz`와 지도·해시를 추후 추가해야 완전한 복원 자료가 됩니다.

## 평가3 — 후발 동적 상자 회피·합류

성공 기준 백업은 `quant_target_3_success_20261106`입니다. 감지 전방70cm, 계획 여유20cm, 정지10cm, lookahead35cm, 최대10RPM, 합류 후 목표2m, 전체360초. 반복 재계획은 꺼져 있습니다. 센서·위치추정·양쪽 Bridge를 기존 구성으로 실행해야 합니다.

후발 드라이버:

```bash
python3 -B ~/follower_diagnostics/follower_drive_node.py \
  --run-hardware --ack-hardware-zero-rpm-tx \
  --input-kind local_avoidance --ack-bounded-pivot-candidate \
  --ack-continuous20-candidate --enable-motion-input \
  --trial-max-rpm 10 --trial-seconds 360
```

시험:

```bash
python3 -B ~/follower_diagnostics/run_local_avoidance_trial.py \
  --run --ack-clear-both-power-cut-ready \
  --apf-pivot --gentle-pivot --map-rejoin \
  --continue-follow-after-rejoin --continuous20 --dynamic-box \
  --post-rejoin-follow-m 2 --pose-source filtered --seconds 360
```

사용자가 회피·합류를 성공으로 확인했습니다. 최신 로그상 경로 추적 진행량 약2.31m지만, 연속 정렬 검증은 최대 약0.897m로 2m 자동판정은 미통과했습니다. 최종 종료는 LEADER_STALE이며 양쪽 정지를 확인했습니다. `evaluation_pass:false`와 사용자 지정 성공을 구분합니다. 휴대폰 거리 증가가 원인이라는 설명은 사용자 추정이며 확정 진단은 아닙니다.

## 백업 검증과 복원

각 `backup/`의 `ARCHIVE.sha256`으로 압축 파일을 확인할 수 있습니다. `manifest.json`에는 원본 파일별 SHA256이 있습니다. 목표3 압축 파일의 최상위는 `follower_diagnostics/`, `robot_ws/` 등입니다. 복원 시 현재 실행 파일에 곧바로 덮어쓰지 말고 별도 디렉터리에 해제해 비교하세요.

이 저장소는 ROS Humble 시스템 패키지/설치 바이너리, 장치 드라이버와 시리얼 권한, 아직 미확보한 선두 최신 지도·원본 전체를 대체하지 않습니다.
