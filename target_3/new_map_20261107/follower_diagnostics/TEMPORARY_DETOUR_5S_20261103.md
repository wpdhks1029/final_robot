# 현재 장면 임시 회피 경로: 첫 5초 연결

현재 장면과 상자 위치를 유지한다. 최근 shadow 18개 정상 구간 모두 경로 여유/입력 시각 통과. 경로 여유 0.32299m는 관측점에 대한 보수적 하한이며 미관측 공간이나 실물 통과를 보증하지 않는다.

새 제어: follower_temporary_detour_preview.py. 명시적인 --run --ack-5s-only 필요. 자체 arm/서비스 호출 없음. 기존 Gateway와 모터 드라이버를 사용한다. 기존 APF 실행을 종료해야 한다. 기존 모터 드라이버를 5초 설정으로 다시 시작한다. 다른 센서/위치 추정/브릿지/Gateway는 유지한다.

- 기록된 current_scene_left_detour_plan.json을 사용하며 SHA256으로 시험 도구와 일치 여부 확인.
- 원래 선두 정지/역진/회전/거리/입력 유효성 판정은 공유하고, 조향만 임시 경로 Pure Pursuit로 계산.
- 원본/수신 시각 0.3초, 원격 시계 여유 0.05초 차감, 계산 후 재검사. 재발행 시 원본 stamp 보존.
- 실제 스캔 시각의 map→base TF와 장착 TF로 모든 유효점을 변환. 점 제외/마스크 없음.
- 새 스캔마다 남은 전체 경로의 회전 차체 여유 검사. 현재 전 방향 0.30m 정지/0.60m 감속 및 명령의 1초 회전 차체 검사.
- 10RPM, 전진 바퀴 조건, 초기 위치 5cm/5도 이내, 위치 급변 차단.
- 모터 드라이버 5초 설정 필수. 운용 도구 최대 4.5초, 제어 노드 무장 관찰 후 4.75초, 드라이버 자체 5초 제한. 한 번 종료한 제어 노드는 재시작 전 재시험 불가.
- 첫 시험 선두 0.04m/s 직진 고정. 가상 시뮬레이션의 선두 협조 감속은 아직 실물에 연결하지 않음.
- 이 시험은 조향 시작/정지 확인이다. 장애물 전체 통과, 재합류+1m, Nav2 평가3 완료가 아님. 결과 evaluation_pass=false 유지.

실행 순서:

1. 후발 기존 모터 드라이버 터미널 Ctrl+C 후:

```bash
python3 -B ~/follower_diagnostics/follower_drive_node.py \
  --run-hardware --ack-hardware-zero-rpm-tx \
  --input-kind guard --enable-motion-input \
  --trial-max-rpm 10 --trial-seconds 5
```

2. 후발 기존 Preview/APF 터미널 Ctrl+C 후:

```bash
python3 -B ~/follower_diagnostics/follower_temporary_detour_preview.py \
  --run --ack-5s-only
```

3. 별도 후발 터미널:

```bash
python3 -B ~/follower_diagnostics/run_joint_follow_trial.py \
  --run --ack-clear-both-power-cut-ready \
  --temporary-detour --trial-seconds 5 \
  --leader-speed 0.04 --leader-turn straight --trace-timing
```

START 입력 후 최대 4.5초 후발 조향/양쪽 정지 관찰. 정지 상태에서는 STOP_LEADER_STOPPED 또는 경로 1점에 따른 PATH_MISSING이 가능하다. 원본 입력 유효 전진 조건이 생겨야 arm한다. 초기 위치 불일치/경로 막힘/시간 만료가 나오면 통과로 간주하지 않는다.

검증: 변경 관련 기존 회귀와 신규 제어 검사 172개 통과. 추가된 새 운용 흐름 3건을 포함한 제어 검사 12개 통과. 실제 ROS 네트워크/직렬 포트/모터를 통한 새 제어의 실물 실행은 수행하지 않았다.

별도 확인 사항: test_pursuit_geometry는 이번 경로에서 사용하지 않는 기존 production pursuit 노드(AST harness)의 후방 목표 전진 검사 1건 실패. 해당 production 파일은 수정하지 않았다. 새로운 임시 경로 노드의 앞쪽 목표/단조 진행 검사 및 기존 preview 후방 목표 차단은 별도 검사한다.
