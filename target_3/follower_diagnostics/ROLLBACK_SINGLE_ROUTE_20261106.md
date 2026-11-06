> 과거 부분 복구 기록입니다. 이후 CURRENT_WORKING_MODE.md의 성공 백업 원본 복원으로 대체됐습니다.

# 추가 장애물 반복 재계획 기능 제거

사용자 요청으로 mid-route 추가 장애물에 대해 계속 재계산하던 실험을 중지했다.
실물 성공 로그 `logs/local_avoidance_trial_1793863618040270489.jsonl` 및 보존 백업과 대조해 주행 흐름을 복구했다. 파일 전체를 과거 버전으로 덮은 것은 아니며, 보정과 센서 수신 수정은 유지한다.

## 제거

- `--persistent-replan` 인수와 `OBSTACLE_WAIT` 운용 상태
- 주행 중 남은 전체 경로 20cm 조건으로 반복 정지하는 추가 검사
- 추가 장애물 대상의 무제한 재계획/5개 스캔 재개 루프
- START 후 비무장 반복 재계획
- 짧은 scan/map 동기 회복마다 가속 램프를 0으로 재설정하고 완전 정지 3회를 기다리던 처리

## 유지

- 기존 곡선 계획 → 실제 map 경로 복귀 → Pure Pursuit 추종
- 앞끝 기준 70cm 감지, 계획 목표 20cm, 현재/예측 정지 최소 기준 10cm, lookahead35cm
- 현재 차체와 1초 예측 이동 영역은 모든 스캔 점으로 계속 검사한다. 막힌 경우 기존 정지/종료 동작이 적용된다.
- 기존의 제한된 근거리 반환 재관측과 초기 주행 중 상자 감지/제한 재계획
- 바퀴 10RPM, 360초, 작업 공간 제한과 source TTL0.8초, 모터 상태/피드백 감시
- raw_linear_feedback_scale1.1474 및 선두 보정. LiDAR/IMU 장착·EKF 파일은 변경하지 않았다.
- 모터 전용 수신, 지속 등록 센서 실행기, 입력 snapshot 시각 순서, 오뎀/IMU 이력 depth10
- 정수 RPM warmup 및 최종 바퀴 상한 처리

현재 `persistent_sensor_executor=true`는 ROS 실행기를 지속 등록한다는 의미이며, 제거한 장애물 반복 대기 기능과 다르다. 운용 결과 `persistent_replan=false`로 표시한다.

변경 전 소스 보존: logs/before_remove_persistent_1793948543259426505.tar.gz
제거한 실험 모듈/테스트 보존: logs/removed_persistent_1793948543259426505/
현재 실행 명령: CURRENT_WORKING_MODE.md
