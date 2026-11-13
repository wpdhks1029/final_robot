# 평가 3: 입력 단절 시 종료 대신 0속도 대기·자동 재개

기존 실물 동작 버전은 evaluation3_clearance10_stop05와
backups/eval3_user_confirmed_1794561585604490550에 보존한다.
이 폴더는 새 후보이며 실제 기체에서 아직 검증하지 않았다.

## 동작

- 선두 위치/경로 또는 지도 기준 후발 위치가 누락·지연되면 양쪽에 속도0을 요청하고
  최신 로컬 센서에 기반한 후발 STOP 명령을 계속 보낸다. 같은 arm·epoch·평가 단계를 유지한다.
- 입력 대기 시간의 상한은 없다. 대기 중360초가 지나 종료되는 일이 없도록
  이 전용 모드의 총 세션 시간도 0(무제한)으로 지정했다.
- 기존0.8초 원본 입력 검사는 삭제하지 않는다. 오래된 입력은 주행에 사용하지 않으며
  송신 전에0.6초 여유선을 적용해 네트워크/처리 지연을 위한0.2초를 남긴다.
- 전송 중 원본시각이0.8초를 넘긴 정상 현재세션 명령은 모터 드라이버가 주행하지 않고
  0속도로 변환한다. 요청의 원본시각은 last_reference_hold에 그대로 기록한다.
- 새 입력이1초 안정되고 각 필요한 원본시각이3회 이상 갱신되며 양쪽 정지가 확인되면
  동일 세션에서 가속 램프를0부터 시작해 재개한다. 지난 명령을 재생하지 않는다.
- 대기 시간은 합류 후 추종 확인/경로계산 단계의 운영 타이머에서 제외한다.
  계산 작업 자체60초 예산, 근접 재관측 한도, 잘못된 경로·지도 점프 검사는 유지한다.
- 로컬 오뎀·IMU·스캔·모터 피드백은 계속 유효해야 한다. 드라이버 명령/피드백/응답
  watchdog0.3초와 엔코더/기기 오류 잠금은 유지한다. 이러한 고장은 자동 재무장하지 않는다.
- 계획 여유10cm, 정지 기준5cm, 최대10RPM·0.025m/s는 유지한다.

## 현재 공개 TF 복구 구성에서 실행

후발 센서·EKF·AMCL·Bridge는 success_tf_recovery/start_localization.bash 구성으로 유지한다.
기존 평가 운영 프로그램과 후발 모터 드라이버만 Ctrl+C로 중지하고 새 드라이버 실행:

```bash
bash ~/nav2_follow_stabilization/evaluation3_reference_wait/start_follower_driver.bash
```

후발 초기 위치를 RViz /follower/initialpose로 다시 지정하고 양쪽 정지·정렬 상태에서:

```bash
bash ~/nav2_follow_stabilization/evaluation3_reference_wait/capture_reference.bash
```

REFERENCE_JSON으로 성공한 후:

```bash
bash ~/nav2_follow_stabilization/success_tf_recovery/start_evaluation3_wait.bash
```

START + Enter로 시험 시작. [WAIT]는 프로그램 종료가 아닌0속도 대기,
[RESUME]는 신선한 입력·정지 확인 후 재개다. Ctrl+C는 양쪽 정지·후발 disarm을 요청하고 종료한다.
정량목표3에서는 선두 Nav2/gate와 정량목표1 일반 추종·teleop을 함께 실행하지 않는다.

새 프로필 CONTINUOUS10_STOP05_WAIT_V1을 양쪽 프로그램이 확인한다.
예전 드라이버와 혼용하면 출발하지 않는다. 최신 센서가 없을 때 지도 위치를
가짜로 만들거나 이전 메시지를 새 시각으로 바꿔 발행하지 않는다.

## 검증

가짜ROS에서25초 참조 단절→0속도 대기→신선한 원본시각 재수신→동일 arm 재개,
지속 단절 중 종료하지 않음, 대기 중 로컬IMU 오류 정지, 가짜모터에서610초
STOP 유지→재개→disarm, 실제 기록의0.800619845초 만료 명령0속도 변환,
잘못된epoch/미래시각/과속 거절과 명령·피드백 watchdog을 확인한다.
실제 모터 구동은 수행하지 않았다. 자세한 검사 결과는 *_tests.log에 기록한다.
