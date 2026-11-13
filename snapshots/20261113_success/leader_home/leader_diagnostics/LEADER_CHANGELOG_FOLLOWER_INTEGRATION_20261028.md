# 선두로봇 수정 이력·검증 결과와 후발로봇 연동 설계

작성일: 2026-10-28 · 대상: Jetson Orin Nano 선두 / Raspberry Pi 후발 / ROS 2 Humble

이 문서는 2026-10-26~27 선두 실기체 작업에서 무엇을 바꿨고, 왜 바꿨으며, 무엇까지 확인했는지 설명한다. 후발 구현에서 재사용할 원칙과 그대로 복사하면 안 되는 기체별 값을 구분한다. 후발 코드에 관한 내용은 **Jetson에 보관된 통합 소스와 10월 21일 검토용 스냅샷의 정적 분석**이며, Raspberry Pi의 현재 소스와 동일하다고 보장하지 않는다. 이 문서 작성 중 모터 명령·노드 실행·후발 코드 수정은 하지 않았다.

## 1. 현재 결론과 증거 수준

**선두는 Rotation Shim 구성에서 회전 → 목표 주행 → 도착 → 자동 정지를 사용자 관찰로 확인했고, 후발 Pi는 선두 위치·경로를 실제로 수신했다. 후발의 실제 자율 추종까지 완료한 것은 아니다.**

| 구분 | 확인 내용 | 증거/한계 |
|---|---|---|
| 선두 센서 | IMU 약 20Hz, LiDAR 약 12Hz, 실측 TF 및 AMCL 지도 정합 | 실제 토픽/TF 출력과 사용자 관찰 |
| 바퀴·EKF | 무명령 상태 피드백 유지, 정지 중 위치 튐 개선 | 180초 관측 및 주행 로그; 장시간 전체 조건 보장은 아님 |
| 거리·각도 | 40cm 직진 5회 실측 일치, 좌우 45° 각 3회 확인 | 사용자 줄자/각도 측정, 정밀 계측기 오차 범위 미확정 |
| Nav2 | 장애물 우회 관찰, 큰 방향 전환 후 도착·정지 | 사용자 확인; 마지막 시험의 최종 action SUCCEEDED 로그 미수집 |
| 선두 bridge | 정지·주행 중 map 위치·경로 발행, 경로 포즈 증가 | path 1→13, 좌표 순변위 약 1.292m |
| 후발 통신 | Pi에서 /leader/odom, /leader/path 실제 수신 | path 21개 포즈; 후발 자체 위치 추정·모터·추종은 별도 |
| 소프트웨어 회귀 | 원본 파일 140개 해시 확인, 모의 테스트 42개 통과 | 모터/센서 실기체 시험과 다른 검증 |
| 새 저장소 재현 | GitHub 스냅샷 및 실행 안내 작성 | export clean build는 전체 완료 못함; 별도 재현 필요 |

수치가 비슷한 두 추정값을 비교하는 것은 **내부 일관성 확인**이다. 실제 거리 정확도는 줄자·회전 표시점 같은 외부 기준으로 확인해야 한다.

## 2. 최종 성공 구성과 보존 기준

### 2.1 실제 검증에 사용한 구성

```text
MD200T 실제 피드백 ──> /odom ─────────────┐
                                         ├─> EKF ─> /odometry/filtered
IMU ─────────────────> /imu/data ─────────┘             └─> odom → base_link

URDF/RSP ─> base_link → imu_link / laser
LiDAR ────> /scan
저장 지도 + AMCL + scan/odom ─> map → odom

Nav2 planner → Rotation Shim → DWB → velocity smoother → /cmd_vel → MD200T
Nav2 recovery behavior도 /cmd_vel에 직접 발행할 수 있음

map → base_link + /odometry/filtered
  → real_leader_bridge → /leader/odom + /leader/path → 후발 Pi
```

`/leader/path`는 Nav2가 계산한 미래 `/plan`이 아니라 **선두가 실제로 지나온 위치를 샘플링한 이력**이다. 둘을 후발 목표 경로로 혼동하지 않는다.

| 항목 | 최종값/파일 | 해석 |
|---|---|---|
| base 실행 | `leader_diagnostics/leader_nav_015_base.launch.py` | 센서/RSP/EKF/별도 polling 제어기, 기본 구동 잠금 |
| Nav2 | `leader_diagnostics/nav2_015_rotation_shim_check.yaml` | 반드시 이 shim 파일을 사용; 이름이 비슷한 DWB-only 파일과 구분 |
| 모터 wrapper | `leader_diagnostics/md200t_polling_controller.py` | 원래 제어기 상속, 같은 포트에서 피드백 조회 |
| EKF | `leader_diagnostics/leader_polling_constrained_ekf.yaml` | 원래 EKF와 별도 후보 파일 |
| bridge | `robot_ws/src/cnu_capstone_2026/convoy_robot/real_leader_bridge_node.py` | freshness 검사 적용 |
| 모터 | ID 1, 57600bps, M1 왼쪽/M2 오른쪽 | 선두 실기체값 |
| 반지름 | 0.085m, 바퀴 지름 170mm | 실제 치수 |
| 유효 윤거 | 0.466m | 회전 환산용 검증 설정; 실물 폭과 다른 개념 |
| 피드백 배율 | 1.18 | 선두의 경험적 환산값 |
| RPM 상한 | 25 | 명령 제한이며 실제 순간 속도 절대 제한 아님 |
| 직진/회전 | 최대 명령 0.15m/s, 0.10rad/s | 실제 속도 계측·제동거리 검증은 미완료 |
| 차체 외곽 | 폭 0.60m, 길이 0.54m, 축에서 앞뒤 0.27m | costmap 반경 0.42m |
| laser TF | xyz=(0.26,0,0.24)m, yaw=π | 실측 위치, 이전 수평 방향 유지 |
| imu TF | xyz=(0,0,0.15)m, 회전 0 | 현재 설정, 후발은 독립 측정 |
| ROS | Domain 10, localhost only 0, 실제 시간 | 장치별 시계 동기는 별도 |

위치는 `base_link`의 x/y 기준이 바퀴축 중앙이다. 현재 URDF의 z 기준은 원래 휠 반지름과 축높이를 같게 둔 지면 기준 구조다. `laser.z=0.24`는 바닥에서 스캔 중심까지의 높이로 설정했다. `base_link`를 바퀴축 높이에 두는 다른 로봇에 0.24를 그대로 복사하면 안 된다.

### 2.2 보존·게시 상태

- 2026-09-06 성공 archive: `~/robot_ws_LEADER_NAV_SUCCESS_20260906.tar.gz`.
- archive SHA256: `1b95a7749e6048e59772b37aca515d5ce62eb5327fb5abe5245955df67c29c5e`.
- 보호 Git 기준: `~/leader_nav_git_backup/repo`, commit `4fe1b49fe9762121c36081500328fa88b7adb766`.
- 새 선두 성공 스냅샷: <https://github.com/wpdhks1029/capstone_leader>.
- 게시 commit: `ec89638be6cfef173b23e3b49448d2d98e6ee56a`.
- export 원본 해시: 저장소 `provenance/SOURCE_MANIFEST.json`.
- export helper/추출 bridge 패키징은 새로 작성한 포팅이며, 원래 실기체 실행과 구분한다. 새 checkout에서 `convoy_robot`, `iahrs_ros2` 빌드는 완료했지만 `leader_robot_control` setup 지연으로 전체 빌드는 중단했다. 전체 재현 성공으로 기록하지 않았다.

성공 archive/보호 clone은 수정하지 않았다. 현재 live 소스와 진단 파일에만 정정을 적용했고 변경 전 복사본은 `~/leader_config_backups/`에 있다.

## 3. 수정 이력: 증상 → 근거 → 수정 → 결과

### 3.1 IMU 포트 파라미터 이름과 장치 고정

**증상:** launch에서 IMU 포트를 지정해도 드라이버가 다른 기본 장치를 사용할 가능성이 있었다.

**근거:** iAHRS 노드가 선언하는 키는 `port`인데 기존 bringup은 `serial_port`를 전달했다. USB 번호는 연결 순서에 따라 달라질 수 있다.

**수정:** `leader_robot_bringup.launch.py`에서 IMU 키를 `serial_port → port`로 정정. 진단 launch도 실제 by-id 장치를 사용했다.

**결과:** iAHRS 연결, `/imu/data`의 `imu_link`, 약20Hz 발행을 확인. LiDAR는 별도 CP2102 by-id, 256000bps에서 정상 scan을 확인.

**후발 교훈:** launch 변수 이름과 노드가 실제로 선언하는 파라미터를 대조한다. by-id까지 같은 문자열을 복사하지 말고 후발 장치에서 다시 찾는다. 이 세션에서 IMU 센서 파싱/단위 변환 코드 자체를 새로 고친 것은 아니다. 기존 종료 시 이중 shutdown 오류는 남아 있다.

### 3.2 MD200T: 무명령 구간의 실제 RPM 피드백 공백

**증상:** 시작 직후 0RPM 응답은 받지만 이후 `RPM FEEDBACK STALE - ODOM INTEGRATION DISABLED`가 반복됐다. `/odom`이20Hz로 보이는 것만으로 정상 측정이라고 판단할 수 없었다.

**근거:** 원래 제어기는 명령 송신 시 응답을 읽고, 무명령 때는 정기 조회하지 않았다. 명령이 끊긴 상태에서 피드백 나이가 0.3초를 넘으면 적분을 금지했다. 출력 주기와 실제 센서 갱신 주기는 달랐다.

시리얼 읽기 전용 조회에서 VERSION/전압/RETURN TYPE 체크섬이 정상이었고 PID216 직접 조회도 성공했다. 같은 연결에서 0RPM 송신에 대한 자동응답도 확인했다. 초기 무응답 한 차례의 원인은 확정하지 않았다.

**수정:** 별도 `PollingMD200TController`에서 같은 시리얼 연결로 PID4를 이용해 PID216을10Hz 조회한다. 포트 소유자와 executor는 하나만 둔다. 새 데이터가 없으면 `last_feedback_time`을 갱신하지 않는다. 관측 없이 실제0RPM을 만들어 넣지 않는다.

- PID4 조회 예: `b7 b8 01 04 01 d8 b3`.
- 피드백 응답: 20byte 프레임, payload14byte.
- M1 signed16: payload[0:2], M2 signed16: payload[7:9].
- 같은 시리얼에서 조회와 명령을 직렬화한다. 외부 조회 스크립트를 동시에 실행하지 않는다.

**결과:** 수정 후 정지180초에서 stale 표본0, 실제 RPM 수신 유지, raw/EKF 위치 변화0 확인.

**후발 교훈:** 통신계층이 실제 측정값을 지속 공급하는지 먼저 확인한다. EKF 튜닝으로 입력 공백을 숨기지 않는다. baud/ID/프레임/timeout은 후발 드라이버와 장치에 맞춰 재검증한다.

### 3.3 EKF 위치 튐: 노이즈 추정에서 재생 비교로 원인 좁히기

**증상:** 바퀴를 들어 약6cm 전진 회전에 해당하는 시험에서 raw는0.0585m인데 EKF 변위가 약7.5m로 표시되는 등 큰 보정이 발생했다.

**초기 가설:** LiDAR 근처 IMU의 진동 가능성을 검토했다. 그러나 IMU는 yaw/yaw-rate만 융합 중이었고, 기록에서 큰 각도 스파이크는 확인하지 못했다. 가속도 데이터가 EKF 선택 비트에서 꺼져 있으므로 선형가속도 진동을 그대로 위치 적분한다고 설명하면 틀린다.

**분석:** rosbag을 읽기 전용 분석한 뒤 격리 Domain217에서 `/odom`, `/imu/data`, `/tf_static`만 software replay했다. 기록된 `/cmd_vel`이나 원래 `/tf`, 원래 filtered 출력은 재생하지 않았다.

첫 재생은 `/tf_static` QoS 불일치로 고정TF가 없어 비교에서 제외했다. transient-local QoS와 실제 고정TF 수신, IMU 각응답을 확인한 후 비교했다.

| 같은 raw x변위 +0.0584965m 입력 | EKF x변위 | 판정 |
|---|---:|---|
| baseline, 긴 무피드백 초기 구간 포함 | -0.0857922m | 새 EKF 하나만 띄워도 충분하지 않음 |
| fresh vx 분산만0→0.0004 | -0.0870884m | 이 단일 값 변경만으로 해결 안 됨 |
| vy=0 제약만 추가 | -0.0857922m | 측방 불확실성은 줄지만 전방 보정은 남음 |
| 초기 무피드백 구간을 줄인 입력 | +0.0587016m | 이 데이터에서 일관성 크게 개선 |

실시간 원래 기록의 사전 위치 분산은 Pxx 약1.86e7, Pyy 약1.16e10이었다. 비교 재생에서도 긴 초기 구간의 불확실성이 컸다. **초기 실제 wheel 측정 부재와 불확실성 누적이 중요한 유발 조건이라는 증거**이며, EKF 내부 gain의 정확한 수학적 원인까지 확정한 것은 아니다. LiDAR 위치 변경만으로 해결됐다고 결론내리지 않았다.

### 3.4 EKF: polling + vy=0 비홀로노믹 제약 결합

**수정 파일:** `leader_polling_constrained_ekf.yaml`, wrapper의 `LateralConstraintPublisher`.

원래 EKF 대비 의미상 설정 변경은 `odom0_config`의 **vy 선택 추가**다. YAML 서식 차이는 동작 변경이 아니다.

```text
상태 순서 = x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az
wheel 입력: vx + vy=0
IMU 입력: yaw + yaw-rate(z)
frequency=30Hz, two_d_mode=true, world_frame=odom, publish_tf=true
imu0_relative=true
```

wrapper는 원래 raw odometry의 `twist.linear.y=0`을 확인하고 `twist.covariance[7]=0.0004`를 부여한다. 이것은 측방 센서 실측이 아닌 **미끄럼이 작다는 차동구동 모델 제약**이다. vx 분산을0.0004로 바꾼 실험과 혼동하지 않는다.

**결과:**

- polling만 적용: 30초 정지 위치는0이지만 Pyy가 약27.65→29718로 증가.
- polling+vy제약: 30초 정지 위치0, xy분산 약0.39→1.89 수준.
- 긴 대기 후180초: raw/EKF 위치 변화·최대 위치 step0, stale0. xy분산은 약20.24→29.16으로 증가했다. 분산0 또는 절대위치 관측이 생긴 것은 아니다.
- 직후 lifted pulse: raw0.0619m, EKF0.0613m. 정지 확인.

**해석:** 같은 기록과 현재 시험조건에서 위치 튐을 억제했다. EKF가 스스로 절대 위치를 알아낸 것이 아니고, raw/EKF 일치는 실제 지상 거리 정확도 증명이 아니다.

**후발 적용:** wheel 입력 연속성, timestamp, 공분산, 축방향, IMU 단위를 확인한 다음 적용한다. 측방 미끄럼이 큰 바닥에 분산0.0004를 무조건 복사하지 않는다. 후발 snapshot에는 wheel yaw-rate와 IMU yaw-rate를 함께 쓰는 구성이 있지만, 두 소스 병용 자체가 항상 오류는 아니다. 오차 상관·신뢰도·bias·공분산을 검증해 결정한다.

### 3.5 바퀴 치수와 경험적 거리 보정 분리

| 단계 | 설정/관측 | 결정 |
|---|---|---|
| 초기 | 반지름0.075m, 약20cm 추정 이동에 실측27cm 반복 | 실물 치수 재확인 |
| 실물 정정 | 사용자 확인 지름170mm → 반지름0.085m | 현재 기본값/bringup/진단 launch 정정 |
| 정정 후 | 약20.79cm 추정에 실측23cm, 3회 비슷한 결과 | 치수를 다시 가짜 값으로 바꾸지 않음 |
| 긴 시험 | raw40.86cm/EKF40.85cm, 실측47cm | 비례 오차 가능성 검토 |
| 회전 표시점 | inferred273.7°, 실제 약305~306° | RPM→이동 환산 차이 추가 근거 |
| 배율1.12 | 40cm 목표에 실측약43cm | 추가 조정 |
| 배율1.18 | 40cm 실측5회 모두40cm | 현재 경험값으로 유지 |

정지 후 전체 bag에서도 바퀴속도 적분과 위치 차이는 약0.7mm였다. 정지 명령 이후 raw추가이동은 약4.9mm였다. 기록상 몇 cm 차이를 단순히 정지 적분 누락으로 설명하기 어려웠다. 다만 실제각도·실측거리의 계측 불확실성 때문에 정확한 내부기어비가1.18이라고 입증한 것은 아니다.

**구현:** wrapper `odom_callback` 안에서 피드백 RPM에만1.18을 곱해 원래 적분기를 호출하고, `finally`에서 원래 RPM을 복원한다. 단일 executor 전제다. 실제 반지름과 송신 RPM 계산식은 유지한다.

```text
기본 바퀴 속도 = RPM × 2πr / 60
최종 피드백 환산 = 기본 바퀴 속도 × 1.18
v = (vR + vL) / 2
w = (vR - vL) / 유효윤거
```

**한계:** 피드백 쪽만 보정했으므로 cmd_vel0.15가 실제0.15m/s라는 보장은 없다. 양쪽 동일배율은 회전속도에도 영향을 준다. 이후 좌우45° 시험은 했지만 모든속도/회전각/바닥에서 검증한 것은 아니다.

**후발 교훈:** 반지름·유효윤거·피드백배율을 독립적으로 기록한다. 선두의0.085/0.466/1.18을 후발에 복사하지 않는다. 좌우 및 전후 방향, 여러 거리·속도에서 독립 실측한다.

### 3.6 LiDAR 실장 TF, 차체 충돌 영역, 시각화 구분

LiDAR를 옮긴 뒤 기존 TF `(0.15,0,0.22)`를 계속 쓰면 스캔과 지도 정합이 잘못될 수 있었다. 실측으로 `(0.26,0,0.24)`로 정정하고 yawπ는 이전과 동일함을 사용자에게 확인했다. 이후 지도 벽/scan 겹침을 확인했다.

차체 외곽을 폭60cm·길이54cm로 확인하고, 기준축이 전후 중앙임을 확인했다. 중심에서 모서리까지 약0.4036m이므로 임시0.42m 반경을 유지했다. 좁은 통로에서 실제 직사각형보다 보수적일 수 있다.

원래 URDF의 **바퀴 시각화** 반지름/높이에는0.075m가 남아 있다. 실제 제어 반지름0.085m와 구분하며, RViz 모양만으로 제어값을 판단하지 않는다. 후발 설계 시 visual·collision·제어 치수·센서 TF를 함께 정합시키는 것이 바람직하다.

### 3.7 Nav2: 느린 회전과 복구 제한시간 불일치

초기 저속 시험에서 회전 상한을0.10rad/s로 낮췄지만 복구 Spin90°의 제한은10초였다. 최소필요시간1.57/0.10≈15.7초보다 짧아 회전 복구가 실패했다.

별도 BT `navigate_015_recovery.xml`에서 Spin allowance를45초로 변경했다. 진행검사도20→45초(기준거리2cm 유지), BT tick10→20ms, action응답대기20→200ms, DWB debug 출력을 껐다.

**결과/한계:** 시간 모순은 해결했지만 이후에도 출발 불능이 남았다. timeout을 늘리는 것은 원인 해결의 대체가 아니다. `Passing new path`가약1초마다 나오는 상황을 경로 계산 정지로 해석하면 안 된다. 실제 BT는1Hz 재계획한다. planner의 expected20Hz 경고만으로 연산병목을 단정하지 않았다.

진행검사는 현재 평면 위치 이동 기준이므로 제자리회전이 길면 시간이 지나 실패할 수 있다. yaw 진행을 반영하는 checker 또는 상태별 제한은 향후 개선 후보이며 현재 구현했다고 주장하지 않는다.

### 3.8 Nav2: 정수 RPM 해상도와 저속 회전 불능

| 원인 분석 단계 | 실제 로그 | 수정/결과 |
|---|---|---|
| 작은 angular명령 | 0.005rad/s → 양쪽0RPM, 한구간699회연속 | 정수 RPM 반올림으로 실제 회전0 확인 |
| 최소후보0.03 | ±1RPM송신, 응답대부분0, 부호 반복 | 비영 명령이라도 실제 구동 보장 아님 |
| 최소후보0.08 | 이전45°시험속도 사용, 일부 방향 출발 개선 | 큰 방향 전환 불능은 남음 |

최종 DWB 후보값: min_speed_xy0.01, min_speed_theta0.08, max_theta0.10, x가감속±0.2, theta가감속±1.6, theta샘플21. smoother 가감속도 같은 크기로 맞췄다. 최소속도 파라미터는 모든 cmd_vel을 강제로 띄우는 하드웨어 floor가 아니라 **DWB 후보 유효성 조건**이다. accel을 함께 조정하지 않으면 한 제어주기 안에 유효 최소후보에 못 도달하는 상태가 생길 수 있다.

후발 모터에도 독립적으로 최소지속회전 RPM, 기동시간, 부호반전, 정지·유격을 측정해야 한다. 단순히 큰 최소명령을 넣는 방식은 과도한 회전/채터링을 만들 수 있으므로 상한·정지우선·실측을 함께 검증한다.

### 3.9 Nav2: 큰 방향차는 먼저 회전하고 주행

45°목표는 되지만180°방향목표에서 반복움찔/출발불능이 남아, 설치된 `nav2_rotation_shim_controller`를 확인하고 별도 YAML을 만들었다.

```yaml
plugin: nav2_rotation_shim_controller::RotationShimController
primary_controller: dwb_core::DWBLocalPlanner
angular_dist_threshold: 0.50
angular_disengage_threshold: 0.20
forward_sampling_distance: 0.25
rotate_to_heading_angular_vel: 0.10
max_angular_accel: 1.6
simulate_ahead_time: 1.0
rotate_to_goal_heading: false
```

경로 방향과 크게 다르면 먼저 방향을 정렬하고 DWB로 전환한다. 마지막 goal방향은 기존 제어/goal checker에 맡기는 설정이다. 회전 충돌 검사는 유지한다.

한 차례 기존 DWB YAML로 실행한 것을 로그의 `Created controller ... DWBLocalPlanner`에서 확인했다. **파일 생성과 실제 적용은 다르다.** 이후 실제 `FollowPath.plugin`이 RotationShimController인 구성을 적용한 뒤 사용자가 버벅거림 해소와 반대방향 회전→이동→도착→자동정지를 확인했다.

후발 Pure Pursuit에도 큰 목표 방향 차를 곧바로 전진 곡률명령으로 처리하지 않고, 정지·방향정렬·주행 상태를 구분하는 원리를 참고할 수 있다. Nav2 plugin을 쓰지 않는 Pure Pursuit에 이 YAML을 붙이는 것으로 기능이 생기지는 않는다.

### 3.10 leader bridge: 좌표계·설치본·입력 freshness

설치본 bridge YAML이 source와 달리 `map_frame=odom`이던 것을 `map`으로 맞췄다. 선두의 공통 지도 위치와 filtered twist를 묶어 발행하는 목적에 맞춘 것이다.

이전에는 cached TF/odom으로 계속 현재시간 메시지를 만들어 후발이 새 측정으로 오인할 수 있었다. 최종 bridge는 다음을 확인한다.

- 실제 filtered odom을 받아야 발행.
- TF와 odom stamp가0이 아니고, 각각0.5초 이내인지 확인.
- 미래 timestamp는100ms까지만 허용. 이후 age판정은 각 장치 시계동기 필요.
- 유효하지 않으면 odom/path 발행을 억제.
- leader odom stamp는 TF 측정시간을 유지.
- path는0.1m 이동마다추가, 최대5000포즈, 약10Hz 재발행.

**결과:** 이전 시험에서 EKF 중단→bridge 발행 중단, 재시작→복구 확인. 최종보정/주행구성에서는 실제경로1→13 누적과 Pi에서path21 수신 확인. 마지막구성의 모든센서손실/복구는 다시 시험해야 한다.

path의 과거 pose stamp가 오래된 것은 이력의 정상 의미다. 반면 현재 상태의 timestamp를 계속 새로 찍어 stale를 숨기면 안 된다. path header가 새로워도 새 경로점이 추가된 뜻은 아니다. 정지 중 path포즈가1개인 것은 정상이다.

## 4. 후발 코드 조사에서 추가로 발견한 연결 조건

### 4.1 현재 Pi 사실과 과거 검토자료의 신뢰도

10월27일 Pi 터미널에서 선두 토픽수신은 성공했다. 당시 `/follower/odom_map`, `/follower/cmd_vel`은 없었고 후발 로컬 ROS/모터프로세스도 없었다. 공용그래프의 `/ekf_filter_node`, `/odom`, `/scan`은 선두 노드일 수 있다.

Jetson의 `cnu_capstone_2026_review/raspberrypi_follower/README.md`는 해당폴더를 **실기기 복사본이 아니라 대화/GitHub/통합기록을 재구성한 검토스냅샷**이라고 명시한다. `current_live_snapshot`이라는 이름이 있어도 최신Pi 원본으로 간주하지 않는다.

과거 런타임 자료의 후발값은19200bps, 반지름0.065m, 윤거0.560m, invert_left=true였다. 이 값 역시 **현재Pi에서 재확인할 참고기록**이며 최종설계치가 아니다.

### 4.2 서로 다른 두 후발 아키텍처

| 항목 | V2 검토용 코드 | 통합형 reference 코드 |
|---|---|---|
| 입력경로 | `/leader/path`, frame `leader_odom`만 허용 | `/leader/path`, 기대frame `map` |
| 자체위치 | `/odometry/filtered`, frame `odom` | `/follower/odom_map`, common map |
| 좌표변환 | 고정phi82.262°,tx0.0585,ty0.9983 | TF `map → follower/base_link` |
| 출력 | `/follower/cmd_vel_test` | `/follower/cmd_vel_raw → APF → /follower/cmd_vel` |
| 모터연결 | 의도적으로 분리된 시험토픽 | 실제연결 후보지만 safety검토 필요 |
| freshness | 수신시각 기반 약1초 watchdog | bridge/PP/APF에 누락사항 확인 |

현재 선두 `/leader/path.header.frame_id=map`이므로 검토용 V2는 이를 거부한다. 조건문을 `leader_odom → map`으로만 바꾸고 예전 고정변환을 유지하면 수학적으로 다른 좌표계를 잘못 섞을 수 있다. reference의고정phi/tx/ty는 당시 초기배치값이지 현재재사용값이 아니다.

### 4.3 권장 인터페이스와 TF 소유권

```text
공유 지도 frame: map

선두:
map --AMCL--> odom --선두 EKF--> base_link --RSP--> laser / imu_link

후발(제안):
map --후발 localization 또는 검증된 정렬--> follower/odom
    --후발 EKF--> follower/base_link --후발 RSP--> follower/laser / follower/imu_link
```

| 계층 | 권장 토픽/이름 | 소유자 |
|---|---|---|
| 선두 상태·경로 | `/leader/odom`, `/leader/path` | 선두 bridge |
| 후발 wheel raw | `/follower/odom_raw` | 후발 모터 제어기 |
| 후발 IMU/scan | `/follower/imu/data`, `/follower/scan` | 후발 드라이버 |
| 후발 EKF | `/follower/odometry/filtered` | 후발 EKF |
| 후발 공통 지도 상태 | `/follower/odom_map` | 후발 bridge |
| 추종 계산 시험 | `/follower/cmd_vel_test` | 실제 모터와 분리한 추종기 |
| 추종 원시 명령 | `/follower/cmd_vel_raw` | 선택한 단일 추종 제어기 |
| 최종 명령 | `/follower/cmd_vel` | watchdog·충돌·구동 허용 gate를 통과한 단일 출력 |

이는 **제안 계약**이며 후발에 이미 적용된 설정이 아니다.

노드 namespace만 바꿔도 절대 토픽 `/cmd_vel`, `/odom`이 자동 격리되지는 않는다. `/tf` 채널을 공유하더라도 frame id는 달라야 하고, 동일 child의 TF 발행자는 하나여야 한다. 파라미터 YAML의 최상위 노드 이름·namespace도 실제 노드와 일치해야 한다.

후발이 선두와 같은 `/odometry/filtered`를 구독하면 선두 위치를 자기 위치로 오인할 수 있다. 또한 같은 지도를 쓰더라도 후발의 AMCL, `initialpose`, pose 출력, 서비스 이름을 분리해야 한다. 서로 다른 지도를 두 map server가 같은 `/map`으로 발행하지 않도록 한다. frame 문자열을 `map`으로 바꾼 것만으로 실제 위치 정합이 이루어지지는 않는다.

### 4.4 P0: 후발 freshness와 최종 정지 gate

정적 분석한 통합형 후발 코드에서 확인한 사항이다.

| 파일 | 현재 코드 동작 | 필요한 검토/개선 |
|---|---|---|
| `real_follower_bridge_node.py` | cached TF를 현재 시간으로 stamp; filtered 입력 없이도 기본 twist 발행 가능 | 측정·수신 시각과 유효성 검사, stale 재포장 금지 |
| `real_pure_pursuit_node.py` | ready flag를 한 번 받으면 유지; 입력 timestamp timeout 없음 | 모든 필수 입력의 freshness, frame, 유한값 검사 |
| 같은 PP | path frame이 달라도 경고 후 사용; odom frame 검사 없음 | 잘못된 좌표는 거부하고 정지 |
| `real_apf_avoidance_node.py` | 한 번 받은 scan·pose·raw 명령을 계속 재사용 가능 | callback 수신 시각과 측정 시각에 기반한 만료 처리 |
| 같은 APF | 장애물 힘이 있으면 raw 명령이 0이어도 angular를 추가할 수 있음 | 추종기의 정지 의도를 회피기가 해제하지 않는 중재 규칙 |

**모터 watchdog만으로 충분하지 않다.** 추종기나 APF가 오래된 데이터를 사용하면서 새 cmd_vel을 계속 발행하면 모터 watchdog는 통신이 정상이라고 판단한다. 입력 유효성 gate와 모터 command watchdog를 각각 두어야 한다.

권장 우선순위는 비상정지/구동 금지 → 입력·TF·좌표계 실패 → 안전거리 위반 → 일반 추종·회피다. 전진·후진·회전 모두에 적용하며, fault 해제와 자동 재출발 정책도 명시한다. 이는 아직 구현하지 않은 개선안이다.

freshness는 monotonic 수신 공백과 동기화된 ROS 측정 시각을 함께 본다. 선두 bridge의 입력이 각각 0.5초 이내라는 것은 TF와 twist가 정확히 같은 시각이라는 보장은 아니다. 빠른 운동에서는 입력 간 시간차 상한이나 동기화도 필요하다. 과거 path pose의 시각과 현재 상태 freshness는 구분한다.

### 4.5 P0: APF의 좌표변환과 선두 제외영역

reference APF는 scan 점을 `follower_pos + r*[cos(scan_angle+yaw), sin(...)]`로 변환한다. **실제 LiDAR의 차체 내 평행이동과 yaw를 적용하지 않는다.** 후발의 scan frame에서 base/map으로 가는 TF를 측정 시각에 적용해야 한다. 센서가 중앙·정방향이 아니면 선두 식별과 회피 방향이 잘못될 수 있다.

또한 선두 중심 반경 0.50m 안의 return을 먼저 제외한 뒤 최소 안전거리를 계산한다. 따라서 가까운 선두나 그 근처의 다른 장애물이 hard-stop 검사에서도 빠질 수 있다. PP가 거리 조절을 담당한다는 사실만으로 이 누락이 안전해지는 것은 아니다.

모든 유효 장애물에 대한 hard-stop을 우선하고, 선두 제외는 신선한 공통 좌표와 실제 외형에 따라 제한하는 방향을 검토한다. APF의 `safety_dist=0.25`는 LiDAR 원점에서 점까지의 거리이지 차체 외곽의 여유거리가 아니다. 후발 크기로 다시 설계한다.

reference APF는 scan을 정수 depth 기본 QoS로 구독한다. 실제 LiDAR가 Best Effort로 발행하면 Reliable 구독과 호환되지 않을 수 있다. `/follower/scan -v`로 확인해, 처음부터 scan을 못 받는 문제와 오래된 scan을 재사용하는 문제를 구분한다.

### 4.6 P1: 선두 회전을 후발이 그대로 모방하면 안 되는 경우

통합 PP는 선두 제자리회전을 먼저 처리하고 선두 각속도의 0.8배를 후발에 준다. 이 분기는 `stop_dist` 검사보다 앞이다. 위치와 주변 장애물이 다른 후발이 선두 yaw를 그대로 따라 돌도록 설계된 셈이다.

후발은 자신의 위치에서 선택한 경로 접선·목표를 향해 회전하는 것을 기본으로 검토한다. 큰 방향차에서는 ALIGN 상태로 들어가 선속도 0, 검증된 최소 회전 명령, 충돌 검사, 시간 상한을 적용한 뒤 TRACK으로 전환한다. 선두 Rotation Shim의 원리는 참고할 수 있지만, Nav2를 쓰지 않는 PP에 YAML만 붙인다고 기능이 생기지는 않는다.

선두 후진 시 후발도 후진시키는 reference 분기도 검토가 필요하다. 거리가 크다는 이유로 후발이 후진하면 일부 배치에서는 간격이 더 커진다. 최초 실추종에서는 전진만 허용하고 선두 후진·제자리회전 시 후발 안전정지를 기본 후보로 두는 단순 정책부터 검증할 수 있다. 정책 제안이지 구현 완료 사항은 아니다.

### 4.7 P1: 경로 이력·재정합·재시작의 세션 관리

- leader bridge에는 2D Pose Estimate 이후 좌표 점프를 자동으로 구분하는 기능이 없다. 과거 경로와 새 좌표가 연결될 수 있다.
- 표준 Path에는 session id가 없다. 재시작·재정합을 명시적으로 전달할 방법이 필요하다.
- reference PP는 전체 경로의 가장 가까운 점을 매번 선택한다. 왕복·교차·U턴에서 과거 구간으로 돌아갈 수 있다.
- 5000점 제한으로 앞부분이 삭제되면 배열 인덱스가 변한다. 인덱스를 영구 식별자로 쓰면 안 된다.

권장 흐름은 위치 정합 → bridge/session 시작이다. 큰 위치 점프 시 정지 → 경로 초기화 → 재정렬 과정을 두고, 누적 경로 거리 기반의 진행도와 session/reset 상태를 설계한다. 후발은 새 session을 확인하기 전 기동하지 않는다. 초기 1.0m 또는 1.5m 간격은 출발 배치값이지 실시간 간격 유지의 증명이 아니다.

### 4.8 P1: 선두/후발 속도·거리 정책의 불일치

현재 선두 직진 명령 상한은 0.15m/s, reference 후발 PP/APF의 `max_linear`는 0.10m/s다. 선두가 계속 더 빠르게 달리면 후발이 간격을 복구할 수 없다. 실제 속도 배율도 기체별로 다르므로 명령값만으로 비교하지 않는다.

최초 연동에서는 선두의 실제 속도를 후발이 여유 있게 따라갈 수 있는 범위로 낮춘다. 후발 최고 속도만 올리는 것을 우선 해법으로 삼지 않는다. 간격은 다음 요소를 포함해 설계한다.

```text
기체 형상 여유 + 위치 오차 + 지연 동안 이동거리 + 상대 제동거리 + 안전 여유
```

평균 수신은 약 10Hz였지만 최대 0.484초 간격이 관측됐다. 샘플 최대값은 최악 지연 보장이 아니다. 단순히 timeout을 0.5초보다 조금 크게 정하지 말고 age 분포·유실률·제동거리·fallback 정책을 함께 결정한다.

reference의 `follow_dist=1.5`, `stop_dist=1.0`은 중심 간 직선거리다. 곡선에서는 경로 호길이 기반 간격·목표 지연도 검토해야 한다. 선두 정지 시 후발도 즉시 멈출지, 잔여 경로를 따라 안전 간격까지 접근할지 정책을 먼저 정한다.

## 5. 후발 제작에 적용할 우선순위

| 우선순위 | 작업 | 완료 판정 |
|---|---|---|
| P0 | 현재 Pi 원본/Git/install/launch 확보 | 재구성본과 실제 차이 기록 |
| P0 | 포트·토픽·TF·노드 격리 | 양쪽 명령이 다른 기체 모터에 전달되지 않음 |
| P0 | 센서/RPM 연속성·EKF·기체 보정 | 정지·재구동·거리·회전과 실제 정지 확인 |
| P0 | 공통 map 또는 검증된 좌표변환 | 두 기체 위치를 같은 실측 기준으로 비교 가능 |
| P0 | 모든 필수 입력의 freshness와 최종 gate | 각각 끊어도 기한 내 0 출력·실제 정지 |
| P0 | APF scan TF·hard-stop·선두 mask 검토 | 가까운 선두/장애물이 정지 조건에서 빠지지 않음 |
| P1 | 모터와 분리한 추종 계산 | 시험 토픽에서 직선·곡선·큰 각도·재시작 검증 |
| P1 | 경로 session·진행도·왕복 처리 | 교차·U턴에서 과거 경로로 복귀하지 않음 |
| P1 | 실제 속도·제동·간격 설계 | 선두 속도에서 간격 오차가 계속 증가하지 않음 |
| P1 | 제한된 실제 추종 | 저속 직선→완만한 곡선→방향 전환을 반복 검증 |
| P2 | 재부팅 재현·정식 launch·로그 | 같은 드라이버와 파라미터로 새 세션 재현 |

## 6. 권장 후발 시험 순서와 기록 양식

아래는 시험 설계이며 자동 실행 명령이 아니다. 각 단계에서 정지 수단과 포트·토픽 격리를 먼저 확인한다.

1. **무구동 조사:** 설치 원본과 review 비교, 그래프·포트·frame·QoS·시간 동기 기록.
2. **센서·고정 TF:** 실제 축과 장착 치수, scan/IMU 단위 확인. 아직 모터를 연결하지 않음.
3. **단독 모터:** 바퀴를 들고 좌우·부호·0 명령·통신 유실 정지 검증.
4. **단독 EKF:** 연속 피드백 상태에서 장기 정지→짧은 구동→재정지, covariance·age·점프 기록.
5. **기체 보정:** 반지름 확인, 외부 거리·각도 측정, 명령 속도와 실제 속도 비교.
6. **좌표 정합:** 후발 위치를 공통 map에 올리고 실제 선후발 간격·방향과 비교.
7. **추종 계산 시험:** 모터와 분리한 출력에서 경로 진행·정지·ALIGN을 검증.
8. **고장 주입:** bridge/EKF/scan/PP/통신 중단, 시계 불일치, 경로 reset 각각의 정지 검증. 소프트웨어와 바퀴를 든 상태부터 수행.
9. **실제 추종:** 선두 저속·짧은 경로·충분한 간격에서 시작하고 조건을 하나씩 확장.

기록할 최소 항목:

```text
날짜 / 기체 / 코드 commit / 실제 실행 경로:
wheel radius / track / scale / min-max RPM:
localization·TF 소유자와 frame:
명령 토픽 publisher/subscriber, QoS:
시험 거리·각도·속도·바닥·초기 간격:
외부 실측 vs raw/EKF/map 변화:
입력별 age / 수신 간격 / 유실 / 최대 pose step:
정지 유발 조건 → 0 명령 시각 → 실제 정지 시각·거리:
결과: 성공 / 실패 / 미검증:
```

합격 임계값은 제동·센서 정확도를 측정해 시험 전에 명시한다. “코드에 stop이 있다”, “topic hz가 나온다”, “EKF와 raw가 같다”만으로 실제 동작 검증을 대체하지 않는다.

## 7. 아직 남아 있는 선두 자체의 제한

1. 수신 jitter 원인과 입력 손실 시 전체 chain의 정지는 최종 구성에서 추가 검증해야 한다.
2. pose covariance가 0인 bridge 출력을 절대 정확한 위치 측정으로 융합하지 않는다. map pose와 body twist의 공분산 모델이 필요하다.
3. 원래 MD의 stale covariance는 [0],[5],[35]에 1e6을 쓰는 비대칭 표기가 남아 있다. 대각 원소·대칭성·양의 준정부호 조건을 검토해야 하며, 성공 snapshot에서 임의 변경하지 않았다.
4. IMU 종료 시 이중 shutdown, URDF wheel visual 치수, source/install 불일치 재발 가능성은 유지보수 항목이다.
5. wrapper의 배율 처리는 단일 executor와 callback 중 원시값 복원에 의존한다. 정식 패키지에서는 명시적인 wheel 변환 계층으로 분리하는 것을 검토한다.
6. controller 출력은 smoother를 지나지만 설치된 Nav2 launch에서 behavior 출력은 `/cmd_vel`에 직접 발행할 수 있다. 현재 BackUp은 0.05m/s 후진을 요청한다. smoother의 min_x=0만 보고 후진 금지라고 판단하면 안 된다. 최종 gate에서 모든 명령을 중재하는 정책이 필요하다.
7. export에는 출처 보존용 legacy bringup도 있다. 실제 plugin·params·TF를 확인해야 하며 새 checkout 전체 clean build·재부팅 재현도 남아 있다.

## 8. 근거 파일과 추적 방법

### 성공 저장소(고정 기준 commit)

- 저장소: <https://github.com/wpdhks1029/capstone_leader/tree/ec89638be6cfef173b23e3b49448d2d98e6ee56a>
- `diagnostics/md200t_polling_controller.py`
- `diagnostics/leader_polling_constrained_ekf.yaml`
- `diagnostics/leader_nav_015_base.launch.py`
- `diagnostics/nav2_015_rotation_shim_check.yaml`
- `diagnostics/navigate_015_recovery.xml`
- `src/convoy_robot/convoy_robot/real_leader_bridge_node.py`
- `src/leader_robot_control/urdf/leader_robot.urdf.xacro`
- `docs/VALIDATION.md`, `provenance/SOURCE_MANIFEST.json`

### Jetson 조사 자료(로컬)

- `~/leader_diagnostics/CURRENT_STATUS_20261026.md` — 누적 기록이므로앞부분의과거미검증표시는최신최종결과와구분한다.
- `~/leader_diagnostics/reports/ekf_comparison_20261026_191925_573468/ANALYSIS.md`：software replay비교.
- `~/leader_diagnostics/reports/ground_distance_review_20261026.md`：거리실측vs추정.
- `~/leader_diagnostics/reports/nav2_deadzone_20261027.md`：정수RPM0 분석.
- `~/.ros/log/python3_5148_1793082082169.log`：angular0.005→RPM0 및±1RPM반응구간.
- `~/.ros/log/controller_server_6426_1793082962169.log`：잘못실행한DWB plugin확인.
- `~/robot_ws/src/cnu_capstone_2026/convoy_robot/real_follower_bridge_node.py`
- `~/robot_ws/src/cnu_capstone_2026/convoy_robot/real_pure_pursuit_node.py`
- `~/robot_ws/src/cnu_capstone_2026/convoy_robot/real_apf_avoidance_node.py`
- `~/robot_ws/src/cnu_capstone_2026/config/real_follower_control.yaml`
- `~/cnu_capstone_2026_review/raspberrypi_follower/README.md`
- `~/cnu_capstone_2026_review/raspberrypi_follower/docs/CURRENT_STATE_20261021.md`
- `~/cnu_capstone_2026_review/raspberrypi_follower/current_live_snapshot/test_nodes/pure_pursuit_v2.py`
- `~/cnu_capstone_2026_review/raspberrypi_follower/candidate_configs/follower_ekf.yaml`

후발 작업 인수자는 먼저 실제 Pi 파일을확보하고이문서의reference와 diff한다. 이문서에정리한개선안은코드에이미반영된사실과명확히구분하여진행한다.
