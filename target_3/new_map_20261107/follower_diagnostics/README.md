# 후발 무구동 관측 후보 — 2026-10-27

**다음 세션은 [HANDOFF_20261027.md](HANDOFF_20261027.md)를 먼저 읽는다.**
해당 문서에 오늘 완료한 실기체 확인과 내일 재개 순서를 저장했다.

## 선두 최종 참조 기준 — 2026-10-28 갱신

- /home/sunuk/LEADER_CHANGELOG_FOLLOWER_INTEGRATION_20261028.md 전체452줄 읽음.
- https://github.com/wpdhks1029/capstone_leader
  main HEAD ec89638be6cfef173b23e3b49448d2d98e6ee56a를 GitHub 연결 도구로 확인.
- 위 고정 commit의 docs/VALIDATION.md 및
  src/convoy_robot/convoy_robot/real_leader_bridge_node.py, 현재 README를 직접 읽었다.
- 선두 성공 운용은 diagnostics/leader_nav_015_base.launch.py 및
  diagnostics/nav2_015_rotation_shim_check.yaml을 기준으로 참고한다.
- 선두 반지름0.085/유효윤거0.466/피드백배율1.18/57600bps는 후발에 복사하지 않는다.
  후발 실측0.065/0.560/19200, 좌측반전true 및 최종 laser yaw0/IMU높이0.16 유지.
- 인터페이스: /leader/odom(map, child base_link), /leader/path(map, 실제 궤적).
  bridge는 TF/filtered stamp 각각0.5초 freshness, 미래100ms 허용, 무효면 발행억제.
  path는0.1m 샘플/최대5000점/약10Hz. 재시작 메모리초기화와 재정합 점프는 별도 관리 필요.
- 후발 /follower/odom_map 및 공통map 정합, freshness/final gate, scan-to-base TF,
  hard-stop 우선 및 경로session 검증을 순서대로 진행한다.
- 선두의 실제 주행 성공 기록과 새 checkout 전체 clean-build/통신유실 안전검증은 구분한다.
  export docs에는 전체 clean build 미완료 및 고장조건 미검증이 명시되어 있다.
- 이번 참조 작업으로 프로젝트/실행 구성 변경, clone/build, 장비 송신은 하지 않았다.

### 현재 재개 지점

수정한 짧은 bond ID 설정으로 재시작 후 사용자 출력에서 후발 map_server와 AMCL
모두 active, 두 bond 연결 및 Managed nodes are active 확인.
한 차례 AMCL configure 응답 송신 timeout으로 inactive에 머문 이력이 있으나
같은 수정 설정의 다음 재시작에서는 성공했다. 해당 일시 오류의 근본 원인은 미확정.
현재 AMCL은 초기 위치 미지정 경고 상태로, 다음 작업은 노트북 RViz
2D Pose Estimate Topic=/follower/initialpose 설정 후 실제 후발 위치/방향 지정이다.
그 후 /follower/amcl_pose 및 map->follower/base_link와 scan/map 정합을 확인한다.
사용자가 실제 지도 위치/방향으로 초기 위치 재지정 후 다음을 확인했다.
/follower/amcl_pose 위치 (2.740248748,0.583726604)m, yaw 약15.631도;
map->follower/base_link TF가 같은 위치/방향으로 수신되며 정지 구간에 유지.
사용자가 RViz에서 실제 벽/모서리와 scan 겹침 확인.
따라서 정지 상태 공통map 위치정합 확인 완료. 이동 중 정확도/반복수렴/장기안정성은 미검증.
초기 TF 수신 중 일시적 past extrapolation 경고 후 정상 출력으로 전환됨.
이 pose 숫자는 해당 세션/배치의 관측값이며 다음 초기위치로 하드코딩하지 않는다.

### 다음: 후발 map bridge 무구동 시험

설치 기본 bridge 대신 기존 real_follower_bridge_tf_candidate.py를 직접 실행하는 후보를 사용한다.
이 후보는 EKF 프레임/값, 수신·측정age 및 TF age를 확인하고 무효일 때 발행하지 않는다.
출력은 map/follower/base_link의 /follower/odom_map. TF/EKF 중 오래된 stamp를 유지한다.
모터 포트를 열거나 cmd_vel을 발행하지 않으며 추종기는 아직 실행하지 않는다.
실제 발행률/TF손실·EKF중단 시 발행억제는 다음 단계에서 검증한다.

사용자 실제 실행에서 /follower/odom_map pub1, Reliable, 약10Hz 확인.
최신 재정합 후 위치(1.470814239,0.297934276)m/yaw약13.865도가 TF와 일치.
약33초 떨어진 두 표본 위치/방향 유지, twist는 수치상 거의0.
중간 위치 변화는 사용자가 기체를 옮겼다고 확인했으며, 무이동 점프로 판정하지 않는다.
이동 후 초기위치를 다시 지정하는 절차를 안내했다.
출력 pose 공분산0은 정확도 보증이 아니며 이 메시지를 절대위치 센서로 EKF에 융합하지 않는다.
입력 중단 시 발행억제/복구는 아직 미검증이며, 추종/실제 구동은 실행하지 않았다.

### EKF 일시 정지 시험 결과 — 2026-10-28

첫 launch 중단 시험은 어느 입력이 중단됐는지 혼동이 있어 전체입력 중단 성공으로
판정하지 않았다. TF 조회 실패 중 bridge 메시지가 안 나온 것만 확인했다.
이후 모든 관련 프로세스를 정리하고 관측(A)/위치추정(C)/bridge(D)를 구분해 재시작했다.
두 lifecycle active, 실제 현재 위치 재지정 및 scan/map 겹침을 사용자 확인.
bridge 기준 위치(1.651065857,0.085237367)m, twist0 출력 확인.

대상 EKF PID4171을 SIGSTOP으로 일시 정지 후 1초 대기, 5초 echo에서 새 header 없음.
현재 bridge 로그1793174353~4357에 EKF missing or stale 경고 확인.
종료 trap의 SIGCONT 재개 후 header stamp1793174367.747219516/map 수신 복구 확인.
즉 EKF 프로세스 갱신중단에서 bridge 발행억제/재개를 이 시험에서 확인했다.
정확한 억제 지연계측, 재개 후 위치연속성, AMCL/TF만 유실, 실제 모터 정지는 별도 미검증.
함께 제공된1793173744~3796 경고와 ExternalShutdownException은
1793174202 bridge 재실행 이전 세션 로그이므로 이번 갱신중단 중 크래시로 판정하지 않는다.
ExternalShutdownException 종료 처리 정리는 별도 유지보수 항목으로 남긴다.

재개 후 사용자 표본1793174506: EKF x/y/yaw0, twist0;
1793174509 bridge map 위치(1.651065857,0.085237367), quaternion(z0.114114210,w0.993467638)
시험 전과 동일. 따라서 재개 후 해당 표본에서 위치 점프 없음 확인.
중단 순간부터 복구까지 연속 pose 최대step/정확한 응답지연을 계측한 결과는 아니다.
EKF pose Pxx/Pyy약33.965, Pyaw약37.457로 커져 있음. 속도/각속도 입력 기반
로컬 적분기의 누적 불확실성을 검토할 항목이며, map 위치 점프나 절대정확도와 혼동하지 않는다.
bridge pose 공분산0은 이 불확실성을 표현하지 않으므로 센서융합용으로 사용하지 않는다.

### 선두 토픽 현재 세션 수신 확인

선두를 기존 성공 base allow_motion=false, 저장지도 localization, 초기위치정합,
leader bridge 순서로 사용자가 실행. Nav2 주행은 실행하지 않도록 안내했다.
후발에서 /leader/odom(map/base_link), /leader/path(map) 실제 수신 확인.
path pub1 Reliable, 수신약10Hz. echo에서 message lost 알림5~8회 관측.
odom hz 출력은 초기 burst로 약17.84Hz에서11.59Hz까지 변함.
평균rate만으로 유실률/지연상한을 판단하지 않는다. 원인 미확정, 추종 미실행.
check_convoy_rx.py는 10초 동안 세 토픽을 함께 구독해 간격/age/경로점수/현재pose를
콘솔에 출력하는 진단 도구다. 파일·시리얼·cmd_vel 접근 없음. 실제 실행 결과는 미수집.
첫 실제10초표본: leader odom99개 gap최대1.6375s/age최대2.1246s,
path96개 gap1.5466s/age1.9387s. age>0.5s 각각17/14개.
follower82개 gap최대0.2680s/age0.2179s, age>0.5s0개.
header 반복/역순0. path1점은 선두정지 및 새 bridge 세션에서 정상일 수 있다.
초기발견/큐/네트워크/선두발행/시계차 원인은 아직 분리되지 않았으므로 추종 보류.
진단 도구에 최대공백 발생시각 및 처음3초 이후 별도통계를 추가. 원표본도 유지한다.
후속20초표본: 초기3초 제외 후 leader odom/path 각170개,
gap최대0.7536/0.7570s가 경과11.92~12.68초에 동시 발생.
age최대0.6634/0.6692s, age>0.5s 각2개. p95 age0.0281/0.0350s.
같은 구간 follower170개 gap최대0.1027s/age최대0.0499s.
초기연결만의 현상은 아니며 선두측발행/원격전달 경로를 추가 분리해야 한다.
추종은 계속 보류. 다음은 동일 진단을 선두 로컬에서도 실행하여 비교하는 단계다.
양 기체 동시20초 비교: 선두 로컬 odom/path200개씩, 초기3초 제외 gap최대0.1252/0.1233s,
age최대0.0934/0.0955s. 후발에서 초기3초 제외 선두 토픽 각170개,
gap최대0.1583/0.1569s, age최대0.0479/0.0494s, age>0.5s0.
같은 후발 로컬map gap최대0.1025s/age0.0505s. 이번 정상구간은 확인했으나
앞선 정상연결 후0.75초공백을 해소했다고 판정하지 않는다.
원격age 최소음수 약-60ms 및 반대방향age가 더 큰 점은 시계차도 검토해야 함을 뜻하며
순수네트워크지연으로 해석하지 않는다. 양 기체 timesync 상태/offset 확인은 다음 단계.
현재 지도좌표 선두(3.1193,0.6493),후발(1.6511,0.0852) 중심거리약1.573m.
이것은 지도추정 기준이며 실제 바퀴축 중앙 간 거리 실측 대조는 아직 미완료.
양 기체 timesync-status: follower ntp.ubuntu.com offset+35.274ms/jitter61.139ms,
leader 1.pool.ntp.org offset-68.762ms/jitter104.564ms. 서로 다른 서버의 최근표본이므로
두 offset 차이를 현재 기체간 정확한 시계차로 단정하지 않는다.
선두 stamp가 후발 시계보다 약간 미래인 표본이 있어 미래시각을 거부하는 추종 후보가
정지할 수 있다. 허용범위를 임의 확대하지 않고 상대시계차를 먼저 확인한다.
check_clock_offset.py는 기존 SSH키로 한 세션에서9회 시간질의(첫 표본제외)를 하고
최소RTT 표본의 상대offset 추정/왕복시간 기반 구간을 출력한다. 시간설정 변경 없음.
추정 midpoint는 왕복대칭 가정이며 정밀시간동기 인증이 아니다. 실행결과 미수집.
첫실행은 BatchMode 키인증 실패로 시계차 미측정. --interactive-auth 옵션을 추가해
일반터미널의 SSH 비밀번호 입력을 허용했다. 비밀번호를 인자로 받거나 파일에 저장하지 않는다.
호스트키확인은 유지하고 첫표본(인증시간포함)은 통계에서 제외한다.
사용자 --interactive-auth 실행결과: 최소RTT9.952ms,
leader-minus-follower midpoint+31.453ms, 인과구간약[+26.476,+36.429]ms.
해당시점의 추정이며 네트워크대칭/시계안정 가정과 시간변화를 고려해야 한다.
과거음수age의 가능 원인 하나지만 0.75~1.6초공백을 이31ms로 설명하지 않는다.
미래stamp를 무조건거부하는 추종 후보는 동기편차로 정지할 수 있으므로
시계관리/유한한허용오차 정책을 실구동 전 검토. age값 임의보정/timeout확대는 아직 적용하지 않음.
SSH/시간서버설정 변경없음. 실제중심간 거리/좌우배치 대조는 여전히 미수집.
사용자 후속실측: 두 기체는 일직선 배치, LiDAR-LiDAR 거리1.56m,
선두뒷부분부터 후발까지 약1m(정확한 후발측 측정끝점은 미확인).
지도중심거리약1.573m와 거리크기는 가깝지만 후발좌표계 좌측약0.215m 편차는 남는다.
양 LiDAR 전방offset0.26m가 같고 자세가 같다면 축중앙거리도 LiDAR간거리와 같다.
이것은 정밀기하보정/배치인증은 아니며 실제방향/위치추정오차를 추가 확인한다.
수동초기위치오차는 가능원인이나 확정하지 않는다. 22cm상수보정/강제좌표일치 적용없음.
다음은 양 기체 정지상태 request_nomotion_update 1회씩 후 새 pose/상대배치 재측정이다.
갱신 후 표본: 선두(3.1206,0.5425),yaw13.962도;
후발(1.6598,0.0085),yaw14.124도. 중심 간 거리 약1.555m,
후발기준 전방약1.547m/좌측약0.161m, 방향차약-0.162도.
실측1.56m와 거리크기는 가까우나 일직선배치의 좌측잔차약16cm는 미해결.
stationary 선두의 AMCL위치보정이약0.107m여서 path1->2점 증가.
이두점은 실제주행증거가 아니며 다음 실주행이력 세션 전 leader bridge 재시작필요.
초기3초 이후 leader age최대0.308s, gap0.325s; follower age0.069s.
이번표본에서0.5초초과없지만 이전간헐공백문제 전체해결판정은 하지 않는다.
양 bridge 구분 후 재실행: leader path1점(3.1206,0.5425), follower map출력 복구 확인.
최신10초표본 초기3초이후 leader age최대0.0867/0.0901s, gap0.1495/0.1520s;
follower age0.0480s/gap0.1049s. 경로초기화는 확인, 장시간지연해결은 미검증.
다음은 기존 safe PP 후보를 output /follower/cmd_vel_test로 지정한 모터미연결 시험이다.
실행 전 시험토픽/실제명령토픽의 구독자를 확인한다. APF/실제구동제어기는 실행하지 않는다.
path1점에서0출력은 정상이나 전체추종성공/모든입력유효검증을 의미하지 않는다.
기존후보의 leader회전모방/후진정책, 미래stamp0허용, 경로session/진행도 문제는 남아 있으므로
실제모터출력으로 연결하지 않는다. 수정은 별도 검증된 진단경로에서 준비한다.
사용자 시험토픽 실행: /follower/cmd_vel_test 발행자 real_pure_pursuit1, 구독자0,
연속 Twist0 확인. path1점/선두정지 조건의 정지이며 전체추종성공은 아니다.
추가 test_pursuit_geometry.py는 기존 AST fake harness로 가상경로만 사용한다.
새6개+기존13개=19개 중18통과/1실패. 직진/좌우부호/회전좌표/각입력끊김/1점reset 통과.
실패: follower가 (0,0),전방+X인데 선택경로가 (-3,0)->(-2,0) 뒤쪽일 때
linear.x=+0.1을 출력. local_x<0에 대한 정지/정렬 분기가 없어 뒤목표에서 앞으로 가는 결함 재현.
실제구동하지 않고 발견. 원본 PP는 수정하지 않음. test 출력만 사용하고 실모터 연결금지.
다음은 전진전용 미리보기에서 후방/큰방향차 목표를 정지시키는 진단후보 보완이다.

### 전진전용 시험 미리보기 — 기존 추종 소스 보존

follower_pursuit_preview.py를 별도로 작성했다. 순수 PreviewCore와 ROS 실행부를 분리했다.
기존 safe PP/파서/센서/EKF/선두파일은 수정하지 않았다.

- 출력은 /follower/cmd_vel_test 고정. resolve_topic_name으로 확인하고 다른 출력 remap을 거부한다.
- 선두정지/후진/제자리회전, 근접거리, 경로1점/잘못된입력/입력만료에서0출력.
- 선택 목표가 후방이면 STOP_TARGET_BEHIND, 방향차45도초과면 STOP_ALIGN_REQUIRED.
  자동 제자리정렬은 아직 구현하지 않았다. 불확실한 상황에서 방향을 모방하지 않는다.
- 계산 상한 전진0.1m/s, 회전0.2rad/s, 추종거리1.5m/정지거리1m,
  lookahead0.8m, freshness0.5s는 시험값이다. 실제속도/제동여유 합격값이 아니다.
- 미래timestamp는 기존처럼 거부한다. 관측된 시계차를 숨기는 상수보정은 적용하지 않았다.
- 입력 수신/측정age, frame, 유한값, quaternion을 검사하고 나쁜 새 입력은 이전값을 무효화한다.
- 정지 이유와 시험 계산값을2초마다 로그에 출력한다. 시리얼/scan/APF/실제모터gate는 없다.
- 기존 전체경로 최근접점선택 원리는 남아 있다. 세션/경로교차/단조진행도,
  위치점프 검출, 시간정책, 장애물보호와 실제 추종 성능은 미완료다.
- test_pursuit_preview.py의 가상 입력15개 테스트 통과. ROS 노드/모터를 실행한 검증은 아니다.
  원래 test_pursuit_geometry.py는 기존후보 결함 재현용으로 보존되어 해당case가 실패한다.

기존 시험 추종기 터미널E만 종료하고, 같은 시험토픽의 발행자가 없는지 확인한 후 사용한다.

```bash
python3 -B ~/follower_diagnostics/follower_pursuit_preview.py
```

출력 파라미터를 넘기지 않는다. APF/실제속도제어기는 켜지 않는다.
정지/경로1점인 현재 실입력에서는 STOP_MISSING_OR_INVALID_PATH 및0출력이 정상이다.
이 상태의0출력은 모든입력이 유효하거나 가상경로시험이 실제에서도 통과함을 의미하지 않는다.
다음은 실행 graph/정지이유 로그를 확인하고, 모터분리 상태에서 가상입력 ROS 통합시험을 준비한다.
사용자 실제 Domain10에서 새 follower_pursuit_preview 발행자1/구독자0 및 Twist0 확인.
정지이유 시작로그는 아직 미수집. 경로1점이므로 현재0출력을 입력유효성완료로 해석하지 않는다.
check_pursuit_preview_ros.py --run은 별도 Domain217/localhost-only에서 가상입력과
미리보기 자식프로세스를 만들고17개 조건을 검사한다. 실제Domain10노드/센서/모터는 접근하지 않는다.
실행전 Domain217의 다른 노드/관련토픽 endpoint 발견 시 거부한다. 실제실행결과는 아직 미수집.
시험완료 시 자식프로세스 종료. 입력중단검사는1초이후출력을 보며 정확한 정지지연 계측은 아니다.
사용자 실제 후발터미널 실행 RESULT17/17 passed, Failures=[] 확인.
직진(0.1,0),좌우(0.1,+/-0.02),후방/큰방향차/선두정지·후진·회전/경로reset/잘못된frame 정지,
leader/path/follower 각입력유실 정지와 복구, 최종복구를 가상ROS에서 확인했다.
Domain10의 현재실입력0출력과 별개의 테스트다. 실제주행/충돌회피/정지지연계측은 미검증.
다음은 scan-to-base TF와 실제차체외곽을 반영한 보호계층의 무구동검증이다.
후발 실제외곽 폭/길이/축중앙에서앞뒤끝거리 미확인. 선두외곽값을 복사하지 않는다.

### 후발 외곽 확인과 보호 미리보기

사용자 실측확인: 바퀴포함 폭0.60m/길이0.54m/바퀴축중앙에서전후각0.27m.
follower_hardware.yaml의 follower_obstacle_preview에 해당값을 저장했다.
이후 기록이 위 미확인 상태를 갱신한다.

follower_obstacle_preview.py는 기존APF를수정하지 않은 별도시험후보다.
입력 /follower/cmd_vel_test + Best Effort /follower/scan,
출력 /follower/cmd_vel_guard_test 고정. 실모터토픽remap거부,시리얼접근없음.

- scan측정시각에 TF follower/base_link<-follower/laser 조회.
  정적TF의stamp0은정상이며 동적측정의stamp0과혼동하지 않는다.
  좌표의평행이동/회전을적용한점으로 직사각형외곽까지유클리드거리를계산한다.
- 모든유효return을정지검사에포함. 선두mask 없음. 선두/주변장애물도제외하지않는다.
- 가까운점의외곽여유<=0.30m이면정지,0.30~0.60m는기존v/w를같은비율로줄인다.
  이두수치는새시험값이며실측제동거리/안전합격값아님.
- 상위0명령을회피회전으로바꾸지않음. 후진/제자리회전은거부.
- TF실패/잘못된scan/frame/stamp/전부무효ranges,scan측정·수신만료,
  시험명령수신만료에서0출력. timeout0.5초는시험값.
-16개오프라인검사통과: 앞/옆/모서리외곽,전방0.26m오프셋,yaw변환,근거리정지,
  입력무효화/만료/0명령유지/출력제한. 실제ROS연결은아직미검증.

```bash
# A/C/D/E와별개 후발터미널F. E의입력은계속시험토픽만사용한다.
python3 -B ~/follower_diagnostics/follower_obstacle_preview.py
```

알려진한계: ROS Twist입력에는원측정timestamp없어수신공백만검사한다.
실제근거리0.28m 미만센서사각/장애물높이/차체자체반사/가림/움직임중scan왜곡,
회전시쓸림영역/제동거리는해결하지않았다. 일부무효ray를안전한빈공간으로보증하지않는다.
전부무효면정지하지만 유효ray도있는부분사각을보증할수없으므로실구동gate로사용금지.
초음파/범퍼등추가보호와명시적latched fault/rearm정책은후속과제다.
사용자실제실행: follower_obstacle_preview pub1/구독자0,guard_test Twist0 확인.
STOP_UPSTREAM은상위시험명령0이므로예상상태이나 clearance0~0.022m를관측.
차체외곽내/매우근접return이있음을뜻하므로자체반사/장애물/데이터오류를분리전에는
ignore_array/자체차체mask/최소거리확대등으로숨기지않는다.
원래보호로직은유지하고가장가까운8점의각도/range/base좌표 진단정보를추가했다.
inspect_near_scan.py는TF를적용한최소거리점5개scan표본을읽기전용출력한다.
실제근접점위치/원인은아직미확인. 이문제로실구동은계속보류한다.
사용자확인: 후발오른쪽물체존재. 제거후 y약-0.32m 근접선은사라짐.
그후 -141도/r약0.315/base(0.015,-0.198) 내부점은간헐적으로남음.
사용자케이블정리후5/5표본에서내부점미관측,최소외곽여유0.5439~0.5581m.
케이블반사가능성과일치하나장시간재발/근본원인확정은별도미검증.
무시mask/threshold변경없이관찰. scan높이24cm주변케이블고정상태유지가필요하다.
다음 check_obstacle_preview_ros.py --run은 Domain217 localhost에서
가상Best Effort scan/정적TF/시험명령으로16조건을검사한다. 실제장비접근없음.
아직실행결과미수집. TF/입력유실검사는1초후출력판정이며정확한stop latency시험아님.
사용자격리실행에서보호계층16/16통과(Failures=[]) 확인.
Best Effort scan/정적TF/정지·감속·잘못된stamp/frame/각입력유실과복구의ROS연결확인.
다음 check_convoy_chain_ros.py --run은 같은격리Domain에서PP와guard를동시실행해
20개연결조건을검사한다. raw가전진이어도장애물/scan유실시최종guard0인지,
PP입력유실/PP프로세스종료시최종0인지확인한다. 실제실행결과는아직미수집.
사용자실행 RESULT20/20 passed, Failures=[] 확인.
가상raw가전진0.1이어도앞/옆장애물·scan오류/유실에서guard0,
각추종입력유실/1점reset/선두정지/후방목표에서raw/guard모두0,
회복후전진복구 및 가상PP SIGKILL종료후guard0 확인.
late-phase판정이므로정확한end-to-end정지시간/실제제동을인증하지않는다.
현재물리모터는PID4조회전용소유자이며시험출력을모터로연결하지않았다.
다음소프트웨어작업의계약은 MOTOR_INTEGRATION_PLAN_20261028.md에정리했다.
후속으로 follower_motor_candidate.py 순수통합상태기계 및 test_motor_candidate.py 작성.
fake serial/clock39개검사통과,실포트/ROS노드실행없음.
기본dry-run/구동잠금,단일transaction,정지우선,명령/피드백만료,latched fault와
재무장epoch,실제RPM기반적분을검사했다. 포트adapter/ROSwrapper는아직미구현.
현재A의조회노드를교체하거나실제모터에시험출력을연결하지않는다.
이후 follower_motor_mock.py ROS연결부를추가했다. 실제포트열기기능없는메모리모델만사용한다.
기본 enable_simulated_motion=false/read-only설정. 명시적으로true로시작해도실제장비접근은없다.
입력은 /follower/motor_mock/manual_command의version1 JSON String 한개에
epoch/frame/stamp_ns/v/w를함께전달하는임시수동시험계약이다. 기존Twist추종출력과연결없음.
출력은 /follower/motor_mock/status, /follower/motor_mock/odom 및mock전용frame,
arm/disarm/set_feedback_enabled 서비스도같은mock접두사. 토픽/서비스remap거부.
실제 /follower/odom_raw, /follower/cmd_vel 또는TF에는발행하지않는다.
새 envelope/transport 모의검사13/13통과. 단순즉시RPM응답모델이므로가속/제동/실물정지모델아님.
check_motor_mock_ros.py --run은Domain217에서기본잠금/명시적재무장/단절/재생명령거부/
모의odom을검사하는19개통합조건을준비했다. 실제ROS실행결과는아직미수집.
사용자 check_motor_mock_ros.py --run에서19/19통과확인.
동시에재실행한보호16/16/연결20/20도재통과했으며별개의새고유case로중복계산하지않는다.

### 실제 포트 잠금 후보 준비 — 아직 실행하지 않음

follower_motor_port.py는장치character검사,fuser전후점유검사,pyserial exclusive,
커널TIOCEXCL을사용하며실패/경쟁발견시닫는다. 모의포트검사12개통과.
실제kernel/USB에서의동작/프로세스가시성/독점성은아직미검증.
송신경계에서ID1 PID4또는ENABLE1/RPM0패킷두종류만허용하며비영RPM을독립거부한다.
follower_motor_locked.py는이포트를쓰는잠금전용ROS실행부다.
default실행은help만출력하며,실포트실행에는 --run-hardware-locked --ack-zero-rpm-tx가모두필요.
속도명령구독/arm서비스/TF발행없음. actual /follower/odom_raw와별도status만발행.
측정RPM이0이아니면fault유지/STOP요구. 측정값은임의로0으로덮어쓰지않는다.
잠금wrapper검사5개통과. 실제실행/시리얼송신은아직안했다.

관측launch에 external_feedback_confirmed(기본false) 옵션추가.
enable_feedback=true와동시선택하면거부. external=true일때관측launch는모터포트를열지않고
새외부odom공급자로EKF를사용하도록선택할수있다. 기본/기존내부모드/외부모드/중복거부정적검사통과.
현재실행중프로세스에는파일변경이자동적용되지않는다.
실물전환은먼저현재PID/포트점유확인 -> 기존조회소유자정리 -> 새잠금노드단독확인 ->
외부odom모드관측재시작 -> AMCL실제위치재지정 순서다. 이단계에서실제속도명령은허용하지않는다.

## 2026-10-28 재개 결과

- 사용자 확인: 재개 전 모든 노드 꺼짐. USB by-path 매핑 어제와 동일.
- 최종 기본값(yaw0, inverted=true)으로 센서TF/LiDAR launch 재실행 성공.
- scan RViz 표시 확인. Reliable 구독으로 QoS 경고가 있었고 Best Effort로 설정함.
- 시작 직후 checksum 오류/intensity 자동조정 재현. 제공된 이후 약1분 로그에는
  추가 checksum 오류 없음. 장시간 통신 품질/오류율은 별도 미검증.
- 바퀴축 중앙 기준 정면1m 상자 앞면을 놓고 RViz Measure에서도 사용자 1m 확인.
  단일 거리의 화면 측정 정합이며, 센서 전구간 정밀도/계측 인증이 아니다.
- 다음 단계 준비: 진단 launch에서 EKF /set_pose(토픽/서비스), /enable, /toggle을
  각각 /follower/set_pose, /follower/enable, /follower/toggle로 remap.
  기존 EKF 소스/YAML은 변경하지 않음.
- 사용자 실행 graph에서 /follower/set_pose(구독/서비스), /follower/enable,
  /follower/toggle 분리 확인. filtered odom 발행자 EKF1, 위치/속도0 확인.
  bridge 미실행 상태 구독자0. 최초 node info 실패 후 실제 노드 재조회 성공.
- map_server/amcl/lifecycle_manager 실행파일 존재 확인. 후발 지도 SHA256:
  YAML 0ece345abf4149f98955ba8b92fd892ced548dac03f82dde542c63d3e9e9b8f6,
  PGM 1afef54bf00023fd64cb41babc6410459f3eeda7f467e848d8374174a9e0f927.
  사용자 선두 터미널 출력에서 두 해시 모두 일치 확인. AMCL/map server는 아직 실행하지 않음.

### 후발 지도 위치 추정 후보

follower_localization.launch.py는 기본 비활성이다. enable_localization:=true 선택 시
후발 namespace 아래 map_server/amcl/localization_manager만 시작한다. 모터·센서는 시작하지 않는다.
기존 관측 launch가 /follower/scan, follower/odom -> follower/base_link,
follower/base_link -> follower/laser를 제공해야 한다.
지도 YAML/PGM 해시가 양 기체에서 확인한 값과 다른 경우 실행을 거부한다.
AMCL 입력 map은 /follower/localization_map, scan은 /follower/scan이다.
초기위치는 /follower/initialpose, pose출력은 /follower/amcl_pose,
TF는 map -> follower/odom이며 선두 odom 프레임과 구분된다.
기본 /initialpose로 위치를 보내지 않는다. 노트북 RViz Tool Properties에서
2D Pose Estimate Topic을 /follower/initialpose로 바꾼 뒤 사용해야 한다.
AMCL은 같은 지도 공간에서 실제 후발 차체축 중앙의 위치/방향을 지정해야 한다.
현재 공간 확인/실행 graph/lifecycle/map 수신/TF/지도정합은 아직 미검증.
AMCL은 노드 시작만으로 map TF 정합 완료가 아니다. 자동 기본위치 지정은 하지 않는다.

첫 실행에서 지도 수신 성공, map_server active/AMCL inactive 확인.
원인: manager node_names에 FQN을 사용해 bond ID /follower/map_server와
서버 ID map_server가 불일치. Humble 공식 lifecycle_manager.cpp 및 lifecycle_node.cpp
소스에서 ID 생성 확인 후 node_names를 [map_server, amcl]로 수정했다.
manager와 서버 모두 /follower namespace 유지. bond 비활성화/타임아웃 증가는 하지 않았다.
정적 구성 검증 통과, 수정 후 실제 lifecycle active 여부는 재시작 확인 필요.
시작 직후 scan timestamp가 TF cache보다 이르다는 경고도 있었으나 지속 여부는 미확인.

이 디렉터리의 파일은 별도 진단 후보다. 기존 소스, EKF A, 설치 목록 및 선두 구성은 수정하지 않았다.
생성 단계에서는 장비 노드 실행, 빌드, 시리얼 송신을 하지 않았다.

## 최종 LiDAR 설정 — 아래 과거 비교 기록보다 우선

사용자가 실물 장착을 확인하고 0도에서 전방/좌우가 모두 맞음을 재확인했다.
기본 laser_rpy를 (0,0,0)으로 정리했다. 위치는 (0.26,0,0.24)m 그대로다.
inverted=true, reversion=true 유지. IMU TF는 (0,0,0.16)m, 회전0 그대로다.
앞선 12도 및 180도 전환은 비교 과정의 임시값이며 최종 설정이 아니다.
다음 실행부터 laser_yaw_offset_deg/laser_flip_180 인자는 생략한다.
특히 예전 laser_flip_180:=true 명령을 재사용하면 현재 기본값에 다시180도가 더해진다.
실행 중 TF는 파일 변경만으로 바뀌지 않는다. 현재의 yaw0 시험은 그대로 유지 가능하다.

```bash
# 기존 센서 launch 종료 후 다음 실행 시 사용. 모터/IMU/EKF는 시작하지 않는다.
ros2 launch ~/follower_diagnostics/follower_observation.launch.py \
  enable_sensor_tf:=true enable_lidar:=true
```

방향은 사용자 실제 상자 관찰로 확인했다. 거리 정확도 및 checksum 오류 지속 여부는 미검증.

## 확인한 값과 증거

사용자가 USB를 하나씩 분리하고 원래 자리에 재연결하여 확인했다.

| 장비 | USB by-path 끝부분 | 확인 당시 tty |
|---|---|---|
| MD200T | usb-0:1.4:1.0-port0 | ttyUSB0 |
| LiDAR | usb-0:1.1:1.0-port0 | ttyUSB1 |
| IMU | usb-0:1.3:1.0-port0 | ttyUSB2 |

by-path의 앞부분은 YAML을 참조한다. USB 소켓/허브를 옮기면 재확인이 필요하다.
LiDAR와 IMU의 CP2102 시리얼 번호는 모두 0001로 by-id가 중복된다.

- 사용자 확인: MD200T 19200bps, ID1, M1 왼쪽/M2 오른쪽.
- 사용자 실측: 바퀴 지름 130mm(반지름 0.065m), 윤거 560mm.
- 사용자 터미널: PID4 조회 5회 모두 유효 20바이트 PID216 응답, 양쪽 RPM0.
- 바퀴를 든 개별 +3RPM/0.5초 명령: M1 왼쪽 후진, M2 오른쪽 전진.
- M1 피드백 +2~+3 후 0 복귀, M2 +1 후 0 복귀. 사용자가 실제 정지 확인.
- 따라서 명령/피드백 좌측 반전 true, 우측 false. 음수 명령 시험은 별도 미검증.
- 위 실기체 결과는 사용자 출력/관찰에 근거한다. 이 도구에서 직접 측정한 결과가 아니다.

## 구성

- follower_hardware.yaml: 실물 확인값. polling 기본 false.
- follower_observation.launch.py: 모든 실행 옵션 기본 false. 속도 제어기 없음.
- PID4 노드는 기존 test_tools 소스를 직접 실행하며, feedback 선택 시에만 polling을 true로 덮어쓴다.
- IMU는 /imu/data를 /follower/imu/data로 remap하고 follower/imu_link를 사용한다.
- EKF는 기존 follower_ekf_A_selected.yaml을 그대로 읽는다.
  입력은 vx/vy/yaw-rate odom 및 IMU yaw-rate, 출력 TF는 follower/odom -> follower/base_link다.
- 사용자 추가 확인: 센서 위치/방향은 선두와 동일, IMU 높이는 바닥부터 16cm.
  base_link 원점은 바퀴축 중앙의 바닥 투영점으로 정의한다(바퀴축 높이 원점이 아님).
  IMU xyz=(0,0,0.16)m, rpy=(0,0,0); laser xyz=(0.26,0,0.24)m, yaw=pi.
- enable_sensor_tf=true는 위 값을 이용해 follower/base_link -> follower/imu_link,
  follower/base_link -> follower/laser를 발행한다. 기본 false. 실제 TF 측정/정합은 미검증.
- EKF 실행은 feedback/IMU 선택과 센서 TF 선택을 요구한다. 외부 실장 TF를 사용하는 경우에는
  센서 TF 선택 대신 imu_tf_confirmed=true를 사용한다. 이 플래그는 실제 TF 검사가 아니다.
- 같은 child frame을 발행하는 기존 RSP/static TF와 중복 실행하지 않는다.
- LiDAR는 enable_lidar=true에서만 시작한다. 기본 false.
  /follower/scan (Best Effort), follower/laser 프레임과 후발 전용 scan 서비스를 사용한다.
- AMCL, bridge, 추종 및 모터 속도 제어는 포함하지 않는다.

## 실행 방법 — 향후 실행 범위를 확인한 뒤 일반 후발 터미널에서 사용

시리얼 소유자는 포트당 하나여야 한다. 기존 시험 스크립트/모터 노드와 중복 실행하지 않는다.
관측용 구성에는 모터 STOP 기능이 없다. 기존에 회전 중인 모터를 정지시키는 용도로 사용하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

# 기본 선택: 장비 프로세스를 시작하지 않는다.
ros2 launch ~/follower_diagnostics/follower_observation.launch.py

# PID4 피드백만: 실제 조회 송신 및 test_logs/md200t_pid216_received.hex 기록.
ros2 launch ~/follower_diagnostics/follower_observation.launch.py enable_feedback:=true

# 센서 TF만: 장비 포트를 열지 않는다.
ros2 launch ~/follower_diagnostics/follower_observation.launch.py enable_sensor_tf:=true

# IMU와 확인한 실장 TF: 실제 시리얼 g/a/e 질의 송신.
ros2 launch ~/follower_diagnostics/follower_observation.launch.py enable_imu:=true enable_sensor_tf:=true

# 센서 TF 및 입력 확인 후 사용하는 전체 관측 구성.
ros2 launch ~/follower_diagnostics/follower_observation.launch.py \
  enable_feedback:=true enable_imu:=true enable_ekf:=true enable_sensor_tf:=true
```

## 미검증 및 다음 단계

### 임시 각도 비교 옵션

사용자가 차체를 벽과 평행하게 놓아도 전체 scan이 base_link +X 기준 시계방향 약15도
기울어진다고 보고했다. 앞선 전방/좌우 확인은 대략적인 방향 확인이며 정밀 정합 완료가 아니다.
laser_yaw_offset_deg:=15.0은 scan을 반시계방향15도 보정하는 비교 후보다.
laser TF yaw에만 더하며 xyz/IMU/inverted/reversion은 유지한다. 기본0으로 원상태다.
후보 yaw195도는 tf2_echo에서 -165도로 표시될 수 있다(같은 회전).
실제 장착 각도인지 센서 각도 오프셋인지는 미확정이며, 후보 효과/여러 방향 거리 정합은 미검증.
같은 child TF 중복을 피하기 위해 기존 관측 launch를 종료한 뒤 재시작해야 한다.

후속 관찰: +12도에서 벽은 X축과 평행하나 실제 차체 정면 상자가 -X에 나타남.
따라서 전방/좌우 정합은 미완료다. laser_flip_180:=true를 선택하면 laser TF yaw에만
180도를 추가한다. 기본 false. +12도와 조합하면 원본180+12+180=372도(12도와 동치)다.
이것은 장착 방향 확정값이 아닌 앞뒤 비교 후보이며 IMU/base_link 및 xyz는 바꾸지 않는다.
시험은 enable_sensor_tf:=true enable_lidar:=true laser_yaw_offset_deg:=12.0
laser_flip_180:=true로 센서만 실행한다. 기존 standalone LiDAR/TF를 먼저 종료한다.
실제 전방 상자 +X, 왼쪽 상자 +Y, 오른쪽 상자 -Y를 모두 확인한 뒤 채택 여부를 결정한다.

### 사용자 터미널 및 RViz에서 추가 확인한 결과

- 센서 static TF: IMU (0,0,0.16), laser (0.26,0,0.24), yaw pi 출력 확인.
- IMU by-path 1.3/115200 연결, follower/imu_link 및 약20Hz 발행 확인.
- 정지 상태 PID216 약400개, timeout/discard 0. 관측 polling 약8.4Hz(10Hz 상한과 구분).
- 반지름0.065, 왼쪽 반전true 실제 ROS 파라미터 확인. raw/EKF 위치·속도0 및 odom TF 확인.
- EKF의 두 입력 구독 확인. Reliable publisher -> Best Effort subscriber로 호환.
- G2B 모델/230400 연결, follower/laser 및 약9.68Hz scan 수신 확인.
- 노트북 RViz Best Effort 선택 후 표시 성공. inverted=false에서는 좌우 반대 관찰.
- inverted=true로 재시작 후 사용자가 전방/좌우 일치 확인. reversion=true, TF yaw pi 유지.
- standalone LiDAR 시험은 사용자 터미널 결과다. 새 enable_lidar launch 옵션의 실제 실행은 아직 미검증.
- 시작 시 checksum 오류 및 intensity 자동 조정이 있었다. 지속 오류 여부/거리 정확도는 미확인.

- 관측 launch의 IMU/feedback/EKF 생성과 위 파라미터는 확인. 종료/재시작 회귀 검증은 미완료.
- PID4 단독 시험 성공은 ROS odom/EKF 성공과 다르다. PID216 응답에는 요청별 식별자가 없다.
- IMU 실장값/방향은 사용자 확인에 근거하며, 런타임 TF/IMU 축 검증은 미완료.
  IMU의 기존 gyro 보정 1.027은 변경하지 않았다.
- 기존 IMU 드라이버에는 /follower/imu/gyro_z_raw 진단 출력도 있다.
- 기존 EKF A 30Hz는 유지했다. 다른 runtime YAML(20Hz)과 혼용하지 않는다.
- 지상 거리/속도/회전 정확도, 통신 단절 시 실제 정지, 강제종료 및 시리얼 단절은 미검증.
- LiDAR 거리 품질/APF scan-to-base 변환, 공통 지도 AMCL, initialpose 격리, map bridge는 후속 단계다.
- 신선도 검증 후보는 정식 설치로 전환하지 않았다. 추종/APF는 실행하지 않는다.
- 기본값만으로 설정한 YAML과 향후 실제 런타임 파라미터를 반드시 대조해야 한다.

이 후보를 사용하지 않으면 기존 실행 경로에는 영향이 없다. 롤백은 후보 launch 종료 후
기존 경로 사용이며, 이 디렉터리 삭제나 기존 파일 복원은 필요하지 않다.
