# 다음 단계: 후발 모터 통합 후보 계약

현재는 계획이며 새 모터 제어기 구현/실행 완료가 아니다.
진행 갱신: follower_motor_candidate.py에 순수 상태기계 코어를 작성하고
test_motor_candidate.py의 fake serial/clock39개 검사를 통과했다.
실제포트를 여는 adapter와 ROS 실행 노드는 아직 구현/전환하지 않았다.
아래후속갱신: 잠금전용 실제포트adapter/ROS실행부는작성·모의검증했으나 실제실행하지않았다.
구동허용실행부는여전히없다.
따라서 현재 실행 중인 조회 노드를 교체하는 단계는 아니다.
추종 가상ROS17/17,보호16/16,두계층연결20/20은 통과했다.
이것은 모터드라이버가 명령단절/프로세스종료/전원문제에서 실제멈춘다는 증거가 아니다.

## 현재 소스에서 확인한 연결 조건

- follower_md200t_pid4_node.py가 실제포트를소유하며조회와raw odom만제공한다.
- 기존 leader_robot_control MD 노드는 /cmd_vel,/odom 및선두프레임기본값을사용한다.
  후발조회노드와동시실행하거나이름만바꾸어실제구동에연결하지않는다.
- 기존 MD 콜백의최대0.3초블로킹RX/STOP반복sleep은watchdog실행을지연시킬수있다.
- 기존코드에서잔여RPM을조건부0으로덮어쓰는처리는실제정지측정증거와구분한다.
- 실물검증값:19200/ID1/by-path1.4,반지름0.065/윤거0.560,M1좌측반전/M2우측정방향.

## 새 진단 후보에서 구현할 조건

1. 기본 dry-run, hardware 열기와 motion 허용을 각각 명시적으로선택.
   ROS노드미실행 모의serial/clock으로회귀검증먼저.
2. 포트소유자하나:PID4조회/PID207명령/응답파싱을하나의상태기계에서직렬화.
   기존포트점유시열기거부,명령/조회각각독립노드실행금지.
3. odom 출력 /follower/odom_raw, 프레임 follower/odom -> follower/base_link.
   EKF가TF유일소유자이며드라이버는해당TF발행하지않음.
4. 첫움직임은추종과분리된명시적수동시험입력으로만준비.
   /follower/cmd_vel_guard_test를자동연결하지않는다.
   최종구동입력은timestamp/좌표/유한값/수신age/epoch/rearm정책을별도확정.
5. 명령이없거나만료/무효/locked이면0요구. 주기적watchdog는callback블로킹에의존하지않도록설계.
6. 실제피드백실종/serial부분쓰기/예외/불가능값은fault.
   요청명령을실제RPM으로오인하지않고last_feedback_time은유효응답에서만갱신.
7. 시작/종료STOP byte구조는검증된 enable1+RPM0을유지.
   serial오류나프로세스SIGKILL에서는소프트웨어STOP을보장할수없다.
   MD200T장치자체command-timeout 지원/실제설정은아직미확인이다.
8. fault후유효입력복구만으로자동출발시키지않는latched fault/명시적rearm정책을설계.
9. RPM/가감속/지상속도/정지거리의상한은실기체보정후결정.
   선두25RPM/배율1.18을복사하지않는다. 제어r/track은후발실측값으로유지.

## 실제 시험으로 전환하기 전

- 후발실제모터구동은아직이번단계에허용/실행된것이아니다.
  앞선허용은개별바퀴+3RPM/0.5초시험이며새지상추종으로확대해석하지않는다.
- 검토가능한후보코드/모의검증/diff를준비한후실장비송신·구동범위를확인한다.
- 조회전용A launch를종료/교체하면EKF가초기화될수있어AMCL재정합/경로세션재시작필요.
- 물리기체준비/즉시전원차단/정지실측은사용자가일반SSH터미널에서진행한다.
- LiDAR근거리사각/높이/가림,초음파/범퍼미통합,지도측방오차약16cm,
  시계편차약31ms/간헐통신공백은남은문제다. 소프트웨어시험성공으로덮지않는다.

## 작성한 코어의 구체 동작 및 검증 범위

- 기본 dry_run=true, allow_motion=false. 직접 실행해도 안내만 출력하며 포트/ROS를 시작하지 않는다.
- 실제 I/O 대신 외부에서 단일 nonblocking transport를 주입하는 코어다.
  timeout0/write_timeout<=20ms를 요구하고 flush/sleep/블로킹응답대기는 사용하지 않는다.
  실제 OS/USB 스케줄링의실시간상한을보증하지는않는다. 단일호출자전제이며thread-safe아님.
- 시작0RPM,잠금중PID4조회,arm후신선한명령이있을때만PID207.
  조회/명령한건응답대기. STOP은대기를취소해우선송신할수있다.
- stop정책은 enable1/RPM0과응답요청을유지. 읽기/부분쓰기오류시최대3회best-effort STOP.
  STOP전송성공을실제정지측정으로보지않는다. I/O오류는재시작전해제불가.
- 초기명령/명령측정·수신만료0.3s,피드백만료0.3s,조회상한10Hz는후보값이다.
  정확한실제반응시간/부하하에서의deadline은미계측.
- max_rpm3은앞선개별바퀴시험을참고한후보명령상한이며실제속도/제동상한아님.
  피드백plausibility상한200RPM 역시시험용이며실측장치허용범위미확정.
- frame=follower/base_link/측정stamp/단조증가stamp/arm이후stamp/epoch를검사한다.
  fault는latched,새응답만으로자동출발안함. 현재0RPM측정+fresh feedback+명시적arm필요.
  arm/fault/disarm에서epoch를변경하고과거명령을무효화한다.
- 실제RPM에좌측반전을적용해odom적분. 유효측정때만갱신,긴공백은적분하지않음.
  timer만으로마지막RPM을새측정으로재발행하거나정지후RPM을강제로0으로쓰지않음.
- 정상/부분frame/잡음/체크섬/중복/잘못된ID/22바이트/타임아웃순서,
  명령/피드백만료·재무장·재생명령거부·I/O실패·정지우선·부호·cap·odom·종료,
  시계점프와500단계무작위arm/fault전이의packet불변조건검사등39개통과.
- PID216에요청별식별자가없어새요청창에들어온이전지연응답을완전히구분할수없음.
  상태바이트의장치오류bit해석/실물watchdog설정/실포트점유상호배제는추가구현·검증필요.
- 실제ROS명령envelope와세션epoch의원자적전달방식은아직연결하지않았다.
  기존Twist시험출력을현재epoch로자동포장해연결하지않는다.

## ROS 모의 연결부 추가

- follower_motor_mock.py는메모리transport만생성. 실제포트adapter는여전히미구현이다.
- enable_simulated_motion 기본false/read-only. 모의동작명시선택뒤arm서비스를호출해야한다.
- 수동JSON envelope는 version1/epoch/frame/stamp_ns/v/w가정확히한메시지에존재해야하며
  중복key/추가key/잘못된type/NaN/큰문자열을거부한다. 생산용메시지계약확정이아니다.
- 별도 /follower/motor_mock/* namespace 및mock frame만사용하고실제토픽/TF발행없음.
- measured_rpm과last_sent_rpm/model_rpm을따로표시한다. 재무장시험은fresh측정0을기다린다.
- envelope/메모리packet검사13개통과. check_motor_mock_ros.py --run은
  격리Domain217에서19개조건을검사하도록작성,실제ROS실행결과미수집.
- 이노드를실행해도물리모터시험이아니다. 기존실행중인조회노드를종료/교체하지않는다.

## 잠금전용 실포트 후보 준비 및 미검증

- 사용자모의ROS19/19통과후 ExclusivePort와follower_motor_locked.py 작성.
- 포트adapter12개+잠금실행부5개모의검사통과. kernel독점/실제포트시험은미수행.
- fuser전후소유자검사+flock+TIOCEXCL. 다른소유자/알수없는출력/권한오류면열기거부.
  권한있는다른프로세스의개입이나OS지연/전기적고장을보장하는안전장치는아니다.
- 바이트송신층에서ID1 PID4와검증된ENABLE1/RPM0만허용.
  core가잘못된비영RPM을요구해도이연결부는거부한다.
- 기본실행은help. 명시적hardware-locked/0RPM송신확인이있어야포트를연다.
- 실제odom공급자를교체하기전현재조회소유자종료와중복publish방지필요.
- 관측launch의 external_feedback_confirmed=true 옵션은외부공급자를선택하며
  내부feedback선택과동시사용거부. 새EKF초기화후AMCL재정합필요.
- 실제송신승인범위는조회/0RPM에서끝나며추종출력의실모터연결로확대하지않는다.
