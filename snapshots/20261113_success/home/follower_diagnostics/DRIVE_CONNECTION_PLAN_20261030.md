# 실제 구동 연결부 후보와 남은 단계

현재 실제 로봇은 follower_motor_locked --publish-position-candidate 유지.
이번 새후보는 파일작성/모의검사만했으며 실포트전환/arm/자동추종안함.

## 준비한 구동 계약

- follower_drive_candidate.py:단일모터Core + 독립바이트gate。
- 기본enabled=false,명시enable와arm서비스및fresh0RPM5샘플없으면움직이지않음.
- arm때epoch교체,JSON원자명령 version/epoch/frame/stamp/v/w검증.
  .3초명령/피드백/응답만료와5초시험상한,latched fault/자동재출발금지.
  fault후서비스재무장도거부,후보프로세스재시작필요.
- v>=0/두바퀴전진만,최대3RPM. 제자리회전/후진차단.
  raw M1[-3,0],M2[0,3],ID1PID207ENABLE1return216허용.
  byte층게이트도5초만열림. 정지/PID4는잠금중에도허용.
- 장치status비영이면보수적으로fault. 해당flag세부해석미완료.
- follower_drive_node.py 기본help. memory는Domain217/localhost만,
  hardware는별도선택/0RPM송신확인필수. motion추가옵션없으면arm거부.
- /follower/motor_candidate/manual_command,StringJSON만입력.
  /arm,/disarm서비스,/status출력. 실제Twist/preview구독없음.
- 같은소유자가RPMraw와positioncandidate odom공급,EKFTF발행은EKF만。
  memory모델위치값은0이므로그모드로position거리정확도는검증못함。
- /cmd_vel_guard_test를epoch붙여자동포장하는gateway는아직없다。
  특히원본측정시각/수신시각/세션을지키는정식보호명령계약이추가로필요。

## 검증

- 새로운순수drive11개/CLI3개+기존코어39개/포트13개=66개통과。
- check_drive_node_ros.py --run은별도Domain217에서새wrapper를메모리로
  기본잠금/arm/부호cap/5초상한/명령유실/epoch거부/disarm조건검사하도록준비。
  실제ROS결과아직미수집,실제장비정지/USB차단보증아님。

후속사용자실행결과:새구동wrapper ROS메모리14/14통과。
기본잠금/명시arm/부호cap/5초상한/명령유실/자동재무장금지/epoch/disarm확인。
이결과는하드웨어/전기적정지/장치watchdog검증이아님。

## 원본 측정 시각을 보존하는 시험 경로 추가

- 기존plainTwist시험토픽유지. 병렬cmd_vel_stamped_test와
  cmd_vel_guard_stamped_test(TwistStamped)추가,모터구독연결없음。
- 추종header는path/leader/follower원본시각중가장오래된값,
  입력누락이면0. publish시각으로덮어쓰지않음。
- 별도stamped보호core는원본추종시각/수신시각/scan시각을.3초로검사,
  유효출력header=min(추종원본,scan원본). 무효입력은0속도/0stamp。
  plainbranch기존동작/timeout은변경없음。
- pure근거6+기존추종15/보호16/launch3=40검사통과。
  check_stamped_guard_ros.py --run은Domain217합성7조건검사준비,
  새stamped실제ROS결과아직미수집. 실행중Domain10preview재시작안함。
- 실제구동envelope/gateway연결은아직미완료。기존manualv1프로토콜에
  원본source_stamp와전송순서stamp를동시에보존하는별도계약검토필요。

## 전달부 후보 준비

- 사용자 stamped guard ROS7/7통과확인。
- follower_guard_gateway.py 추가。기본no-node,명시--run시status/StampedGuard구독。
  arm/disarm서비스호출/serial/실cmd_vel출력없음。armed/명시enable/guard입력모드/
  fresh상태와fresh측정/arm이후측정일때만guard_command JSON발행。
- version2는epoch/frame/v/w와stamp_ns(전송순서)와source_stamp_ns(원본측정)를
  함께보존。구동코어도두시각을각각검사해새전송시각이오래된source를감추지못함。
- follower_drive_node --input-kind guard에서만새v2채널구독。
  기본manual v1채널계약유지。gateway는재무장/자동구동허용을하지않음。
- 새gateway9/drive11/CLI3/stamped6=29개pure검사통과。
  check_guard_gateway_ros.py --run은Domain217합성stampedguard->gateway->메모리
  실제구동wrapper 9조건을확인하도록준비。현재실물의노드/포트변경없음。
  해당ROS통합결과는아직미수집。물리모터정지/장치watchdog미검증유지。

## 실물 구동 금지 연결 완료 후 첫 시작 조건

- 사용자gateway메모리통합9/9및실물guard_command발행1/구독1확인。
  실제driver enabledfalse/armedfalse/faultnull/fresh0RPM 상태유지。
- check_follow_start.py는실제leader/path/follower/guard/status를10초수신해
  경로점수/원본시각/선두속도/상대위치/추종stop이유를한번에기록。
  명령발행/arm/모터포트접근/주행없음. 첫 실제 추종 주행은 아직 미실행.
- 경로1점/선두정지상태에서는추종출력없음。준비없이arm하면명령TTL로fault가능。
- 기존leader정지판정vx<=.03m/s,새driver3RPM상한은지름.13m기준약.0204m/s。
  이한정값으로지속간격유지성능을보증할수없으며첫5초연결시험용。
  이후속도범위/추종정지판정의실기체조정필요。

## 최종까지 남은 주요 단계

1. 실제구동연결:새wrapper모의ROS확인,원본시각포함보호명령계약/gateway,
   명시arm/중단정책,locked소유자에서새owner전환후정지확인。
2. 실제저속추종:선두실주행경로누적/후발짧은직선/간격유지/커브확인。
   회전윤거실기체검증은사용자반복시험생략요청으로미완료인상태를명시。
3. 최종통합검증:장애물/각입력단절/고장복귀/재시작시나리오와실행문서정리。
   장치watchdog185=0의정확한정지동작미확인,통신단절시실제정지는미검증。

날짜/시간백분율을확정할근거없음. 하드웨어문제없을때도여러실물검증단계가남음。
현재완료는정지상태의센서부터보호preview까지이며자동추종완료가아님。
