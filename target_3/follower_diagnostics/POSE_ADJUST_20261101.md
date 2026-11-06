# 초기 위치 숫자 미세 조정

실행 위치는 후발 터미널이다. --robot으로 선두와 후발을 선택한다.
선두 소스 변경이나 파일 복사는 필요 없다. 기존ROS 환경 Domain10을 사용한다.

## 기본 미리보기

```bash
python3 -B ~/follower_diagnostics/adjust_initial_pose.py --robot follower
python3 -B ~/follower_diagnostics/adjust_initial_pose.py --robot leader
```

현재 map TF에서 바퀴축 중앙의x/y/yaw를 읽는다. 단위는m/도다.
기본 실행은 초기 위치 메시지를 발행하지 않는다.

## 증분 조정

```bash
python3 -B ~/follower_diagnostics/adjust_initial_pose.py --robot follower --dyaw-deg 1
python3 -B ~/follower_diagnostics/adjust_initial_pose.py --robot follower --dx-cm 1
python3 -B ~/follower_diagnostics/adjust_initial_pose.py --robot follower --dy-cm -1
```

--dyaw-deg 양수는 왼쪽/반시계, 음수는 오른쪽/시계 회전이다.
기본 offset-frame=base에서dx는로봇정면(+)/후방(-),dy는좌측(+)/우측(-)이다.
--offset-frame map을 선택하면dx/dy를지도X/Y축 기준으로 해석한다.
증분은 명령을 실행한 시점의 현재TF 기준이다. 미리보기를 여러 번 실행해도 반영되지 않는다.
한 번의 증분은 이동20cm/회전10도 이내로 제한한다.

## 실제 반영

현재 위치를 유지하고 각도만13.72도로 지정하는 미리보기:

```bash
python3 -B ~/follower_diagnostics/adjust_initial_pose.py --robot follower --set-yaw-deg 13.72
```

반영 명령:

```bash
python3 -B ~/follower_diagnostics/adjust_initial_pose.py --robot follower --set-yaw-deg 13.72 --apply --ack-stationary
```

이 옵션은 실행 시점의 현재TF x/y를 유지한다. 다른 증분/절대 위치 옵션과는 혼용하지 않는다.
입력 각도는 초기값이며 AMCL의 이후 추정을 고정하지 않는다. 기본 불확실성은 그대로다.

먼저 양쪽 실물 정지를 확인하고 통합 시험 도구를 종료한다. 드라이버와 센서/AMCL은 유지한다.
미리보기 값을 확인한 뒤 같은 명령에 --apply --ack-stationary를 추가한다.

```bash
python3 -B ~/follower_diagnostics/adjust_initial_pose.py --robot follower --dyaw-deg 1 --apply --ack-stationary
```

양쪽 native filtered odom의최신0속도와 후발 미무장/피드백/RPM0을0.5초 이상 확인한다.
대상 AMCL active와 초기 위치 구독 연결도 확인한 뒤 메시지를1회 발행한다.
확인 실패 시 발행하지 않는다. 모터 명령/arm/TF/설정 변경은 하지 않는다.
사용자 정지 확인을 대체하지 않는다.

## 절대 좌표

지도 기준x/y(m)와yaw(도)를 모두 지정한다. 현재map TF가 없어도 초기 지정에 사용할 수 있다.

```bash
python3 -B ~/follower_diagnostics/adjust_initial_pose.py --robot follower --x 1.5 --y 0.2 --yaw-deg 16
```

위 숫자는 형식 예시이며 실제 위치에 맞게 입력한다. 절대 좌표와 증분은 혼용하지 않는다.
반영하려면 동일하게 --apply --ack-stationary를 추가한다.

## 정확도와 불확실성

숫자 지정은 초기 추정값을 정밀하게 입력하는 기능이며 실제 위치·각도 정확도를 보장하지 않는다.
AMCL은 이후 scan과지도에 따라 다시 추정한다. 스캔·지도 정합으로 판단한다.
기본 표준편차는RViz 기본과같은xy0.5m/yaw15도다. 숫자 증분1cm/1도와 추정 불확실성은 별개다.
--xy-std-m/--yaw-std-deg로 불확실성을 명시할 수 있지만 실측 근거 없이 줄이지 않는다.
선두 초기 위치 조정을 모두 마친 뒤에는 선두bridge를 한 번 재시작해 이전 경로를 초기화한다.
후발 조정만 할 때는 선두 경로를 초기화할 필요가 없다.

순수 계산·쿼터니언·공분산·정지조건/미무장 검사9개 통과. 실제 pose 발행은 에이전트가 실행하지 않았다.
