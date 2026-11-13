# 선두 바퀴 각속도 EKF 후보

현행 dump: leader_ekf_current.yaml (원본 보존).
후보: leader_ekf_wheel_yaw_candidate.yaml.
변경: odom0_config의 Z축 각속도(index11) 추가, imu0 입력 제거.
전진·횡방향 속도와 다른 유효 EKF 설정은 유지. dump의 null 미설정 항목 및 내부QoS/diagnostic 항목은 재설정하지 않음.

기록: bags/recopy/no_deadband_left_20261101_211457.
마지막 이동 명령 구간만 사용. 원본 Odometry/Imu 메시지 및 covariance 보존,
전체 시각을 공통 오프셋으로 이동하여 수신 시각 순서·간격대로 재생.
새로운 초기 EKF, 기본 IMU 장착TF z=.15 yaw0, publish_tf=false.
Domain217 localhost, 실제 모터·Domain10 명령·TF 발행 없음.
에이전트 환경 UDP 네트워크 제한 경고가 있었으나 로컬 통신으로 양쪽 EKF 출력 수신 확인.

결과 logs/leader_ekf_comparison_1793536204647331037/result.json:
- 기존 EKF: 15.2613도 (출력905개)
- 바퀴 각속도 후보: 37.5777도 (출력697개)
- 사용자 실측: 약35도 (대략값이며 정밀 참값 아님)
- 후보 파라미터 검사2개 통과.

출발 전 마지막 출력과 명령 종료2~4초 구간 마지막 출력을 비교.
초기상태·실제 실행 지연·원본 필터 내부상태는 복원하지 않으므로 정확한 원본 재현은 아님.
실물 적용·회전 정합·미끄럼·장시간 안정성은 미검증.
선두 실제 launch의 EKF params-file 경로를 확인한 뒤 해당 설정만 백업·교체할 것.
IMU 드라이버 원본 백업 및 deadband 제거 후보는 별도 보존. 모터/AMCL/laser TF 변경 없음.
