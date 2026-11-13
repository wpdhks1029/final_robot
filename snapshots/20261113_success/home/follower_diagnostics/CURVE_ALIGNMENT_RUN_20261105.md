# 곡선 회피 중 방향 비교

직선 이동 구간 map/filtered 회전량 차이는 약 0.12도였다. 다음은 곡선 주행에서 방향 차이가 커지는 시점을 관찰하는 시험이다. 제어 이득/작업 공간/완료 기준은 변경하지 않는다.

1. 일반 회피 시험의 시작 배치를 복원하고 RViz 실제 위치·방향을 맞춘다. 위치 복원으로 선두 경로 이력이 역방향으로 이어졌으면 Bridge를 재시작하고 유효한 전진 경로를 확보한다.
2. 사람 투입을 변수에서 빼기 위해 상자를 미리 배치한다. 차체 앞끝 70cm 이내 감지 범위에 있고 라이다 높이에 걸려야 한다. 사람이 왼쪽 회피 공간에 남아 있지 않도록 한다.
3. 새 후발 터미널: python3 -B ~/follower_diagnostics/watch_follow_alignment.py --seconds 400 --observe-motion
4. 시험 터미널: 기존 continuous20 회피 실행 명령을 사용하고 --follow-only 및 --follow-check-seconds는 넣지 않는다.
5. 시험이 종료되면 관찰 터미널 Ctrl+C로 저장한다. 운영자 TRIAL_LOG와 관찰 JSON을 함께 보존한다.

관찰은 /follower/local_avoidance/status를 구독해 FOLLOW/AVOID와 MOVE_OUT/PASS_BOX/REJOIN/TRACK_REFERENCE를 구분한다. 구간 표시는 수신된 최근 제어 상태이며 물리 전환 시각의 정확한 증명은 아니다. 회전량은 각 구간의 센서 공통 측정 시각으로 비교한다. 각 구간의 자세 변환 변화도 함께 저장한다. 관찰 도구는 발행/arm/서비스/보정 기능이 없다.

이번 시험은 완료 사유가 작업 공간 제한이어도 기록으로 곡선 구간을 분석할 수 있다. 공간 제한이나 정량 성공 조건을 완화하지 않는다.
