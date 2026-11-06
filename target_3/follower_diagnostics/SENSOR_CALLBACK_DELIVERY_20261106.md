# 운영자 센서 콜백 지연 분석 및 실행기 수정

시험: logs/local_avoidance_trial_1793946629972058104.jsonl
독립 관찰: logs/follow_alignment_1793946881121732073.json
비교 결과: logs/local_avoidance_trial_1793946629972058104_input_delivery.json

INPUT_AGED_DURING_COMPUTE 당시 제어 계산은 약 2.6ms, 입력 나이는 0.858초였다. 명령은 이미 STOP/WAIT_OBSTACLE_ROUTE 상태였다.
동일 실패 시각 주변 운영자 filtered/IMU 수신 공백은 약 0.766/0.767초, 독립 관찰 노드는 약 0.073/0.061초다. 센서가 두 구독자 모두에서 멈춘 경우가 아니라 운영자 전달/실행 지연에 해당한다.

로컬 Humble rclpy.spin_once는 기본 실행기에 node를 추가하고 호출 후 제거한다. 기존 운영자는 이를 반복하면서 6개/3ms의 제한된 추가 콜백만 처리했다. 수정 후 운영자 node를 SingleThreadedExecutor에 한 번 등록해 유지하고, 동일 timeout_sec=.001 인자로 최대32개/10ms 배치를 처리한다. 입력 callback은 여전히 운영자 주 흐름에서만 core/history를 갱신하므로 동시 자료구조 변경을 추가하지 않는다. 모터 status 전용 thread는 유지한다.

TTL 0.8초와 최종 명령 전 실제 입력 시각 검사는 유지한다. 오래된 자료의 시각을 바꾸거나 stale 입력을 허용하지 않는다.

검증:
- fake ROS 회귀 21개 통과
- 실제 ROS 실행기 격리 Domain217/localhost 시험: 가상 센서 8개 모두176회, 최대 callback age 약0.00576초
- 이 격리 시험은 같은 프로세스의 합성 publisher/subscriber다. 도구 환경의 네트워크 transport 권한 경고가 있었으나 프로세스 내부 전달 검사는 통과했다. 실제 여러 프로세스 센서 전달과 CPU 부하는 실물 재시험으로 확인해야 한다.

드라이버 재시작은 필요 없다. 운영자 명령 재실행으로 적용되며 요약 persistent_sensor_executor=true로 표시한다.
