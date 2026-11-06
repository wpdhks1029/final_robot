# 모터 상태 전용 수신 실행기

대상 로그: logs/local_avoidance_trial_1793945528065375415.jsonl

운영자 노드 상태 수신 간격이 0.352초로 벌어져 기존 0.3초 수신 감시에서 종료됐다. 이후 메시지는 fault 없음, feedback_age_s 약 0.0072초를 보고했고 최종 양쪽 정지도 확인됐다. 이것만으로 드라이버 발행 지연과 운영자 callback 지연을 완전히 구분할 수는 없다.

운영자에 별도 read-only ROS node/executor/thread를 추가해 모터 status 구독을 경로 계산/로그 기록과 분리했다. 모터 callback은 최신 snapshot과 실제 수신 시각만 갱신하고, 디스크 로그·motion_seen·종료 fault 처리는 운영자 주 실행 흐름에서 수행한다. 로그에는 received_ros_ns/received_mono_s를 함께 저장한다.

모터 상태 수신 TTL 0.3초, 실제 피드백/명령/응답 감시 및 fault latch는 변경하지 않았다. 전용 수신기가 실패하면 시험을 종료한다. 전용 node에는 주행 publisher, arm/service client가 없다. 종료 시 로봇 정지 확인을 마친 뒤 수신 thread를 종료하고 마지막 상태 로그를 처리한다.

드라이버 재시작은 필요 없다. 운영자 시험 명령을 새로 실행하면 반영된다. 실물 상태 수신 개선 여부는 재시험 로그로 확인해야 한다.

## 수신 snapshot 시각 순서 수정

후속 로그 local_avoidance_trial_1793946094102934953.jsonl에서는 실제 수신 최대 간격이 약 0.182초로 TTL 이내였다. 전용 callback이 loop의 mono 저장 이후 새 motor tuple을 갱신하면 기존 motor_problem(motor, mono, ...)가 음수 나이를 계산해 MISSING_OR_STALE로 오인할 수 있음을 재현했다. 상태 tuple을 먼저 snapshot하고 이후 검사 시각을 읽는 공통 함수를 모든 상태 검사에 적용했다. 주행 로직도 검사한 동일 snapshot을 사용한다. 진짜 미수신/0.3초 초과/미래 시각은 계속 거부하며 검사 실패 로그에 실제 rx_age_s를 기록한다. 드라이버 및 임계값 변경은 없다.
