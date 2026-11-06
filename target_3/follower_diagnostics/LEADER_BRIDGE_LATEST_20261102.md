# 선두 브릿지 최신 상태 후보

원본 leader_bridge_current.py: 사용자로부터 실제 설치본 복사, 원본 보존.
후보 leader_bridge_latest_candidate.py: 별도 선두진단파일로 실행, 기존 package 실행파일 덮어쓰지 않음.

수정:
- 동적TF BestEffort KeepLast10, static TF의ReliableTransientLocal 기본값 유지.
- filtered odom 수신 BestEffort KeepLast1.
- /leader/odom 및 /leader/path 발행 BestEffort KeepLast1.
- TF조회 timeout .05->0초. 미수신시현재주기건너뛰고다음callback처리.
- TF/filtered twist 중 오래된 원본 stamp를 odom과path 모두에 보존.
- context종료RCLError만 처리, 실행중오류는재전파.

2026-11-02 추가 수정: 기본 publish_rate를 10Hz에서 20Hz로 변경했다.
타이머 주기의 대기는 최대100ms에서50ms로 줄어든다. 실행 지연·전송 지연은 별도이며 실제 지연 개선은 적용 후 측정해야 한다.
발행 주기는 1~30Hz의 유한값만 허용한다. 10Hz로 복원하려면 --ros-args -p publish_rate:=10.0을 사용한다.
이전 파일: backups/leader_bridge_20hz_20261102/leader_bridge_latest_candidate.py.

유지: path_min_distance.10m/max5000points/defaulttf&odomtimeout.5초,
경로점생성/좌표/속도/모터/EKF/AMCL/기하보정/후발.3초제한.
발행 BestEffort가돼 Reliable-only 구독자는수신못하므로 상태조회도BestEffort사용.
현재후발소비노드와 convoy_view.rviz 프로필은이미BestEffort.

검사: 모의34개, 격리 Domain217 ROS7/7(누락TF/오래된입력/소실/복구/오래된원본stamp보존).
로그 logs/leader_bridge_ros_1793612980576509823.log.
로컬시험은실제무선/모터정지검증이아님. 실물미적용.

적용: 선두기존 bridge터미널만Ctrl+C, python3 -B ~/leader_diagnostics/leader_bridge_latest_candidate.py
--ros-args -p map_frame:=map -p base_frame:=base_link.
선두기본/AMCL/후발노드유지. pose재지정불필요. path이력은재시작으로초기화됨.
rollback: 후보bridge종료 후 기존 ros2 run convoy_robot real_leader_bridge_node.py ...실행.
두bridge동시실행금지. 적용후정지상태 source_budget20초로실측비교.
