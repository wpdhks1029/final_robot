# 추종 중간 출력 지연 감소 후보

수정:
- follower_pursuit_preview.py: plain/stamped preview 출력 Best Effort KeepLast1.
- follower_obstacle_preview.py: plain/stamped guard 출력 Best Effort KeepLast1.
- 두 노드 모두 stamped출력을 일반미리보기보다 먼저발행.
- guard 동적TF수신 BestEffort1, 장착staticTF 기본ReliableTransientLocal 유지.
- guard TF조회timeout0: 단일executor에서 .05초 기다리지 않고 현재주기정지/다음수신재시도.
- 문맥 종료 후 waitset RCLError만 정상종료로 취급. 실행중예외는재전파.

보존: 실제원본stamp, remote50ms보수여유, .3초소스/scan기한, 장애물범위, 속도상한,
게이트웨이->드라이버 String의Reliable QoS, arm/disarm서비스, 모터코드·선두기하설정.
실행중기존프로세스에는아직반영안됨. preview launch만재시작필요.
일반ros2topic echo미리보기조회에도 --qos-reliability best_effort사용.

검증:
- pure59개통과 /tmp/latest_preview_qos_tests.log
- Domain217 syntheticchain20/20 통과. 종료시ROS문맥닫힘예외가있어후속정리함.
- 후속stampedguard7/7 통과 /tmp/latest_stamped_guard_ros.log. 원본기한만료/미래/scan/장애물/소실/복구확인.
- 에이전트 UDP권한제한경고. 로컬격리연결만검사했고 실제무선/정지지연은검증안됨.
- 체크용ROSsubscriber도BestEffort1로맞춤.

백업: backups/latest_preview_qos_20261102/
실물적용후check_source_budget20초조회로나이와수신확인. 오래된source재타임스탬프/기한완화없음.
