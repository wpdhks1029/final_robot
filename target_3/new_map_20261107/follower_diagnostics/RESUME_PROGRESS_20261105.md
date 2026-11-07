# 11/5 경로 추종 재개 변경

선후발 하드웨어/지도/bridge/RViz 실행 완료라는 사용자 보고 후 작업.
실물 주행/arm 도구 호출 없음. 다음 단계는 watch_directed_follow_shadow.py --seconds 10. 후발 driver는 일반local_avoidance,
거리시험ignore-clearance 옵션이 없는 상태여야 한다.

완료한 코드 변경:
- directed_path_tracker.py: 누적 호길이로 목표 선택, progress가 뒤로 가지 않음,
  한 주기에서 먼 교차분기로 뛰지 않음.120deg 초과 급반전 이력은 삭제하지 않고
  PATH_SHARP_REVERSAL_RESTART_BRIDGE. 끝 접선과 leader 자세 반대면 거부.
  경로 reset/rolling prefix 변경은 현재 후보에서 새세션 필요로 거부.
  완만한U곡선은 전역방향이 바뀌어도 유지한다. Nav2 전체 경로 적합성은 미검증.
- tracked_pursuit_candidate.py: 기존 속도게이트/RPM/출력상한 유지,
  arc target으로 Pure Pursuit 곡률 계산. 뒤 목표/큰 정렬오차는 정지.
- map-rejoin의 목표 선택과 완료 검사가 같은 선분을 사용하도록 연결.
- --continue-follow-after-rejoin 옵션: map복귀+1m 검증후양쪽STOP,
  실제zero3회 후 trackedPP 인계.10초 이상 및 map path 진행10cm 이상 시
  REJOIN_AND_FOLLOW_CONFIRMED.20초 내진행미확인 종료. 기존360초상한 유지.
  기본값에서는 기존처럼 복귀후종료하며 물리 확인전 무제한운용하지 않는다.
- watch_directed_follow_shadow.py: 읽기전용, map follower/leader/path/motorstatus
  검사. 전체path/선택구간/호길이/좌표/시간및owner를 JSON 저장. publisher/service없음.
  정지선두path1점이면 PATH_NEEDS_TWO_POINTS는 예상 가능. 자동 이동 없음.

검증: 경로/선분방향/map목표/operator기존회귀30개, trackedPP/영속도인계/readonly4개,
실제memory-contract 전체회피→map복귀→PP인계→추종 시험1개 통과.
Memory시험은55cm map/local 차이를 넣음. 유효한fresh입력/정적장면 가정이며
무선/AMCL/실물주행 성공을 증명하지 않음. 테스트이름:
 test_directed_path_tracker, test_tracked_pursuit_candidate,
 test_rejoin_to_pursuit_memory.Tests,
 test_map_rejoin_guidance.Tests, test_live_rejoin_verification,
 test_local_avoidance_operator.

알려진 제한: map경로 짧음/이력오염이면 출발조건준비가 필요.
오래된frozenreference파일은 initial alignment에 여전히 쓰임.
MAP_FILTERED 보간은 실제support만 사용; 대기.2s/sourceTTL.8/motordeadline.3유지.
동적회피계획은현재작은직선실험기준; Nav2곡선통합은아직미완료.
