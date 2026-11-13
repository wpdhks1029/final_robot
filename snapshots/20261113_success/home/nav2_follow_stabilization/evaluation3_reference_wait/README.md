

스캔 정렬 대기 수정: reference-wait 모드에서는 FILTERED_SCAN_ALIGNMENT_TIMEOUT의 0.4초 종료를 제거했다. 양쪽 속도0을 유지하고 새 스캔/오뎀 정렬이 1초간 안정되고 정지 피드백이 확인되면 같은 arm에서 재개한다. 로컬 센서 불량은 계속 종료한다. 이전 코드는 backups/before_scan_alignment_wait에 보관했다.
