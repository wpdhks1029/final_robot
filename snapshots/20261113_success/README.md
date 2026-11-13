# 전체 로봇: 정량목표1~3

2026-11-13 사용자 확인 성공 구성의 원본 소스 백업입니다.

- `home/`: 후발 작업 환경 원본 파일. 경로는 후발 `$HOME` 기준입니다.
- `leader_home/`: 실제 선두 SSH 수집 원본. 경로는 선두 `$HOME` 기준이며 후발 설정과 혼합하지 않습니다.
- `RUNBOOK_KO.md`: 현재 목표 정의, 공통 노드, 평가별 시작/전환 명령.
- `MANIFEST.json`: 파일별 SHA256, 크기, 실행 권한. `verify_snapshot.py`로 검사합니다.

ROS2 Humble/Ubuntu 및 기존 시스템 ROS 패키지·장치 권한은 별도로 필요합니다.
`robot_ws/build`, `install`은 소스가 아니므로 제외했습니다. 복원 후 해당 장치에서 colcon build가 필요합니다.
후발 LiDAR 실행파일도 소스와 CMakeLists.txt를 보존했으며 현 장치에서 다시 빌드합니다:
```bash
source /opt/ros/humble/setup.bash
cmake -S ~/follower_diagnostics/lidar_sdk_trace -B ~/follower_diagnostics/lidar_sdk_trace/build
cmake --build ~/follower_diagnostics/lidar_sdk_trace/build -j2
```
전체 개인 홈·인증키·비밀번호·캐시·대용량 원시 로그는 저장하지 않습니다.
오래된 실험 파일도 의존성 누락 방지를 위해 관련 소스 폴더 안에서는 보존했습니다.
실행할 진입점은 RUNBOOK에 지정된 현재 파일만 사용합니다.
테스트의 대용량 기록 fixture는 제외될 수 있으므로 이 백업에서 전체 회귀시험이 재현된다고 주장하지 않습니다.

복원은 로봇 실행을 모두 종료한 뒤 별도 위치에서 해시를 검증하고 필요한 호스트 디렉터리를 검토하여 복사하세요.
기존 성공 파일을 자동으로 덮어쓰거나 ROS·모터를 실행하는 설치 스크립트는 포함하지 않습니다.
고정 `/home/sunuk` 경로와 현재 IP는 현 성공 구성을 보존했습니다. 다른 계정/네트워크에서는 수정해야 합니다.
실물 동작 확인과 정식 평가 합격은 별개이며 자동 실패 결과를 성공으로 수정하지 않았습니다.
