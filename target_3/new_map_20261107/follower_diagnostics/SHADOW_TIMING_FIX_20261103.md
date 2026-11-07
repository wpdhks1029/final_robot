# 실제 입력 관찰의 계산 지연/표시 수정

입력 로그: logs/detour_shadow_1793706543046531232.json.
20개 관찰 중 계산 후 만료13, 경로 차단3, 경로 유효2, 초기 leader 미수신/만료각1.
원본 시각 나이 증가 중앙값134.84ms, 최대344.69ms. 이 증가에는 관찰 코드 자체의 연산이 포함되므로 무선만의 문제로 분류하지 않는다.
만료된6개 행에 가상 guarded 후보가 양수로 남아 있었다. 관찰 코드는 publisher/client/service가 없으므로 모터 전송은 없었다.

## 수정

- PreparedPathReview: 고정 경로의 회전/이동 상계/구간 경계는 시작 때 한번 계산.
- 각64자세 구간의 사각 차체를 감싸는 원과30cm+2cm+샘플 사이 이동 여유 밖의 점은 거리 연산만 생략한다.
  이 점은 해당구간의 모든 자세에 대해 보호 영역 밖임이 보장된다. 원본 스캔을 필터하거나 지우지 않는다.
- 모든 점을 계산한 기준과80개 무작위 장면에서 같은 정지 판정을 확인한다.
- 출력 min_path_clearance_m는 공간 생략이 있으면 보수적 하한이며 clearance_is_lower_bound=true로 표시한다.
- 계산 후 시각 만료가 있으면 guarded_candidate_v/w, leader_candidate_v를 모두0으로 한다.
- geometry_reason과timing_reason을 분리해 둘 다 실패한 경우 원인을 보존한다.
- 변화가 있던 실제 스캔 전체는 scenes에 최대30개 저장한다. 새 근접 점을 실제 물체/노이즈라고 임의 단정하지 않는다.
- compute_s와source_age_after_compute_s를 출력한다. 0.3초 기준은 변경하지 않는다.

로컬 개발 환경의1353경로자세/432점 검사20회 중앙5.84ms, 최대9.74ms.
이는 로봇의수정전134.84ms와 동일 조건 성능 비교가 아니며, 실제 Pi에서 재확인이 필요하다.
모터 연결/arm/주행/네트워크설정/하드웨어 제한 변경 없음.

```bash
python3 -B ~/follower_diagnostics/watch_temporary_detour_shadow.py --seconds 20 --virtual-forward-probe --plan ~/follower_diagnostics/logs/current_scene_left_detour_plan.json
```
