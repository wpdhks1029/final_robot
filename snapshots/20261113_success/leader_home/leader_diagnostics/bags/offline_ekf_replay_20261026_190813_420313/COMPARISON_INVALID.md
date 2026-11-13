# 비교 결과 판정 보류

이 실행은 정상 IMU 융합 조건의 fresh-EKF 비교로 사용하지 않는다.

- 출력 bag의 /tf_static 메시지 수가 0.
- ekf.log, player.log, recorder.log에 DURABILITY_QOS_POLICY 충돌 기록.
- 원본 /tf_static의 offered_qos_profiles가 비어 있어 재생기가 volatile로
  발행했고 transient-local 구독자에게 전달하지 못함.
- 출력 EKF의 yaw 및 yaw-rate가 모두 0으로, 기록된 IMU 각도 변화가 반영되지 않음.

재생기에 lifted_bag_qos.yaml을 명시하도록 수정했다. 향후 실행은 필수 고정 TF와
이 기록에 대한 EKF 각도 반응을 확인한 뒤에만 비교 분석을 수행한다.
이 디렉터리의 원래 bag 및 로그는 변경하지 않고 보존한다.
