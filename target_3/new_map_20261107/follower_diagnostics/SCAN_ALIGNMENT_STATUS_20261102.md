# 선두 스캔 정합 문제 재검토

짧은 직진/좌회전 trial_success는 양쪽 정지와 추종 계약 확인이며 지도 정합 검증이 아님.
사용자 관측: wheel-yaw EKF 뒤에도 선두 스캔 회전 잔류.

저장된 no_deadband_left 마지막30초 raw LaserScan으로 인접 스캔 정합을 탐색:
- point-to-point ICP(220점 downsample)은 range4/8m에서11.49/14.16도로 방법 의존성이 컸음. 참값 사용 금지.
- point-to-plane ICP(최대600점, 약.7초간격42쌍)는 range4/8m에서24.642/24.576도.
  중앙 정합 잔차 약.008/.013m. 실내합성점군 +/-회전·평행이동 검사 및 기존검사총3개통과.
- 환경의 평행벽·가림·누적정합·대응점모호성에 민감하므로24.6도를 물리참값 또는 보정계수로 사용하지 않음.
- IMU15.2/바퀴36.8/사용자대략35도와 서로 불일치. 단일대략실측으로 wheel후보를 정확하다고 확정하지 않음.

선두 polling wrapper odom_callback은 actual_m1/m2_rpm 모두 feedback_speed_scale배해 부모odom_callback호출.
직진거리용배율이 부모의회전환산에도 들어갈 수 있으므로 부모 실제소스와현재파라미터덤프확보필요.
레이저yaw180은 원인확정안됨. TF고정/임의yaw보정/새배율/30초추가주행은 아직 수행하지 않음.
후발serial timeout도 간헐오류 미해결이며 최근5초직진·좌회전에서만 미재현.

## 실제 선두 소스/파라미터 확인
leader_controller_current.yaml: radius.085m, track.466m, feedback_speed_scale1.18.
부모 odom_callback은 RPM으로 v와w를계산하고 midpoint적분, fresh한측정 공분산은기본0.
wrapper가 양쪽 actualRPM을1.18배한뒤부모를호출하여 v,w 모두1.18배가됨.
이는 정당한 공통 바퀴속도 보정일 수도 있으므로 결합자체를 결함으로 단정하지 않음.
모터 /cmd_vel 환산에도 같은radius/track이 사용되므로 회전odom만 바꾸기 위해 실제geometry를 임의변경하면 안됨.

별도 후보 leader_md200t_polling_split_scale_candidate.py 준비:
- 기존 covariance후보에 feedback_yaw_scale 추가, 기본값은 기존feedback_speed_scale과동일하여기본회전불변.
- 단일executor의 odom콜백내에서만 환산을분리하고 finally로RPM과track원복.
- 모터프로토콜·명령환산·포트·기한·폴링불변.
- 분리환산/예외시원복/기본보존등관련27개모의검사통과. 실물미적용.
- 정확한 yaw계수 아직선정하지 않음. 선두 실제바퀴외경/중심간격 및 신뢰할 물리회전참조 필요.
