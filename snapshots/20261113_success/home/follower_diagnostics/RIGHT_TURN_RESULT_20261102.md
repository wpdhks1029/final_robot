# 기하 수정 후 우회전 시험

logs/joint_follow_trial_1793546478200401815.jsonl
leader_start1793546484030002345, follower_arm1793546484412094215,
scene_failure1793546506511132635. 추종 약22.099초.
종료 GUARD_SOURCE_OR_RX_INVALID. source_age.302045101, rx_age.000323627.
원본stamp1793546506207688530은 remote50ms보수여유를차감한
선두/path 원본stamp1793546506257688530과일치.
새leader/path stamp1793546506353411435도 종료직전수신됐지만
새pursuit source1793546506303411435 관측은 종료후약.004초.
따라서 게이트가 실제로관측한보호출력은그시점기한초과. TTL을자동완화하지않음.
당시후발position/EKF/TF는새값수신중.
지도기준선두yaw 약-19.618도. 실제물리회전참값은미측정.
간격1.66351->1.56160m (종료정지구간포함).
양쪽최신0피드백확인true, post_stop_faults없음, trial_successfalse.
사용자시각관측: 선두레이저스캔거의안움직임(지도정합이거의유지된것으로해석).
30초전체성공은아님. 기하후보의시각정합개선사례이며장시간정밀도확정아님.
