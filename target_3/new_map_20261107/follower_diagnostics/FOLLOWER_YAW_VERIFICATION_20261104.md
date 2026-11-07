# Follower pivot heading verification — priority before avoidance changes

User visually observed about 80–90 degrees when wheel-based control reported roughly70 degrees. The local avoidance controller uses raw wheel odometry after freezing its reference; that value is not ground-truth physical angle.

Recorded evidence: local_avoidance_trial_1793774746886900549.jsonl remained in outward stage0, ended at raw reference yaw78.0 degrees, x0.211m/y0.643m. Offline initial/stop scan registration (local_scan_alignment_1793775872105133855.json) requires approximately26.5–27.7 degrees correction for its good fits. Best median nearest residual improves from0.367m to0.009m, with193/222 current points within0.10m after registration versus21 before. Translation is ambiguous along surfaces, multiple local minima exist, and moving objects/visibility changes remain possible. This is evidence of a heading/reference mismatch, not a calibration coefficient or external angle measurement.

The earlier1.14m passage estimate used the inconsistent raw-odometry frame and retained box. It is a model distance, not an established physical passage width. Do not move the box or change wheel track solely on that estimate. Experiments with stage switching/short steps did not complete the captured scene; they are isolated in apf_pivot_stage_candidate.py and not enabled in the physical operator. Test test_apf_stopped_scene_memory retains the known failed scene as an expected failure. The normal APF implementation remains the previous baseline; further avoidance tuning is deferred until heading verification.

## Tools

watch_follower_yaw.py --seconds 30 is subscriptions/file-output only. It reads raw wheel odometry, IMU orientation, filtered odometry, map odometry, motor status, scan and the verified static mount. It does not arm or create a motor publisher. Stationary observation alone cannot validate a turn scale.

run_follower_yaw_trial.py is a separate follower-only bounded pivot. Default opens nothing. --plan-only creates no motor publisher or arm client. --run requires --ack-clear-pivot-area-power-cut-ready and START. It never publishes leader/cmd_vel or forward velocity. It accepts the existing local_avoidance driver at10RPM/30,60,360s and uses its unchanged strict v3 contract. This trial requests at most5 wheel RPM, turns toward raw target20 or30 degrees (either sign), stops at12 seconds of turning, an IMU45-degree increment, unexpected opposite yaw or raw translation>3cm. A full circumscribed body circle plus2cm reserve must have>30cm clearance from every scan point: approximately75cm clear radius from body center. At least100 valid points and330 degrees scan coverage are required. Near points abort; there is no near-return hold in this calibration trial. Sources retain their actual stamps, <=0.8s lifetime; nonzero uses post-arm stamps. Fresh zero STOP maintains startup while waiting. The existing generated command/feedback/serial0.3s watchdogs remain unchanged. Finally the follower is disarmed and zero feedback is confirmed; no automatic return rotation or rearm.

The raw and IMU angular changes use a common timestamp window beginning at the stationary pre-arm baseline and ending after settling. Quaternion wraparound is handled; gaps>0.3s are reported, not silently bridged. Filtered/map sources are optional comparison channels and may incorporate the same IMU/encoder inputs; they are not independent physical ground truth. IMU roll/pitch are recorded; >20 degrees tilt blocks the test. The installed iAHRS driver applies a0.02rad/s gyro-Z deadband, so this tool compares quaternion orientation changes, not blind gyro integration. No wheel radius/track, EKF, AMCL, pose, TF or IMU calibration settings are changed.

Mark the starting body direction on the floor. Remove the test box for this calibration and provide an unobstructed radius of at least75cm around the follower. Keep leader stationary and other follower command tools stopped. Existing local_avoidance motor/sensor/localization/bridge terminals may remain; no motor driver restart is needed.

    python3 -B ~/follower_diagnostics/run_follower_yaw_trial.py --run --ack-clear-pivot-area-power-cut-ready --target-deg 20

Output includes sources.raw.delta_deg, sources.imu.delta_deg, imu_minus_raw_deg and imu_over_raw_ratio. Save the JSON path and compare the external floor-angle observation before considering any calibration coefficient.

Validation:10 pure and fake-ROS tests passed, covering raw20/IMU26 comparison, source-window synchronization/wrap, data-gap rejection, quaternion validity, independent circle/time/IMU/direction limits,1–5RPM executable pivots, absence of leader/forward commands, disarm/zero confirmation, no commands in plan-only, and stale IMU abort. These are not a physical calibration result.

## Left/right results and IMU-referenced verification option

Left raw30 test: raw30.269°, IMU36.869°, EKF37.640°, map38.421°, user visually estimated35°. Right raw-30 test: raw-30.291°, IMU-38.179°, EKF-38.806°, map-38.925°, user visually estimated-37°. Both confirmed stopped. The two IMU/raw magnitude ratios are1.2180 and1.2604. First raw20 test had raw20.381°/IMU25.790° while user visually estimated20°, so these visual values remain approximate and are not a precise geometry calibration. Summary: logs/follower_yaw_comparison_20261104.json. Do not turn this into an automatic wheel-track/radius change.

The dedicated yaw trial now supports explicit --angle-source imu (default remains raw). It stops at the IMU quaternion yaw increment target, retaining wheel angle as an independently logged comparison channel. All scan-circle, input freshness,12-second,5RPM,opposite-direction,disarm/zero and IMU45-degree limits remain. A separate raw40-degree cap is also enforced. The IMU source is only for this dedicated verification; normal APF pose integration is not changed.

    python3 -B ~/follower_diagnostics/run_follower_yaw_trial.py --run --ack-clear-pivot-area-power-cut-ready --angle-source imu --target-deg 30

Compare actual body direction to a measured30-degree floor line. No driver restart or parameter edits required.14 pure/fake-ROS tests passed, including both signed IMU-target pivots reaching +/-30° while raw reports approximately+/-23.1° in the1.3 ratio model, independent raw cap, and stop cleanup. This is not yet a physical IMU-target validation result.

## Physical IMU-target checks completed

Left IMU30 trial follower_yaw_trial_1793777697627706832.json: IMU30.371°, raw24.817°, EKF31.022°, user approximately28°. Right IMU-30 trial follower_yaw_trial_1793777854458447417.json: IMU-30.624°, raw-24.795°, EKF-31.228°, user approximately-30°. Both successfully confirmed stopped. This supports using IMU-informed heading instead of uncorrected wheel heading for the next local-control candidate. These are approximate external observations, not a precise track/radius calibration. No geometry or main APF pose-source parameter was changed. Future control must use the same improved pose history for both robot pose and timestamped scan projection; replacing only the final turn stop angle would leave obstacle coordinates inconsistent.
