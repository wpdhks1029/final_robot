# EKF pose source for local avoidance (explicit candidate)

CLI: run_local_avoidance_trial.py --pose-source filtered. Default remains raw. Motor driver, radius0.065m, track0.560m,10RPM and trial limits are unchanged. EKF/AMCL/IMU YAML and TF are not edited.

Both current robot pose and the measured-time scan transformation use the same /follower/odometry/filtered history in follower/odom. The frozen map/reference alignment is initialized from the selected EKF pose, not wheel pose. Scan interpolation still requires measured brackets <=0.2s or bounded endpoint hold <=0.05s; original stamps, including the older supporting sample, are retained. The source set includes raw wheel odometry, filtered odometry, IMU and scan plus leader; all must satisfy existing0.8s lifetime and post-arm motion rules. Loss of filtered or IMU data aborts; no raw fallback occurs.

Raw wheel odometry is still subscribed/validated and archived separately. IMU quaternion/frame/covariance/tilt are checked. Relative EKF/IMU heading divergence above8 degrees plus0.5rad/s times their timestamp separation aborts. This accounts for timing mismatch without treating wheel yaw as truth. Filtered-pose jumps >0.10m/0.12rad abort. No map->odom corrections are applied during the frozen local maneuver.

The existing30cm actual body/quantized sweep guard and near-return pause are retained. In filtered mode, the sweep also forecasts nominal quantized motion scaled1.35, reserving above the observed1.22–1.27 yaw/raw ratios without modifying wheel conversions. Held-pose bounds use0.09m/s and0.33rad/s; additional position-age uncertainty uses0.24m/s after0.1s. These are conservative prediction reserves, not calibrated physical constants. Archive fields distinguish selected EKF pose history, raw monitor history and IMU orientation history.

Validation:
- Filtered pose/scan round-trip with a fixed marker stays consistent while20%-underreported wheel yaw produces a displaced marker.
- Missing EKF, missing raw health, missing IMU and EKF/IMU divergence stop rather than falling back.
- Recorded left/right IMU-target30 tests replay through selected-pose history and pass heading consistency (<2 degrees difference).
- Full actual APF + bounded driver contract in memory completes body pass, rejoin and additional1m when wheel yaw deliberately reports20% below the physical simulated/fused pose.
- Fake ROS confirms selected EKF anchor (different from raw), one selected publisher, source labels and both stops.
- Original raw mode regressions remain passing.

Test invocations: filtered pose/operator/archive17 tests passed (before the additional recorded-data case); driver/reference/pause28 passed; raw core/archive15 passed; recorded data/filtered units/archive7 passed. These overlap and are not four independent trial sets. No physical EKF-pose APF completion has been observed yet.

Restore both robots to the original straight starting arrangement and return the visible raised20cm box to the test location. Keep existing10RPM/360s local_avoidance driver, sensor/EKF/localization and bridge terminals. Driver/EKF restart is unnecessary. First obtain a fresh current-scene plan without motion:

    python3 -B ~/follower_diagnostics/run_local_avoidance_trial.py --plan-only --apf-pivot --pose-source filtered --seconds 360

Expected reason PLAN_ONLY_READY with pose_source=filtered. The APF plan certificate covers its initial checked step, not the entire maneuver. After reviewing that output, the bounded pilot uses --run --ack-detour-area-power-cut-ready --apf-pivot --pose-source filtered --near-return-hold --seconds360 (use separate --seconds 360 arguments).

The separate stage/short-step experiment remains isolated and is not enabled here. The priority change is consistent IMU-informed pose, addressing the wheel-angle underestimate confirmed by physical left/right observations.

## First current-scene preflight after EKF selection

Trial local_avoidance_trial_1793779076449623920.jsonl accepted filtered pose/IMU inputs, then found no checked initial step; no arm or leader command occurred. Current body clearance is0.33688m, below the extra0.34m planning reserve (hard body guard remains0.30m). The initially nearest point is body[-0.598,-0.376]m, but the actual worst rotating point is different: body[-0.642,+0.055]m, producing about0.2411m body clearance near a43.5-degree left turn. Thus current-nearest and swept-blocking points must not be conflated. New APF plan-failure diagnostics now report both, with per-direction minima and blocking point/pose. Saved analysis: logs/apf_initial_space_1793779076449623920.json. Current default plan still refuses this geometry; no thresholds or control behavior were relaxed. Offline straight advances of0.10–0.12m create checked subsequent pivot candidates in the recorded scene. Prefer moving the rear obstacle farther away, or shifting the starting robot arrangement forward about0.15m while maintaining the convoy gap, before another read-only plan check. This is a geometry constraint, not evidence that the EKF mode failed.

## Measured-time callback alignment correction

Trial local_avoidance_trial_1793779550209965190.jsonl stopped after four control reports with FILTERED_ODOM_SCAN_BRACKET_MISSING. Before the failure, EKF source ages varied0.053–0.161s, IMU0.044–0.152s and scans0.150–0.249s. The exact failing interval was not captured in that old exception path, so a delayed callback is supported but not conclusively separated from EKF processing lag.

In filtered mode the operator now drains up to6 nonblocking pending callbacks with a3ms budget after its normal spin, before pose computations. If the newest scan is ahead of the last selected pose and cannot use the existing<=50ms bounded hold, both robots receive zero commands and the operator waits up to0.2s for a new EKF sample. Sources retain original timestamps; no motion uses a missing bracket. New measured samples must satisfy the original<=0.2s interpolation bracket. Persistent lag aborts as FILTERED_SCAN_ALIGNMENT_TIMEOUT. A historical gap, missing/stale sensor, IMU divergence or other fault still aborts immediately. No old scan is substituted and no new point is ignored. The source0.8s lifetime, generated-command/feedback/response0.3s watchdogs,50ms endpoint hold and30cm body clearance are unchanged. No YAML or EKF prediction parameter was changed.

Alignment wait/failure/recovery logs report the scan timestamp, first/last selected-pose timestamps, scan lead, classification and recent pose times; stop evidence includes pre/post scans and selected/raw/IMU histories. Tests cover a pending EKF callback only delivered by a nonblocking drain, a scan80ms ahead that cannot be held but is correctly projected after its real future bracket arrives, zero-only output during transient wait, single-arm recovery, persistent-gap both-stop behavior and refusal to wait across missing historical data.
