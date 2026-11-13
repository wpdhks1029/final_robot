# Provisional leader yaw correction and1m distance check

Source: user-reported stationary-pivot results. IMU/raw ratios1.18034,
1.19284 and1.16819; visual angles approximately65,63,78deg. Use rounded
feedback_yaw_scale1.18 provisionally. This is not a precise external-angle
calibration or a right-turn validation. New leader_nav_yaw_calibrated.launch.py
keeps the original launch unchanged. Wheel radius.085, track.560, forward
feedback_speed_scale1.18 and outgoing command conversion remain unchanged.
No follower parameters change. Requires restarting leader hardware launch.

run_leader_distance_trial.py measures an odom-referenced1m forward displacement,
not a guaranteed physical metre. Operator measures actual travel from a marked
fixed chassis point. Speed.04m/s, final10cm at.02m/s; maximum40s.15deg heading
and15cm lateral drift limits. Fresh raw, filtered and IMU streams required,
sole leader cmd_vel publisher/subscriber; finally zero and five stationary
raw feedback samples. No follower publishers or arm services. Operator-clear-
path mode records near scan evidence without aborting for the near circle;
restricted to this diagnostic, normal robot protection code is unchanged.

Forward correction is intentionally not changed until tape-measured distance
is available: candidate_new_forward_scale = current_scale * physical/odom.
