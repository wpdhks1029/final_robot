# EKF software-only comparison

Original input: lifted_ekf_20261026_175317. All validated replays received the
base_link -> imu_link fixed TF and showed IMU angular response. No hardware
nodes or recorded /cmd_vel were replayed. Success source/config unchanged.

## Same motion window

Raw wheel odometry x delta is +0.0584965m in every case.

| Case | EKF x delta (m) | EKF body-vx integral (m) |
| --- | ---: | ---: |
| Baseline, full prefix | -0.0857922 | +0.0583023 |
| Fresh vx covariance = 0.0004 | -0.0870884 | +0.0585961 |
| vy=0 constraint, variance 0.0004 | -0.0857922 | +0.0579595 |
| Short warmup, otherwise baseline | +0.0587016 | +0.0590060 |

Largest pose/velocity step residual:
baseline 0.234273m; finite covariance 0.194613m;
vy constraint 0.234273m; short warmup 0.0004145m.
The latter is sample-time consistency, not measured ground-travel accuracy.

## Covariance observations

At roughly t=1793004814.9, before the pulse:

- Baseline Pxx ~101.58, Pyy ~257.34.
- Finite covariance Pxx ~102.18, Pyy ~257.34.
- vy constraint Pxx ~101.58, Pyy ~0.683. The constraint was applied and reduced
  lateral uncertainty, but did not fix this forward-position correction.
- Short warmup Pxx ~0.1033, Pyy ~0.1924.

The original live filter had still larger uncertainty before this recording,
with Pxx ~1.86e7 and Pyy ~1.16e10. This supports inspecting initialization,
missing wheel measurements, and covariance evolution before tuning geometry.

## Interpretation and limits

The original bag's first command was zero at t=1793004813.01705, with motion
beginning at 1793004815.0184271. Short warmup begins at 1793004812.81705,
so most of the initial interval with stale raw feedback is excluded. Remaining
recorded messages are unchanged, and the original fixed TF is seeded at start.

This isolates the long no-feedback prefix as a key triggering condition for
this dataset. It does not prove an exact internal Kalman-gain mechanism or
establish that all covariance choices are irrelevant. The finite variance
test uses one trial value, not a calibrated sensor model.

LiDAR was operating for the source recording, and the same recorded IMU data
after the cutoff produces a consistent short-warmup result. Repositioning the
LiDAR alone is not established as the solution.

## Next development direction, not yet deployed

Keep success motor/IMU/EKF files intact. Consider a separate diagnostic wrapper
that requests real PID216 through PID4 while no cmd_vel is arriving, using the
same serial owner/executor as the existing motor controller. Never fabricate
stationary zero RPM when feedback is missing. Verify sustained stationary RX,
uncertainty during idle, loss/recovery behavior, and then a bounded lifted-wheel
pulse before ground travel. A reset immediately before each motion is only a
diagnostic workaround and does not establish robust operation.
