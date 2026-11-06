# Outward corner-chasing correction

Recorded trial1793856740477958211: initial diagonal55deg; target corner(.5432,.7954).
Stop pose(.4938,.7165,.9751rad), corner distance9.31cm. Repeated aiming at the
same final corner raised desired heading55→58deg. Current filtered/IMU deltas
57.37/57.69deg agree; this does not indicate a new yaw calibration failure.
Hard current body gap after uncertainty33.57cm, desired58deg pivot/step
minimum33.7cm fails standard34cm planning reserve. Original55deg remaining
advance passes; subsequent rotation and full forward leg also pass the stored
snapshot (about36.5cm after a10cm step).

GentlePivot stage0 now preserves the initial checked diagonal heading. Project
remaining travel onto that line and stop if cross-track error exceeds6cm or
forward progress is exhausted without reaching the corner. At every selection
recheck the full leg and subsequent corner at the predicted actual leg endpoint,
not the ideal saved corner. No increase in speed, clearance relaxation, mask,
or yaw-scale change. Stage1 limited checked side repair remains available.
Test_outward_heading_hold replays the stop cloud and tests large side-error stop.
Existing side-repair and measured-feedback full-rejoin/PP-handoff tests retained.
Physical success remains unverified.
