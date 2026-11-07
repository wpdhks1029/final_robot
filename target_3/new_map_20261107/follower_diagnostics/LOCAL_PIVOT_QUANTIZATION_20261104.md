# Pivot stall diagnosis, 2026-11-04

Recorded trial: local_avoidance_trial_1793770003949580477.jsonl.
At goal 2, yaw stopped at 57.9745 degrees. Last ready command: w=0.0104097 rad/s, conservative clearance=0.315615 m. Last 100 motor status messages all reported sent_rpm=[0,0]. With radius 0.065 m and track 0.560 m the request rounds to zero wheel RPM. The no-progress timer then stops the trial.

LocalCore now tries the smallest executable 1 RPM pivot when a nonzero pivot rounds to zero. It must pass the unchanged all-direction current and one-second swept-body clearance checks, including measured current motion and uncertainty; otherwise it stops. The 0.30 m hard clearance and input/device timeouts are unchanged. Logs now include nearest scan point in body coordinates, its laser range, candidate wheel RPM, and minimum pivot activation.

Initial snapshot nearest points at the stalled pose have about 0.378 m body clearance and 0.664 m laser range. This is not the live scan at the stop (0.326 m before uncertainty); the exact live object cannot be identified from the old log. Do not infer a distant wall or mask points from that snapshot. New nearest-point diagnostics permit attribution on a subsequent trial.

This fixes an executable-command dead zone, not proof of a collision-free completed real maneuver. Restore the starting location and heading before replanning; do not reuse the midpoint pose as a new trial start.
