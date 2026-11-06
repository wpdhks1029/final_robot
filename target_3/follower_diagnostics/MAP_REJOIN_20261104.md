# Live map/path rejoin candidate

Incident: local_avoidance_trial_1793783686459932449 reported completion against
frozen reference coordinates. User observed no rejoin. Later stationary audit
rejoin_map_audit_1793784139085451761 measured49.6cm path distance and55.5cm
map-vs-projected-odom offset,11.1deg heading mismatch. The old trial did not log
live map/path throughout; this is not a historical localization drift trace.
Original success is not evidence of physical rejoin.

Explicit opt-in: --apf-pivot --gentle-pivot --pose-source filtered --map-rejoin.
Bypass remains checked45deg/10cm stop-turn-step. After side passage, rejoin
selection projects measured map position onto live /leader/path and targets a
point0.5m forward along its segments. Never extends beyond leader history.
Map pose and filtered pose are matched at the measured map timestamp; target
is transformed into local control coordinates. Scans, EKF and mount TF remain
untouched. Same whole-body full-leg/corner/live-command clearance guards apply.
Missing/stale map/path or missing measured odom support stops the run. Abrupt
map pose jumps stop; no automatic pose reset/rearm. Sources retain old stamps.

Completion in this mode uses fresh measured map position within10cm and10deg
of actual leader path, whole-body passage of box, and1m net forward measured
map progress. Frozen-frame mismatch remains logged but is not treated as a
requirement to move to the wrong frozen line. Original mode retains its
independent mismatch check. evaluation_pass remainsfalse.

Validation: test_map_rejoin_guidance.Tests (4), MapMemory (1), plus19 map
verification/operator regressions. Full actual-memory-contract test injects
55cm map/local offset; body passage, map-path rejoin and1m progress complete.
Transform unit test injects55.5cm offset and11.1deg yaw discrepancy. Tests also
reject stale/missing inputs, missing pose timestamp support and map jumps.
Physical behavior and localization accuracy remain to be tested.
