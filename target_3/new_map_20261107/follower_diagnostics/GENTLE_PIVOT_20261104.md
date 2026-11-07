# Smaller initial pivot candidate

Explicit flag: `--apf-pivot --gentle-pivot --pose-source filtered`.
The original APF pivot remains available without the new flag. This candidate
uses a geometry-checked diagonal route, not an unconstrained APF force heading.

Search 20..80 degrees in 5 degree increments; select the smallest angle whose
entire diagonal, pivot to forward, forward bypass, diagonal rejoin, and final
forward 1.1m remain clear against all captured points and known box boundary.
Require >40cm static planning clearance, keeping the existing 30cm live hard
stop, full rectangular swept-body checks, source lifetime and motor deadlines.
Offset is box high edge +74cm. Stop and confirm zero feedback between phases.
Live selection checks the remaining leg and its next corner; each motor cycle
retains existing live scan and swept-command checks. No unchecked fallback.

Recorded filtered scene: local_avoidance_trial_1793781323794712578.jsonl.
Original initial heading 55 degrees; accepted candidate 45 degrees.
An earlier 40-degree candidate failed noisy replay near the corner; it is not
used. Final three seeds 7/11/19 with 3mm pose noise and 100ms delay completed
body passage, rejoin, and 1m forward, minimum simulated clearance 39.5..40.7cm,
virtual time 161.8..165.3s. Artifact:
logs/gentle_pivot_replay_1793782479754812142.json.
Static captured scene, ideal communication, follower-only kinematic replay;
these results are not a physical evaluation pass or wireless validation.

Reproduce: python3 -B verify_gentle_pivot.py
Tests: python3 -m unittest test_gentle_pivot.Tests test_gentle_pivot.CapturedFilteredMemory -q
The captured-scene memory test uses the actual local command contract and
feedback/zero gates, filtered pose and deliberately under-reporting raw yaw.

Plan only on current scene before bounded physical trial:
python3 -B ~/follower_diagnostics/run_local_avoidance_trial.py --plan-only --apf-pivot --gentle-pivot --pose-source filtered --seconds 360
