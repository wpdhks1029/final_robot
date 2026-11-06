# APF-guided pivot candidate — offline and observation only

Implemented in apf_pivot_candidate.py. The local motor contract now includes the zero-only startup handshake described below; restart the driver to load it. The local operator now offers an explicit --apf-pivot flag; the default controller remains unchanged.

The candidate combines APF heading preference, locked left bypass, intermediate attraction targets, and a discrete heading feasibility search. It is not pure APF. During this bounded candidate it returns to the straight reference line and continues for the additional 1m; general curved-route Pure Pursuit resumption is not implemented. Every rotation and 10cm straight segment is checked against every recorded point with the full 0.54x0.60m rectangle, 0.33m planning clearance and an extra 1cm sample bound. The replay retains the existing 0.30m hard guard, 1cm pose uncertainty, integer RPM, one-second swept check, bounded 10RPM and explicit direction-change braking. Minimum executable pivot is 1RPM. The final reference target advances forward to prevent circling at a fixed endpoint.

Replay source: logs/local_avoidance_trial_1793770821520744310.jsonl, initial planning_input and checked_local_plan point cloud, including known 20cm box depth. No measured point is removed or translated.

History is retained:
- apf_pivot_replay_1793771315497708421.json: all 3 failed by time limit, circling at final attraction target.
- apf_pivot_replay_1793771345033112268.json: numerical completion only after 11–12m forward travel; unsuitable for this scene. Rejected as a useful candidate despite the old geometric result label.
- apf_pivot_replay_1793771375614636379.json: final reference tracking correction; seeds 7,11,19 complete body pass, rejoin and additional 1m in 206.0,197.8,200.5s; minimum conservative clearance 0.3842,0.3926,0.3895m. Final x=3.62–3.70m. New replays also enforce x<=4.5m and |y|<=1.25m.

Model limitations: fixed initial scan, unseen regions not modelled, follower only (no moving leader), ideal fresh input, 3mm Gaussian pose noise, 0.1s delay, no unequal wheel slip, no radio losses. These are simulations, not quantitative evaluation passes. The snapshot model cannot explain the changed live rear-right return in the failed physical turn. Current-scan observation is required before motion integration.

Tests: python3 -m unittest test_apf_pivot_candidate -q (6 passing). Tests cover rear corner rotation obstruction, diagonal selection, blocked stop, turn/drive brake, forward continuation at old final target, and absence of control publisher/service calls in the shadow tool.

With both robots stationary and the follower restored to the trial starting location and heading:

    python3 -B ~/follower_diagnostics/watch_apf_pivot_shadow.py --seconds 20

This subscribes to scan/TF and saves complete observed point clouds with candidate headings in logs/apf_pivot_shadow_*.json. It publishes no command, requests no arm, and calls no motor service. Existing local stop/turn/go trials without --apf-pivot still use the old controller.

## Bounded live pilot integration

run_local_avoidance_trial.py --apf-pivot uses the unchanged LOCAL_STOP_TURN_GO_V1 bounded motor contract and source/command/feedback timeouts. Result controller_kind is APF_PIVOT_STEP_V1. Initial geometry covers the first step only; it is not a complete global route certificate. At confirmed zero wheel feedback, a new heading is selected; during rotation/drive the pending complete rotation/step is rechecked against the current scan and the retained known box boundary. The existing quantized one-second body sweep remains authoritative. New directions cannot skip three zero feedback samples. Every next 10cm step is selected again. A failed safety check stops the session, without autonomous retry/rearm. Stop logs now include current point cloud and control diagnostics.

Current user observation: logs/apf_pivot_shadow_1793771828792258657.json, all 19 samples select 65 degrees, checked clearance 0.40388–0.40781m, compute 0.0149–0.0471s, source age after compute 0.1376–0.3576s (within existing 0.8s lifetime).

Current scene full memory driver/contract run: logs/apf_pivot_memory_1793772097378034195.json. Body pass, reference rejoin and additional 1m completed at 207.55 virtual seconds; min guarded clearance 0.37984m, 4147 accepted envelopes, 220 phase brake ticks, final RPM [0,0]. No live services or motor traffic were used. The normal bounded motor envelope, including post-arm source timestamps, is checked in this run. Moving leader geometry, live drift and wireless behavior remain physical test requirements.

With the existing local_avoidance 10RPM/360s driver, and both robots restored to the initial test configuration:

    python3 -B ~/follower_diagnostics/run_local_avoidance_trial.py --run --ack-detour-area-power-cut-ready --apf-pivot --seconds 360

The startup must show APF_PIVOT_STEP_V1. The log reports APF target heading at each phase. Do not interpret TRIAL_TIME_LIMIT as proof of obstacle clearance; whole geometry is separately reported as GEOMETRIC_PASS_REJOIN_PLUS_1M and still evaluation_pass=false.

## Post-arm startup correction

The physical trial local_avoidance_trial_1793772205412738028.jsonl stopped with POST_ARM_MEASUREMENTS_NOT_READY without motor fault or motion. The old 0.25s startup wait was shorter than the supported sensor/interpolation latency. The operator now waits up to the existing 0.8s source lifetime, continuously publishing exact zero STOP envelopes with fresh post-arm headers and current epoch. Zero STOP alone may reference pre-arm source/scan stamps, still <=0.8s old and <=header. Every nonzero DRIVE/TURN continues requiring both measurements after the current arm. No measurement is restamped. Header, command, feedback and serial response watchdogs remain 0.3s.

The driver advertises zero_stop_prearm_supported=true. An old loaded driver is rejected before leader start/arm, requiring a restart. Waiting is logged as post_arm_wait with source owners. Sensor support delayed 0.4s is reproduced in the fake operator and now produces only zero until ready; nonzero then requires post-arm measurements. Stale stop, pre-arm nonzero, nonzero STOP and missing-command watchdog rejection remain tested. Current targeted suite: 31 tests passed, including the full APF memory contract/control run. Existing live driver process must be restarted to load this contract update.

## Robot-anchor startup alignment correction

Reported diagnostic in local_avoidance_trial_1793773570343396504.jsonl: follower lateral=0.107877m, leader lateral=0.094633m, yaw errors -0.790/-1.093 degrees. The two startup anchors were incorrectly treated as a measured leader path, applying the stricter 0.10m path test to a robot pose allowed up to 0.15m. reference_from_plan now takes an explicit source_kind. Only the no-valid-path fallback may use robot_anchors, and the exact two points must match the current follower/leader poses. Those anchors retain the 0.15m lateral and 20-degree heading checks; actual live paths retain their 0.10m limit. The saved line is never shifted sideways, replaced or re-fitted. The source kind is included in failed-reference diagnostics. Same reported inputs now pass anchor validation while the same points tagged as live_path still fail. Arbitrary anchor substitution and excessive robot lateral/yaw remain rejected. Reference refresh, diagnostics and operator regression: 23 tests passed. Driver restart is not needed for this operator/reference-helper correction.

## Optional zero-output near-return hold

Trial local_avoidance_trial_1793774222043810461.jsonl provides 24 raw scans: 19 prior scans without points within 0.30m of the body; one 0.334m return at -79.5 degrees (index144), two clear scans, one 0.301m return in the same bin, then clear. This establishes intermittency, not proof that no physical object existed. Extracted evidence: apf_near_return_evidence_1793774222043810461.json.

Explicit --near-return-hold (requires --apf-pivot) now converts a footprint intrusion into immediate zero STOP output, never point deletion. While paused, all normal input freshness, body, pending-step and quantized swept checks still run. Resume requires 5 distinct new scans with >=0.35m uncertainty-adjusted current body clearance and >0.30m swept clearance, LOCAL_AVOIDANCE_READY including the pending-step validation, and actual RPM[0,0] with 3 zero feedback samples. Any unsafe observation resets the streak. Reusing one scan cannot advance it. At 2 seconds without resume, the session aborts; unrelated safety faults abort immediately. The 360s trial deadline, 0.8s source life and 0.3s device watchdogs are unchanged; there is no rearm. Every near return first causes zero output, including confirmed real obstacles. Hold start/resume evidence is recorded.

Validation: 18 distinct tests passed (17 operator/pause/evidence tests plus the full APF memory-driver test; 6 pause units overlap the separate 7-test memory invocation). The memory run injected two short near-return bursts during the actual APF/control/driver-contract loop, verified zero while paused, a single start/resume pair, and body-clear rejoin plus1m completion. This is memory-only evidence, not a physical evaluation result. Driver is unchanged for this optional operator behavior. Restore start position and heading before a new bounded trial.

    python3 -B ~/follower_diagnostics/run_local_avoidance_trial.py --run --ack-detour-area-power-cut-ready --apf-pivot --near-return-hold --seconds 360
