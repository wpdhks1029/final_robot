# Short frame diagnostic and side-pass corrections

Frame check1793858373313526816 + IMU return1793858643094477410 completed with
both zero confirmed. User observed return to initial forward direction.
Pure return IMU-47.74deg, filtered-48.79deg. Rotation/20cm/return aggregate
scan comparison median nearest distance1.135cm; diagnostic rigid fit yaw-0.078deg,
position correction about1cm, residual0.584cm. Artifact:
logs/frame_round_trip_analysis_1793858743260533384.json.
This short static-scene registration does not rule out longer-run drift, moving
objects or ambiguous correspondence. No additional sensor calibration applied.

Latest failed trial1793857108908785996 exposed two controller issues:
1. PASS_BOX actual localy.82614 vs plannedy.80819. Aiming at the fixed waypoint
   requested-1.065deg, heading back towards the box. Full leg minimum.33989m
   fails34cm planning reserve. Holding actual lateral height with0deg straight
   gives minimum about.345m in the identical recorded cloud. Preserve actual
   lateral height while passing; map-path correction starts after body passage.
2. If live scan invalidates an already selected step, old FSM aborts before
   choosing the available safe replacement. Gentle controller now BRAKEs and
   enters SELECT; LocalCore requires3 actual wheel-zero samples before choosing
   again. Maximum2 blocked reselections without5cm progress and6 total/trial.
   If no checked candidate exists, stop. Original non-gentle controller unchanged.

No scan points removed. Runtime30cm hard clearance,34cm checked planning
reserve,36cm repaired forward corridor reserve, sourceTTL.8, motor deadlines.3,
RPM/speed limits and all-direction body checking remain unchanged. Existing
maximum2x10cm side repair retained. No new long unchecked side motion.

Validation:27 recorded-scene, side-repair, initial-heading, driver and full
measured-feedback memory rejoin→PP handoff checks passed. New replay proves
holding acquired height passes the formerly blocked cloud; new core test
proves moving/only2zero feedback cannot release replacement command.
Physical whole avoidance/rejoin/PP success is still unverified.

Next: frame diagnostic finished; restore physical starting poses and matching
RViz pose, keep normal driver(no distance-test flag), re-plan current obstacle
scene with --map-rejoin --continue-follow-after-rejoin --plan-only. Then bounded
physical run if ready, checking actual RViz/physical path rejoin, not just reason.
