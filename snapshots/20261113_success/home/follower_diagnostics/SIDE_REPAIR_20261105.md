# Forward-corridor repair after the initial pivot

Source local_avoidance_trial_1793856109394840671.jsonl.
Stopped at local pose(.7735,.8313,.0364rad), PASS_BOX SELECT.
Current uncertainty-adjusted body clearance.34634m. Pending full forward leg
minimum.3219m; first10cm forward minimum.3343m. No direct straight continuation
passes checked_step's34cm reserve.
Old recovery only allowed current headings25..65deg. Here current heading2deg
prevented checking a viable diagonal correction. Initial plan box high.0903m;
current blocking returns y.18..21m. Why those world positions changed cannot be
resolved from this snapshot alone; not necessarily a moving object or noise.

New recovery tests small heading changes10..50deg, ordered by change from
current heading, plus current diagonal heading if applicable. Maximum2x10cm
repair steps remains. Each candidate checks complete rotation,10cm translation,
rotation back to forward, and all remaining forward corridor against all live
scan points and known box boundary. No obstacle points removed. Standard
34cm planning reserve unchanged, repaired forward leg must exceed36cm. Runtime
hard30cm and uncertainty/measured/quantized sweep checks remain unchanged.

Recorded stopped scene selects25deg, step minimum.3563m and following complete
forward corridor.3646m. Selection failure now logs all tested repair headings
and respective step/forward minima, rather than only the previous good choice.
No claim of physical success. Follower raw calibration1.1474 remains active only
in hardware driver, selected avoidance pose remains filtered.
