# Outstanding zero reply vs new motion intent

Physical trial1793859382916676252 stopped during turn back to forward due to
FEEDBACK_STALE. Fault evidence: feedback age.31963s; callback interval.01982s;
zero response waiting.21982s;19byte parser prefix with last checksum absent;
no new bytes for.20005s. Full driver trace archived in
logs/motor_zero_reply_hold_audit_1793859382916676252.json.

Trace: moving request5549.7441, immediate zero preemption5549.7643,
19byte RX5549.7841, stale at5549.9841. Operator log kept sending negative TURN
intents throughout. Existing armed retry was permitted only when core.command
was exactSTOP; changing the pending intent toTURN disabled that retry despite
an outstanding zero transaction. This describes the uncovered handling path;
physical source of the missing checksum remains unconfirmed.

LocalAvoidanceContract now validates each envelope as before, then clamps a
nonzero request to actual zero while last_sent iszero and a response is pending.
No motion intent is queued; a later fresh envelope is needed after full feedback.
This preserves eligibility of the existing single zero-only retry. No nonzero
retransmission, synthetic byte, restamping, timeout extension or automatic rearm.
Original sensor/feedback/response/command and body clearance guards unchanged.
Normal core/manual/guard behavior not changed. Generic local and dedicated
bounded distance mode both retain this hold before new motion.

Status zero_reply_hold_supported/active added. Operator detects an old driver
before arm with REQUIRE_ZERO_REPLY_HOLD_DRIVER_RESTART. Restart normal
local_avoidance driver, without the distance-test ignore-clearance flag.

Tests:43 other driver/retry/operator/full memory-handoff checks passed;
2 dedicated hold tests pass including both pivot signs, exact19byte prefix,
original deadline preservation and persistent loss fault without rearming.
One dedicated test was initially too strict after full feedback (waiting_since
correctly becameNone); assertion narrowed to the outstanding-reply interval.
Physical recovery is still unverified and serial truncation may recur.
