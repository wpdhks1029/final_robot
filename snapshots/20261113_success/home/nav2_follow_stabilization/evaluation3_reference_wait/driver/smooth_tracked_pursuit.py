"""Optional continuous gap/speed controller; no ROS or hardware interfaces.

Freshness, near-gap, reverse, rotation and path-geometry stops are inherited.
Only the ordinary leader-stopped/distance speed policy is replaced.
"""
import math
from tracked_pursuit_candidate import TrackedPreviewCore


class SmoothTrackedPreviewCore(TrackedPreviewCore):
    def __init__(self, lookahead=.8):
        super().__init__(lookahead)
        self.filtered_leader_v=0.
        self.filtered_stamp=None
        self.acceleration=.12
        self.deceleration=.20
        self.gap_deadband=.05
        self.speed_diagnostic={}
        self.reset_motion()

    def reset_motion(self):
        self.previous_v=0.
        self.previous_w=0.
        self.previous_at=None

    def speed_gate(self, now_ns, received):
        _,_,reason=super().speed_gate(now_ns,received)
        if reason not in ('FORWARD_ALLOWED','STOP_LEADER_STOPPED','STOP_FOLLOW_DISTANCE'):
            self.reset_motion()
            self.speed_diagnostic=dict(reason=reason)
            return 0.,0.,reason
        row=self.inputs['leader']
        ns=row[0].sec*10**9+row[0].nanosec
        measured=max(0.,min(self.max_linear,row[2][3]))
        if self.filtered_stamp is None or ns<self.filtered_stamp or ns-self.filtered_stamp>300_000_000:
            self.filtered_leader_v=measured
        elif ns>self.filtered_stamp:
            dt=(ns-self.filtered_stamp)/1e9
            alpha=1.-math.exp(-dt/.30)
            self.filtered_leader_v+=alpha*(measured-self.filtered_leader_v)
        self.filtered_stamp=ns
        gap=math.hypot(row[2][0]-self.inputs['follower'][2][0],
                       row[2][1]-self.inputs['follower'][2][1])
        error=gap-self.follow_distance
        # Continuous dead zone: centimetre-scale gap changes must not subtract
        # all of a small but nonzero leader speed. Outside the band, subtract
        # only the excess error so there is no velocity step at the boundary.
        excess=math.copysign(max(0.,abs(error)-self.gap_deadband),error)
        correction=.5*excess
        desired=max(0.,min(self.max_linear,self.filtered_leader_v+correction))
        self.speed_diagnostic=dict(gap_m=gap,target_gap_m=self.follow_distance,
            gap_deadband_m=self.gap_deadband,gap_error_m=error,
            gap_correction_mps=correction,filtered_leader_v_mps=self.filtered_leader_v,
            raw_leader_v_mps=row[2][3],desired_v_mps=desired,leader_source_ns=ns)
        return desired,0.,'SMOOTH_FORWARD_ALLOWED' if desired>0. else 'SMOOTH_GAP_HOLD'

    def limit_drive(self, v, w, reason, mono):
        if reason not in ('TRACKED_PURSUIT','SMOOTH_GAP_HOLD'):
            self.reset_motion()
            return 0.,0.,reason
        if self.previous_at is None:
            dt=0.
        elif not 0<=mono-self.previous_at<=.25:
            self.reset_motion();dt=0.
        else:
            dt=min(.05,mono-self.previous_at)
        curvature=w/v if v>0. else (self.previous_w/self.previous_v if self.previous_v>0. else 0.)
        low=max(0.,self.previous_v-self.deceleration*dt)
        high=self.previous_v+self.acceleration*dt
        result=max(low,min(high,v))
        self.previous_v=result;self.previous_w=curvature*result;self.previous_at=mono
        return result,self.previous_w,'TRACKED_PURSUIT' if result>0. else reason

    def command(self, now_ns, received):
        v,w,reason=super().command(now_ns,received)
        return self.limit_drive(v,w,reason,received)

    def feedback_bounded(self, v, w, mono):
        """Anchor the next ramp to the final RPM-bounded output, not PP intent."""
        self.previous_v=v;self.previous_w=w;self.previous_at=mono
