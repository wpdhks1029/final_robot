"""Fail-closed gate between Nav2 and the leader motor topic.

Pure Python core: no ROS imports, motor access, or side effects. Nav2 is allowed
to drive the leader only while the follower trial is freshly in FOLLOW and the
follower driver is armed with fresh healthy feedback. During planning/avoidance,
or whenever an input is stale, this gate returns an immediate zero command.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class GateOutput:
    linear_x: float
    angular_z: float
    reason: str


class Nav2ConvoyGateCore:
    def __init__(self, *, nav_timeout_s=.25, follower_timeout_s=.35,
                 motor_timeout_s=.30, odom_timeout_s=.30,
                 max_linear=.04, max_angular=.10, min_gap=.85,
                 full_speed_gap=1.20):
        self.nav_timeout_s = float(nav_timeout_s)
        self.follower_timeout_s = float(follower_timeout_s)
        self.motor_timeout_s = float(motor_timeout_s)
        self.odom_timeout_s = float(odom_timeout_s)
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.min_gap = float(min_gap)
        self.full_speed_gap = float(full_speed_gap)
        if (not all(math.isfinite(x) and x > 0 for x in (
                self.nav_timeout_s, self.follower_timeout_s,
                self.motor_timeout_s, self.odom_timeout_s,
                self.max_linear, self.max_angular, self.min_gap,
                self.full_speed_gap)) or self.full_speed_gap <= self.min_gap):
            raise ValueError('INVALID_GATE_CONFIGURATION')
        self.nav = self.follower = self.motor = None
        self.leader = self.follower_pose = None
        self.graph_ok = False

    def update_nav(self, linear_x, angular_z, received_mono):
        self.nav = (float(linear_x), float(angular_z), float(received_mono))

    def update_follower(self, payload, received_mono):
        try:
            value = json.loads(payload) if isinstance(payload, str) else payload
            if not isinstance(value, dict):
                raise ValueError
            self.follower = (value, float(received_mono))
        except (ValueError, TypeError, json.JSONDecodeError):
            self.follower = None

    def update_motor(self, payload, received_mono):
        try:
            value = json.loads(payload) if isinstance(payload, str) else payload
            if not isinstance(value, dict):
                raise ValueError
            self.motor = (value, float(received_mono))
        except (ValueError, TypeError, json.JSONDecodeError):
            self.motor = None

    @staticmethod
    def _fresh(sample, now, timeout):
        return sample is not None and 0. <= now - sample[-1] <= timeout

    def update_leader_pose(self, x, y, received_mono):
        self.leader = (float(x), float(y), float(received_mono))

    def update_follower_pose(self, x, y, received_mono):
        self.follower_pose = (float(x), float(y), float(received_mono))

    def evaluate(self, now_mono):
        now = float(now_mono)
        if not self.graph_ok:
            return GateOutput(0., 0., 'STOP_GRAPH_NOT_EXCLUSIVE')
        if not self._fresh(self.nav, now, self.nav_timeout_s):
            return GateOutput(0., 0., 'STOP_NAV2_COMMAND_STALE')
        if not self._fresh(self.follower, now, self.follower_timeout_s):
            return GateOutput(0., 0., 'STOP_FOLLOWER_STATUS_STALE')
        status = self.follower[0]
        if status.get('trial_phase') != 'FOLLOW':
            return GateOutput(0., 0., 'STOP_FOLLOWER_NOT_IN_FOLLOW')
        if status.get('reason', '').startswith('STOP_'):
            return GateOutput(0., 0., 'STOP_FOLLOWER_SAFETY_STOP')
        if status.get('reason') in ('NEAR_RETURN_PAUSE', 'CONTINUOUS_RECHECK_COMPLETE'):
            return GateOutput(0., 0., 'STOP_FOLLOWER_REOBSERVATION')
        if not self._fresh(self.motor, now, self.motor_timeout_s):
            return GateOutput(0., 0., 'STOP_MOTOR_STATUS_STALE')
        motor = self.motor[0]
        if (motor.get('armed') is not True or motor.get('fault')
                or motor.get('io_failed') is not False
                or motor.get('feedback_fresh') is not True):
            return GateOutput(0., 0., 'STOP_FOLLOWER_MOTOR_UNHEALTHY')
        if (not self._fresh(self.leader, now, self.odom_timeout_s)
                or not self._fresh(self.follower_pose, now, self.odom_timeout_s)):
            return GateOutput(0., 0., 'STOP_ROBOT_POSE_STALE')
        lx, ly = self.leader[:2]
        fx, fy = self.follower_pose[:2]
        gap = math.hypot(lx - fx, ly - fy)
        v, w = self.nav[:2]
        if not all(math.isfinite(x) for x in (gap, v, w)):
            return GateOutput(0., 0., 'STOP_NONFINITE_INPUT')
        if gap <= self.min_gap:
            return GateOutput(0., 0., 'STOP_CONVOY_GAP')
        scale = min(1., max(0., (gap - self.min_gap) /
                            (self.full_speed_gap - self.min_gap)))
        linear_limit = self.max_linear
        # Keep leader speed near the follower's live requested speed so the
        # convoy gap does not grow while following. Allow only a slow startup
        # crawl while the follower is waiting for its first leader motion.
        follow_reason = status.get('follower_follow_reason')
        follower_v = status.get('follower_requested_v')
        if follow_reason == 'STOP_NEAR_LEADER':
            return GateOutput(0., 0., 'STOP_FOLLOWER_TOO_CLOSE')
        if follow_reason == 'STOP_LEADER_STOPPED':
            # PreviewCore needs measured leader speed >0.03 m/s before it
            # leaves STOP_LEADER_STOPPED. This bounded startup pulse is .04.
            linear_limit = min(linear_limit, .04)
        elif follow_reason == 'STOP_FOLLOW_DISTANCE':
            # The follower is intentionally waiting for the leader to open its
            # target gap. Move slowly until PP asks to follow again.
            linear_limit = min(linear_limit, .01)
        elif isinstance(follower_v, (int, float)) and math.isfinite(follower_v) and follower_v > 0:
            linear_limit = min(linear_limit, follower_v + .003)
        elif v > 1e-4:
            return GateOutput(0., 0., 'STOP_FOLLOWER_SPEED_UNAVAILABLE')
        # Reverse recovery remains disabled. Nav2's in-place alignment is
        # allowed only with a generous measured convoy gap.
        if v < -1e-4:
            return GateOutput(0., 0., 'STOP_UNSUPPORTED_NAV2_MANEUVER')
        if abs(w) > .02 and abs(v) < .005 and gap < 1.20:
            return GateOutput(0., 0., 'STOP_ROTATION_GAP_TOO_SMALL')
        # Apply every speed bound to both components together. Independent
        # clipping changes w/v after Nav2 has already smoothed the command.
        command_scale = min(1., linear_limit / abs(v) if v else 1.,
                            self.max_angular / abs(w) if w else 1.)
        scale *= command_scale
        return GateOutput(v * scale, w * scale, 'NAV2_FOLLOWER_GATE_OPEN')

    def diagnostic(self, now_mono, output):
        """Snapshot receipt ages and velocities without changing gate state."""
        def finite(value):
            return value if math.isfinite(value) else None
        sources = dict(nav=self.nav, follower_status=self.follower,
                       motor=self.motor, leader_pose=self.leader,
                       follower_pose=self.follower_pose)
        ages = {name: None if row is None else finite(now_mono-row[-1])
                for name, row in sources.items()}
        gap = None
        if self.leader is not None and self.follower_pose is not None:
            gap = finite(math.hypot(self.leader[0]-self.follower_pose[0],
                                    self.leader[1]-self.follower_pose[1]))
        return dict(reason=output.reason, graph_ok=self.graph_ok,
                    receive_age_s=ages, gap_m=gap,
                    nav_command=None if self.nav is None else [finite(x) for x in self.nav[:2]],
                    output_command=[output.linear_x, output.angular_z])
