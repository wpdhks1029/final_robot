import math
import json
import unittest

from nav2_convoy_gate_core import Nav2ConvoyGateCore


class GateTests(unittest.TestCase):
    def test_measured_reverse_wait_keeps_zero_until_status_recovers(self):
        c=self.ready(gap=1.5,nav=(.06,.08),follower_v=0.,
            follower_reason='STOP_LEADER_REVERSE')
        c.update_follower(dict(trial_phase='FOLLOW',reason='FOLLOW_WAITING_LEADER_REVERSE',
            follower_follow_reason='STOP_LEADER_REVERSE',follower_requested_v=0.),10.)
        out=c.evaluate(10.05)
        self.assertEqual((out.linear_x,out.angular_z),(0.,0.))
        c.update_follower(dict(trial_phase='FOLLOW',reason='FOLLOW_WAITING_LEADER',
            follower_follow_reason='STOP_LEADER_STOPPED',follower_requested_v=0.),10.06)
        self.assertGreater(c.evaluate(10.07).linear_x,0.)
        c.update_nav(-.02,0.,10.08)
        self.assertEqual(c.evaluate(10.09).linear_x,0.)

    def test_rotation_wait_allows_nav2_turn_then_forward_with_gap_guard(self):
        c=self.ready(gap=1.5,nav=(0.,.08),follower_v=0.,
            follower_reason='STOP_LEADER_ROTATING')
        c.update_follower(dict(trial_phase='FOLLOW',reason='FOLLOW_WAITING_LEADER_ROTATION',
            follower_follow_reason='STOP_LEADER_ROTATING',follower_requested_v=0.),10.)
        out=c.evaluate(10.05)
        self.assertEqual(out.reason,'NAV2_FOLLOWER_GATE_OPEN')
        self.assertEqual(out.linear_x,0.)
        self.assertGreater(out.angular_z,0.)
        c.update_nav(.06,0.,10.06)
        self.assertGreater(c.evaluate(10.07).linear_x,0.)
        c.update_follower_pose(.4,0.,10.08)
        c.update_leader_pose(1.5,0.,10.08)
        c.update_nav(0.,.08,10.08)
        self.assertEqual(c.evaluate(10.09).reason,'STOP_ROTATION_GAP_TOO_SMALL')

    def test_optional_006_speed_and_startup_preserve_curvature(self):
        c=self.ready(nav=(.15,.08),follower_v=.1)
        c.max_linear=.06
        r=c.evaluate(10.05)
        self.assertAlmostEqual(r.linear_x,.06)
        self.assertAlmostEqual(r.angular_z*.15,r.linear_x*.08)
        c.update_follower(dict(trial_phase='FOLLOW',reason='FOLLOW_WAITING_LEADER',
            follower_follow_reason='STOP_LEADER_STOPPED',follower_requested_v=0.),10.)
        self.assertAlmostEqual(c.evaluate(10.05).linear_x,.06)

    def test_optional_005_speed_preserves_curvature_and_startup(self):
        c = self.ready(nav=(.15, .08), follower_v=.1)
        c.max_linear = .05
        r = c.evaluate(10.05)
        self.assertAlmostEqual(r.linear_x, .05)
        self.assertAlmostEqual(r.angular_z*.15, r.linear_x*.08)
        c.update_follower(dict(trial_phase='FOLLOW', reason='FOLLOW_WAITING_LEADER',
                               follower_follow_reason='STOP_LEADER_STOPPED',
                               follower_requested_v=0.), 10.)
        self.assertAlmostEqual(c.evaluate(10.05).linear_x, .04)

    def test_scaling_preserves_nav2_curvature_for_all_limits(self):
        for gap in (1.0, 1.3, 1.8):
            for v in (.006, .04, .15):
                for w in (-.4, -.08, 0., .08, .4):
                    for reason in ('FORWARD_ALLOWED', 'STOP_LEADER_STOPPED', 'STOP_FOLLOW_DISTANCE'):
                        c = self.ready(gap=gap, nav=(v, w), follower_reason=reason)
                        out = c.evaluate(10.05)
                        self.assertEqual(out.reason, 'NAV2_FOLLOWER_GATE_OPEN')
                        self.assertAlmostEqual(out.angular_z*v, out.linear_x*w)
                        self.assertLessEqual(out.linear_x, .04)
                        self.assertLessEqual(abs(out.angular_z), .10)

    def test_stale_input_stops_immediately_after_driving(self):
        c = self.ready()
        self.assertGreater(c.evaluate(10.05).linear_x, 0.)
        out = c.evaluate(10.26)
        self.assertEqual((out.linear_x, out.angular_z), (0., 0.))
        self.assertEqual(out.reason, 'STOP_NAV2_COMMAND_STALE')
        self.assertAlmostEqual(c.diagnostic(10.26, out)['receive_age_s']['nav'], .26)

    def test_diagnostic_is_json_safe_for_invalid_inputs(self):
        c = self.ready(nav=(math.nan, math.inf))
        out = c.evaluate(10.05)
        d = c.diagnostic(10.05, out)
        json.dumps(d, allow_nan=False)
        self.assertEqual(d['nav_command'], [None, None])

    def ready(self, gap=1.5, phase='FOLLOW', follower_reason='FORWARD_ALLOWED',
              follower_v=.02, nav=(.15, .08)):
        c = Nav2ConvoyGateCore()
        c.graph_ok = True
        c.update_nav(*nav, 10.)
        c.update_follower(dict(trial_phase=phase, reason='CONTINUOUS_READY',
                               follower_follow_reason=follower_reason,
                               follower_requested_v=follower_v), 10.)
        c.update_motor(dict(armed=True, fault=None, io_failed=False,
                            feedback_fresh=True), 10.)
        c.update_leader_pose(gap, 0., 10.)
        c.update_follower_pose(0., 0., 10.)
        return c

    def test_follow_caps_nav2_speed_and_turn(self):
        c = self.ready()
        out = c.evaluate(10.05)
        self.assertEqual(out.reason, 'NAV2_FOLLOWER_GATE_OPEN')
        self.assertAlmostEqual(out.linear_x, .023)
        self.assertLessEqual(abs(out.angular_z), .10)

    def test_planning_and_avoidance_hold_leader(self):
        for phase in ('PLAN_BRAKE', 'PLANNING', 'AVOID', 'STOP'):
            with self.subTest(phase=phase):
                c = self.ready(phase=phase)
                out = c.evaluate(10.05)
                self.assertEqual((out.linear_x, out.angular_z), (0., 0.))

    def test_fail_closed_on_graph_or_stale_inputs(self):
        c = self.ready()
        c.graph_ok = False
        self.assertEqual(c.evaluate(10.05).reason, 'STOP_GRAPH_NOT_EXCLUSIVE')
        c.graph_ok = True
        c.update_nav(.15, .08, 10.40)
        self.assertEqual(c.evaluate(10.40).reason, 'STOP_FOLLOWER_STATUS_STALE')

    def test_motor_fault_disarm_and_stale_stop(self):
        for update, expected in (
            (dict(armed=False, fault=None, io_failed=False, feedback_fresh=True),
             'STOP_FOLLOWER_MOTOR_UNHEALTHY'),
            (dict(armed=True, fault='FAULT', io_failed=False, feedback_fresh=True),
             'STOP_FOLLOWER_MOTOR_UNHEALTHY'),
        ):
            c = self.ready()
            c.update_motor(update, 10.)
            self.assertEqual(c.evaluate(10.05).reason, expected)
        c = self.ready()
        c.update_nav(.15, .08, 10.31)
        self.assertEqual(c.evaluate(10.31).reason, 'STOP_MOTOR_STATUS_STALE')

    def test_minimum_gap_is_hard_stop_and_soft_gap_scales(self):
        c = self.ready(gap=.84)
        self.assertEqual(c.evaluate(10.05).reason, 'STOP_CONVOY_GAP')
        c = self.ready(gap=1.025)
        out = c.evaluate(10.05)
        self.assertAlmostEqual(out.linear_x, .023 * .5)

    def test_nav_timeout_bad_values_and_pose_staleness_stop(self):
        c = self.ready()
        self.assertEqual(c.evaluate(10.26).reason, 'STOP_NAV2_COMMAND_STALE')
        c = self.ready(nav=(math.nan, 0.))
        self.assertEqual(c.evaluate(10.05).reason, 'STOP_NONFINITE_INPUT')
        c = self.ready()
        c.update_nav(.15, .08, 10.36)
        self.assertEqual(c.evaluate(10.36).reason, 'STOP_FOLLOWER_STATUS_STALE')
        c = self.ready()
        c.update_nav(.15, .08, 10.31)
        c.update_follower(dict(trial_phase='FOLLOW', reason='CONTINUOUS_READY',
                               follower_follow_reason='FORWARD_ALLOWED',
                               follower_requested_v=.02), 10.05)
        self.assertEqual(c.evaluate(10.31).reason, 'STOP_MOTOR_STATUS_STALE')

    def test_slow_leader_opening_target_gap_and_startup(self):
        c = self.ready(gap=1.1, follower_reason='STOP_FOLLOW_DISTANCE', follower_v=0.)
        out = c.evaluate(10.05)
        self.assertAlmostEqual(out.linear_x, .01 * (.25 / .35))
        c = self.ready(gap=1.5, follower_reason='STOP_LEADER_STOPPED', follower_v=0.)
        self.assertAlmostEqual(c.evaluate(10.05).linear_x, .04)
        c = self.ready(gap=1.1, follower_reason='STOP_NEAR_LEADER', follower_v=.02)
        self.assertEqual(c.evaluate(10.05).reason, 'STOP_FOLLOWER_TOO_CLOSE')

    def test_unsupported_reverse_and_tight_spin_stop(self):
        c = self.ready(nav=(-.02, 0.))
        self.assertEqual(c.evaluate(10.05).reason, 'STOP_UNSUPPORTED_NAV2_MANEUVER')
        c = self.ready(gap=1.0, nav=(0., .1))
        self.assertEqual(c.evaluate(10.05).reason, 'STOP_ROTATION_GAP_TOO_SMALL')

    def test_reobservation_and_terminal_status_hold(self):
        for reason in ('NEAR_RETURN_PAUSE', 'CONTINUOUS_RECHECK_COMPLETE'):
            c = self.ready()
            c.update_follower(dict(trial_phase='FOLLOW', reason=reason,
                                   follower_follow_reason='FORWARD_ALLOWED',
                                   follower_requested_v=.02), 10.)
            self.assertEqual(c.evaluate(10.05).reason, 'STOP_FOLLOWER_REOBSERVATION')
        c = self.ready(phase='STOP')
        self.assertEqual(c.evaluate(10.05).reason, 'STOP_FOLLOWER_NOT_IN_FOLLOW')


if __name__ == '__main__':
    unittest.main()
