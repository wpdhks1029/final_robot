import math
import unittest
from types import SimpleNamespace as S

from tracked_pursuit_candidate import TrackedPreviewCore
from follower_stamped_preview import oldest_pursuit_source
from nav2_convoy_gate_core import Nav2ConvoyGateCore


def path_message(points, stamp_ns=10_000_000_000, frame='map'):
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    poses = []
    for x, y in points:
        poses.append(S(header=S(frame_id=frame), pose=S(position=S(x=x, y=y))))
    return S(header=S(frame_id=frame, stamp=S(sec=sec, nanosec=nanosec)), poses=poses)


class Nav2GoalPathTests(unittest.TestCase):
    def make_core(self):
        core = TrackedPreviewCore(.8)
        stamp = S(sec=10, nanosec=0)
        core.inputs['leader'] = (stamp, 100., (2., 0., 0., .05, 0.))
        core.inputs['follower'] = (stamp, 100., (-1.5, 0., 0., 0., 0.))
        core.leader_moving = True
        return core

    def test_valid_map_goal_plan_drives_forward_pursuit(self):
        core = self.make_core()
        core.path_callback(path_message([(0., 0.), (1., 0.), (3., .2), (5., .5)]),
                           10_000_000_000, 100.)
        v, w, reason = core.command(10_000_000_000, 100.)
        self.assertEqual(reason, 'TRACKED_PURSUIT')
        self.assertGreater(v, 0.)
        self.assertLessEqual(v, .10)
        self.assertTrue(math.isfinite(w))
        self.assertLessEqual(abs(w), .20)

    def test_missing_plan_stays_stopped(self):
        core = self.make_core()
        self.assertEqual(core.command(10_000_000_000, 100.),
                         (0., 0., 'STOP_MISSING_OR_INVALID_PATH'))

    def test_wrong_frame_plan_is_rejected(self):
        core = self.make_core()
        core.path_callback(path_message([(0., 0.), (2., 0.)], frame='odom'),
                           10_000_000_000, 100.)
        self.assertEqual(core.command(10_000_000_000, 100.),
                         (0., 0., 'STOP_MISSING_OR_INVALID_PATH'))

    def test_stale_plan_is_rejected(self):
        core = self.make_core()
        core.path_callback(path_message([(0., 0.), (2., 0.)], stamp_ns=1_000_000_000),
                           10_000_000_000, 100.)
        self.assertEqual(core.command(10_000_000_000, 100.),
                         (0., 0., 'STOP_MISSING_OR_INVALID_PATH'))

    def test_changed_goal_plan_fails_closed_during_trial(self):
        core = self.make_core()
        core.path_callback(path_message([(0., 0.), (2., 0.), (5., 0.)]),
                           10_000_000_000, 100.)
        self.assertEqual(core.command(10_000_000_000, 100.)[2], 'TRACKED_PURSUIT')
        core.path_callback(path_message([(0., 0.), (0., 2.), (0., 5.)]),
                           10_000_000_000, 100.)
        command = core.command(10_000_000_000, 100.)
        self.assertEqual(command[:2], (0., 0.))
        self.assertEqual(command[2], 'STOP_PATH_SESSION_CHANGED_RESTART')

    def test_plan_is_session_persistent_but_sensor_command_stamp_stays_fresh(self):
        core = self.make_core()
        core.path_callback(path_message([(0., 0.), (2., 0.), (5., 0.)]),
                           10_000_000_000, 100., persistent=True)
        self.assertEqual(core.command(10_000_000_000, 100.)[2], 'TRACKED_PURSUIT')

        current = S(sec=20, nanosec=0)
        core.inputs['leader'] = (current, 110., (2., 0., 0., .05, 0.))
        core.inputs['follower'] = (current, 110., (-1.5, 0., 0., 0., 0.))
        command = core.command(20_000_000_000, 110.)
        self.assertEqual(command[2], 'TRACKED_PURSUIT')
        source = oldest_pursuit_source(core.inputs, remote_clock_margin_ns=50_000_000,
                                       include_path=False)
        self.assertEqual(source, 19_950_000_000)

    def test_stale_republished_goal_plan_invalidates_persistent_session(self):
        core = self.make_core()
        core.path_callback(path_message([(0., 0.), (2., 0.)]),
                           10_000_000_000, 100., persistent=True)
        core.path_callback(path_message([(0., 0.), (2., 0.)], stamp_ns=1_000_000_000),
                           10_000_000_000, 101., persistent=True)
        self.assertEqual(core.command(10_000_000_000, 101.),
                         (0., 0., 'STOP_MISSING_OR_INVALID_PATH'))

    def test_valid_goal_path_opens_only_bounded_follower_gate(self):
        pursuit = self.make_core()
        pursuit.path_callback(path_message([(0., 0.), (2., 0.), (5., .2)]),
                              10_000_000_000, 100., persistent=True)
        follower_v, follower_w, follow_reason = pursuit.command(10_000_000_000, 100.)
        self.assertEqual(follow_reason, 'TRACKED_PURSUIT')
        gate = Nav2ConvoyGateCore()
        gate.graph_ok = True
        gate.update_nav(.15, .08, 100.)
        gate.update_follower(dict(trial_phase='FOLLOW', reason='LOCAL_AVOIDANCE_READY',
            follower_follow_reason=follow_reason, follower_requested_v=follower_v), 100.)
        gate.update_motor(dict(armed=True, fault=None, io_failed=False,
            feedback_fresh=True), 100.)
        gate.update_leader_pose(1.8, 0., 100.)
        gate.update_follower_pose(0., 0., 100.)
        output = gate.evaluate(100.05)
        self.assertEqual(output.reason, 'NAV2_FOLLOWER_GATE_OPEN')
        self.assertGreater(output.linear_x, 0.)
        self.assertLessEqual(output.linear_x, .04)
        self.assertLessEqual(abs(output.angular_z), .10)

    def test_missing_goal_path_keeps_both_robots_held(self):
        pursuit = self.make_core()
        follower_v, follower_w, follow_reason = pursuit.command(10_000_000_000, 100.)
        self.assertEqual(follow_reason, 'STOP_MISSING_OR_INVALID_PATH')
        gate = Nav2ConvoyGateCore()
        gate.graph_ok = True
        gate.update_nav(.15, .08, 100.)
        gate.update_follower(dict(trial_phase='FOLLOW', reason=follow_reason,
            follower_follow_reason=follow_reason, follower_requested_v=follower_v), 100.)
        gate.update_motor(dict(armed=True, fault=None, io_failed=False,
            feedback_fresh=True), 100.)
        gate.update_leader_pose(1.8, 0., 100.)
        gate.update_follower_pose(0., 0., 100.)
        output = gate.evaluate(100.05)
        self.assertEqual(output.linear_x, 0.)
        self.assertEqual(output.angular_z, 0.)


if __name__ == '__main__':
    unittest.main()
