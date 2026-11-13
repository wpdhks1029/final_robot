import unittest
import math
import numpy as np

from nav2_follow_safety import build_scene, bounded_pursuit_result


class FollowSafetyTests(unittest.TestCase):
    def test_optional_speed_floor_preserves_curvature_and_does_not_override_stops(self):
        r=bounded_pursuit_result(self.scene(),.005,.001,'TRACKED_PURSUIT',
            min_linear=.04,max_linear=.06)
        self.assertAlmostEqual(r['v'],.04)
        self.assertAlmostEqual(r['w']/r['v'],.2)
        for reason in ('STOP_STALE_LEADER','STOP_NEAR_LEADER','STOP_LEADER_REVERSE'):
            stopped=bounded_pursuit_result(self.scene(),.005,.001,reason,
                min_linear=.04,max_linear=.06)
            self.assertEqual((stopped['v'],stopped['w']),(0.,0.))
        waiting=bounded_pursuit_result(self.scene(),0.,0.,'STOP_LEADER_STOPPED',
            min_linear=.04,max_linear=.06)
        self.assertEqual(waiting['v'],0.)
        blocked=bounded_pursuit_result(self.scene(point=(.5,0.)),.005,0.,
            'TRACKED_PURSUIT',min_linear=.04,max_linear=.06)
        self.assertEqual(blocked['reason'],'STOP_FOLLOW_CLEARANCE')

    def test_floor_candidate_still_keeps_wheel_limit_and_default_is_unchanged(self):
        original=bounded_pursuit_result(self.scene(),.005,0.,'TRACKED_PURSUIT')
        self.assertAlmostEqual(original['v'],.005)
        for curvature in (-3.,0.,3.):
            r=bounded_pursuit_result(self.scene(),.005,.005*curvature,
                'TRACKED_PURSUIT',min_linear=.04,max_linear=.06)
            rpm=max(abs(r['v']-.28*r['w']),abs(r['v']+.28*r['w']))/.065*60/(2*math.pi)
            self.assertLessEqual(rpm,10.+1e-9)

    def test_explicit_clearance_disable_retains_real_distance_and_other_stops(self):
        scene=self.scene(point=(.50,0.))
        result=bounded_pursuit_result(scene,.06,0.,'TRACKED_PURSUIT',
            max_linear=.06,enforce_clearance=False)
        self.assertEqual(result['motion'],'DRIVE')
        self.assertLess(result['clearance_m'],.3)
        self.assertFalse(result['clearance_stop_enabled'])
        for reason in ('STOP_STALE_LEADER','STOP_NEAR_LEADER'):
            stopped=bounded_pursuit_result(scene,.06,0.,reason,enforce_clearance=False)
            self.assertEqual(stopped['reason'],reason)
            self.assertEqual(stopped['v'],0.)

    def test_reverse_wait_never_requests_motion_and_can_resume(self):
        waiting=bounded_pursuit_result(self.scene(),.06,.1,'STOP_LEADER_REVERSE',
            max_linear=.06,allow_leader_reverse_wait=True)
        self.assertEqual(waiting['reason'],'FOLLOW_WAITING_LEADER_REVERSE')
        self.assertEqual((waiting['v'],waiting['w'],waiting['motion']),(0.,0.,'STOP'))
        blocked=bounded_pursuit_result(self.scene(point=(.5,0.)),0.,0.,
            'STOP_LEADER_REVERSE',allow_leader_reverse_wait=True)
        self.assertEqual(blocked['reason'],'STOP_FOLLOW_CLEARANCE')
        resumed=bounded_pursuit_result(self.scene(),.06,0.,'TRACKED_PURSUIT',
            max_linear=.06,allow_leader_reverse_wait=True)
        self.assertEqual(resumed['motion'],'DRIVE')
        legacy=bounded_pursuit_result(self.scene(),0.,0.,'STOP_LEADER_REVERSE')
        self.assertEqual(legacy['reason'],'STOP_LEADER_REVERSE')

    def test_nav2_leader_rotation_waits_at_zero_then_resumes_forward(self):
        scene=self.scene()
        waiting=bounded_pursuit_result(scene,.06,.1,'STOP_LEADER_ROTATING',
            max_linear=.06,allow_leader_rotation_wait=True)
        self.assertEqual(waiting['reason'],'FOLLOW_WAITING_LEADER_ROTATION')
        self.assertEqual((waiting['v'],waiting['w'],waiting['motion']),(0.,0.,'STOP'))
        resumed=bounded_pursuit_result(scene,.06,0.,'TRACKED_PURSUIT',
            max_linear=.06,allow_leader_rotation_wait=True)
        self.assertEqual(resumed['motion'],'DRIVE')

    def test_rotation_wait_keeps_exterior_obstacle_stop_and_other_faults(self):
        blocked=bounded_pursuit_result(self.scene(point=(.5,0.)),0.,0.,
            'STOP_LEADER_ROTATING',allow_leader_rotation_wait=True)
        self.assertEqual(blocked['reason'],'STOP_FOLLOW_CLEARANCE')
        for reason in ('STOP_LEADER_REVERSE','STOP_STALE_LEADER','STOP_NEAR_LEADER'):
            result=bounded_pursuit_result(self.scene(),.06,0.,reason,
                allow_leader_rotation_wait=True)
            self.assertEqual(result['reason'],reason)
            self.assertEqual((result['v'],result['w']),(0.,0.))
        legacy=bounded_pursuit_result(self.scene(),0.,0.,'STOP_LEADER_ROTATING')
        self.assertEqual(legacy['reason'],'STOP_LEADER_ROTATING')

    def filtered_scene(self,points,pose=(0.,0.,0.,0.,0.)):
        return build_scene(pose_row=(1_000_000_000,10.,pose),
            scan_row=(1_100_000_000,10.1,points),now_ns=1_200_000_000,
            now_mono=10.2,exclude_body_internal=True)

    def test_internal_returns_are_excluded_without_modifying_raw_points(self):
        points=np.array([[-.115,.083],[-.149,.086],[2.,0.]])
        original=points.copy()
        scene=self.filtered_scene(points)
        self.assertEqual(len(scene['excluded_body_points']),2)
        np.testing.assert_array_equal(points,original)
        self.assertEqual(scene['points'].shape,(1,2))
        result=bounded_pursuit_result(scene,.06,0.,'TRACKED_PURSUIT',max_linear=.06)
        self.assertEqual(result['motion'],'DRIVE')
        self.assertAlmostEqual(result['v'],.06)

    def test_boundary_and_near_exterior_points_still_stop(self):
        for point in ((.27,0.),(-.27,0.),(0.,.30),(0.,-.30),(.50,0.)):
            result=bounded_pursuit_result(self.filtered_scene([(-.1,.08),point]),
                .06,0.,'TRACKED_PURSUIT',max_linear=.06)
            self.assertEqual(result['reason'],'STOP_FOLLOW_CLEARANCE')

    def test_filter_uses_body_coordinates_before_rotated_map_projection(self):
        scene=self.filtered_scene([(-.115,.083),(2.,0.)],pose=(3.,4.,math.pi/2,0.,0.))
        np.testing.assert_allclose(scene['points'],[[3.,6.]])

    def test_scan_with_only_internal_returns_stops(self):
        with self.assertRaisesRegex(ValueError,'FOLLOW_SCAN_NO_EXTERNAL_POINTS'):
            self.filtered_scene([(-.115,.083)])

    def test_006_curves_keep_ten_rpm_limit_and_external_stop(self):
        for w in (-.2,0.,.2):
            r=bounded_pursuit_result(self.scene(),.1,w,'TRACKED_PURSUIT',max_linear=.06)
            rpm=max(abs(r['v']-.28*r['w']),abs(r['v']+.28*r['w']))/.065*60/(2*math.pi)
            self.assertLessEqual(rpm,10.+1e-9)
            self.assertLessEqual(r['v'],.06)

    def test_optional_005_speed_keeps_rpm_and_obstacle_bounds(self):
        straight = bounded_pursuit_result(self.scene(), .1, 0., 'TRACKED_PURSUIT', max_linear=.05)
        self.assertAlmostEqual(straight['v'], .05)
        for w in (-.2, 0., .2):
            r = bounded_pursuit_result(self.scene(), .1, w, 'TRACKED_PURSUIT', max_linear=.05)
            rpm = max(abs(r['v']-.28*r['w']), abs(r['v']+.28*r['w']))/.065*60/(2*math.pi)
            self.assertLessEqual(rpm, 10.+1e-9)
        blocked = bounded_pursuit_result(self.scene(point=(.50, 0.)), .05, 0.,
                                         'TRACKED_PURSUIT', max_linear=.05)
        self.assertEqual((blocked['v'], blocked['w']), (0., 0.))

    def scene(self, point=(2.0, 0.0), pose_stamp=1_000_000_000,
              scan_stamp=1_100_000_000, now_ns=1_200_000_000):
        return build_scene(
            pose_row=(pose_stamp, 10.0, (0.0, 0.0, 0.0, 0.0, 0.0)),
            scan_row=(scan_stamp, 10.1, [point]),
            now_ns=now_ns, now_mono=10.2)

    def test_map_pose_and_base_scan_make_fresh_scene_without_odom_scan_bracket(self):
        scene = self.scene()
        self.assertAlmostEqual(scene['pose_scan_delta_s'], .1)
        self.assertGreater(scene['clearance'], .3)
        self.assertEqual(scene['source_stamps']['follower_map'], 1_000_000_000)

    def test_numpy_scan_points_are_accepted(self):
        scene = build_scene(
            pose_row=(1_000_000_000, 10.0, (0.0, 0.0, 0.0, 0.0, 0.0)),
            scan_row=(1_100_000_000, 10.1, np.asarray([[2.0, 0.0], [2.0, 0.1]])),
            now_ns=1_200_000_000, now_mono=10.2)
        self.assertEqual(scene['points'].shape, (2, 2))

    def test_former_303ms_filtered_scan_bracket_gap_no_longer_requires_interpolation(self):
        scene = self.scene(pose_stamp=1_000_000_000, scan_stamp=1_303_000_000,
                           now_ns=1_400_000_000)
        self.assertAlmostEqual(scene['pose_scan_delta_s'], .303)
        self.assertLess(scene['uncertainty'], .15)

    def test_stale_pose_or_scan_fails_closed(self):
        with self.assertRaisesRegex(ValueError, 'FOLLOW_POSE_STALE'):
            build_scene(pose_row=(1, 10., (0., 0., 0., 0., 0.)),
                        scan_row=(1_100_000_000, 10.1, [(2., 0.)]),
                        now_ns=1_200_000_000, now_mono=10.2)
        with self.assertRaisesRegex(ValueError, 'FOLLOW_SCAN_STALE'):
            build_scene(pose_row=(1_000_000_000, 10., (0., 0., 0., 0., 0.)),
                        scan_row=(1, 10.1, [(2., 0.)]),
                        now_ns=1_200_000_000, now_mono=10.2)

    def test_large_scan_pose_skew_fails_closed(self):
        with self.assertRaisesRegex(ValueError, 'FOLLOW_POSE_SCAN_TIME_MISMATCH'):
            self.scene(scan_stamp=1_500_000_000, now_ns=1_600_000_000)

    def test_near_obstacle_stops_without_detour(self):
        result = bounded_pursuit_result(self.scene(point=(.50, 0.)), .04, 0.,
                                        'TRACKED_PURSUIT')
        self.assertEqual(result['reason'], 'STOP_FOLLOW_CLEARANCE')
        self.assertEqual((result['motion'], result['v'], result['w']), ('STOP', 0., 0.))

    def test_swept_path_stops_and_forward_command_is_bounded(self):
        scene = self.scene(point=(.68, 0.))
        result = bounded_pursuit_result(scene, .10, .20, 'TRACKED_PURSUIT')
        self.assertIn(result['reason'], ('STOP_FOLLOW_SWEPT_CLEARANCE', 'TRACKED_PURSUIT'))
        self.assertLessEqual(result['v'], .04)
        self.assertLessEqual(abs(result['w']), result['v'] / .28 + 1e-9)

    def test_leader_wait_reason_does_not_request_follower_motion(self):
        result = bounded_pursuit_result(self.scene(), 0., 0., 'STOP_LEADER_STOPPED')
        self.assertEqual(result['reason'], 'FOLLOW_WAITING_LEADER')
        self.assertEqual((result['motion'], result['v'], result['w']), ('STOP', 0., 0.))


if __name__ == '__main__':
    unittest.main()
