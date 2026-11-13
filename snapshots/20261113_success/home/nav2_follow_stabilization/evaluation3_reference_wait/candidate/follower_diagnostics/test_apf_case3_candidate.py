import math,unittest
from apf_case3_candidate import APF,Settings,RejoinEvidence

class Tests(unittest.TestCase):
    def test_nearest_side_point_is_not_front_blocker(self):
        from apf_case3_candidate import scan_diagnostic
        c=Settings();d=scan_diagnostic([(0.,-.65),(1.,.15),(2.,0.)],c)
        self.assertEqual(d['front_blocker_count'],1)
        self.assertEqual(d['nearest_points'][0]['x_m'],0.)
        self.assertEqual(d['nearest_front_points'][0]['x_m'],1.)
    def make(self):return APF(Settings(lidar_height=.15,obstacle_height=.20))
    def test_confirmed_raised_obstacle_allowed(self):
        r=APF(Settings(lidar_height=.25,visibility_confirmed=True)).step(.04,0.,[(1.,0.)])
        self.assertEqual(r['reason'],'APF_CANDIDATE')
    def test_preferred_side_does_not_enter_pass_before_obstacle(self):
        c=APF(Settings(lidar_height=.25,visibility_confirmed=True),preferred_side=-1)
        self.assertEqual(c.step(.04,0.,[(2.,1.)])['phase'],'TRACK')
        self.assertLess(c.step(.04,0.,[(1.,0.)])['w'],0.)
    def test_scan_projection_uses_mount_and_excludes_invalid_ranges(self):
        from types import SimpleNamespace as S
        from watch_apf_case3 import project
        m=S(header=S(frame_id='follower/laser'),angle_min=0.,angle_increment=.1,range_min=.28,range_max=16.,ranges=[1.,0.,float('nan')])
        tf=S(header=S(frame_id='follower/base_link'),child_frame_id='follower/laser',transform=S(translation=S(x=.26,y=0.,z=.24),rotation=S(x=0.,y=0.,z=0.,w=1.)))
        self.assertEqual(project(m,tf),[(1.26,0.)])
        tf.child_frame_id='other'
        with self.assertRaises(ValueError):project(m,tf)
    def test_unknown_height_blocks(self):self.assertEqual(APF().step(.04,0.,[(1.,0.)])['reason'],'STOP_LIDAR_HEIGHT_UNCONFIRMED')
    def test_short_obstacle_not_declared_visible(self):
        for h in (.20,.24):self.assertEqual(APF(Settings(lidar_height=h,obstacle_height=.20)).step(.04,0.,[(1.,0.)])['reason'],'STOP_OBSTACLE_BELOW_OR_AT_SCAN_PLANE')
    def test_clear_keeps_pursuit_turn(self):
        r=self.make().step(.04,.03,[(2.,1.)]);self.assertAlmostEqual(r['v'],.04);self.assertAlmostEqual(r['w'],.03)
    def test_center_obstacle_breaks_symmetric_deadlock(self):
        r=self.make().step(.04,0.,[(1.,-.1),(1.,.1)]);self.assertEqual(r['phase'],'AVOID');self.assertGreater(r['w'],0)
    def test_left_obstacle_selects_right(self):self.assertLess(self.make().step(.04,0.,[(1.,.2)])['w'],0)
    def test_hard_stop_not_masked(self):
        for p in [(0.,0.),(.55,0.),(-.5,0.)]:self.assertEqual(self.make().step(.04,0.,[p])['reason'],'STOP_FOOTPRINT_CLEARANCE')
    def test_missing_or_invalid_scan_stops(self):
        for points in [[],[(float('nan'),0.)]]:self.assertEqual(self.make().step(.04,0.,points)['v'],0.)
    def test_side_lock_prevents_oscillation(self):
        c=self.make();a=c.step(.04,0.,[(1.,-.1),(1.,.1)]);b=c.step(.04,0.,[(1.,.12)])
        self.assertEqual(a['side'],b['side'])
    def test_wheel_forward_envelope_kept(self):
        for v in (.005,.02,.1):
            r=self.make().step(v,.2,[(1.,0.)]);self.assertLessEqual(abs(r['w'])*.560/2,r['v']+1e-12)
    def test_upstream_stop_not_overridden(self):self.assertEqual(self.make().step(0.,0.,[(1.,0.)])['v'],0.)
    def test_centered_box_2_5m_not_enough_for_current_margins(self):
        c=Settings();r=c.passage(0.);self.assertFalse(r['left_possible']);self.assertFalse(r['right_possible']);self.assertAlmostEqual(r['required_gap_m'],1.2)
        self.assertTrue(c.passage(.1)['right_possible'])
    def tracker(self):return RejoinEvidence((0.,0.),0.,1.,cross_tolerance=.1,heading_tolerance=math.radians(10))
    def test_front_crossing_alone_not_full_pass(self):self.assertFalse(self.tracker().step(1,1.1,0.,0.)['whole_body_passed'])
    def test_body_pass_rejoin_and_net_one_meter(self):
        t=self.tracker();self.assertEqual(t.step(1,1.3,.4,0.)['phase'],'REJOIN')
        self.assertEqual(t.step(2,1.4,0.,0.)['phase'],'FOLLOW_AFTER_REJOIN')
        r=t.step(3,2.41,0.,0.);self.assertEqual(r['phase'],'GEOMETRIC_COMPLETE_CANDIDATE');self.assertFalse(r['evaluation_pass'])
    def test_deviation_resets_rejoin_distance(self):
        t=self.tracker();t.step(1,1.4,0.,0.);t.step(2,2.,.2,0.);r=t.step(3,2.1,0.,0.)
        self.assertEqual(r['post_rejoin_net_progress_m'],0.)
    def test_backtrack_below_obstacle_plane_revokes_pass(self):
        t=self.tracker();t.step(1,1.4,0.,0.);r=t.step(2,.8,0.,0.);self.assertFalse(r['whole_body_passed']);self.assertEqual(r['post_rejoin_net_progress_m'],0.)
    def test_rotated_corners_must_clear(self):self.assertFalse(self.tracker().step(1,1.29,0.,math.pi/2)['whole_body_passed'])
    def test_old_pose_rejected(self):
        t=self.tracker();t.step(1,1.4,0.,0.)
        with self.assertRaises(ValueError):t.step(1,2.5,0.,0.)

if __name__=='__main__':unittest.main()
