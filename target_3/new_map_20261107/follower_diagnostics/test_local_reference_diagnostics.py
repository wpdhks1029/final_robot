import math,unittest
from local_reference_diagnostics import alignment_diagnostics

class Tests(unittest.TestCase):
    reference=dict(snapshot=dict(reference_origin=[1.,2.],reference_heading=math.pi/2))
    def test_shifted_saved_line_is_distinguished_from_heading(self):
        d=alignment_diagnostics(self.reference,[(.8,2.),(.8,3.)],(.8,2.,math.pi/2),(.8,3.,math.pi/2))
        self.assertAlmostEqual(d['follower_lateral_m'],.2)
        self.assertEqual(set(d['failed_checks']),{'follower_lateral_m','leader_lateral_m','path_max_lateral_m'})
    def test_old_path_history_can_fail_while_both_robot_poses_align(self):
        d=alignment_diagnostics(self.reference,[(1.,2.),(.8,3.)],(1.,2.,math.pi/2),(1.,3.,math.pi/2))
        self.assertEqual(d['failed_checks'],['path_max_lateral_m'])
    def test_heading_failure_is_separate_and_reference_is_not_modified(self):
        d=alignment_diagnostics(self.reference,[(1.,2.),(1.,3.)],(1.,2.,0.),(1.,3.,math.pi/2))
        self.assertEqual(d['failed_checks'],['follower_heading_error_deg'])
        self.assertFalse(d['reference_changed']);self.assertFalse(d['control_connected'])

class AnchorReferenceTests(unittest.TestCase):
    def setUp(self):
        self.reference=dict(reason='DETOUR_PLANNED',snapshot=dict(
            reference_origin=[1.9443607661575613,-.02223755341924044],
            reference_heading=math.radians(17.24979905717249),line_fit_residual_m=.0328))
        self.follower=(1.862700127182709,.06536448982962076,.28727482708164465)
        self.leader=(3.3132994212209312,.5019143072220287,.28198838055920844,0.,0.)
        self.anchors=(self.follower[:2],self.leader[:2])
    def call(self,path,kind='live_path',follower=None):
        from check_temporary_detour import reference_from_plan
        from temporary_detour_candidate import Pose
        return reference_from_plan(self.reference,path,Pose(*(follower or self.follower)),self.leader,source_kind=kind)
    def test_reported_10_point_8cm_robot_anchor_is_within_15cm_limit(self):
        origin,heading,cross,residual=self.call(self.anchors,'robot_anchors')
        self.assertAlmostEqual(cross,.10787729056769996)
        self.assertAlmostEqual(heading,self.reference['snapshot']['reference_heading'])
        d=alignment_diagnostics(self.reference,self.anchors,self.follower,self.leader,source_kind='robot_anchors')
        self.assertEqual(d['failed_checks'],[]);self.assertIsNone(d['path_limit_m'])
    def test_real_path_at_same_offset_still_fails_10cm_limit(self):
        with self.assertRaisesRegex(ValueError,'LIVE_ALIGNMENT_MISMATCH'):self.call(self.anchors)
    def test_robot_anchor_label_cannot_be_used_on_arbitrary_path(self):
        with self.assertRaisesRegex(ValueError,'ANCHORS_INVALID'):
            self.call([(0.,0.),(1.,1.)],'robot_anchors')
    def test_lateral_and_heading_robot_limits_remain_enforced(self):
        h=self.reference['snapshot']['reference_heading'];a=self.reference['snapshot']['reference_origin']
        f=(a[0]-.16*math.sin(h),a[1]+.16*math.cos(h),h)
        with self.assertRaisesRegex(ValueError,'LIVE_ALIGNMENT_MISMATCH'):
            self.call((f[:2],self.leader[:2]),'robot_anchors',f)
        f=(*self.follower[:2],h+math.radians(21))
        with self.assertRaisesRegex(ValueError,'HEADING_MISMATCH'):
            self.call((f[:2],self.leader[:2]),'robot_anchors',f)

if __name__=='__main__':unittest.main()
