import ast,math,unittest
from pathlib import Path
from temporary_detour_candidate import transition,path_clear,Settings
from watch_temporary_detour_shadow import review_path,PreparedPathReview,finalize_row


class Tests(unittest.TestCase):
    def test_clear_and_blocked_match_scalar_rectangle_check(self):
        path=transition(0.,1.,0.,0.)
        for points in ([(.5,1.)],[(.5,.5)],[(.5,.6)],[(.5,.65)],[(0.,0.)]):
            result=review_path(path,points)
            self.assertEqual(result['reason']=='LIVE_PATH_GEOMETRY_CLEAR',path_clear(path,points,Settings()))
    def test_obstacle_blocking_reports_location(self):
        r=review_path(transition(0.,1.,0.,0.),[(.8,0.)])
        self.assertEqual(r['reason'],'STOP_LIVE_PATH_BLOCKED')
        self.assertEqual(r['blocking']['point'],[.8,0.])
    def test_invalid_and_empty_input_refused(self):
        self.assertEqual(review_path([],[])['reason'],'STOP_PATH_OR_SCAN_MISSING')
        self.assertEqual(review_path([(0.,0.,0.,10.)],[(2.,0.)])['reason'],'STOP_PATH_OR_SCAN_INVALID')
    def test_shadow_has_no_ros_publisher_or_actuation_calls(self):
        tree=ast.parse(Path(__file__).with_name('watch_temporary_detour_shadow.py').read_text())
        calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertFalse(calls & {'create_publisher','create_client','create_service','call_async','arm','open_gate'})
    def test_spatial_pruning_matches_full_point_checks(self):
        import numpy as np
        rng=np.random.RandomState(4);path=transition(0.,1.2,0.,.1)
        cached=PreparedPathReview(path)
        for _ in range(80):
            points=rng.uniform([-2.,-2.],[3.,2.],size=(30,2)).tolist()
            result=cached.review(points)
            self.assertEqual(result['reason']=='LIVE_PATH_GEOMETRY_CLEAR',path_clear(path,points,Settings()))
            actual=min(math.hypot(max(abs(math.cos(a)*(px-x)+math.sin(a)*(py-y))-.27,0),
                                 max(abs(-math.sin(a)*(px-x)+math.cos(a)*(py-y))-.3,0))
                       for x,y,a,k in path for px,py in points)
            self.assertLessEqual(result['min_path_clearance_m'],actual+1e-10)
    def test_expired_review_zeroes_all_guarded_proposals(self):
        row=dict(reason='LIVE_PATH_GEOMETRY_CLEAR',source_ns=1_000_000_000,tracking_reason='DETOUR_TRACK',
                 guarded_candidate_v=.02,guarded_candidate_w=.03,leader_candidate_v=.012)
        finalize_row(row,1_310_000_000,.13)
        self.assertEqual(row['reason'],'STOP_INPUT_AGED_DURING_REVIEW')
        self.assertEqual([row[k] for k in ('guarded_candidate_v','guarded_candidate_w','leader_candidate_v')],[0.,0.,0.])
    def test_geometry_and_timing_failures_both_preserved(self):
        row=dict(reason='STOP_LIVE_PATH_BLOCKED',source_ns=1_000_000_000,tracking_reason='DETOUR_TRACK')
        finalize_row(row,1_400_000_000,.2)
        self.assertEqual(row['geometry_reason'],'STOP_LIVE_PATH_BLOCKED')
        self.assertEqual(row['timing_reason'],'STOP_INPUT_AGED_DURING_REVIEW')
