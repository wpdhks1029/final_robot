import math,unittest
from temporary_detour_candidate import Settings,Pose,plan,path_clear,pursue,reference_frame,arc_transition,lateral_arc_transition


def scene():
    points=[]
    for i in range(121):points.extend([(-1+i*.05,-1.25),(-1+i*.05,1.25)])
    for i in range(21):points.append((1.6,.25+.2*i/20))
    return points


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.points=scene();cls.result=plan(cls.points,Settings(),side=-1)
    def test_measured_box_has_right_detour_and_straight_rejoin(self):
        r=self.result;self.assertEqual(r['reason'],'DETOUR_PLANNED');self.assertLess(r['offset_m'],0)
        self.assertEqual(r['path'][-1][1],0.)
        self.assertGreater(r['path'][-1][0]-r['join_x_m'],1.)
    def test_all_points_checked_including_right_wall(self):
        self.assertTrue(path_clear(self.result['path'],self.points,Settings()))
        x,y,*_=self.result['path'][60]
        self.assertFalse(path_clear(self.result['path'],self.points+[(x,y)],Settings()))
    def test_too_close_box_refused_instead_of_shrinking_margin(self):
        self.assertEqual(plan([(.7,.25+i*.01) for i in range(21)],Settings(),side=-1)['path'],[])
    def test_recorded_short_right_obstacle_still_has_no_checked_path(self):
        r=plan([(.905,-.29+i*.008) for i in range(31)],Settings(),side=-1)
        self.assertEqual(r['reason'],'STOP_NO_COLLISION_CHECKED_DETOUR')
        self.assertAlmostEqual(r['observed_center_y_m'],-.17)
    def test_short_arc_respects_start_heading_and_parallel_endpoint(self):
        p=arc_transition(0.,.01,.05,-.43,.30)
        self.assertAlmostEqual(p[0][2],.05)
        self.assertAlmostEqual(p[-1][1],-.43)
        self.assertAlmostEqual(p[-1][2],0.)
        self.assertLess(max(abs(r[3]) for r in p),.98*2/.560)
    def test_short_turn_outer_wheel_remains_below_ten_rpm(self):
        path=arc_transition(0.,0.,0.,-.4,.30)
        v,w,_,reason=pursue(path,Pose(0.,0.,0.),.04)
        self.assertEqual(reason,'DETOUR_TRACK')
        self.assertLess((v+abs(w)*.560/2)/(.065*2*math.pi)*60,10.)
    def test_one_meter_body_gap_has_short_detour_candidate(self):
        points=[]
        for i in range(121):points.extend([(-1+i*.05,-1.25),(-1+i*.05,1.25)])
        for i in range(21):points.append((.87,.25+.2*i/20))
        result=plan(points,Settings(),side=-1)
        self.assertEqual(result['reason'],'DETOUR_PLANNED')
        self.assertTrue(path_clear(result['path'],points,Settings()))
    def test_static_leader_is_not_masked_for_one_meter_layout(self):
        points=[]
        for i in range(21):
            points.extend([(.87,.25+.2*i/20),(1.27,-.3+.6*i/20)])
        # 정지 선두가 경로를 막으면 통과 경로라고 선언하지 않는다.
        result=plan(points,Settings(),side=-1)
        if result['path']:self.assertTrue(path_clear(result['path'],points,Settings()))
    def test_pursuit_never_moves_backwards_or_skips_to_distant_end(self):
        v,w,index,reason=pursue(self.result['path'],Pose(0.,0.,0.),.04)
        self.assertEqual(reason,'DETOUR_TRACK');self.assertEqual(index,0)
        self.assertGreater(v,abs(w)*.560/2)
    def test_tracking_deviation_stops(self):
        self.assertEqual(pursue(self.result['path'],Pose(0.,.3,0.),.04)[3],'STOP_DETOUR_TRACKING')
    def test_reference_fit_tolerates_short_segment_noise(self):
        origin,h,e,residual=reference_frame([(i*.1,.006*(-1)**i) for i in range(11)],Pose(0.,0.,0.))
        self.assertLess(abs(h),.02);self.assertLess(residual,.02)
    def test_reference_curve_and_short_history_refused(self):
        for points in ([(0.,0.),(.1,0.),(.2,0.)],[(0.,0.),(1.,.7),(2.,0.)]):
            with self.assertRaises(ValueError):reference_frame(points,Pose(0.,0.,0.))
    def test_sparse_points_not_declared_box(self):
        self.assertEqual(plan([(1.6,.3)],Settings(),side=-1)['reason'],'NO_OBSTACLE_CLUSTER')
    def test_box_crossing_corridor_edge_is_not_truncated(self):
        r=plan([(.9,.54+i*.01) for i in range(23)],Settings(),side=-1)
        self.assertAlmostEqual(r['observed_center_y_m'],.65)
        self.assertAlmostEqual(r['observed_width_m'],.22)
    def test_large_left_shift_uses_forward_arcs_and_sideways_translation(self):
        path=lateral_arc_transition(0.,0.,-.06,1.,.32)
        self.assertAlmostEqual(path[-1][1],1.)
        self.assertAlmostEqual(path[-1][2],0.)
        self.assertTrue(any(abs(p[2]-math.pi/2)<1e-6 and p[3]==0. for p in path))
        self.assertLess(max(abs(p[3]) for p in path),.98*2/.560)
    def test_tracking_curvature_is_limited_without_reverse_wheel(self):
        path=[(i*.01,.1,0.,0.) for i in range(100)]
        v,w,_,reason=pursue(path,Pose(0.,0.,0.),.04,lookahead=.08)
        self.assertEqual(reason,'DETOUR_TRACK_CURVATURE_LIMITED')
        self.assertGreater(v-abs(w)*.28,0.)
        self.assertLess((v+abs(w)*.28)/(.065*2*math.pi)*60,10.)
    def test_time_objective_keeps_collision_checks_and_improves_estimate(self):
        result=plan(self.points,Settings(),side=-1,objective='time')
        self.assertEqual(result['reason'],'DETOUR_PLANNED')
        self.assertTrue(path_clear(result['path'],self.points,Settings()))
        self.assertLessEqual(result['estimated_follow_time_s'],self.result['estimated_follow_time_s']+1e-8)
