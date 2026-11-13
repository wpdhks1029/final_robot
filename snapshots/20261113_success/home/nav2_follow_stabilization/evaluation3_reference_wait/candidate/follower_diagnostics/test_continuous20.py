import json,unittest
from pathlib import Path
from test_local_avoidance_drive import Tests as Setup
from continuous_local_core import ContinuousCore,build_plan
from test_filtered_local_pose import FilteredMemory
from map_rejoin_guidance import guidance
from test_map_rejoin_guidance import inputs

class ContractTests(unittest.TestCase):
    def test_twenty_cm_plan_and_ten_cm_stop_are_separate(self):
        c,t,p,k=Setup().contract();k.continuous20=True
        obj=Setup().envelope(c,k,'DRIVE',.02,.1);obj.update(clearance_m=.15,swept_clearance_m=.12)
        self.assertTrue(k.command(json.dumps(obj),c.ros))

    def test_curve_and_twenty_cm_require_exclusive_driver_mode(self):
        for enabled in (False,True):
            c,t,p,k=Setup().contract();k.continuous20=enabled
            obj=Setup().envelope(c,k,'DRIVE',.02,.15);obj.update(clearance_m=.25,swept_clearance_m=.23)
            self.assertEqual(k.command(json.dumps(obj),c.ros),enabled)
    def test_actual_curve_packets_allow_inner_wheel_reverse_only_in_curve_port(self):
        c,t,p,k=Setup().contract();k.continuous20=True
        for _ in range(40):
            c.now+=.02
            obj=Setup().envelope(c,k,'DRIVE',.02,.15);obj.update(clearance_m=.25,swept_clearance_m=.23)
            self.assertTrue(k.command(json.dumps(obj),c.ros));k.tick(c.ros)
        self.assertGreater(t.rpm[0],0);self.assertGreater(t.rpm[1],t.rpm[0])
        self.assertEqual(p.motion_kind,'CURVE')
    def test_pivot_and_sub_twenty_cm_still_rejected(self):
        for motion,v,w,gap in [('TURN',0.,.1,.3),('DRIVE',.02,.1,.1)]:
            c,t,p,k=Setup().contract();k.continuous20=True
            obj=Setup().envelope(c,k,motion,v,w);obj.update(clearance_m=gap,swept_clearance_m=gap)
            self.assertFalse(k.command(json.dumps(obj),c.ros))

class CoreTests(unittest.TestCase):
    def core(self):
        c=ContinuousCore(pose_source='filtered');c.freeze((0,0,0),(0,0,0));ns=100_000_000_000
        c.raw(ns,1.,(0,0,0,0,0));c.filtered(ns,1.,(0,0,0,0,0));c.imu(ns,1.,(0,0,0,0,0));c.inputs['leader']=(ns,1.,(3,0,0,.04,0))
        return c,ns
    def test_follow_roundoff_never_exceeds_driver_speed_bound(self):
        for speed in (.031939,.039997,.06,.1):
            c,ns=self.core();c.offer_scan(ns,1.,[(5.,5.)]);c.last_curve=(.99,.025,0.)
            r=c.command(ns,1.,{},(speed,.001))
            self.assertEqual(r['reason'],'CONTINUOUS_READY');self.assertLessEqual(r['v'],.025)
            clock,t,port,contract=Setup().contract();contract.continuous20=True
            obj=Setup().envelope(clock,contract,'DRIVE',r['v'],r['w'])
            self.assertTrue(contract.command(json.dumps(obj),clock.ros))
    def test_twenty_cm_guard_and_workspace_apply_after_handoff(self):
        c,ns=self.core();c.offer_scan(ns,1.,[(.31,0.)])
        self.assertEqual(c.command(ns,1.,{},(.02,0.))['reason'],'STOP_FOOTPRINT_CLEARANCE')
    def test_workspace_and_seamless_handoff_keep_limits_and_curve_state(self):
        c,ns=self.core();c.freeze((0,0,0),(4.51,0,0));c.offer_scan(ns,1.,[(5.,5.)])
        self.assertEqual(c.command(ns,1.,{},(.02,0.))['reason'],'STOP_APF_WORKSPACE_BOUND')
        c.last_curve=(1.,.02,.05)
        with self.assertRaises(ValueError):c.handoff_to_follow({'continuous20':False,'armed':True,'feedback_fresh':True})
        c.handoff_to_follow({'continuous20':True,'armed':True,'feedback_fresh':True,'measured_rpm':[3,5]})
        self.assertEqual(c.last_curve,(1.,.02,.05));self.assertIsNone(c.fsm)
    def test_two_metre_follow_extension_starts_only_at_handoff(self):
        c,ns=self.core()
        self.assertEqual(c.follow_workspace_max_x,4.5)
        c.last_pose=(3.8,0.,0.)
        c.handoff_to_follow({'continuous20':True,'armed':True,'feedback_fresh':True},follow_distance_m=2.)
        self.assertAlmostEqual(c.follow_workspace_max_x,6.3)
        c.freeze((0,0,0),(6.31,0,0));c.offer_scan(ns,1.,[(10.,5.)])
        self.assertEqual(c.command(ns,1.,{},(.02,0.))['reason'],'STOP_APF_WORKSPACE_BOUND')
    def test_plan_from_recorded_scene_requires_replay_success(self):
        rows=list(map(json.loads,(Path(__file__).parent/'logs/local_avoidance_trial_1793859841865297024.jsonl').read_text().splitlines()))
        r=next(x for x in rows if x['event']=='planning_input');p=build_plan(r['points'],r['start'])
        self.assertGreater(p['geometry']['min_path_clearance_m'],.1)
        self.assertEqual(p['config']['profile'],'CONTINUOUS10_STOP05_WAIT_V1')

if __name__=='__main__':unittest.main()

class PlanningDiagnosticsTests(unittest.TestCase):
    def test_budget_is_not_reported_as_geometry_failure(self):
        from unittest.mock import patch
        from continuous_avoidance_candidate import search
        with patch('continuous_avoidance_candidate.time.monotonic',side_effect=[0.,61.]):
            result=search([], (0.,0.,0.), {'high':0.1})
        self.assertEqual(result['reason'],'CONTINUOUS_PLANNING_BUDGET_EXCEEDED')
    def test_continuous_review_uses_twenty_cm_profile(self):
        from continuous_local_core import route_still_clear
        plan={'continuous_path':[(0.,0.,0.),(.01,0.,0.)]}
        self.assertTrue(route_still_clear(plan,[(0.,.55)]))
        self.assertFalse(route_still_clear(plan,[(0.,.429)]))
