import json,math,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'candidate/follower_diagnostics'))
from continuous_local_core import ContinuousCore,build_plan,route_still_clear
from continuous_avoidance_candidate import CLEARANCE
from continuous_near_return_pause import ContinuousNearReturnPause
import test_continuous20 as fixture
from run_local_avoidance_trial import motor_problem

class PolicyTests(unittest.TestCase):
    def test_planning_clearance_and_three_cm_recheck_reserve(self):
        self.assertEqual(CLEARANCE,.1)
        plan={'continuous_path':[(0.,0.,0.),(.01,0.,0.)]}
        self.assertTrue(route_still_clear(plan,[(0.,.431)]))
        self.assertFalse(route_still_clear(plan,[(0.,.429)]))
    def test_runtime_five_cm_stop_uses_uncertainty_adjusted_body_distance(self):
        for gap in (.049,.051,.075):
            with self.subTest(gap=gap):
                probe,ns=fixture.CoreTests().core();probe.offer_scan(ns,1.,[(5.,5.)])
                uncertainty=probe.scene(ns,1.)['uncertainty']
                c,ns=fixture.CoreTests().core();c.offer_scan(ns,1.,[(0.,.30+gap+uncertainty)])
                row=c.command(ns,1.,{},(.025,0.))
                self.assertAlmostEqual(row['clearance_m'],gap)
                self.assertEqual(row['reason']=='STOP_FOOTPRINT_CLEARANCE',gap<=.05)
    def test_old_ten_cm_driver_is_rejected_before_arm(self):
        state={'continuous20':True,'continuous_stop_clearance_m':.1}
        self.assertEqual(motor_problem((state,1.),1.,360,continuous20=True),'REQUIRE_CONTINUOUS10_STOP05_DRIVER_RESTART')
    def test_new_profile_plans_recorded_scene(self):
        d=json.loads(next((ROOT/'fixtures').glob('*1794032200970674450*')).read_text())
        p=build_plan(d['points'],d['start'])
        self.assertEqual(p['config']['profile'],'CONTINUOUS10_STOP05_WAIT_V1')
        self.assertEqual(p['config']['clearance_m'],.1)
        self.assertEqual(p['config']['stop_clearance_m'],.05)
        self.assertGreater(p['geometry']['min_path_clearance_m'],.1)
    def test_driver_and_operator_policy_files_match(self):
        self.assertEqual((ROOT/'driver/evaluation3_clearance_policy.py').read_bytes(),(ROOT/'candidate/follower_diagnostics/evaluation3_clearance_policy.py').read_bytes())

if __name__=='__main__':unittest.main()
