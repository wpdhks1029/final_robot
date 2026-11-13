import json,math,unittest
from pathlib import Path
import numpy as np
from gentle_pivot_candidate import build_gentle_plan,checked_route,GentlePivot
from apf_pivot_candidate import build_apf_pivot_plan
from test_filtered_local_pose import FilteredMemory

LOG=Path(__file__).parent/'logs/local_avoidance_trial_1793781323794712578.jsonl'
def scene():
    return next(r for r in map(json.loads,LOG.read_text().splitlines()) if r['event']=='planning_input')

class Tests(unittest.TestCase):
    def test_recorded_filtered_scene_reduces_turn_and_checks_whole_route(self):
        r=scene();old=build_apf_pivot_plan(r['points'],r['start']);new=build_gentle_plan(r['points'],r['start'])
        self.assertEqual(r['pose_source'],'filtered')
        self.assertLess(new['config']['initial_heading_deg'],old['config']['initial_heading_deg'])
        self.assertEqual(new['config']['initial_heading_deg'],45)
        self.assertGreater(new['geometry']['min_path_clearance_m'],.34)
        self.assertEqual(new['points'],old['points'])
    def test_blocked_rotation_never_accepted(self):
        with self.assertRaises(ValueError):checked_route(np.array([[0.,0.]]),(0,0,0),dict(near=1.,far=1.2,high=.1))
    def test_new_obstacle_on_remaining_leg_stops_before_ten_cm_step(self):
        r=scene();p=build_gentle_plan(r['points'],r['start']);c=GentlePivot(r['start'],p['box'],p['gentle_route'])
        corner=p['gentle_route']['targets'][0]
        cloud=np.array(p['points']+[[corner[0],corner[1]]])
        self.assertIsNone(c.choose(r['start'],cloud))

class CapturedFilteredMemory(FilteredMemory):
    def data_for_test(self):return scene()
    def plan_for_test(self,d):return build_gentle_plan(d['points'],d['start'])

if __name__=='__main__':unittest.main()
