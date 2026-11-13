import json,math,unittest
from pathlib import Path
import numpy as np
from gentle_pivot_candidate import GentlePivot
from apf_pivot_candidate import checked_step

class Tests(unittest.TestCase):
    def scene(self):
        r=list(map(json.loads,(Path(__file__).parent/'logs/local_avoidance_trial_1793856109394840671.jsonl').read_text().splitlines()))
        p=next(x['plan'] for x in r if x['event']=='checked_local_plan');s=next(x for x in r if x['event']=='stopped_control')
        c=GentlePivot(p['start'],p['box'],p['gentle_route']);c.stage=1;c.map_rejoin=True
        return c,tuple(s['control']['pose']),np.array(s['points']+p['box_boundary'])
    def test_recorded_forward_heading_can_repair_without_returning_to_fifty_degrees(self):
        c,p,points=self.scene();h,end,_=c.choose(p,points)
        self.assertAlmostEqual(math.degrees(h),25)
        self.assertAlmostEqual(math.dist(end,p[:2]),.1)
        self.assertTrue(checked_step(points,p,h,.1)[0])
        self.assertGreater(c.last_diagnostic['forward_corridor_clearance_m'],.36)
        next_choice=c.choose((*end,h),points);self.assertIsNotNone(next_choice)
        self.assertAlmostEqual(next_choice[0],0.)
    def test_two_attempt_cap_and_new_blocker_are_not_bypassed(self):
        c,p,points=self.scene();c.side_adjustments=2;self.assertIsNone(c.choose(p,points))
        c,p,points=self.scene();self.assertIsNone(c.choose(p,np.vstack((points,p[:2]))))
        self.assertTrue(c.last_diagnostic['selection_failed'])

if __name__=='__main__':unittest.main()
