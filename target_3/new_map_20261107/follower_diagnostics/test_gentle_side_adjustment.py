import json,math,unittest
from pathlib import Path
import numpy as np
from gentle_pivot_candidate import GentlePivot

class Tests(unittest.TestCase):
    def scene(self):
        rows=list(map(json.loads,(Path(__file__).parent/'logs/local_avoidance_trial_1793784599680490643.jsonl').read_text().splitlines()))
        p=next(r['plan'] for r in rows if r['event']=='checked_local_plan');s=next(r for r in rows if r['event']=='stopped_control')
        c=GentlePivot(p['start'],p['box'],p['gentle_route']);c.stage=1;c.map_rejoin=True
        return c,tuple(s['control']['pose']),np.asarray(s['points']+p['box_boundary'])
    def test_recorded_blocked_forward_gets_checked_ten_cm_extension_then_forward(self):
        c,p,pts=self.scene();choice=c.choose(p,pts);self.assertIsNotNone(choice)
        h,end,_=choice;self.assertAlmostEqual(h,p[2]);self.assertAlmostEqual(math.dist(end,p[:2]),.1)
        self.assertGreater(c.last_diagnostic['checked_clearance_m'],.36)
        next_choice=c.choose((*end,h),pts);self.assertIsNotNone(next_choice)
        self.assertAlmostEqual(next_choice[0],0.);self.assertEqual(c.side_adjustments,1)
    def test_extra_obstacle_and_retry_bound_still_stop(self):
        c,p,pts=self.scene();self.assertIsNone(c.choose(p,np.vstack((pts,p[:2]))))
        c,p,pts=self.scene();c.side_adjustments=2;self.assertIsNone(c.choose(p,pts))

if __name__=='__main__':unittest.main()
