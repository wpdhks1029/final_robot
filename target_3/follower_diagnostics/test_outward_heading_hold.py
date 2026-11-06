import json,math,unittest
from pathlib import Path
import numpy as np
from gentle_pivot_candidate import GentlePivot
from apf_pivot_candidate import checked_step

class Tests(unittest.TestCase):
    def test_recorded_corner_chase_is_replaced_by_checked_original_diagonal(self):
        rows=list(map(json.loads,(Path(__file__).parent/'logs/local_avoidance_trial_1793856740477958211.jsonl').read_text().splitlines()))
        plan=next(r['plan'] for r in rows if r['event']=='checked_local_plan');stop=next(r for r in rows if r['event']=='stopped_control')
        points=np.array(stop['points']+plan['box_boundary']);p=stop['control']['pose']
        old=stop['control']['apf']['selected_heading_rad']
        self.assertFalse(checked_step(points,p,old,.093)[0])
        c=GentlePivot(plan['start'],plan['box'],plan['gentle_route']);c.map_rejoin=True
        choice=c.choose(p,points);self.assertIsNotNone(choice)
        h,end,_=choice;self.assertAlmostEqual(math.degrees(h),55.)
        self.assertTrue(checked_step(points,p,h,math.dist(end,p[:2]))[0])
        forward=plan['gentle_route']['targets'][1][0]-end[0]
        self.assertTrue(checked_step(points,(*end,h),0.,forward)[0])
    def test_large_side_error_does_not_become_unchecked_heading_correction(self):
        c=GentlePivot((0,0,0),dict(near=1.,far=1.2,high=.1),dict(targets=[(.5,.7),(2.,.7),(3.,0.)]))
        self.assertIsNone(c.choose((.1,-.2,0.),np.array([[5.,5.]])))

if __name__=='__main__':unittest.main()
