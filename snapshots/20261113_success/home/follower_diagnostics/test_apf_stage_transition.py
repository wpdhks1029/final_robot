import json,math,unittest
from pathlib import Path
import numpy as np
from apf_pivot_stage_candidate import APFPivot,APFPivotFSM

class Tests(unittest.TestCase):
    def setUp(self):
        p=Path(__file__).parent/'logs/local_avoidance_trial_1793774746886900549.jsonl'
        rows=[json.loads(s) for s in p.read_text().splitlines()]
        self.plan=next(r['plan'] for r in rows if r['event']=='checked_local_plan')
        stop=next(r for r in rows if r['event']=='stopped_control')
        self.pose=stop['control']['pose'];self.points=np.array(stop['points']+self.plan['box_boundary'])
    def test_recorded_stuck_pose_can_enter_pass_phase_with_checked_forward_step(self):
        c=APFPivot(self.plan['start'],self.plan['box']);choice=c.choose(self.pose,self.points)
        self.assertIsNotNone(choice);self.assertEqual(c.stage,1)
        self.assertLessEqual(math.degrees(choice[0]),65)
        self.assertGreater(choice[1][0]-self.pose[0],0.)
        self.assertLessEqual(c.last_diagnostic['step_length_m'],.1)
        self.assertGreater(c.last_diagnostic['checked_clearance_m'],.34)
    def test_blocked_selected_step_first_brakes_and_reselects_then_is_bounded(self):
        f=APFPivotFSM(self.plan['start'],self.plan['box']);f.points=self.points
        for i in range(4):
            f.controller.phase='TURN';f.controller.heading=math.radians(75)
            f.controller.target=(self.pose[0]+.1*math.cos(math.radians(75)),self.pose[1]+.1*math.sin(math.radians(75)))
            v,w,phase=f.command(self.pose)
            self.assertEqual((v,w),(0.,0.))
            self.assertEqual(phase,'BRAKE' if i<3 else 'STOP_APF_STEP_BLOCKED')
    def test_initial_pose_does_not_skip_outward_phase(self):
        c=APFPivot(self.plan['start'],self.plan['box'])
        c.choose(self.plan['start'],np.array(self.plan['points']))
        self.assertEqual(c.stage,0)

if __name__=='__main__':unittest.main()
