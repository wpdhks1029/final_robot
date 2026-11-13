import json,math,unittest
from pathlib import Path
import numpy as np
from gentle_pivot_candidate import GentlePivot
from apf_pivot_candidate import APFPivotFSM,checked_step

class Tests(unittest.TestCase):
    def scene(self):
        rows=list(map(json.loads,(Path(__file__).parent/'logs/local_avoidance_trial_1793857108908785996.jsonl').read_text().splitlines()))
        plan=next(r['plan'] for r in rows if r['event']=='checked_local_plan');stop=next(r for r in rows if r['event']=='stopped_control')
        c=GentlePivot(plan['start'],plan['box'],plan['gentle_route']);c.map_rejoin=True;c.stage=1
        return c,plan,stop['control']['pose'],np.array(stop['points']+plan['box_boundary'])
    def test_recorded_snapshot_keeps_actual_lateral_room_and_passes(self):
        c,plan,p,cloud=self.scene();old_heading=math.atan2(c.targets[1][1]-p[1],c.targets[1][0]-p[0])
        self.assertFalse(checked_step(cloud,p,old_heading,math.dist(c.targets[1],p[:2]))[0])
        h,end,_=c.choose(p,cloud)
        self.assertAlmostEqual(h,0.);self.assertAlmostEqual(end[1],p[1]);self.assertEqual(c.side_adjustments,0)
        self.assertGreater(c.last_diagnostic['checked_clearance_m'],.34)
    def test_blocked_pending_step_brakes_and_does_not_reselect_while_wheels_move(self):
        from local_stop_turn_go import LocalCore
        from compare_avoidance_strategies import body_points
        controller,plan,p,cloud=self.scene();core=LocalCore();core.freeze((0,0,0),p)
        ns=100_000_000_000;core.raw(ns,1.,(0.,0.,0.,0.,0.));core.inputs['leader']=(ns,1.,(5.,0.,0.,.04,0.))
        core.offer_scan(ns,1.,body_points(cloud,p).tolist());core.install(plan);core.fsm.controller=controller
        controller.phase='DRIVE';controller.heading=-.01859500607556752
        controller.target=(p[0]+.1*math.cos(controller.heading),p[1]+.1*math.sin(controller.heading))
        r=core.command(ns,1.,{'measured_rpm':[2,2],'zero_feedback_samples':0})
        self.assertEqual((r['v'],r['w']),(0.,0.));self.assertEqual(controller.phase,'SELECT')
        for motor in ({'measured_rpm':[2,2],'zero_feedback_samples':0},{'measured_rpm':[0,0],'zero_feedback_samples':2}):
            self.assertEqual(core.command(ns,1.,motor)['reason'],'BRAKE_BEFORE_APF_SELECTION')
        motor={'measured_rpm':[0,0],'zero_feedback_samples':3}
        core.command(ns,1.,motor);r=core.command(ns,1.,motor)
        self.assertEqual(r['reason'],'LOCAL_AVOIDANCE_READY');self.assertGreater(r['v'],0.);self.assertEqual(r['w'],0.)
    def test_obstacle_or_reselection_budget_still_stops(self):
        c,plan,p,cloud=self.scene();self.assertIsNone(c.choose(p,np.vstack((cloud,p[:2]))))
        fsm=APFPivotFSM(plan['start'],plan['box']);fsm.controller=c;fsm.points=cloud
        c.phase='DRIVE';c.heading=-.01859500607556752;c.target=(p[0]+.1,p[1]-.002)
        fsm.blocked_total=6
        self.assertEqual(fsm.command(p)[2],'STOP_APF_STEP_BLOCKED')

if __name__=='__main__':unittest.main()
