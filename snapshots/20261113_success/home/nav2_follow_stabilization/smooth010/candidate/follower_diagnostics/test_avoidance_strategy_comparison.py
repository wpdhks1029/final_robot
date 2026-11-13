import math,unittest
import numpy as np
from compare_avoidance_strategies import clearance,advance,dogleg,StopTurnGo,simulate,R

class Tests(unittest.TestCase):
    def test_rectangular_clearance_checks_rotating_corners(self):
        points=np.array([[.60,0.]])
        self.assertAlmostEqual(clearance(points,(0.,0.,0.)),.33)
        self.assertLess(clearance(points,(0.,0.,math.pi/4)),.30)
    def test_pivot_changes_heading_without_translation(self):
        x,y,a=advance((1.,2.,0.),0.,math.pi/2,1.)
        self.assertEqual((x,y),(1.,2.));self.assertAlmostEqual(a,math.pi/2)
    def test_drive_commands_respect_ten_rpm_and_forward_wheels(self):
        rng=np.random.RandomState(11)
        for _ in range(300):
            c=StopTurnGo([('DRIVE',(3.,0.,0.))],v=.06,w=.2,pulse=True)
            v,w,state=c.command((0.,rng.uniform(-.09,.09),rng.uniform(-.2,.2)))
            rpm=max(abs(v-w*.28),abs(v+w*.28))/R*60/(2*math.pi)
            self.assertLessEqual(rpm,10.)
            self.assertGreaterEqual(v-abs(w)*.28,0.)
    def test_turn_command_is_explicitly_separate_from_forward_driver_mode(self):
        c=StopTurnGo([('TURN',(0.,0.,math.pi/2))],v=.06,w=.2,pulse=True)
        v,w,state=c.command((0.,0.,0.))
        self.assertEqual(v,0.);self.assertGreater(w,0.)
        self.assertLessEqual(abs(w)*.28/R*60/(2*math.pi),10.)
    def test_initial_collision_never_reports_success(self):
        c=StopTurnGo([('DRIVE',(3.,0.,0.))])
        r=simulate(np.array([[0.,0.]]),(0.,0.,0.),c,.2,.5,limit=1.)
        self.assertFalse(r['passed']);self.assertEqual(r['reason'],'STOP_TRUE_CLEARANCE')
    def test_evaluation_requires_body_pass_and_extra_meter(self):
        class Straight:
            def command(self,p):return .04,0.,'DRIVE'
        r=simulate(np.array([[-10.,-10.]]),(0.,0.,0.),Straight(),.2,.5,noise=0.,delay=0.,limit=80.)
        self.assertTrue(r['passed']);self.assertGreaterEqual(r['final_pose'][0],1.5)
        self.assertFalse(r['evaluation_pass']);self.assertTrue(r['simulation_only'])
    def test_dogleg_returns_to_original_line_and_adds_meter(self):
        path,goals=dogleg((0.,.04,0.),.16,.82,1.94)
        self.assertAlmostEqual(path[-1][0],3.14);self.assertEqual(path[-1][1:3],(0.,0.))
        self.assertTrue(any(phase=='TURN' for phase,target in goals))

if __name__=='__main__':unittest.main()
