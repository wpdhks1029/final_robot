import math,unittest
import numpy as np
from apf_pivot_candidate import APFPivot,checked_step

class Tests(unittest.TestCase):
    def test_front_clear_does_not_remove_rear_corner_rotation_obstacle(self):
        ok,d,path=checked_step(np.array([[-.45,-.45]]),(0.,0.,0.),math.pi/2,.1)
        self.assertFalse(ok)
    def test_open_space_chooses_a_diagonal_instead_of_fixed_right_angle(self):
        c=APFPivot((0.,0.,0.),dict(near=1.,far=1.2,high=.1))
        h,target,path=c.choose((0.,0.,0.),np.array([[4.,4.]]))
        self.assertGreater(h,0.);self.assertLess(h,math.pi/2)
        self.assertGreater(target[0],0.);self.assertGreater(target[1],0.)
    def test_blocked_scene_returns_stop(self):
        c=APFPivot((0.,0.,0.),dict(near=1.,far=1.2,high=.1))
        v,w,reason=c.command((0.,0.,0.),np.array([[0.,0.]]))
        self.assertEqual((v,w),(0.,0.));self.assertEqual(reason,'STOP_APF_NO_CHECKED_STEP')
    def test_turn_to_drive_has_explicit_brake(self):
        c=APFPivot((0.,0.,0.),dict(near=1.,far=1.2,high=.1))
        c.phase='TURN';c.heading=.5;c.target=(.1,.1)
        self.assertEqual(c.command((0.,0.,.5),np.array([[4.,4.]])),(0.,0.,'BRAKE'))
    def test_rejoined_reference_continues_forward_at_old_final_target(self):
        c=APFPivot((0.,0.,0.),dict(near=1.,far=1.2,high=.1));c.stage=3
        x=c.targets[-1][0]
        h,target,path=c.choose((x,0.,0.),np.array([[-4.,-4.]]))
        self.assertGreater(target[0],x);self.assertAlmostEqual(h,0.)
    def test_shadow_has_no_control_publishers_or_services(self):
        import ast
        from pathlib import Path
        tree=ast.parse(Path(__file__).with_name('watch_apf_pivot_shadow.py').read_text())
        calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertFalse({'create_publisher','create_client','publish','call_async'} & set(calls))

# Reuse the full measured-feedback driver/phase-contract harness. This is a
# separate memory transport run, not the kinematic replay above.
import test_local_stop_turn_go as _memory_tests

class APFMemoryClosedLoop(_memory_tests.MemoryClosedLoop):
    def plan_for_test(self,d):
        from apf_pivot_candidate import build_apf_pivot_plan
        return build_apf_pivot_plan(d['points'],d['start'])

if __name__=='__main__':unittest.main()
