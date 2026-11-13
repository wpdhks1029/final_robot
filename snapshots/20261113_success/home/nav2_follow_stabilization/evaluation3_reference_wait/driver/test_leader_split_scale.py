import ast
import math
from pathlib import Path
import unittest

class Parent:
    def odom_callback(self):
        left=self.actual_m1_rpm*2*math.pi*self.wheel_radius/60
        right=self.actual_m2_rpm*2*math.pi*self.wheel_radius/60
        if self.fail:raise RuntimeError('injected')
        return (left+right)/2,(right-left)/self.wheel_track

path=Path(__file__).with_name('leader_md200t_polling_split_scale_candidate.py')
tree=ast.parse(path.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PollingMD200TController')
cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='odom_callback']
ns={'MD200TController':Parent};exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),ns)
Node=ns['PollingMD200TController']

class Tests(unittest.TestCase):
    def node(self,yaw):
        n=Node();n.actual_m1_rpm=3.;n.actual_m2_rpm=5.;n.wheel_radius=.085;n.wheel_track=.466
        n.feedback_speed_scale=1.18;n.feedback_yaw_scale=yaw;n.fail=False
        return n
    def test_translation_constant_and_yaw_independent(self):
        a=self.node(1.18);b=self.node(1.)
        va,wa=a.odom_callback();vb,wb=b.odom_callback()
        self.assertAlmostEqual(va,vb);self.assertAlmostEqual(wa/wb,1.18)
        for n in (a,b):
            self.assertEqual(n.wheel_track,.466)
            self.assertEqual((n.actual_m1_rpm,n.actual_m2_rpm),(3.,5.))
    def test_measured_track_and_independent_yaw_scale(self):
        n=self.node(1.);n.wheel_track=.560
        v,w=n.odom_callback()
        self.assertAlmostEqual(v,(3.+5.)/2*1.18*2*math.pi*.085/60)
        self.assertAlmostEqual(w,(5.-3.)*2*math.pi*.085/60/.560)
        self.assertEqual(n.wheel_track,.560)

    def test_exception_restores_shared_command_geometry(self):
        n=self.node(.8);n.fail=True
        with self.assertRaises(RuntimeError):n.odom_callback()
        self.assertEqual(n.wheel_track,.466)
        self.assertEqual((n.actual_m1_rpm,n.actual_m2_rpm),(3.,5.))
    def test_default_retains_previous_combined_scaling(self):
        n=self.node(1.18);del n.feedback_yaw_scale
        v,w=n.odom_callback()
        self.assertAlmostEqual(w,(5.-3.)*1.18*2*math.pi*.085/60/.466)

if __name__=='__main__':unittest.main()
