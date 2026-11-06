import ast,unittest
from pathlib import Path
from types import SimpleNamespace as S
from tracked_pursuit_candidate import TrackedPreviewCore
from local_stop_turn_go import LocalCore

class Tests(unittest.TestCase):
    def core(self,points,pose=(0.,.1,0.,0.,0.)):
        c=TrackedPreviewCore();c.leader_moving=True
        t=S(sec=100,nanosec=0)
        c.inputs={'leader':(t,1.,(5.,0.,0.,.04,0.)),'follower':(t,1.,pose),'path':(t,1.,points)}
        return c
    def test_pure_pursuit_turns_towards_map_path(self):
        c=self.core([(0,0),(1,0),(2,0)])
        v,w,r=c.command(100_000_000_000,1.)
        self.assertGreater(v,0);self.assertLess(w,0);self.assertEqual(r,'TRACKED_PURSUIT')
        self.assertAlmostEqual(w,2*(-.1)/(.8**2+.1**2)*v)
    def test_reverse_history_never_drives(self):
        c=self.core([(1,0),(0,0),(2,0)])
        v,w,r=c.command(100_000_000_000,1.)
        self.assertEqual((v,w),(0.,0.));self.assertIn('SHARP_REVERSAL',r)
    def test_handoff_requires_measured_zero(self):
        c=LocalCore()
        with self.assertRaises(ValueError):c.handoff_to_follow({'measured_rpm':[0,1],'zero_feedback_samples':0})
        c.handoff_to_follow({'measured_rpm':[0,0],'zero_feedback_samples':3})
        self.assertIsNone(c.fsm);self.assertIsNone(c.plan)
    def test_shadow_no_publishers_or_services(self):
        tree=ast.parse(Path(__file__).with_name('watch_directed_follow_shadow.py').read_text())
        calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertFalse({'create_publisher','create_client','publish','call_async'}&calls)

if __name__=='__main__':unittest.main()
