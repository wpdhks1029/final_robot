from pathlib import Path
import sys,unittest
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parent/'candidate/follower_diagnostics'))
from reference_wait import ReferenceWait,reference_stamps,ReferenceUnavailable
import test_continuous20 as fixture

class WaitTests(unittest.TestCase):
    def test_wait_has_no_expiry_and_requires_new_samples_and_zero(self):
        w=ReferenceWait();self.assertTrue(w.enter(1.,invalid=True))
        for t in (10.,100.,1000.):w.enter(t,invalid=True)
        self.assertTrue(w.active)
        for i in range(15):
            self.assertFalse(w.ready(1000.+i*.1,{'leader':1,'follower_map':2,'scan':3},True))
        for i in range(20):
            result=w.ready(1002.+i*.1,{'leader':100+i,'follower_map':200+i,'scan':300+i},False)
            self.assertFalse(result)
        self.assertTrue(w.ready(1004.,{'leader':121,'follower_map':221,'scan':321},True))
        self.assertGreater(w.finish(1004.),1000.);self.assertFalse(w.active)
    def test_backward_source_requires_stabilization_again(self):
        w=ReferenceWait();w.enter(1.)
        w.ready(1.,{'leader':5},True)
        self.assertFalse(w.ready(1.1,{'leader':4},True));self.assertIsNone(w.stable_since)
    def test_local_zero_scene_is_available_without_remote_leader_but_motion_scene_is_not(self):
        c,ns=fixture.CoreTests().core();c.offer_scan(ns,1.,[(5.,5.)]);c.inputs.pop('leader')
        local=c.scene(ns,1.,require_leader=False)
        self.assertNotIn('leader',local['source_stamps'])
        with self.assertRaisesRegex(ValueError,'LEADER_MISSING'):c.scene(ns,1.)
    def test_local_stale_imu_is_not_masked_by_reference_wait(self):
        c,ns=fixture.CoreTests().core();c.offer_scan(ns,1.,[(5.,5.)]);c.inputs.pop('imu')
        with self.assertRaisesRegex(ValueError,'IMU_MISSING'):c.scene(ns,1.,require_leader=False)

if __name__=='__main__':unittest.main()
