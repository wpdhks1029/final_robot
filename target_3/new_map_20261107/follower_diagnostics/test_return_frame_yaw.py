import unittest
from test_follower_yaw_trial import Tests as FlowTests

class Tests(unittest.TestCase):
    def test_forty_seven_degree_imu_return_is_bounded_and_disarms(self):
        f,code=FlowTests().run_flow(angle_source='imu',target=-47,ratio=1.2)
        self.assertEqual(code,0);self.assertTrue(f.summary['follower_zero_confirmed'])
        self.assertFalse(f.armed);self.assertFalse(f.summary['leader_commanded'])
        angle=f.summary['sources']['imu']['delta_deg']
        self.assertLessEqual(angle,-47.);self.assertGreater(angle,-48.)
        self.assertTrue(all(c['v']==0 and abs(c['w'])<=.2 for c in f.commands))

if __name__=='__main__':unittest.main()
