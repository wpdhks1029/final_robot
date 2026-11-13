import unittest
from compare_leader_turn_ekf import parameters

class ParametersTests(unittest.TestCase):
    def test_candidate_uses_wheel_yaw_rate_and_keeps_translation(self):
        current=parameters();candidate=parameters(True)
        self.assertEqual([i for i,x in enumerate(current['odom0_config']) if x],[6,7])
        self.assertEqual([i for i,x in enumerate(candidate['odom0_config']) if x],[6,7,11])
        self.assertFalse(any(k.startswith('imu0') for k in candidate))
        self.assertTrue(current['imu0_config'][5]);self.assertTrue(current['imu0_config'][11])
        for k,v in candidate.items():
            if k!='odom0_config':self.assertEqual(v,current[k])
    def test_dump_null_parameters_are_not_reapplied(self):
        self.assertFalse(any(v is None for v in parameters(True).values()))

if __name__=='__main__':unittest.main()
