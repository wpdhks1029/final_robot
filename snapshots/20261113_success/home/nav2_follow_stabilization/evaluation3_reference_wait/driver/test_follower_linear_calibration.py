import unittest
from follower_motor_candidate import MotorCore,Config

class Tests(unittest.TestCase):
    def test_linear_only_and_outgoing_wheels_unchanged(self):
        a=MotorCore(config=Config(max_rpm=10));b=MotorCore(config=Config(max_rpm=10,raw_linear_feedback_scale=1.1474))
        data=dict(m1_rpm=-4,m2_rpm=6)
        for t in (1.,1.1):a._record_feedback(data,t);b._record_feedback(data,t)
        self.assertAlmostEqual(b.odom_v,a.odom_v*1.1474)
        self.assertAlmostEqual(b.odom_x,a.odom_x*1.1474)
        self.assertEqual(b.odom_w,a.odom_w);self.assertEqual(b.odom_yaw,a.odom_yaw)
        self.assertEqual(a.rpm(.04,.02),b.rpm(.04,.02))
    def test_invalid_scale_rejected(self):
        for v in (0.,2.,float('nan')):
            with self.assertRaises(ValueError):Config(raw_linear_feedback_scale=v)

if __name__=='__main__':unittest.main()
